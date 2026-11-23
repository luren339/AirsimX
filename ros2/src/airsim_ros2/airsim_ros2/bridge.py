"""ROS 2 bridge between AirSim and standard ROS message types.

The bridge talks to the simulator through the repository's ``airsim``
Python client, so it works locally or against a remote simulator (for
example over an SSH tunnel). It publishes odometry in both the AirSim NED
convention and ROS ENU, plus GPS, IMU, barometer and environment data,
and accepts velocity commands, pose goals and the legacy
``airsim_interfaces`` takeoff/land/reset services.
"""

import math
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Image, Imu, NavSatFix
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from airsim_interfaces.msg import Altimeter, Environment, VelCmd
from airsim_interfaces.srv import Land, Reset, Takeoff

import airsim
from . import conversions


def _qos(depth=10, reliable=True):
    return QoSProfile(depth=depth,
                      reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


class AirsimBridge(Node):
    def __init__(self):
        super().__init__('airsim_bridge')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 41451)
        self.declare_parameter('vehicle_name', 'PX4SITL')
        self.declare_parameter('odom_rate_hz', 30.0)
        self.declare_parameter('imu_rate_hz', 50.0)
        self.declare_parameter('gps_rate_hz', 1.0)
        self.declare_parameter('baro_rate_hz', 2.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('arm_wait_seconds', 30.0)
        # Takeoff and land block on the server until the manoeuvre finishes;
        # a firmware vehicle descends for tens of seconds, so the RPC timeout
        # must cover that plus the disarm that follows.
        self.declare_parameter('rpc_timeout', 60.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('world_frame', 'world_ned')
        # Camera frames. image_rate_hz=0 disables publishing.
        self.declare_parameter('camera_name', '0')
        self.declare_parameter('image_rate_hz', 10.0)
        self.declare_parameter('image_type', int(airsim.ImageType.Scene))

        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)
        self.vehicle = str(self.get_parameter('vehicle_name').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.camera_name = str(self.get_parameter('camera_name').value)
        self.image_rate_hz = float(self.get_parameter('image_rate_hz').value)
        self.image_type = int(self.get_parameter('image_type').value)
        self.rpc_timeout = float(self.get_parameter('rpc_timeout').value)

        self.client = airsim.MultirotorClient(ip=host, port=port,
                                              timeout_value=self.rpc_timeout)
        self.prefix = '/' + self.vehicle
        self.lock = threading.Lock()

        self.odom_ned_pub = self.create_publisher(Odometry, self.prefix + '/odom_local_ned', _qos())
        self.odom_enu_pub = self.create_publisher(Odometry, self.prefix + '/odom', _qos())
        self.gps_pub = self.create_publisher(NavSatFix, self.prefix + '/global_gps', _qos())
        self.imu_pub = self.create_publisher(Imu, self.prefix + '/imu', _qos(50, reliable=False))
        self.altimeter_pub = self.create_publisher(Altimeter, self.prefix + '/altimeter', _qos())
        self.pressure_pub = self.create_publisher(FluidPressure, self.prefix + '/air_pressure', _qos())
        self.env_pub = self.create_publisher(Environment, self.prefix + '/environment', _qos())
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.camera_frame = '%s_camera_%s_optical' % (self.vehicle, self.camera_name)
        # ROS topic segments may not start with a digit, and AirSim names its
        # default camera "0", so the image namespace is camera_<name>.
        self.camera_topic = self.prefix + '/camera_' + self.camera_name + '/image_raw'
        self.image_pub = None
        self._static_tf = StaticTransformBroadcaster(self) if self.publish_tf else None
        self._camera_tf_sent = False
        self._images_published = 0

        # Commands share one group: takeoff and land block for tens of
        # seconds, and a velocity command must not run in the middle of one.
        self.command_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Takeoff, self.prefix + '/takeoff', self.on_takeoff,
                            callback_group=self.command_group)
        self.create_service(Land, self.prefix + '/land', self.on_land,
                            callback_group=self.command_group)
        self.create_service(Reset, self.prefix + '/reset', self.on_reset,
                            callback_group=self.command_group)
        self.create_service(Trigger, self.prefix + '/arm', self.on_arm,
                            callback_group=self.command_group)
        self.create_service(Trigger, self.prefix + '/disarm', self.on_disarm,
                            callback_group=self.command_group)
        self.create_service(Trigger, self.prefix + '/enable_api_control', self.on_enable_api,
                            callback_group=self.command_group)
        self.create_service(Trigger, self.prefix + '/disable_api_control', self.on_disable_api,
                            callback_group=self.command_group)
        self.create_service(Trigger, self.prefix + '/hover', self.on_hover,
                            callback_group=self.command_group)

        self.create_subscription(VelCmd, self.prefix + '/vel_cmd_world_frame', self.on_vel_cmd_world,
                                 _qos(1), callback_group=self.command_group)
        self.create_subscription(Twist, self.prefix + '/cmd_vel_enu', self.on_cmd_vel_enu,
                                 _qos(1), callback_group=self.command_group)
        self.create_subscription(PoseStamped, self.prefix + '/goal_pose_enu', self.on_goal_pose_enu,
                                 _qos(1), callback_group=self.command_group)

        # Every call to the simulator blocks for one round trip. Against a
        # remote simulator that is tens of milliseconds, so the telemetry
        # timers must not share a callback group: with the default
        # mutually-exclusive group they run one after another and the topic
        # rates collapse to the sum of the RPC latencies. One group per timer
        # lets them overlap without letting a single timer re-enter itself.
        self.odom_group = MutuallyExclusiveCallbackGroup()
        self.imu_group = MutuallyExclusiveCallbackGroup()
        self.gps_group = MutuallyExclusiveCallbackGroup()
        self.baro_group = MutuallyExclusiveCallbackGroup()
        self.image_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / float(self.get_parameter('odom_rate_hz').value), self.publish_odometry,
                          callback_group=self.odom_group)
        self.create_timer(1.0 / float(self.get_parameter('imu_rate_hz').value), self.publish_imu,
                          callback_group=self.imu_group)
        self.create_timer(1.0 / float(self.get_parameter('gps_rate_hz').value), self.publish_gps,
                          callback_group=self.gps_group)
        self.create_timer(1.0 / float(self.get_parameter('baro_rate_hz').value), self.publish_environment,
                          callback_group=self.baro_group)
        if self.image_rate_hz > 0:
            self.image_pub = self.create_publisher(Image, self.camera_topic,
                                                   _qos(1, reliable=False))
            self.create_timer(1.0 / self.image_rate_hz, self.publish_image,
                              callback_group=self.image_group)
        self.get_logger().info(
            'AirsimBridge for %s via %s:%d (ENU frame %s, NED frame %s)'
            % (self.vehicle, host, port, self.map_frame, self.world_frame))
        if self.image_pub is not None:
            self.get_logger().info(
                'Publishing camera %s at %.1f Hz on %s'
                % (self.camera_name, self.image_rate_hz, self.camera_topic))

    # ---------------------------------------------------------------- helpers
    def state(self):
        return self.client.getMultirotorState(vehicle_name=self.vehicle)

    def call(self, label, function, *args, **kwargs):
        try:
            return function(*args, **kwargs)
        except Exception as error:  # keep the node alive when the simulator drops
            self.get_logger().error('%s failed: %r' % (label, error))
            return None

    def set_api_control(self, enabled):
        # enableApiControl returns None on success, so wrap it in a bool.
        def apply():
            self.client.enableApiControl(enabled, self.vehicle)
            return True
        return self.call('enable api' if enabled else 'disable api', apply) is True

    def fill_header(self, header, timestamp_ns, frame):
        header.stamp.sec, header.stamp.nanosec = conversions.stamp_from_nanoseconds(int(timestamp_ns))
        header.frame_id = frame

    # ------------------------------------------------------------- publishing
    def publish_odometry(self):
        state = self.call('odometry', self.state)
        if state is None:
            return
        k = state.kinematics_estimated
        n, e, d = k.position.x_val, k.position.y_val, k.position.z_val
        vn, ve, vd = k.linear_velocity.x_val, k.linear_velocity.y_val, k.linear_velocity.z_val
        w, x, y, z = k.orientation.w_val, k.orientation.x_val, k.orientation.y_val, k.orientation.z_val
        timestamp = state.timestamp

        ned = Odometry()
        self.fill_header(ned.header, timestamp, self.world_frame)
        ned.child_frame_id = self.base_frame
        ned.pose.pose.position.x, ned.pose.pose.position.y, ned.pose.pose.position.z = n, e, d
        ned.pose.pose.orientation.w, ned.pose.pose.orientation.x = w, x
        ned.pose.pose.orientation.y, ned.pose.pose.orientation.z = y, z
        ned.twist.twist.linear.x, ned.twist.twist.linear.y, ned.twist.twist.linear.z = vn, ve, vd
        self.odom_ned_pub.publish(ned)

        east, north, up = conversions.ned_to_enu_vector(n, e, d)
        v_east, v_north, v_up = conversions.ned_to_enu_vector(vn, ve, vd)
        qw, qx, qy, qz = conversions.ned_quaternion_to_enu(w, x, y, z)
        enu = Odometry()
        self.fill_header(enu.header, timestamp, self.map_frame)
        enu.child_frame_id = self.base_frame
        enu.pose.pose.position.x, enu.pose.pose.position.y, enu.pose.pose.position.z = east, north, up
        enu.pose.pose.orientation.w, enu.pose.pose.orientation.x = qw, qx
        enu.pose.pose.orientation.y, enu.pose.pose.orientation.z = qy, qz
        enu.twist.twist.linear.x, enu.twist.twist.linear.y, enu.twist.twist.linear.z = v_east, v_north, v_up
        self.odom_enu_pub.publish(enu)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            self.fill_header(transform.header, timestamp, self.map_frame)
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = east
            transform.transform.translation.y = north
            transform.transform.translation.z = up
            transform.transform.rotation.w, transform.transform.rotation.x = qw, qx
            transform.transform.rotation.y, transform.transform.rotation.z = qy, qz
            self.tf_broadcaster.sendTransform(transform)

    def publish_imu(self):
        data = self.call('imu', self.client.getImuData, imu_name='Imu', vehicle_name=self.vehicle)
        if data is None:
            return
        ax, ay, az = conversions.frd_to_flu_vector(
            data.linear_acceleration.x_val, data.linear_acceleration.y_val, data.linear_acceleration.z_val)
        gx, gy, gz = conversions.frd_to_flu_vector(
            data.angular_velocity.x_val, data.angular_velocity.y_val, data.angular_velocity.z_val)
        w, x, y, z = conversions.ned_quaternion_to_enu(
            data.orientation.w_val, data.orientation.x_val, data.orientation.y_val, data.orientation.z_val)
        message = Imu()
        self.fill_header(message.header, data.time_stamp, self.base_frame)
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = ax, ay, az
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = gx, gy, gz
        message.orientation.w, message.orientation.x = w, x
        message.orientation.y, message.orientation.z = y, z
        # sensor_msgs/Imu uses flat 3x3 covariance arrays; -1 on the first
        # element marks the value as unavailable, per REP 145.
        message.orientation_covariance[0] = -1.0
        message.angular_velocity_covariance[0] = -1.0
        message.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(message)

    def publish_gps(self):
        gps = self.call('gps', self.client.getGpsData, gps_name='Gps', vehicle_name=self.vehicle)
        if gps is None:
            return
        geo = gps.gnss.geo_point
        message = NavSatFix()
        self.fill_header(message.header, gps.time_stamp, self.map_frame)
        message.latitude, message.longitude, message.altitude = geo.latitude, geo.longitude, geo.altitude
        message.status.service = 1
        message.status.status = conversions.gps_fix_type(gps.gnss.fix_type)
        message.position_covariance = conversions.covariances(max(gps.gnss.eph, gps.gnss.epv, 0.01))
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(message)

    def publish_image(self):
        """One raw camera frame as sensor_msgs/Image.

        AirSim returns tightly packed B, G, R bytes for compress=False, which
        is exactly ROS's ``bgr8`` encoding, so no decode step is involved.
        The frame carries AirSim's capture timestamp (simulation time) so it
        lines up with odometry and point clouds taken from the same moment.
        """
        request = airsim.ImageRequest(self.camera_name, self.image_type,
                                      pixels_as_float=False, compress=False)
        responses = self.call('image', self.client.simGetImages, [request],
                              vehicle_name=self.vehicle)
        if not responses:
            return
        response = responses[0]
        height, width = int(response.height), int(response.width)
        message_data = response.image_data_uint8
        if height <= 0 or width <= 0 or len(message_data) != height * width * 3:
            return
        message = Image()
        self.fill_header(message.header, response.time_stamp, self.camera_frame)
        message.height = height
        message.width = width
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = width * 3
        message.data = bytes(message_data)
        self.image_pub.publish(message)
        self._images_published += 1
        if self._static_tf is not None and not self._camera_tf_sent:
            self.publish_camera_transform(response)
        if self._images_published % 100 == 0:
            self.get_logger().info('published %d camera frames' % self._images_published)

    def publish_camera_transform(self, response):
        """Static base_link -> camera optical frame, from one captured frame.

        The camera is rigidly mounted, so the relative pose measured once is
        valid for the rest of the run. Vehicle and camera poses are converted
        from AirSim's NED/FRD with the same helpers the odometry uses.
        """
        state = self.call('odometry', self.state)
        if state is None:
            return
        k = state.kinematics_estimated
        vehicle_position = conversions.ned_to_enu_vector(k.position.x_val, k.position.y_val, k.position.z_val)
        vehicle_orientation = conversions.ned_quaternion_to_enu(
            k.orientation.w_val, k.orientation.x_val, k.orientation.y_val, k.orientation.z_val)
        camera = response.camera_position
        camera_orientation = response.camera_orientation
        camera_position = conversions.ned_to_enu_vector(camera.x_val, camera.y_val, camera.z_val)
        camera_quaternion = conversions.ned_quaternion_to_enu(
            camera_orientation.w_val, camera_orientation.x_val,
            camera_orientation.y_val, camera_orientation.z_val)
        translation, rotation = conversions.camera_optical_pose(
            vehicle_position, vehicle_orientation, camera_position, camera_quaternion)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.camera_frame
        transform.transform.translation.x, transform.transform.translation.y, \
            transform.transform.translation.z = (float(v) for v in translation)
        transform.transform.rotation.w, transform.transform.rotation.x, \
            transform.transform.rotation.y, transform.transform.rotation.z = \
            (float(v) for v in rotation)
        self._static_tf.sendTransform(transform)
        self._camera_tf_sent = True

    def publish_environment(self):
        baro = self.call('barometer', self.client.getBarometerData, barometer_name='Barometer', vehicle_name=self.vehicle)
        if baro is None:
            return
        altimeter = Altimeter()
        self.fill_header(altimeter.header, baro.time_stamp, self.map_frame)
        altimeter.altitude, altimeter.pressure, altimeter.qnh = baro.altitude, baro.pressure, baro.qnh
        self.altimeter_pub.publish(altimeter)
        pressure = FluidPressure()
        self.fill_header(pressure.header, baro.time_stamp, self.map_frame)
        pressure.fluid_pressure = baro.pressure
        self.pressure_pub.publish(pressure)
        self.env_pub.publish(self.environment_message(baro))

    def environment_message(self, baro):
        environment = self.call('environment', self.client.simGetGroundTruthEnvironment, vehicle_name=self.vehicle)
        message = Environment()
        self.fill_header(message.header, baro.time_stamp, self.map_frame)
        if environment is not None:
            message.position.x, message.position.y, message.position.z = (
                environment.position.x_val, environment.position.y_val, environment.position.z_val)
            message.geo_point.latitude = environment.geo_point.latitude
            message.geo_point.longitude = environment.geo_point.longitude
            message.geo_point.altitude = environment.geo_point.altitude
            message.gravity.x, message.gravity.y, message.gravity.z = (
                environment.gravity.x_val, environment.gravity.y_val, environment.gravity.z_val)
            message.air_pressure = environment.air_pressure
            message.temperature = environment.temperature
            message.air_density = environment.air_density
        else:
            message.air_pressure = baro.pressure
        return message

    # -------------------------------------------------------------- commands
    def on_vel_cmd_world(self, message):
        twist = message.twist
        self.call('vel_cmd_world', self.client.moveByVelocityAsync,
                  twist.linear.x, twist.linear.y, twist.linear.z, 1.0,
                  yaw_mode=airsim.YawMode(False, 0), vehicle_name=self.vehicle)

    def on_cmd_vel_enu(self, message):
        linear, angular = conversions.twist_enu_to_ned(
            (message.linear.x, message.linear.y, message.linear.z),
            (message.angular.x, message.angular.y, message.angular.z))
        # angular[2] is already in AirSim's clockwise-from-north/down-positive
        # convention; YawMode expects degrees.
        yaw_rate = math.degrees(angular[2])
        self.call('cmd_vel_enu', self.client.moveByVelocityAsync,
                  linear[0], linear[1], linear[2], 1.0,
                  yaw_mode=airsim.YawMode(True, yaw_rate), vehicle_name=self.vehicle)

    def on_goal_pose_enu(self, message):
        position = message.pose.position
        north, east, down = conversions.enu_to_ned_vector(position.x, position.y, position.z)
        self.call('goal_pose_enu', self.client.moveToPositionAsync,
                  north, east, down, 3.0, vehicle_name=self.vehicle)

    def on_takeoff(self, request, response):
        response.success = self.takeoff(wait=request.wait_on_last_task)
        return response

    def arm_with_retry(self):
        # PX4 denies arming while its estimator settles after the first GPS fix,
        # and an RC-less PX4 in Position mode after landing also denies it; Hold
        # mode removes that second requirement. Retry within a bounded window.
        deadline = time.monotonic() + float(self.get_parameter('arm_wait_seconds').value)
        attempted_hold = False
        while True:
            if self.call('arm', self.client.armDisarm, True, self.vehicle) is True:
                return True
            if not attempted_hold:
                self.call('hold mode before arming', lambda: self.client.hoverAsync(vehicle_name=self.vehicle).get())
                attempted_hold = True
            if time.monotonic() >= deadline:
                return False
            time.sleep(1.0)

    def takeoff(self, wait=True):
        with self.lock:
            started = time.monotonic()
            self.get_logger().info('takeoff requested')
            if not self.set_api_control(True):
                return False
            if not self.arm_with_retry():
                self.get_logger().error('takeoff aborted: vehicle did not accept arming')
                return False
            self.get_logger().info('armed after %.1fs' % (time.monotonic() - started))
            result = self.call('takeoff', lambda: self.client.takeoffAsync(timeout_sec=20, vehicle_name=self.vehicle).get())
            self.get_logger().info('takeoff result %r after %.1fs' % (result, time.monotonic() - started))
            if result is not True:
                return False
            if not wait:
                return True
            hovered = self.call('hover', lambda: self.client.hoverAsync(vehicle_name=self.vehicle).get())
            self.get_logger().info('hover result %r after %.1fs' % (hovered, time.monotonic() - started))
            return hovered is True

    def on_land(self, request, response):
        response.success = self.land()
        return response

    def land(self):
        with self.lock:
            if self.call('land', lambda: self.client.landAsync(timeout_sec=30, vehicle_name=self.vehicle).get()) is not True:
                return False
            if self.call('disarm', self.client.armDisarm, False, self.vehicle) is not True:
                return False
            return self.set_api_control(False)

    def on_reset(self, request, response):
        response.success = self.call('reset', self.client.reset) is not None
        return response

    def on_arm(self, request, response):
        response.success = self.call('arm', self.client.armDisarm, True, self.vehicle) is True
        response.message = 'armed' if response.success else 'arm rejected'
        return response

    def on_disarm(self, request, response):
        response.success = self.call('disarm', self.client.armDisarm, False, self.vehicle) is True
        response.message = 'disarmed' if response.success else 'disarm rejected'
        return response

    def on_enable_api(self, request, response):
        response.success = self.set_api_control(True)
        response.message = 'api control enabled' if response.success else 'enable failed'
        return response

    def on_disable_api(self, request, response):
        response.success = self.set_api_control(False)
        response.message = 'api control disabled' if response.success else 'disable failed'
        return response

    def on_hover(self, request, response):
        response.success = self.call('hover', lambda: self.client.hoverAsync(vehicle_name=self.vehicle).get()) is True
        response.message = 'holding' if response.success else 'hover failed'
        return response

    def destroy_node(self):
        self.client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AirsimBridge()
    # The node's callback groups are per-timer (see AirsimBridge.__init__), so
    # a multi-threaded executor is what actually lets a slow RPC overlap with
    # the other topics.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
