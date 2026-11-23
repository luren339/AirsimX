"""Publish AirSim's clock and sensors in the form a ROS 2 navigation stack expects.

The stack this serves was written against a Gazebo world: it reads a
simulation clock, an organized or hit-only 3D lidar cloud, a barometer and a
magnetometer, and it judges arrival against ground truth. AirSim (Unreal
Engine) can supply all of that, but not on the same topics or in the same
conventions, so this node translates:

* ``/clock`` from AirSim's simulation clock. Every node in the stack runs
  ``use_sim_time``, and PX4 is started with ``UXRCE_DDS_SYNCT=0`` so its
  stamps are ROS time already; the clock published here is the same AirSim
  time PX4 stamps its HIL messages with.
* ``/lidar_3d/points`` from an AirSim lidar. AirSim reports hits only (a ray
  that reaches nothing is not in the response) and in NED body axes, so the
  cloud is converted to the body FLU the stack consumes and the stack is
  expected to run with ``lidar_3d_hit_only_returns: true``.
* ``/barometer/air_pressure`` and ``/magnetometer/magnetic_field``.

Frames: the cloud is expressed in the lidar's own frame with body-FLU axes,
which is what the stack's ``config/sensor_extrinsics.yaml`` assumes; the
AirSim lidar must be mounted at the pose that file states. The barometer
and magnetometer carry ``base_link``.

The node is deliberately read-only against the simulator: it never arms,
moves or resets the vehicle.
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import FluidPressure, MagneticField, PointCloud2, PointField

import airsim


def sensor_qos(depth=5):
    """Best-effort, as the stack's sensor consumers subscribe."""
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


class AirsimSource(Node):
    def __init__(self):
        super().__init__('airsim_source')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 41452)
        self.declare_parameter('vehicle_name', 'PX4SITL')
        self.declare_parameter('lidar_name', 'lidar_3d')
        self.declare_parameter('lidar_frame', 'lidar_3d')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('clock_rate_hz', 50.0)
        self.declare_parameter('lidar_rate_hz', 10.0)
        self.declare_parameter('baro_rate_hz', 20.0)
        self.declare_parameter('mag_rate_hz', 20.0)
        self.declare_parameter('rpc_timeout', 10.0)

        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        self.vehicle = str(self.get_parameter('vehicle_name').value)
        self.lidar_name = str(self.get_parameter('lidar_name').value)
        self.lidar_frame = str(self.get_parameter('lidar_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.client = airsim.MultirotorClient(
            ip=host, port=port, timeout_value=float(self.get_parameter('rpc_timeout').value))
        self.lock = threading.Lock()
        self.frames = 0
        self.failures = 0

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/lidar_3d/points', sensor_qos())
        self.baro_pub = self.create_publisher(FluidPressure, '/barometer/air_pressure', sensor_qos())
        self.mag_pub = self.create_publisher(MagneticField, '/magnetometer/magnetic_field', sensor_qos())

        # One group per timer, as in airsim_ros2: every call blocks for one
        # round trip, and a shared group would serialize the topics.
        self.groups = {name: MutuallyExclusiveCallbackGroup()
                       for name in ('clock', 'lidar', 'baro', 'mag')}
        self.create_timer(1.0 / float(self.get_parameter('clock_rate_hz').value),
                          self.publish_clock, callback_group=self.groups['clock'])
        if float(self.get_parameter('lidar_rate_hz').value) > 0:
            self.create_timer(1.0 / float(self.get_parameter('lidar_rate_hz').value),
                              self.publish_lidar, callback_group=self.groups['lidar'])
        self.create_timer(1.0 / float(self.get_parameter('baro_rate_hz').value),
                          self.publish_barometer, callback_group=self.groups['baro'])
        self.create_timer(1.0 / float(self.get_parameter('mag_rate_hz').value),
                          self.publish_magnetometer, callback_group=self.groups['mag'])
        self.get_logger().info(
            'AirSim source for %s via %s:%d (clock %.0f Hz, lidar %.0f Hz)'
            % (self.vehicle, host, port,
               float(self.get_parameter('clock_rate_hz').value),
               float(self.get_parameter('lidar_rate_hz').value)))

    # ------------------------------------------------------------- helpers
    def call(self, label, function, *args, **kwargs):
        try:
            return function(*args, **kwargs)
        except Exception as error:  # keep publishing after a simulator restart
            self.failures += 1
            if self.failures % 20 == 1:
                self.get_logger().error('%s failed: %r' % (label, error))
            return None

    def sim_time_ns(self):
        state = self.call('state', self.client.getMultirotorState, vehicle_name=self.vehicle)
        if state is None:
            return None
        return int(state.timestamp)

    # ---------------------------------------------------------- publishing
    def publish_clock(self):
        stamp = self.sim_time_ns()
        if stamp is None:
            return
        message = Clock()
        message.clock.sec, message.clock.nanosec = divmod(stamp, 10**9)
        self.clock_pub.publish(message)

    def publish_lidar(self):
        data = self.call('lidar', self.client.getLidarData, lidar_name=self.lidar_name,
                         vehicle_name=self.vehicle)
        if data is None:
            return
        points = np.asarray(data.point_cloud, dtype=np.float32)
        if points.size % 3:
            return
        points = points.reshape(-1, 3)
        # AirSim reports NED body axes; the stack consumes FLU body axes.
        points = np.column_stack((points[:, 0], -points[:, 1], -points[:, 2]))
        message = PointCloud2()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(int(data.time_stamp), 10**9)
        message.header.frame_id = self.lidar_frame
        message.height = 1
        message.width = int(points.shape[0])
        message.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.is_dense = bool(np.isfinite(points).all())
        message.data = np.ascontiguousarray(points).tobytes()
        self.lidar_pub.publish(message)
        self.frames += 1
        if self.frames % 100 == 0:
            self.get_logger().info('published %d lidar frames (%d points last)'
                                   % (self.frames, message.width))

    def publish_barometer(self):
        baro = self.call('barometer', self.client.getBarometerData, barometer_name='Barometer',
                         vehicle_name=self.vehicle)
        if baro is None:
            return
        message = FluidPressure()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(int(baro.time_stamp), 10**9)
        message.header.frame_id = self.base_frame
        message.fluid_pressure = float(baro.pressure)
        message.variance = 0.0
        self.baro_pub.publish(message)

    def publish_magnetometer(self):
        mag = self.call('magnetometer', self.client.getMagnetometerData,
                        magnetometer_name='Magnetometer', vehicle_name=self.vehicle)
        if mag is None:
            return
        body = mag.magnetic_field_body
        message = MagneticField()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(int(mag.time_stamp), 10**9)
        message.header.frame_id = self.base_frame
        # AirSim reports the field in NED body axes; the stack reads FLU and
        # negates y and z back to FRD itself.
        message.magnetic_field.x = float(body.x_val)
        message.magnetic_field.y = float(-body.y_val)
        message.magnetic_field.z = float(-body.z_val)
        self.mag_pub.publish(message)

    def destroy_node(self):
        self.client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AirsimSource()
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
