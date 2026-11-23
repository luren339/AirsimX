"""Record AirSim ground truth in the CSV shape the stack's validator reads.

The navigation stack judges arrival and crash outcomes against the simulation's
true pose, and it reads that from ``gz_pose.csv`` in the run directory (written
by a Gazebo pose recorder). This node writes the same file from AirSim ground
truth and publishes the same pose as ``nav_msgs/Odometry`` for inspection.

Columns, matching the Gazebo record the validators parse:
``sim_seconds, x, y, z, qx, qy, qz, qw, wall_seconds`` with the position in the
planner's map frame (east, north, up) derived from AirSim's NED truth.

The file is flushed on every sample so a flight that ends badly still leaves
its record.
"""

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import airsim
from airsim.frames import ned_quaternion_to_enu, ned_to_enu_vector


class TruthRecorder(Node):
    def __init__(self, output_path):
        super().__init__('airsim_truth_recorder')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 41452)
        self.declare_parameter('vehicle_name', 'PX4SITL')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        self.vehicle = str(self.get_parameter('vehicle_name').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.client = airsim.MultirotorClient(ip=host, port=port, timeout_value=10.0)

        self.stream = output_path.open('w', encoding='utf-8', newline='')
        self.lock = threading.Lock()
        self.samples = 0
        self.odom_pub = self.create_publisher(Odometry, '/drone_city_nav/truth/odom', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.sample)
        self.get_logger().info('recording AirSim ground truth to %s' % output_path)

    def sample(self):
        try:
            state = self.client.simGetGroundTruthKinematics(vehicle_name=self.vehicle)
        except Exception as error:
            self.get_logger().error('ground truth failed: %r' % (error,))
            return
        stamp = int(self.client.getMultirotorState(vehicle_name=self.vehicle).timestamp)
        p = state.position
        q = state.orientation
        east, north, up = ned_to_enu_vector(p.x_val, p.y_val, p.z_val)
        qw, qx, qy, qz = ned_quaternion_to_enu(q.w_val, q.x_val, q.y_val, q.z_val)
        wall = time.time()
        with self.lock:
            self.stream.write('%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n'
                              % (stamp / 1e9, east, north, up, qx, qy, qz, qw, wall))
            self.stream.flush()
            self.samples += 1

        message = Odometry()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
        message.header.frame_id = self.map_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = east, north, up
        message.pose.pose.orientation.w, message.pose.pose.orientation.x = qw, qx
        message.pose.pose.orientation.y, message.pose.pose.orientation.z = qy, qz
        vx, vy, vz = state.linear_velocity.x_val, state.linear_velocity.y_val, state.linear_velocity.z_val
        v_east, v_north, v_up = ned_to_enu_vector(vx, vy, vz)
        message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = v_east, v_north, v_up
        self.odom_pub.publish(message)
        if self.samples % 200 == 0:
            self.get_logger().info('%d truth samples' % self.samples)

    def close(self):
        with self.lock:
            self.stream.close()
        self.client.close()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='path of the gz_pose.csv-compatible record')
    parsed, ros_args = parser.parse_known_args(args)
    from pathlib import Path
    rclpy.init(args=ros_args)
    node = TruthRecorder(Path(parsed.output))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
