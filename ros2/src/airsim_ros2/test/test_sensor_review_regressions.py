"""Review triggers using real Jazzy messages and actual local RPC sockets."""
import os
from pathlib import Path
import subprocess
import sys
import unittest

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
except ImportError:
    raise unittest.SkipTest('sensor node regressions require ROS 2 Jazzy')

import airsim
from airsim_ros2.sensor_bridge import AirSimRos2SensorBridge

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'PythonClient/tests'))
from rpc_sensor_server import SensorRpcServer, wait_for_idle_disconnect


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class SensorReviewRegressionTests(unittest.TestCase):
    def setUp(self):
        self.context = Context()
        rclpy.init(context=self.context)
        self.nodes = []

    def tearDown(self):
        for node in self.nodes:
            node.destroy_node()
        self.context.try_shutdown()

    def node(self, port):
        node = AirSimRos2SensorBridge(context=self.context, start_workers=False,
            parameter_overrides=[Parameter('port', value=port),
                                 Parameter('rpc_timeout_s', value=.08)])
        self.nodes.append(node)
        for name in ('lidar_pub', 'imu_pub', 'image_pub', 'camera_info_pub', 'clock_pub'):
            setattr(node, name, _Publisher())
        return node

    def test_idle_reconnect_to_inertial_lidar_cannot_publish_a_local_frame_cloud(self):
        with SensorRpcServer() as first:
            node = self.node(first.port)
            client = node._new_client('lidar')
            node._lidar_loop(client)
            first.close()
            wait_for_idle_disconnect(client)
            with SensorRpcServer(port=first.port, frame='VehicleInertialFrame',
                                 stamp=2_000_000_000) as second:
                with self.assertRaisesRegex(ValueError, 'SensorLocalFrame'):
                    node._lidar_loop(client)
                self.assertEqual(len(node.lidar_pub.messages), 1)
                self.assertEqual(len(node.clock_pub.messages), 1)
                self.assertIn('getSettingsString', second.calls)

    def test_idle_reconnect_to_valid_local_lidar_is_revalidated_and_published(self):
        with SensorRpcServer() as first:
            node = self.node(first.port)
            client = node._new_client('lidar')
            node._lidar_loop(client)
            first.close()
            wait_for_idle_disconnect(client)
            with SensorRpcServer(port=first.port, stamp=2_000_000_000) as second:
                node._lidar_loop(client)
                self.assertIn('getSettingsString', second.calls)
                self.assertEqual(len(node.lidar_pub.messages), 2)
                self.assertEqual(node.lidar_pub.messages[-1].header.stamp.sec, 2)

    def test_camera_calibration_timeout_leaves_same_stamp_retryable(self):
        with SensorRpcServer(camera_timeout_once=True) as server:
            node = self.node(server.port)
            client = node._new_client('camera')
            with self.assertRaises(airsim.RPCTimeoutError):
                node._camera_loop(client)
            clock_after_failure = len(node.clock_pub.messages)
            accepted_after_failure = node._gates['camera'].accepted
            node._release_client('camera', client)
            client = node._new_client('camera')
            node._camera_loop(client)
            node._camera_loop(client)  # The successful T is now a duplicate.
            self.assertEqual(len(node.image_pub.messages), 1)
            self.assertEqual(len(node.camera_info_pub.messages), 1)
            self.assertEqual(node.image_pub.messages[0].header,
                             node.camera_info_pub.messages[0].header)
            self.assertEqual(node.image_pub.messages[0].header.stamp.sec, 1)
            self.assertEqual((clock_after_failure, accepted_after_failure), (0, 0))
            self.assertEqual(len(node.clock_pub.messages), 1)

    def test_accelerated_unix_epoch_stops_cli_once_instead_of_reconnecting(self):
        # Current-era Unix ns doubled by ScalableClock; valid uint64, not ROS sec.
        with SensorRpcServer(stamp=3_590_000_000_000_000_000, clock_speed=2) as server:
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = '190'
            env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
            command = [sys.executable, '-m', 'airsim_ros2.sensor_bridge', '--ros-args',
                       '-p', 'port:=' + str(server.port), '-p', 'rpc_timeout_s:=0.1',
                       '-p', 'reconnect_delay_s:=0.02', '-p', 'diagnostic_period_s:=0.05']
            try:
                result = subprocess.run(command, env=env, text=True, capture_output=True, timeout=8)
            except subprocess.TimeoutExpired:
                self.fail('unsupported source epoch kept the bridge reconnecting instead of exiting')
            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            errors = [line for line in (result.stdout + result.stderr).splitlines()
                      if '[ERROR]' in line and 'unsupported source epoch' in line]
            self.assertEqual(len(errors), 1, result.stdout + result.stderr)
            self.assertLessEqual(server.connections, 3)
