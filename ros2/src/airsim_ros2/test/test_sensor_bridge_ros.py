"""Real Jazzy nodes/messages with injected sensor-only RPC; never connects to a vehicle."""
import json
import time
import math
import threading
from unittest.mock import Mock
from types import SimpleNamespace as Obj
import unittest

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
except ImportError:
    raise unittest.SkipTest('Jazzy node tests require ROS 2')

import cv2
import numpy as np

from airsim_ros2.sensor_bridge import AirSimRos2SensorBridge


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class SensorClient:
    def __init__(self, frame='SensorLocalFrame'):
        self.calls, self.closed, self.frame = [], False, frame
        self.stamp = 1_000_000_000
        self.connection_generation = 1

    def confirmConnection(self):
        self.calls.append('confirmConnection')

    def ping(self):
        self.calls.append('ping')
        return True

    def listVehicles(self):
        self.calls.append('listVehicles')
        return ['UAV_A', 'UAV_B']

    def getSettingsString(self):
        self.calls.append('getSettingsString')
        return json.dumps({'Vehicles': {v: {'Sensors': {'Lidar1': {'DataFrame': self.frame}}}
                                       for v in ('UAV_A', 'UAV_B')}})

    def getLidarData(self, **kwargs):
        self.calls.append('getLidarData')
        return Obj(time_stamp=self.stamp, point_cloud=[1., 2., 3.])

    def getImuData(self, **kwargs):
        self.calls.append('getImuData')
        return Obj(time_stamp=self.stamp, angular_velocity=Obj(x_val=0., y_val=0., z_val=0.),
                   linear_acceleration=Obj(x_val=0., y_val=0., z_val=-9.81))

    def simGetImages(self, *args, **kwargs):
        self.calls.append('simGetImages')
        image = np.zeros((4, 8, 3), dtype=np.uint8)
        image[0, 0] = (10, 20, 30)
        ok, data = cv2.imencode('.png', image)
        assert ok
        return [Obj(time_stamp=self.stamp, image_data_uint8=data.tobytes())]

    def simGetCameraInfo(self, *args, **kwargs):
        self.calls.append('simGetCameraInfo')
        scale = math.sqrt(3.)
        return Obj(fov=90., proj_mat=Obj(matrix=[
            [0., scale, 0., 0.], [0., 0., -2 * scale, 0.],
            [0., 0., 0., 10.], [-1., 0., 0., 0.]]))

    def close(self):
        self.closed = True


class SensorBridgeTests(unittest.TestCase):
    def setUp(self):
        self.context = Context()
        rclpy.init(context=self.context)
        self.nodes = []

    def tearDown(self):
        for node in self.nodes:
            node.destroy_node()
        self.context.try_shutdown()

    def node(self, **parameters):
        node = AirSimRos2SensorBridge(
            context=self.context, start_workers=False,
            parameter_overrides=[Parameter(k, value=v) for k, v in parameters.items()])
        self.nodes.append(node)
        for attr in ('lidar_pub', 'imu_pub', 'image_pub', 'camera_info_pub'):
            setattr(node, attr, RecordingPublisher())
        if node.clock_pub is not None:
            node.clock_pub = RecordingPublisher()
        return node

    def test_existing_imu_selection_and_optional_raw_stream_are_preserved(self):
        node = self.node(imu_raw_topic='imu_raw', imu_target_hz_sim_time=200.)
        node.imu_raw_pub = RecordingPublisher()
        client = SensorClient()
        for stamp in (1_000_000_000, 1_001_000_000, 1_001_000_000, 1_005_000_000):
            client.stamp = stamp
            node._imu_loop(client)
        self.assertEqual(len(node.imu_raw_pub.messages), 3)
        self.assertEqual(len(node.imu_pub.messages), 2)
        self.assertEqual(node._gates['imu'].rejected, 1)

    def test_cloud_and_camera_keep_real_source_stamp_and_camera_info(self):
        node, client = self.node(), SensorClient()
        node._lidar_loop(client)
        node._camera_loop(client)
        cloud = node.lidar_pub.messages[0]
        image, info = node.image_pub.messages[0], node.camera_info_pub.messages[0]
        self.assertEqual(cloud.header.frame_id, 'uav_a/lidar_link')
        self.assertEqual((image.encoding, image.width, image.height), ('rgb8', 8, 4))
        self.assertEqual(list(image.data[:3]), [30, 20, 10])
        self.assertEqual(image.header, info.header)
        self.assertEqual((info.width, info.height), (image.width, image.height))
        self.assertEqual(image.header.stamp.sec, 1)
        self.assertAlmostEqual(info.k[0], 4 * math.sqrt(3.))
        self.assertEqual(set(client.calls), {'getSettingsString', 'getLidarData', 'simGetImages', 'simGetCameraInfo'})

    def test_clock_does_not_regress_and_a_nonmaster_never_publishes_clock(self):
        node, client = self.node(), SensorClient()
        client.stamp = 2_000_000_000
        node._imu_loop(client)
        client.stamp = 1_000_000_000
        node._lidar_loop(client)
        self.assertEqual(len(node.clock_pub.messages), 1)
        other = self.node(publish_clock=False, frame_prefix='uav_b', vehicle_name='UAV_B')
        other._imu_loop(client)
        self.assertIsNone(other.clock_pub)
        self.assertEqual(other.imu_pub.messages[0].header.frame_id, 'uav_b/imu_link')

    def test_mislabelled_inertial_lidar_cloud_is_rejected_before_publishing(self):
        node = self.node()
        client = SensorClient(frame='VehicleInertialFrame')
        node._client_factory = lambda **kw: client
        with self.assertRaisesRegex(ValueError, 'SensorLocalFrame'):
            node._new_client('lidar')
        self.assertTrue(client.closed)

    def test_shutdown_releases_every_independent_rpc_client(self):
        clients = []
        def factory(**kwargs):
            client = SensorClient()
            clients.append(client)
            return client
        node = AirSimRos2SensorBridge(context=self.context, client_factory=factory)
        deadline = time.monotonic() + 3
        while len(clients) < 3 and time.monotonic() < deadline:
            time.sleep(.01)
        node.destroy_node()
        self.assertEqual(len(clients), 3)
        self.assertTrue(all(c.closed for c in clients))
        self.assertTrue(all(not worker.is_alive() for worker in node._workers))

    def test_invalid_poll_rate_fails_before_starting_workers(self):
        with self.assertRaisesRegex(ValueError, 'poll_hz'):
            AirSimRos2SensorBridge(context=self.context, parameter_overrides=[
                Parameter('imu_poll_hz', value=0.)], start_workers=False)

    def test_worker_recovers_from_transient_rpc_error_in_its_own_context(self):
        node = self.node()
        clients = []
        def factory(**kwargs):
            client = SensorClient()
            clients.append(client)
            return client
        node._client_factory = factory
        calls = []
        def callback(client):
            calls.append(client)
            if len(calls) == 1:
                raise ConnectionError('injected temporary transport failure')
            node._stop.set()
        node._worker('imu', 100., callback)
        self.assertEqual(len(calls), 2)
        self.assertEqual(len(clients), 2)
        self.assertTrue(all(c.closed for c in clients))

    def test_invalid_stamp_cannot_poison_the_global_clock(self):
        node = self.node()
        with self.assertRaises(ValueError):
            node._accept_timestamp('imu', 2**63)
        with self.assertRaises(ValueError):
            node._publish_clock(2**63)
        self.assertTrue(node._accept_timestamp('imu', 100))
        self.assertTrue(node._accept_timestamp('lidar', 101))
        self.assertEqual(len(node.clock_pub.messages), 2)

    def test_slow_handshake_is_cancelled_before_destroying_ros_entities(self):
        entered = threading.Event()
        clients = []
        class SlowClient(SensorClient):
            def __init__(self):
                super().__init__()
                self.done = threading.Event()
            def wait(self):
                entered.set()
                self.done.wait(.5)
            def confirmConnection(self):
                self.wait()
            def ping(self):
                self.wait()
                return True
            def listVehicles(self):
                self.wait()
                return super().listVehicles()
            def getSettingsString(self):
                self.wait()
                return super().getSettingsString()
            def close(self):
                super().close()
                self.done.set()
        def factory(**kw):
            c = SlowClient(); clients.append(c); return c
        node = AirSimRos2SensorBridge(context=self.context, client_factory=factory,
            parameter_overrides=[Parameter('rpc_timeout_s', value=.05),
                                 Parameter('publish_camera', value=False)])
        self.assertTrue(entered.wait(2))
        node.destroy_node()
        alive = [worker.is_alive() for worker in node._workers]
        for c in clients:c.close()
        for worker in node._workers:worker.join(timeout=2)
        self.assertFalse(any(alive))
        self.assertTrue(all(c.closed for c in clients))

    def test_failed_reconnect_uses_backoff_not_the_imu_poll_period(self):
        node = self.node(reconnect_delay_s=.2)
        clients = []
        def factory(**kw):
            c = SensorClient(); clients.append(c); return c
        node._client_factory = factory
        def fail(client):
            raise ConnectionError('unavailable IMU')
        worker = threading.Thread(target=node._worker, args=('imu', 1000., fail))
        worker.start()
        time.sleep(.11)
        node._stop.set(); worker.join(timeout=2)
        self.assertLessEqual(len(clients), 2)

    def test_duplicate_only_stream_still_reports_wall_time_diagnostics(self):
        node = self.node()
        node._report_rate_if_due = Mock()
        node._accept_timestamp('imu', 100)
        node._accept_timestamp('imu', 100)
        self.assertEqual(node._report_rate_if_due.call_count, 2)

    def test_camera_metadata_returning_after_stop_cannot_publish(self):
        node = self.node()
        class StoppingClient(SensorClient):
            def simGetCameraInfo(self, *args, **kwargs):
                node._stop.set()
                return super().simGetCameraInfo(*args, **kwargs)
        node._camera_loop(StoppingClient())
        self.assertEqual(node.image_pub.messages, [])
        self.assertEqual(node.camera_info_pub.messages, [])

    def test_two_namespaces_isolate_topics_and_only_one_clock_owner(self):
        first = AirSimRos2SensorBridge(context=self.context, start_workers=False,
                                      namespace='uav_a')
        second = AirSimRos2SensorBridge(context=self.context, start_workers=False,
            namespace='uav_b', parameter_overrides=[Parameter('frame_prefix', value='uav_b'),
                                                    Parameter('publish_clock', value=False)])
        self.nodes.extend((first, second))
        self.assertEqual(first.lidar_pub.topic_name, '/uav_a/lidar/points')
        self.assertEqual(second.lidar_pub.topic_name, '/uav_b/lidar/points')
        self.assertIsNone(second.clock_pub)

    def test_connected_rpc_clients_do_not_shadow_ros_executor_clients(self):
        from rclpy.executors import SingleThreadedExecutor
        node = self.node()
        node._client_factory = lambda **kw: SensorClient()
        node._new_client('imu')
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(node)
        try:
            executor.spin_once(timeout_sec=.01)
        finally:
            executor.remove_node(node)
            executor.shutdown()


if __name__ == '__main__':
    unittest.main()
