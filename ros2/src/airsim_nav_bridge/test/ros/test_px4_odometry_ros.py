"""Explicit Jazzy/px4_msgs checks; not discovered by the ROS-free offline suite.

Use an isolated ROS_DOMAIN_ID. These tests publish synthetic odometry, never
connect to a flight controller or an AirSim process, and never send commands.
"""

import importlib
import importlib.util
import math
import os
import signal
import subprocess
import sys
import time
import unittest
import uuid

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import SensorCombined, VehicleOdometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message, serialize_message
    from rosgraph_msgs.msg import Clock
except ImportError as error:
    raise unittest.SkipTest('source Jazzy and matching px4_msgs for this explicit ROS suite') from error

from airsim_nav_bridge.px4_odometry_conversion import ros_odom_to_px4_vehicle_odometry


def odometry(stamp_ns=10_000_000_000, position=(0.0, 0.0, 0.0)):
    msg = Odometry()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    msg.header.frame_id = 'fastlio_nwu'
    msg.child_frame_id = 'body_flu'
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
    msg.pose.pose.orientation.w = 1.0
    return msg


class VehicleOdometryMessageTests(unittest.TestCase):
    def test_nwu_pose_and_body_flu_twist_become_local_frd(self):
        source = odometry(1_234_567_890, (1.0, 2.0, 3.0))
        half = math.sqrt(0.5)
        source.pose.pose.orientation.w = half
        source.pose.pose.orientation.z = half
        source.twist.twist.linear.x, source.twist.twist.linear.y, source.twist.twist.linear.z = (4.0, 5.0, 6.0)
        source.twist.twist.angular.x, source.twist.twist.angular.y, source.twist.twist.angular.z = (0.1, 0.2, 0.3)
        msg = ros_odom_to_px4_vehicle_odometry(source, now_us=1_234_568, input_has_body_twist=True)
        # Exercise Jazzy's generated setters and wire serialization, not stand-in messages.
        msg = deserialize_message(serialize_message(msg), VehicleOdometry)
        self.assertEqual((msg.timestamp, msg.timestamp_sample), (1_234_568, 1_234_567))
        self.assertEqual(msg.pose_frame, VehicleOdometry.POSE_FRAME_FRD)
        self.assertEqual(msg.velocity_frame, VehicleOdometry.VELOCITY_FRAME_BODY_FRD)
        self.assertEqual(tuple(msg.position), (1.0, -2.0, -3.0))
        for actual, expected in zip(msg.q, (half, 0.0, 0.0, -half)):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertEqual(tuple(msg.velocity), (4.0, -5.0, -6.0))
        for actual, expected in zip(msg.angular_velocity, (0.1, -0.2, -0.3)):
            self.assertAlmostEqual(actual, expected, places=6)
        for values in (msg.position_variance, msg.orientation_variance, msg.velocity_variance):
            for value in values:
                self.assertAlmostEqual(value, 0.01, places=6)
        self.assertEqual((msg.reset_counter, msg.quality), (0, 0))

    def test_normalizes_quaternion_and_changes_both_bases(self):
        source = odometry()
        orientation = source.pose.pose.orientation
        orientation.w = orientation.x = orientation.y = orientation.z = 1.0
        self.assertEqual(tuple(ros_odom_to_px4_vehicle_odometry(source).q), (0.5, 0.5, -0.5, -0.5))

    def test_covariance_retains_diagonals_and_sanitizes_unknown_values(self):
        source = odometry()
        for index, value in ((0, 0.25), (7, -1.0), (14, math.nan),
                             (21, 0.04), (28, 0.09), (35, math.inf)):
            source.pose.covariance[index] = value
        source.pose.covariance[1] = 99.0  # Off-diagonal terms are not sent by VehicleOdometry.
        for index, value in ((0, 0.36), (7, 0.49), (14, 0.64)):
            source.twist.covariance[index] = value
        msg = ros_odom_to_px4_vehicle_odometry(source, variance_floor=0.02, input_has_body_twist=True)
        for values, expected in ((msg.position_variance, (0.25, 0.02, 0.02)),
                                 (msg.orientation_variance, (0.04, 0.09, 0.02)),
                                 (msg.velocity_variance, (0.36, 0.49, 0.64))):
            for actual, want in zip(values, expected):
                self.assertAlmostEqual(actual, want, places=6)

    def test_message_uses_mapped_sample_and_publish_timestamp(self):
        msg = ros_odom_to_px4_vehicle_odometry(odometry(), now_us=30_100_000,
                                               timestamp_sample_us=30_000_000)
        self.assertEqual((msg.timestamp, msg.timestamp_sample), (30_100_000, 30_000_000))
        msg = ros_odom_to_px4_vehicle_odometry(odometry(), now_us=1, timestamp_sample_us=30_000_000)
        self.assertEqual((msg.timestamp, msg.timestamp_sample), (30_000_000, 30_000_000))

    def test_rejects_zero_and_non_finite_quaternions(self):
        for value in (0.0, math.nan, math.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                source = odometry()
                source.pose.pose.orientation.w = value
                ros_odom_to_px4_vehicle_odometry(source)

    def test_rejects_invalid_header_timestamp(self):
        for stamp in (0, -1_000_000_000):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                ros_odom_to_px4_vehicle_odometry(odometry(stamp))

    def test_moving_pose_only_does_not_invent_zero_velocity(self):
        for stamp, position in ((10_000_000_000, (0.0, 0.0, 0.0)),
                                (10_100_000_000, (0.2, 0.0, 0.0))):
            source = odometry(stamp, position)
            msg = deserialize_message(serialize_message(ros_odom_to_px4_vehicle_odometry(source)),
                                      VehicleOdometry)
            self.assertEqual(msg.velocity_frame, VehicleOdometry.VELOCITY_FRAME_UNKNOWN)
            for values in (msg.velocity, msg.angular_velocity, msg.velocity_variance):
                self.assertTrue(all(math.isnan(value) for value in values))

    def test_pose_only_ignores_unmeasured_twist_and_its_covariance(self):
        source = odometry()
        source.twist.twist.linear.x = math.inf
        source.twist.twist.angular.z = 1e100
        source.twist.covariance[0] = 1e100
        msg = ros_odom_to_px4_vehicle_odometry(source)
        self.assertEqual(msg.velocity_frame, VehicleOdometry.VELOCITY_FRAME_UNKNOWN)
        for values in (msg.velocity, msg.angular_velocity, msg.velocity_variance):
            self.assertTrue(all(math.isnan(value) for value in values))

    def test_rejects_non_finite_and_float32_overflow_positions(self):
        for value in (math.nan, math.inf, 1e100, -1e100):
            with self.subTest(value=value), self.assertRaises(ValueError):
                ros_odom_to_px4_vehicle_odometry(odometry(position=(value, 0.0, 0.0)))

    def test_rejects_invalid_trusted_twist_before_float32_cast(self):
        for attribute in ('linear', 'angular'):
            for value in (math.nan, math.inf, 1e100, -1e100):
                with self.subTest(attribute=attribute, value=value), self.assertRaises(ValueError):
                    source = odometry()
                    getattr(source.twist.twist, attribute).x = value
                    ros_odom_to_px4_vehicle_odometry(source, input_has_body_twist=True)

    def test_rejects_finite_covariance_overflow(self):
        for field, index in (('pose', 0), ('pose', 21), ('twist', 0)):
            with self.subTest(field=field, index=index), self.assertRaises(ValueError):
                source = odometry()
                getattr(source, field).covariance[index] = 1e100
                ros_odom_to_px4_vehicle_odometry(source, input_has_body_twist=True)

    def test_rejects_out_of_range_px4_message_timestamps(self):
        for sample in (0, -1, (1 << 64)):
            with self.subTest(sample=sample), self.assertRaises(ValueError):
                ros_odom_to_px4_vehicle_odometry(odometry(), timestamp_sample_us=sample)
        with self.assertRaises(ValueError):
            ros_odom_to_px4_vehicle_odometry(odometry(), now_us=(1 << 64))

    def test_float32_boundary_values_remain_finite_after_serialization(self):
        maximum = 3.4028234663852886e38
        source = odometry(position=(maximum, -maximum, 0.0))
        source.pose.pose.orientation.w = source.pose.pose.orientation.x = 1e100
        source.twist.twist.linear.x = maximum
        source.twist.twist.angular.z = -maximum
        for covariance in (source.pose.covariance, source.twist.covariance):
            for index in (0, 7, 14, 21, 28, 35):
                covariance[index] = maximum
        msg = ros_odom_to_px4_vehicle_odometry(source, input_has_body_twist=True)
        msg = deserialize_message(serialize_message(msg), VehicleOdometry)
        for values in (msg.position, msg.q, msg.velocity, msg.angular_velocity,
                       msg.position_variance, msg.orientation_variance, msg.velocity_variance):
            self.assertTrue(all(math.isfinite(value) for value in values))
        self.assertEqual(msg.position[0], maximum)
        self.assertAlmostEqual(msg.q[0], math.sqrt(0.5), places=6)


class _OdometryNodeTestCase(unittest.TestCase):
    extra_parameters = ()

    def setUp(self):
        name = 'airsim_nav_bridge.fastlio_to_px4_odometry'
        self.assertIsNotNone(importlib.util.find_spec(name), 'odometry node must be installable')
        api = importlib.import_module(name)
        self.prefix = '/pr3_' + uuid.uuid4().hex
        self.arguments = ['--ros-args']
        for parameter, suffix in (('input_topic', '/odom'), ('output_topic', '/px4_odom'),
                                  ('clock_topic', '/clock'), ('px4_time_topic', '/px4_time')):
            self.arguments += ['-p', parameter + ':=' + self.prefix + suffix]
        for parameter, value in self.extra_parameters:
            self.arguments += ['-p', parameter + ':=' + value]
        rclpy.init(args=self.arguments)
        self.addCleanup(rclpy.try_shutdown)
        self.bridge = api.FastLioToPx4Odometry()
        self.addCleanup(self.bridge.destroy_node)
        self.probe = Node('pr3_test_driver')
        self.addCleanup(self.probe.destroy_node)
        self.executor = SingleThreadedExecutor()
        self.addCleanup(self.executor.shutdown)
        self.executor.add_node(self.bridge)
        self.executor.add_node(self.probe)
        self.received = []
        self.odom_pub = self.probe.create_publisher(Odometry, self.prefix + '/odom', 10)
        self.clock_pub = self.probe.create_publisher(Clock, self.prefix + '/clock', 10)
        self.time_pub = self.probe.create_publisher(SensorCombined, self.prefix + '/px4_time', 10)
        self.output = self.probe.create_subscription(VehicleOdometry, self.prefix + '/px4_odom',
                                                       self.received.append, 10)
        self.wait_until(lambda: all(pub.get_subscription_count() for pub in
                                   (self.odom_pub, self.clock_pub, self.time_pub, self.bridge.publisher)))

    def wait_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not predicate():
            self.executor.spin_once(timeout_sec=0.02)
        self.assertTrue(predicate(), 'ROS condition timed out')

    def settle(self):
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def clocks(self, sim_ns=10_000_000_000, px4_us=30_000_000):
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(sim_ns, 1_000_000_000)
        px4 = SensorCombined()
        px4.timestamp = px4_us
        self.clock_pub.publish(clock)
        self.time_pub.publish(px4)
        # Wait for the actual subscriber callbacks before sending the sample.
        self.wait_until(lambda: self.bridge._sim_clock_ns == sim_ns and self.bridge._px4_now_us == px4_us)


class OdometryNodeTests(_OdometryNodeTestCase):
    def test_requires_both_clocks_then_publishes_static_estimate(self):
        self.odom_pub.publish(odometry())
        self.settle()
        self.assertEqual(self.received, [])
        clock = Clock()
        clock.clock.sec = 10
        self.clock_pub.publish(clock)
        self.wait_until(lambda: self.bridge._sim_clock_ns == 10_000_000_000)
        self.odom_pub.publish(odometry())
        self.settle()
        self.assertEqual(self.received, [])
        self.clocks()
        self.odom_pub.publish(odometry())
        self.wait_until(lambda: len(self.received) == 1)
        self.assertEqual((self.received[0].timestamp, self.received[0].timestamp_sample),
                         (30_000_000, 30_000_000))
        self.assertEqual(tuple(self.received[0].position), (0.0, 0.0, 0.0))

    def test_pose_only_motion_keeps_twist_unknown_on_wire(self):
        self.clocks()
        self.odom_pub.publish(odometry())
        self.wait_until(lambda: len(self.received) == 1)
        self.clocks(10_100_000_000, 30_100_000)
        self.odom_pub.publish(odometry(10_100_000_000, (0.2, 0.0, 0.0)))
        self.wait_until(lambda: len(self.received) == 2)
        self.assertAlmostEqual(self.received[-1].position[0], 0.2, places=6)
        for msg in self.received:
            self.assertEqual(msg.velocity_frame, VehicleOdometry.VELOCITY_FRAME_UNKNOWN)
            for values in (msg.velocity, msg.angular_velocity, msg.velocity_variance):
                self.assertTrue(all(math.isnan(value) for value in values))

    def test_rejects_duplicate_backward_and_implausible_samples(self):
        self.clocks()
        self.odom_pub.publish(odometry())
        self.wait_until(lambda: len(self.received) == 1)
        for msg in (odometry(), odometry(9_900_000_000), odometry(10_050_000_000, (2.0, 0.0, 0.0))):
            self.odom_pub.publish(msg)
        self.settle()
        self.assertEqual(len(self.received), 1)
        self.clocks(10_100_000_000, 30_100_000)
        self.odom_pub.publish(odometry(10_100_000_000, (0.2, 0.0, 0.0)))
        self.wait_until(lambda: len(self.received) == 2)
        self.assertEqual([msg.timestamp_sample for msg in self.received], [30_000_000, 30_100_000])

    def test_rejected_startup_position_and_quaternion_do_not_poison_state(self):
        self.clocks()
        invalid_orientation = odometry()
        invalid_orientation.pose.pose.orientation.w = 0.0
        for msg in (odometry(position=(223.554, 144.730, -82.334)),
                    odometry(position=(math.nan, 0.0, 0.0)), invalid_orientation):
            self.odom_pub.publish(msg)
        self.settle()
        self.assertEqual(self.received, [])
        self.odom_pub.publish(odometry(position=(1.0, 2.0, 3.0)))
        self.wait_until(lambda: len(self.received) == 1)
        self.assertEqual(tuple(self.received[0].position), (1.0, -2.0, -3.0))

    def test_stale_px4_publish_time_can_retry_same_sample_when_clock_advances(self):
        self.clocks()
        self.odom_pub.publish(odometry())
        self.wait_until(lambda: len(self.received) == 1)
        self.clocks(10_100_000_000, 30_000_000)
        self.odom_pub.publish(odometry(10_100_000_000))
        self.settle()
        self.assertEqual(len(self.received), 1)
        self.clocks(10_100_000_000, 30_100_000)
        self.odom_pub.publish(odometry(10_100_000_000))
        self.wait_until(lambda: len(self.received) == 2)
        self.assertEqual(self.received[-1].timestamp, 30_100_000)

    def test_non_positive_mapped_sample_is_not_published(self):
        self.clocks(100_000_000_000, 100)
        self.odom_pub.publish(odometry(1_000_000_000))
        self.settle()
        self.assertEqual(self.received, [])
        self.clocks()
        self.odom_pub.publish(odometry())
        self.wait_until(lambda: len(self.received) == 1)

    def test_mapped_sample_rollback_is_rejected_without_advancing_input_highwater(self):
        self.clocks()
        self.odom_pub.publish(odometry())
        self.wait_until(lambda: len(self.received) == 1)
        # ROS 10.01 s -> PX4 29,965,000 us, though publish time is 30,005,000 us.
        self.clocks(10_050_000_000, 30_005_000)
        self.odom_pub.publish(odometry(10_010_000_000))
        self.settle()
        self.assertEqual(len(self.received), 1)
        # Equal mapped sample is also invalid despite an advancing publication time.
        self.clocks(10_050_000_000, 30_040_000)
        self.odom_pub.publish(odometry(10_010_000_000))
        self.settle()
        self.assertEqual(len(self.received), 1)
        # Retry the identical ROS stamp after the mapping advances.
        self.clocks(10_050_000_000, 30_060_000)
        self.odom_pub.publish(odometry(10_010_000_000))
        self.wait_until(lambda: len(self.received) == 2)
        self.assertEqual(self.received[-1].timestamp_sample, 30_020_000)
        self.assertEqual(self.received[-1].timestamp, 30_060_000)

    @unittest.skipUnless(os.name == 'posix', 'Jazzy Linux SIGINT lifecycle check')
    def test_node_process_exits_cleanly_on_sigint(self):
        process = subprocess.Popen(
            [sys.executable, '-m', 'airsim_nav_bridge.fastlio_to_px4_odometry',
             *self.arguments, '-r', '__node:=pr3_shutdown'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            self.wait_until(lambda: 'pr3_shutdown' in self.probe.get_node_names())
            process.send_signal(signal.SIGINT)
            output, _ = process.communicate(timeout=5)
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()
        self.assertEqual(process.returncode, 0, output)
        self.assertNotIn('Traceback', output)


class BodyTwistNodeTests(_OdometryNodeTestCase):
    extra_parameters = (('input_has_body_twist', 'true'),)

    def test_explicit_body_flu_twist_is_forwarded(self):
        self.clocks()
        source = odometry()
        source.twist.twist.linear.x, source.twist.twist.linear.y, source.twist.twist.linear.z = (4.0, 5.0, 6.0)
        source.twist.twist.angular.x, source.twist.twist.angular.y, source.twist.twist.angular.z = (0.1, 0.2, 0.3)
        self.odom_pub.publish(source)
        self.wait_until(lambda: len(self.received) == 1)
        msg = self.received[0]
        self.assertEqual(msg.velocity_frame, VehicleOdometry.VELOCITY_FRAME_BODY_FRD)
        self.assertEqual(tuple(msg.velocity), (4.0, -5.0, -6.0))
        for value, expected in zip(msg.angular_velocity, (0.1, -0.2, -0.3)):
            self.assertAlmostEqual(value, expected, places=6)

    def test_invalid_measurements_are_dropped_without_poisoning_sample_state(self):
        self.clocks()
        invalid = []
        for attribute, value in (('linear', math.inf), ('linear', 1e100), ('angular', 1e100)):
            source = odometry()
            getattr(source.twist.twist, attribute).x = value
            invalid.append(source)
        for field, index in (('pose', 0), ('pose', 21), ('twist', 0)):
            source = odometry()
            getattr(source, field).covariance[index] = 1e100
            invalid.append(source)
        for source in invalid:
            self.odom_pub.publish(source)
        self.settle()
        self.assertEqual(self.received, [])
        self.odom_pub.publish(odometry())
        self.wait_until(lambda: len(self.received) == 1)
        self.assertEqual(self.received[0].timestamp_sample, 30_000_000)


class OdometryConfigurationTests(unittest.TestCase):
    def test_refuses_invalid_numeric_configuration_at_startup(self):
        api = importlib.import_module('airsim_nav_bridge.fastlio_to_px4_odometry')
        for parameter in ('variance_floor', 'max_initial_position_norm_m',
                          'max_position_speed_m_s', 'diagnostic_period_s'):
            values = ['0.0', '-0.01', '.nan', '.inf']
            if parameter == 'variance_floor':
                values += ['1.0e+100', '1.0e-100']
            for value in values:
                with self.subTest(parameter=parameter, value=value):
                    rclpy.init(args=['--ros-args', '-p', parameter + ':=' + value])
                    node = None
                    try:
                        with self.assertRaisesRegex(ValueError, parameter):
                            node = api.FastLioToPx4Odometry()
                    finally:
                        if node is not None:
                            node.destroy_node()
                        rclpy.try_shutdown()


if __name__ == '__main__':
    unittest.main()
