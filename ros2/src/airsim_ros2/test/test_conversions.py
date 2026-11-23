import math
import unittest

import numpy as np

from airsim_ros2 import conversions


class VectorConversionTests(unittest.TestCase):
    def test_ned_to_enu_and_back(self):
        east, north, up = conversions.ned_to_enu_vector(3.0, 4.0, -5.0)
        self.assertEqual((east, north, up), (4.0, 3.0, 5.0))
        self.assertEqual(conversions.enu_to_ned_vector(east, north, up), (3.0, 4.0, -5.0))

    def test_enu_to_ned_yaw(self):
        # ROS yaw 90 deg means north; AirSim heading north is 0.
        w, x, y, z = conversions.enu_yaw_to_ned_quaternion(math.pi / 2)
        for got, want in zip((w, x, y, z), (1.0, 0.0, 0.0, 0.0)):
            self.assertAlmostEqual(got, want, places=9)
        # ROS yaw 0 means east; AirSim heading east is +90 deg.
        w, x, y, z = conversions.enu_yaw_to_ned_quaternion(0.0)
        self.assertAlmostEqual(w, math.cos(math.pi / 4), places=9)
        self.assertAlmostEqual(z, math.sin(math.pi / 4), places=9)

    def test_twist_conversion(self):
        # ROS linear x (east) becomes AirSim NED east; ROS yaw rate is about up,
        # AirSim's angular z is about down, so it is negated.
        linear, angular = conversions.twist_enu_to_ned((1.0, 0.0, 0.0), (0.1, 0.2, 0.5))
        self.assertEqual(linear, (0.0, 1.0, 0.0))
        self.assertEqual(angular, (0.1, -0.2, -0.5))
        # Forward and right/left map directly with sign flips only.
        self.assertEqual(conversions.frd_to_flu_vector(1.0, 2.0, 3.0), (1.0, -2.0, -3.0))


class QuaternionTests(unittest.TestCase):
    def assertQuaternionEqual(self, got, want):
        sign = 1.0 if sum(a * b for a, b in zip(got, want)) >= 0 else -1.0
        for a, b in zip(got, want):
            self.assertAlmostEqual(a, sign * b, places=9)

    def test_ned_identity_is_ros_north(self):
        # An unrotated AirSim body faces north, which is ROS yaw +90 deg.
        expected = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
        self.assertQuaternionEqual(conversions.ned_quaternion_to_enu(1.0, 0.0, 0.0, 0.0), expected)

    def test_round_trip(self):
        for raw in [(1, 0, 0, 0), (0.5, 0.5, 0.5, 0.5), (0.7, -0.1, 0.2, 0.66)]:
            norm = math.sqrt(sum(value * value for value in raw))
            quaternion = tuple(value / norm for value in raw)
            enu = conversions.ned_quaternion_to_enu(*quaternion)
            back = conversions.enu_quaternion_to_ned(*enu)
            self.assertQuaternionEqual(back, quaternion)

    def test_ninety_degree_ned_yaw_becomes_ros_yaw(self):
        # AirSim heading 90 deg (clockwise from north) points east, ROS yaw 0.
        half = math.radians(90) / 2
        enu = conversions.ned_quaternion_to_enu(math.cos(half), 0.0, 0.0, math.sin(half))
        yaw = 2 * math.atan2(enu[3], enu[0])
        self.assertAlmostEqual(yaw, 0.0, places=9)
        # AirSim heading 0 points north, ROS yaw 90 deg.
        enu = conversions.ned_quaternion_to_enu(1.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(2 * math.atan2(enu[3], enu[0]), math.pi / 2, places=9)

    def test_matrix_matches_scipy_style_construction(self):
        matrix = conversions.quaternion_to_matrix(0.5, 0.5, 0.5, 0.5)
        np.testing.assert_allclose(matrix @ matrix.T, np.eye(3), atol=1e-12)
        self.assertQuaternionEqual(conversions.matrix_to_quaternion(matrix), (0.5, 0.5, 0.5, 0.5))

    def test_degenerate_input_rejected(self):
        with self.assertRaises(ValueError):
            conversions.quaternion_to_matrix(0.0, 0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            conversions.quaternion_to_matrix(float('nan'), 0.0, 0.0, 0.0)


class MessageHelperTests(unittest.TestCase):
    def test_timestamp_split(self):
        self.assertEqual(conversions.stamp_from_nanoseconds(1_700_000_000_123_456_789),
                         (1_700_000_000, 123_456_789))
        self.assertEqual(conversions.stamp_from_nanoseconds(0), (0, 0))
        with self.assertRaises(ValueError):
            conversions.stamp_from_nanoseconds(-1)

    def test_gps_fix_mapping(self):
        self.assertEqual(conversions.gps_fix_type(0), 0)
        self.assertEqual(conversions.gps_fix_type(1), 0)
        self.assertEqual(conversions.gps_fix_type(3), 2)

    def test_covariance_is_diagonal_variance(self):
        covariance = conversions.covariances(0.5)
        self.assertAlmostEqual(covariance[0], 0.25)
        self.assertAlmostEqual(covariance[4], 0.25)
        self.assertEqual(covariance[1], 0.0)

    def test_camera_optical_axes_follow_ros_convention(self):
        # Identity mounts: optical x is right, y is down, z is forward.
        _, q = conversions.camera_optical_pose((0, 0, 0), (1, 0, 0, 0),
                                               (0, 0, 0), (1, 0, 0, 0))
        for axis, expected in (((1, 0, 0), (0, 1, 0)),
                               ((0, 1, 0), (0, 0, 1)),
                               ((0, 0, 1), (1, 0, 0))):
            got = conversions._rotate(q, axis)
            for value, want in zip(got, expected):
                self.assertAlmostEqual(float(value), float(want), places=9)

    def test_camera_relative_pose_is_body_fixed(self):
        import math
        yaw = math.radians(90.0)
        q_vehicle = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
        # Camera 0.1 m ahead of the vehicle, in world ENU.
        camera = conversions._rotate(q_vehicle, (0.1, 0.0, 0.0))
        base, _ = conversions.camera_optical_pose((0, 0, 0), (1, 0, 0, 0),
                                                  (0.1, 0, 0), (1, 0, 0, 0))
        rotated, _ = conversions.camera_optical_pose((0, 0, 0), q_vehicle,
                                                     camera, q_vehicle)
        # Rotating the whole vehicle must not change the camera's relative pose.
        for a, b in zip(base, rotated):
            self.assertAlmostEqual(float(a), float(b), places=9)


if __name__ == '__main__':
    unittest.main()
