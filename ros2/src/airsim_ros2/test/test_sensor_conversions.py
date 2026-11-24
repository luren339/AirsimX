import math
import struct
from types import SimpleNamespace as Obj
import unittest

try:
    from sensor_msgs.msg import Imu
except ImportError:
    raise unittest.SkipTest('ROS message conversion tests require a sourced Jazzy installation')

from airsim_ros2 import sensor_conversions as convert


class SensorMessageTests(unittest.TestCase):
    def test_cloud_uses_sensor_flu_not_world_enu_and_filters_nonfinite_points(self):
        msg = convert.pointcloud2_from_xyz([1, 2, 3, math.nan, 1, 2], 100, 'a/lidar_link')
        self.assertEqual(struct.unpack('<ffff', bytes(msg.data)), (1., -2., -3., 0.))
        self.assertEqual((msg.width, msg.height, msg.point_step), (1, 1, 16))
        self.assertEqual([f.name for f in msg.fields], ['x', 'y', 'z', 'intensity'])
        self.assertEqual(msg.header.frame_id, 'a/lidar_link')
        self.assertEqual(msg.header.stamp.nanosec, 100)
        with self.assertRaises(ValueError):
            convert.pointcloud2_from_xyz([1, 2], 100, 'a/lidar_link')

    def test_imu_orientation_is_unavailable_and_vectors_are_flu(self):
        sample = Obj(angular_velocity=Obj(x_val=1, y_val=2, z_val=3),
                     linear_acceleration=Obj(x_val=4, y_val=5, z_val=6))
        msg = convert.imu_from_airsim(sample, 1234, 'a/imu_link')
        self.assertIsInstance(msg, Imu)
        self.assertEqual(msg.orientation_covariance[0], -1.)
        self.assertEqual((msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
                         (1., -2., -3.))

    def test_camera_info_keeps_image_stamp_dimensions_and_frame(self):
        msg = convert.camera_info_from_fov(640, 480, 90, 1_234_567_890, 'a/camera_optical_frame')
        self.assertEqual((msg.width, msg.height), (640, 480))
        self.assertEqual((msg.header.stamp.sec, msg.header.stamp.nanosec), (1, 234_567_890))
        self.assertEqual(msg.header.frame_id, 'a/camera_optical_frame')
        self.assertAlmostEqual(msg.k[0], 320.)
        self.assertEqual(list(msg.d), [0.] * 5)

    def test_extrinsics_are_parameters_and_do_not_leak_the_original_rig(self):
        transforms = convert.static_sensor_transforms('b', lidar_translation=(.1, .2, .3))
        by_child = {t.child_frame_id: t for t in transforms}
        lidar = by_child['b/lidar_link']
        self.assertEqual(lidar.header.frame_id, 'b/body_flu')
        self.assertEqual((lidar.transform.translation.x, lidar.transform.translation.y,
                          lidar.transform.translation.z), (.1, .2, .3))
        self.assertEqual(by_child['b/camera_optical_frame'].transform.translation.x, 0.)
        with self.assertRaises(ValueError):
            convert.static_sensor_transforms('')
        with self.assertRaises(ValueError):
            convert.static_sensor_transforms('a', lidar_rotation=(0, 0, 0, 0))

    def test_stamp_overflow_and_negative_values_are_not_replaced_by_wall_time(self):
        for value in (-1, (2**31) * 1_000_000_000):
            with self.subTest(value=value), self.assertRaises(ValueError):
                convert.stamp_from_ns(value)

    def test_native_body_imu_cannot_be_labelled_as_a_rotated_or_displaced_sensor(self):
        for overrides in ({'imu_rotation': (1., 0., 0., 0.)},
                          {'imu_translation': (0., 0., .1)}):
            with self.subTest(overrides=overrides), self.assertRaisesRegex(ValueError, 'IMU'):
                convert.static_sensor_transforms('a', **overrides)

    def test_camera_uses_scene_projection_even_if_main_camera_fov_differs(self):
        scale = math.sqrt(3.)
        projection = [[0., scale, 0., 0.], [0., 0., -scale * 640 / 480, 0.],
                      [0., 0., 0., 10.], [-1., 0., 0., 0.]]
        msg = convert.camera_info_from_projection(640, 480, projection, 100, 'a/camera_optical_frame')
        self.assertAlmostEqual(msg.k[0], 640 / 2 * scale)
        self.assertAlmostEqual(msg.k[4], 640 / 2 * scale)
        with self.assertRaises(ValueError):
            convert.camera_info_from_projection(640, 480, np_identity(), 100, 'a/camera_optical_frame')


def np_identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


if __name__ == '__main__':
    unittest.main()
