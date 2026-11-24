"""ROS sensor envelopes migrated from the readonly acquisition bridge.

Math uses the SDK's shared frame helpers. ROS imports stay inside message
factories so ordinary camera-parameter tests also run in the offline CI.
"""

import math
import struct

from .conversions import frd_to_flu_vector, stamp_from_nanoseconds
from .sensor_samples import SensorConfigurationError, validate_ros_timestamp_ns


def pinhole_intrinsics(width, height, fov_deg):
    """Ideal horizontal-FOV pinhole with the acquisition code's pixel centres.

This is not a physical-camera calibration or support for lens distortion.
"""
    if width <= 0 or height <= 0 or not 0.0 < float(fov_deg) < 180.0:
        raise ValueError('positive image dimensions and FOV in (0, 180) required')
    focal = float(width) / (2 * math.tan(math.radians(float(fov_deg)) / 2))
    return focal, focal, (float(width) - 1) / 2, (float(height) - 1) / 2


def stamp_from_ns(timestamp_ns):
    from builtin_interfaces.msg import Time
    sec, nanosec = stamp_from_nanoseconds(validate_ros_timestamp_ns(timestamp_ns))
    return Time(sec=sec, nanosec=nanosec)


def pointcloud2_from_xyz(points_frd, timestamp_ns, frame_id):
    """SensorLocalFrame FRD -> FLU, with placeholder intensity and no point time."""
    from sensor_msgs.msg import PointCloud2, PointField
    if len(points_frd) % 3:
        raise ValueError('LiDAR point buffer must contain XYZ triplets')
    packed = bytearray()
    for index in range(0, len(points_frd), 3):
        point = tuple(float(v) for v in points_frd[index:index + 3])
        if all(math.isfinite(v) for v in point):
            packed.extend(struct.pack('<ffff', *frd_to_flu_vector(*point), 0.0))
    msg = PointCloud2()
    msg.header.stamp, msg.header.frame_id = stamp_from_ns(timestamp_ns), frame_id
    msg.height, msg.width = 1, len(packed) // 16
    msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    msg.point_step, msg.row_step = 16, len(packed)
    msg.is_bigendian, msg.is_dense, msg.data = False, True, bytes(packed)
    return msg


def imu_from_airsim(sample, timestamp_ns, frame_id):
    from sensor_msgs.msg import Imu
    msg = Imu()
    msg.header.stamp, msg.header.frame_id = stamp_from_ns(timestamp_ns), frame_id
    msg.orientation_covariance[0] = -1.0
    for name in ('angular_velocity', 'linear_acceleration'):
        source, target = getattr(sample, name), getattr(msg, name)
        target.x, target.y, target.z = frd_to_flu_vector(
            source.x_val, source.y_val, source.z_val)
    return msg


def camera_info_from_fov(width, height, fov_deg, timestamp_ns, frame_id):
    fx, fy, cx, cy = pinhole_intrinsics(width, height, fov_deg)
    return _camera_info(width, height, fx, fy, cx, cy, timestamp_ns, frame_id)


def camera_info_from_projection(width, height, projection, timestamp_ns, frame_id):
    """Use AirSim's Scene projection, not the independent CineCamera FOV.

AirSim exports the transposed UE projection after its NED -> EUS change.
The supported centred perspective layout has horizontal scale at [0,1],
vertical scale at [1,2], and perspective divisor at [3,0].
"""
    if width <= 0 or height <= 0 or len(projection) != 4 or any(len(row) != 4 for row in projection):
        raise ValueError('positive image dimensions and a 4x4 Scene projection required')
    if width < height:
        raise SensorConfigurationError(
            'portrait SceneCapture is unsupported: the RPC projection differs from the renderer; '
            'use image width >= height until the projection conventions are aligned')
    matrix = [[float(v) for v in row] for row in projection]
    if not all(math.isfinite(v) for row in matrix for v in row):
        raise ValueError('Scene projection must be finite')
    zero_fields = ((0, 0), (0, 2), (0, 3), (1, 0), (1, 1), (1, 3),
                   (3, 1), (3, 2), (3, 3))
    if any(abs(matrix[i][j]) > 1e-8 for i, j in zero_fields) or abs(matrix[3][0]) < 1e-12:
        raise ValueError('only centred perspective Scene projections are supported')
    fx = abs(matrix[0][1] / matrix[3][0]) * width / 2
    fy = abs(matrix[1][2] / matrix[3][0]) * height / 2
    if not all(math.isfinite(v) and v > 0 for v in (fx, fy)):
        raise ValueError('Scene projection has invalid focal scales')
    return _camera_info(width, height, fx, fy, (width - 1) / 2, (height - 1) / 2,
                        timestamp_ns, frame_id)


def _camera_info(width, height, fx, fy, cx, cy, timestamp_ns, frame_id):
    from sensor_msgs.msg import CameraInfo
    msg = CameraInfo()
    msg.header.stamp, msg.header.frame_id = stamp_from_ns(timestamp_ns), frame_id
    msg.width, msg.height = int(width), int(height)
    msg.distortion_model, msg.d = 'plumb_bob', [0.0] * 5
    msg.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
    msg.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    msg.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
    return msg


def static_sensor_transforms(frame_prefix, *,
                             lidar_translation=(0., 0., 0.), lidar_rotation=(0., 0., 0., 1.),
                             imu_translation=(0., 0., 0.), imu_rotation=(0., 0., 0., 1.),
                             camera_translation=(0., 0., 0.), camera_rotation=(.5, -.5, .5, -.5)):
    """Body-FLU -> sensor-FLU / camera-optical transforms; caller supplies rig pose."""
    from geometry_msgs.msg import TransformStamped
    prefix = str(frame_prefix).strip('/')
    if not prefix:
        raise ValueError('frame_prefix must not be empty')
    transforms = []
    for child, xyz, quat in (
        ('lidar_link', lidar_translation, lidar_rotation),
        ('imu_link', imu_translation, imu_rotation),
        ('camera_optical_frame', camera_translation, camera_rotation),
    ):
        if len(xyz) != 3 or len(quat) != 4 or not all(math.isfinite(v) for v in (*xyz, *quat)):
            raise ValueError('extrinsics need finite xyz[3] and xyzw[4]')
        norm = math.sqrt(sum(v * v for v in quat))
        if not math.isfinite(norm) or norm == 0:
            raise ValueError('extrinsic quaternion must have a finite nonzero norm')
        if child == 'imu_link' and (any(abs(v) > 1e-10 for v in xyz)
                                    or any(abs(v / norm) > 1e-10 for v in quat[:3])):
            raise ValueError('native AirSim IMU is body-fixed; IMU extrinsics must be identity/co-located')
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = prefix + '/body_flu', prefix + '/' + child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, xyz)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = (
            float(v) / norm for v in quat)
        transforms.append(tf)
    return transforms
