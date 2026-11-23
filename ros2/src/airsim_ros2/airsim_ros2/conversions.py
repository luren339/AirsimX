"""ROS message helpers layered over the SDK's shared frame conversions.

The NED/ENU and FRD/FLU math lives in ``airsim.frames`` so the ROS bridge
and the Gymnasium environment cannot drift apart. This module re-exports
those helpers and adds the message-specific pieces (GPS fix status, 3x3
covariance layout) that only make sense for ROS messages.
"""

from airsim.frames import (  # noqa: F401  (re-exported for the bridge)
    FRD_TO_FLU,
    NED_TO_ENU,
    enu_quaternion_to_ned,
    enu_to_ned_vector,
    enu_yaw_to_ned_quaternion,
    flu_to_frd_vector,
    frd_to_flu_vector,
    matrix_to_quaternion,
    ned_quaternion_to_enu,
    ned_to_enu_vector,
    ned_yaw_to_enu,
    quaternion_to_matrix,
    stamp_from_nanoseconds,
    twist_enu_to_ned,
)


# AirSim camera poses use the body convention (x forward, y right, z down);
# ROS camera frames use the optical convention (x right, y down, z forward).
# This permutation maps the former onto the latter, row by row.
FRD_TO_OPTICAL = (
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
)


def _quaternion_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quaternion_conjugate(q):
    return (q[0], -q[1], -q[2], -q[3])


def _rotate(q, v):
    """Rotate a 3-vector by quaternion q (w, x, y, z)."""
    r = quaternion_to_matrix(*q)
    return tuple(sum(r[i][j] * v[j] for j in range(3)) for i in range(3))


def camera_optical_pose(vehicle_position_enu, vehicle_quaternion_enu,
                        camera_position_enu, camera_quaternion_enu):
    """Camera pose expressed in the vehicle base frame, in optical axes.

    Both inputs must already be in the ROS ENU/FLU world frame (the bridge
    converts AirSim's NED/FRD values with ``frames``). The result is relative
    to the vehicle body, so it can be published once as a static transform
    from ``base_link`` to the camera's optical frame.
    """
    inverse = _quaternion_conjugate(vehicle_quaternion_enu)
    delta = tuple(c - v for c, v in zip(camera_position_enu, vehicle_position_enu))
    translation = _rotate(inverse, delta)
    rotation = _quaternion_multiply(inverse, camera_quaternion_enu)
    # R_optical must have the optical axes as its *columns* (their body-frame
    # directions), so the permutation is applied transposed.
    rotation_matrix = quaternion_to_matrix(*rotation)
    optical = matrix_to_quaternion(
        [[sum(FRD_TO_OPTICAL[k][i] * rotation_matrix[k][j]
              for k in range(3)) for j in range(3)] for i in range(3)])
    return translation, optical


def gps_fix_type(fix_type):
    """Map AirSim GNSS fix type to the sensor_msgs/NavSatFix status constants."""
    return 2 if int(fix_type) >= 2 else 0


def covariances(accuracy_m):
    """Diagonal position covariance from a scalar 1-sigma accuracy in meters."""
    variance = float(accuracy_m) ** 2
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]
