"""Convert FAST-LIO2 ROS odometry from NWU/FLU to PX4 local FRD odometry."""
from __future__ import annotations

import math

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import VehicleOdometry


FLOAT32_MAX = 3.4028234663852886e38
FLOAT32_MIN_POSITIVE = 2.0 ** -149
PX4_TIMESTAMP_MAX = (1 << 64) - 1


class ClockPairingError(ValueError):
    """A FAST-LIO sample is too far from the latest simulation-clock sample."""


def positive_finite_parameter(name: str, value: float) -> float:
    """Validate numeric bridge settings before subscriptions can publish data."""
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    if name == 'variance_floor' and not FLOAT32_MIN_POSITIVE <= value <= FLOAT32_MAX:
        raise ValueError('variance_floor must remain positive and finite in float32')
    return value


def _finite_float32_values(values, field: str) -> tuple[float, ...]:
    values = tuple(float(value) for value in values)
    if any(not math.isfinite(value) or abs(value) > FLOAT32_MAX for value in values):
        raise ValueError(f'{field} must contain finite values within the float32 range')
    return values


def _px4_timestamp(value: int, field: str) -> int:
    value = int(value)
    if not 0 < value <= PX4_TIMESTAMP_MAX:
        raise ValueError(f'{field} must be a positive uint64 timestamp')
    return value


def sim_sample_to_px4_us(
    sample_stamp_ns: int,
    sim_clock_ns: int,
    px4_now_us: int,
    *,
    max_clock_pairing_skew_s: float | None = None,
) -> int:
    """Map a simulation timestamp to PX4 time, optionally bounding its /clock skew."""
    sample_ns, sim_ns, px4_us = int(sample_stamp_ns), int(sim_clock_ns), int(px4_now_us)
    if sample_ns <= 0 or sim_ns <= 0 or px4_us <= 0:
        raise ValueError("AirSim sample, /clock and PX4 local clock timestamps must be positive")
    if max_clock_pairing_skew_s is not None:
        max_skew_s = float(max_clock_pairing_skew_s)
        if not math.isfinite(max_skew_s) or max_skew_s < 0.0:
            raise ValueError('max_clock_pairing_skew_s must be finite and non-negative')
        skew_s = abs(sample_ns - sim_ns) / 1_000_000_000.0
        if skew_s > max_skew_s:
            raise ClockPairingError(
                f'clock_pairing_skew={skew_s:.6f}_s_exceeds_{max_skew_s:.6f}_s')
    sample_us = sample_ns // 1_000
    sim_us = sim_ns // 1_000
    return sample_us + (px4_us - sim_us)


def _finite_covariance(values, floor: float = 0.01) -> tuple[float, float, float]:
    floor = positive_finite_parameter('variance_floor', floor)
    result = []
    for value in values:
        value = float(value)
        if not math.isfinite(value):
            raise ValueError('covariance variances must be finite')
        if value < 0.0:
            raise ValueError('covariance variances must be non-negative')
        result.append(max(value, floor))
    return _finite_float32_values(result, 'covariance')


def _orientation_variance_body_axes(
    covariance_fixed: tuple[float, ...],
    quaternion_body_to_fixed: tuple[float, float, float, float],
    floor: float = 0.01,
) -> tuple[float, float, float]:
    """Rotate a fixed-axis orientation covariance into body axes and return its diagonal."""
    covariance_values = _finite_float32_values(covariance_fixed, 'orientation covariance')
    if len(covariance_values) != 9:
        raise ValueError('orientation covariance must contain a 3x3 matrix')

    qx, qy, qz, qw = map(float, quaternion_body_to_fixed)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('orientation covariance rotation quaternion is invalid')
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    # Rotation matrix maps body axes into the fixed parent frame.
    rotation = (
        (1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)),
        (2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)),
        (2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)),
    )
    covariance = tuple(tuple(covariance_values[3 * row + column] for column in range(3))
                       for row in range(3))
    # C_body = R_fixed_from_body^T * C_fixed * R_fixed_from_body.
    diagonal = []
    for body_axis in range(3):
        value = math.fsum(
            rotation[fixed_row][body_axis] * covariance[fixed_row][fixed_column] *
            rotation[fixed_column][body_axis]
            for fixed_row in range(3)
            for fixed_column in range(3)
        )
        # An orthonormal rotation can move a variance at float32 max by a few
        # ulps due to round-off; preserve that representable boundary value.
        if FLOAT32_MAX < value <= FLOAT32_MAX * (1.0 + 1e-12):
            value = FLOAT32_MAX
        diagonal.append(value)
    return _finite_covariance(diagonal, floor)


def ros_odom_to_px4_vehicle_odometry(
    odom: Odometry,
    *,
    now_us: int | None = None,
    variance_floor: float = 0.01,
    timestamp_sample_us: int | None = None,
    input_has_body_twist: bool = False,
) -> VehicleOdometry:
    """Convert FAST-LIO2 odometry to PX4 EV input without truth or GNSS data.

    Assumption: FAST-LIO2 consumes FLU sensor measurements and publishes its
    local world in NWU axes. PX4 FRD conversion flips Y and Z for both the
    local pose frame and, only when explicitly supplied, body-frame twist.
    FAST-LIO2's pose-only output leaves twist unknown rather than measured zero.
    Finite non-negative variances use a conservative floor; negative or
    non-finite covariance entries reject the sample.
    """
    from px4_msgs.msg import VehicleOdometry

    stamp_ns = int(odom.header.stamp.sec) * 1_000_000_000 + int(odom.header.stamp.nanosec)
    if stamp_ns <= 0:
        raise ValueError(f"FAST-LIO odometry has invalid header timestamp: {stamp_ns}")
    raw_sample_us = stamp_ns // 1_000
    sample_us = _px4_timestamp(raw_sample_us if timestamp_sample_us is None else timestamp_sample_us,
                               'timestamp_sample')
    message_time_us = sample_us if now_us is None else max(sample_us, _px4_timestamp(now_us, 'timestamp'))
    variance_floor = positive_finite_parameter('variance_floor', variance_floor)

    position = odom.pose.pose.position
    orientation = odom.pose.pose.orientation
    qx, qy, qz, qw = map(float, (orientation.x, orientation.y, orientation.z, orientation.w))
    q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(q_norm) or q_norm < 1e-8:
        raise ValueError("FAST-LIO odometry quaternion is zero or non-finite")
    qx, qy, qz, qw = qx / q_norm, qy / q_norm, qz / q_norm, qw / q_norm

    msg = VehicleOdometry()
    msg.timestamp = message_time_us
    msg.timestamp_sample = sample_us
    msg.pose_frame = int(VehicleOdometry.POSE_FRAME_FRD)
    msg.position = _finite_float32_values((position.x, -position.y, -position.z), 'position')
    # Quaternion is Hamiltonian (w,x,y,z); conjugation by the 180-degree X-axis
    # basis change maps NWU/FLU attitude into local FRD without changing yaw origin.
    msg.q = _finite_float32_values((qw, qx, -qy, -qz), 'normalized quaternion')
    if input_has_body_twist:
        linear = odom.twist.twist.linear
        angular = odom.twist.twist.angular
        msg.velocity_frame = int(VehicleOdometry.VELOCITY_FRAME_BODY_FRD)
        msg.velocity = _finite_float32_values((linear.x, -linear.y, -linear.z), 'velocity')
        msg.angular_velocity = _finite_float32_values((angular.x, -angular.y, -angular.z), 'angular_velocity')
        msg.velocity_variance = _finite_covariance(
            (odom.twist.covariance[0], odom.twist.covariance[7], odom.twist.covariance[14]), variance_floor
        )
    else:
        msg.velocity_frame = int(VehicleOdometry.VELOCITY_FRAME_UNKNOWN)
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.angular_velocity = [math.nan, math.nan, math.nan]
        msg.velocity_variance = [math.nan, math.nan, math.nan]
    msg.position_variance = _finite_covariance(
        (odom.pose.covariance[0], odom.pose.covariance[7], odom.pose.covariance[14]), variance_floor
    )
    orientation_covariance_indices = (21, 22, 23, 27, 28, 29, 33, 34, 35)
    msg.orientation_variance = _orientation_variance_body_axes(
        tuple(odom.pose.covariance[index] for index in orientation_covariance_indices),
        (qx, qy, qz, qw), variance_floor,
    )
    msg.reset_counter = 0
    msg.quality = 0  # quality is unreported by FAST-LIO2; do not invent it.
    return msg


def fastlio_pose_rejection_reason(
    position,
    *,
    sample_us: int,
    previous_position=None,
    previous_sample_us: int | None = None,
    max_initial_position_norm_m: float = 5.0,
    max_position_speed_m_s: float = 15.0,
) -> str | None:
    """Reject non-finite, far-from-origin startup, or physically implausible pose jumps."""
    if len(position) != 3:
        return 'invalid_position_shape'
    xyz = tuple(float(value) for value in position)
    if not all(math.isfinite(value) for value in xyz):
        return 'non_finite_position'
    if any(abs(value) > FLOAT32_MAX for value in xyz):
        return 'position_out_of_float32_range'
    if int(sample_us) <= 0:
        return 'invalid_sample_timestamp'

    if previous_position is None and previous_sample_us is None:
        norm = math.sqrt(sum(value * value for value in xyz))
        if norm > float(max_initial_position_norm_m):
            return f'initial_position_norm={norm:.3f}_m_exceeds_{max_initial_position_norm_m:.3f}_m'
        return None
    if previous_position is None or previous_sample_us is None:
        return 'incomplete_previous_pose_state'

    dt_s = (int(sample_us) - int(previous_sample_us)) / 1_000_000.0
    if dt_s <= 0.0:
        return 'non_increasing_sample_timestamp'
    previous = tuple(float(value) for value in previous_position)
    if len(previous) != 3 or not all(math.isfinite(value) and abs(value) <= FLOAT32_MAX for value in previous):
        return 'invalid_previous_position'
    distance_m = math.sqrt(sum((xyz[i] - previous[i]) ** 2 for i in range(3)))
    speed_m_s = distance_m / dt_s
    if speed_m_s > float(max_position_speed_m_s):
        return f'position_speed={speed_m_s:.3f}_m_s_exceeds_{max_position_speed_m_s:.3f}_m_s'
    return None


def monotonic_px4_output_timestamp(
    px4_now_us: int,
    sample_px4_us: int,
    previous_output_timestamp_us: int | None,
    previous_sample_timestamp_us: int | None = None,
) -> int | None:
    """Require both the PX4 publish time and mapped sample time to advance."""
    px4_now_us = int(px4_now_us)
    sample_px4_us = int(sample_px4_us)
    if not 0 < px4_now_us <= PX4_TIMESTAMP_MAX or not 0 < sample_px4_us <= PX4_TIMESTAMP_MAX:
        return None
    if previous_sample_timestamp_us is not None and sample_px4_us <= int(previous_sample_timestamp_us):
        return None
    timestamp_us = max(px4_now_us, sample_px4_us)
    if previous_output_timestamp_us is not None and timestamp_us <= int(previous_output_timestamp_us):
        return None
    return timestamp_us
