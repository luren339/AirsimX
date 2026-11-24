"""Convert FAST-LIO2 ROS odometry from NWU/FLU to PX4 local FRD odometry."""
from __future__ import annotations

import math

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import VehicleOdometry


def sim_sample_to_px4_us(sample_stamp_ns: int, sim_clock_ns: int, px4_now_us: int) -> int:
    """Map an AirSim sample time into PX4's local HRT clock using /clock and sensor_combined."""
    sample_ns, sim_ns, px4_us = int(sample_stamp_ns), int(sim_clock_ns), int(px4_now_us)
    if sample_ns <= 0 or sim_ns <= 0 or px4_us <= 0:
        raise ValueError("AirSim sample, /clock and PX4 local clock timestamps must be positive")
    sample_us = sample_ns // 1_000
    sim_us = sim_ns // 1_000
    return sample_us + (px4_us - sim_us)


def _finite_covariance(values, floor: float = 0.01) -> tuple[float, float, float]:
    result = []
    for value in values:
        value = float(value)
        result.append(max(value, floor) if math.isfinite(value) else floor)
    return tuple(result)


def ros_odom_to_px4_vehicle_odometry(
    odom: Odometry,
    *,
    now_us: int | None = None,
    variance_floor: float = 0.01,
    timestamp_sample_us: int | None = None,
) -> VehicleOdometry:
    """Convert FAST-LIO2 odometry to PX4 EV input without truth or GNSS data.

    Assumption: FAST-LIO2 consumes FLU sensor measurements and publishes its
    local world in NWU axes. PX4 FRD conversion flips Y and Z for both the
    local pose frame and body-frame twist. Variances use a conservative floor
    because this FAST-LIO2 build publishes zero covariance.
    """
    from px4_msgs.msg import VehicleOdometry

    stamp_ns = int(odom.header.stamp.sec) * 1_000_000_000 + int(odom.header.stamp.nanosec)
    if stamp_ns <= 0:
        raise ValueError(f"FAST-LIO odometry has invalid header timestamp: {stamp_ns}")
    raw_sample_us = stamp_ns // 1_000
    sample_us = raw_sample_us if timestamp_sample_us is None else int(timestamp_sample_us)
    message_time_us = sample_us if now_us is None else max(sample_us, int(now_us))

    position = odom.pose.pose.position
    orientation = odom.pose.pose.orientation
    qx, qy, qz, qw = map(float, (orientation.x, orientation.y, orientation.z, orientation.w))
    q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(q_norm) or q_norm < 1e-8:
        raise ValueError("FAST-LIO odometry quaternion is zero or non-finite")
    qx, qy, qz, qw = qx / q_norm, qy / q_norm, qz / q_norm, qw / q_norm

    linear = odom.twist.twist.linear
    angular = odom.twist.twist.angular
    msg = VehicleOdometry()
    msg.timestamp = message_time_us
    msg.timestamp_sample = sample_us
    msg.pose_frame = int(VehicleOdometry.POSE_FRAME_FRD)
    msg.position = [float(position.x), -float(position.y), -float(position.z)]
    # Quaternion is Hamiltonian (w,x,y,z); conjugation by the 180-degree X-axis
    # basis change maps NWU/FLU attitude into local FRD without changing yaw origin.
    msg.q = [qw, qx, -qy, -qz]
    msg.velocity_frame = int(VehicleOdometry.VELOCITY_FRAME_BODY_FRD)
    msg.velocity = [float(linear.x), -float(linear.y), -float(linear.z)]
    msg.angular_velocity = [float(angular.x), -float(angular.y), -float(angular.z)]
    msg.position_variance = _finite_covariance(
        (odom.pose.covariance[0], odom.pose.covariance[7], odom.pose.covariance[14]), variance_floor
    )
    msg.orientation_variance = _finite_covariance(
        (odom.pose.covariance[21], odom.pose.covariance[28], odom.pose.covariance[35]), variance_floor
    )
    msg.velocity_variance = _finite_covariance(
        (odom.twist.covariance[0], odom.twist.covariance[7], odom.twist.covariance[14]), variance_floor
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
    if len(previous) != 3 or not all(math.isfinite(value) for value in previous):
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
) -> int | None:
    """Return a strictly advancing PX4 publish timestamp, or None if the clock is stale."""
    px4_now_us = int(px4_now_us)
    sample_px4_us = int(sample_px4_us)
    if px4_now_us <= 0 or sample_px4_us <= 0:
        return None
    timestamp_us = max(px4_now_us, sample_px4_us)
    if previous_output_timestamp_us is not None and timestamp_us <= int(previous_output_timestamp_us):
        return None
    return timestamp_us
