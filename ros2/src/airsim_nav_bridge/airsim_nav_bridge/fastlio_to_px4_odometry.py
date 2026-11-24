#!/usr/bin/env python3
"""Forward FAST-LIO2 estimates to PX4 external-vision input; never commands flight."""
from __future__ import annotations

import threading
import time
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorCombined, VehicleOdometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from .px4_odometry_conversion import (
    fastlio_pose_rejection_reason,
    monotonic_px4_output_timestamp,
    ros_odom_to_px4_vehicle_odometry,
    sim_sample_to_px4_us,
)


class FastLioToPx4Odometry(Node):
    def __init__(self) -> None:
        super().__init__('fastlio_to_px4_odometry_bridge')
        self.declare_parameter('input_topic', '/Odometry')
        self.declare_parameter('output_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('variance_floor', 0.01)
        self.declare_parameter('max_initial_position_norm_m', 5.0)
        self.declare_parameter('max_position_speed_m_s', 15.0)
        self.declare_parameter('diagnostic_period_s', 5.0)
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('px4_time_topic', '/fmu/out/sensor_combined')
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.variance_floor = float(self.get_parameter('variance_floor').value)
        self.max_initial_position_norm_m = float(self.get_parameter('max_initial_position_norm_m').value)
        self.max_position_speed_m_s = float(self.get_parameter('max_position_speed_m_s').value)
        self.diagnostic_period_s = float(self.get_parameter('diagnostic_period_s').value)
        output_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            VehicleOdometry, str(self.get_parameter('output_topic').value), output_qos
        )
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self._on_odometry, qos_profile_sensor_data
        )
        self._sim_clock_ns = 0
        self._px4_now_us = 0
        self.clock_subscription = self.create_subscription(
            Clock, str(self.get_parameter('clock_topic').value), self._on_clock, qos_profile_sensor_data
        )
        self.px4_time_subscription = self.create_subscription(
            SensorCombined, str(self.get_parameter('px4_time_topic').value), self._on_px4_time, qos_profile_sensor_data
        )
        self._last_sample_us = -1
        self._last_accepted_position = None
        self._last_accepted_sample_us = None
        self._last_output_timestamp_us = None
        self._timestamp_rejection_count = 0
        self._last_rejection_log = 0.0
        self._count = 0
        self._last_report = time.monotonic()
        self._lock = threading.Lock()
        self.get_logger().info(
            f'Forwarding {self.input_topic} to {self.publisher.topic_name} as local FRD; '
            'AirSim sample stamps are mapped to PX4 local time using /clock and sensor_combined. '
            'No simulator truth, GNSS, arming, or setpoints are used.'
        )

    def _on_clock(self, msg: Clock) -> None:
        self._sim_clock_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)

    def _on_px4_time(self, msg: SensorCombined) -> None:
        self._px4_now_us = int(msg.timestamp)

    def _on_odometry(self, odom: Odometry) -> None:
        sample_us = int(odom.header.stamp.sec) * 1_000_000 + int(odom.header.stamp.nanosec) // 1_000
        position = (
            float(odom.pose.pose.position.x),
            float(odom.pose.pose.position.y),
            float(odom.pose.pose.position.z),
        )
        with self._lock:
            if sample_us <= self._last_sample_us:
                return
            rejection_reason = fastlio_pose_rejection_reason(
                position,
                sample_us=sample_us,
                previous_position=self._last_accepted_position,
                previous_sample_us=self._last_accepted_sample_us,
                max_initial_position_norm_m=self.max_initial_position_norm_m,
                max_position_speed_m_s=self.max_position_speed_m_s,
            )
            if rejection_reason is not None:
                now = time.monotonic()
                if now - self._last_rejection_log >= self.diagnostic_period_s:
                    self.get_logger().warning(
                        'Rejected FAST-LIO odometry before PX4 publication: ' + rejection_reason
                    )
                    self._last_rejection_log = now
                return
            try:
                if self._sim_clock_ns <= 0 or self._px4_now_us <= 0:
                    return
                sample_sim_ns = int(odom.header.stamp.sec) * 1_000_000_000 + int(odom.header.stamp.nanosec)
                sample_px4_us = sim_sample_to_px4_us(sample_sim_ns, self._sim_clock_ns, self._px4_now_us)
                now_us = monotonic_px4_output_timestamp(
                    self._px4_now_us, sample_px4_us, self._last_output_timestamp_us
                )
                if now_us is None:
                    self._timestamp_rejection_count += 1
                    now = time.monotonic()
                    if now - self._last_rejection_log >= self.diagnostic_period_s:
                        self.get_logger().warning(
                            'Skipped FAST-LIO odometry because PX4 output timestamp did not advance: '
                            f'px4_now={self._px4_now_us} us, sample={sample_px4_us} us, '
                            f'previous_output={self._last_output_timestamp_us} us'
                        )
                        self._last_rejection_log = now
                    return
                msg = ros_odom_to_px4_vehicle_odometry(
                    odom, now_us=now_us, timestamp_sample_us=sample_px4_us, variance_floor=self.variance_floor
                )
            except (TypeError, ValueError, OverflowError) as exc:
                self.get_logger().error(f'Dropping invalid FAST-LIO odometry: {exc}')
                return
            self._last_sample_us = sample_us
            self._last_accepted_position = position
            self._last_accepted_sample_us = sample_us
            self._last_output_timestamp_us = int(msg.timestamp)
            self.publisher.publish(msg)
            self._count += 1
            now = time.monotonic()
            if now - self._last_report >= self.diagnostic_period_s:
                self.get_logger().info(
                    f'Published {self._count} unique FAST-LIO odometry samples; '
                    f'latest sample time={sample_us} us; '
                    f'timestamp_wait_drops={self._timestamp_rejection_count}; '
                    f'input frame={odom.header.frame_id}/{odom.child_frame_id}'
                )
                self._count = 0
                self._timestamp_rejection_count = 0
                self._last_report = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastLioToPx4Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
