"""Offline checks for the existing NWU/FLU external-odometry conversion helpers."""

import importlib
import importlib.util
import math
import unittest


class Px4OdometryConversionTests(unittest.TestCase):
    def api(self):
        name = 'airsim_nav_bridge.px4_odometry_conversion'
        self.assertIsNotNone(importlib.util.find_spec(name), 'odometry helpers must be installable')
        return importlib.import_module(name)

    def test_sample_time_maps_into_px4_local_clock(self):
        self.assertEqual(self.api().sim_sample_to_px4_us(10_000_000_000, 10_100_000_000, 30_100_000),
                         30_000_000)

    def test_time_mapping_preserves_a_sample_ahead_of_latest_clock(self):
        self.assertEqual(self.api().sim_sample_to_px4_us(10_200_000_000, 10_100_000_000, 30_100_000),
                         30_200_000)

    def test_time_mapping_truncates_nanoseconds_to_microseconds(self):
        self.assertEqual(self.api().sim_sample_to_px4_us(1_234_567_890, 1_234_567_900, 100_000),
                         100_000)

    def test_time_mapping_requires_three_positive_clocks(self):
        for values in ((0, 1000, 1), (1000, 0, 1), (1000, 1000, 0), (-1, 1000, 1)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.api().sim_sample_to_px4_us(*values)

    def test_publish_timestamp_advances_only_when_px4_time_advances(self):
        timestamp = self.api().monotonic_px4_output_timestamp
        self.assertEqual(timestamp(500, 400, None), 500)
        self.assertIsNone(timestamp(500, 450, 500))
        self.assertEqual(timestamp(501, 490, 500), 501)
        self.assertEqual(timestamp(501, 510, 500), 510)

    def test_non_positive_px4_timestamps_are_not_published(self):
        for now, sample in ((0, 1), (1, 0), (-1, 2), (2, -1)):
            with self.subTest(now=now, sample=sample):
                self.assertIsNone(self.api().monotonic_px4_output_timestamp(now, sample, None))

    def test_rejects_large_first_local_pose(self):
        reason = self.api().fastlio_pose_rejection_reason((223.554, 144.730, -82.334),
                                                        sample_us=1_000_000)
        self.assertIn('initial_position_norm', reason)

    def test_rejects_impossible_position_jump(self):
        reason = self.api().fastlio_pose_rejection_reason(
            (2.0, 0.0, 0.0), sample_us=1_050_000, previous_position=(0.0, 0.0, 0.0),
            previous_sample_us=1_000_000)
        self.assertIn('position_speed', reason)

    def test_accepts_plausible_initial_pose_and_motion(self):
        guard = self.api().fastlio_pose_rejection_reason
        self.assertIsNone(guard((1.0, 1.0, 1.0), sample_us=1_000_000))
        self.assertIsNone(guard((0.2, 0.0, 0.0), sample_us=1_050_000,
                                previous_position=(0.0, 0.0, 0.0), previous_sample_us=1_000_000))

    def test_accepts_positions_at_configured_limits(self):
        guard = self.api().fastlio_pose_rejection_reason
        self.assertIsNone(guard((5.0, 0.0, 0.0), sample_us=1_000_000))
        self.assertIsNone(guard((0.75, 0.0, 0.0), sample_us=1_050_000,
                                previous_position=(0.0, 0.0, 0.0), previous_sample_us=1_000_000))

    def test_rejects_non_finite_position(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                self.assertEqual(self.api().fastlio_pose_rejection_reason(
                    (value, 0.0, 0.0), sample_us=1_000_000), 'non_finite_position')

    def test_rejects_malformed_position_and_incomplete_previous_state(self):
        guard = self.api().fastlio_pose_rejection_reason
        self.assertEqual(guard((0.0, 0.0), sample_us=1), 'invalid_position_shape')
        self.assertEqual(guard((0.0, 0.0, 0.0), sample_us=0), 'invalid_sample_timestamp')
        self.assertEqual(guard((0.0, 0.0, 0.0), sample_us=1, previous_sample_us=1),
                         'incomplete_previous_pose_state')

    def test_rejects_non_increasing_sample_times(self):
        for sample in (1_000_000, 999_999):
            with self.subTest(sample=sample):
                self.assertEqual(self.api().fastlio_pose_rejection_reason(
                    (0.0, 0.0, 0.0), sample_us=sample, previous_position=(0.0, 0.0, 0.0),
                    previous_sample_us=1_000_000), 'non_increasing_sample_timestamp')

    def test_rejects_invalid_previous_position(self):
        for previous in ((math.nan, 0.0, 0.0), (0.0, 0.0)):
            with self.subTest(previous=previous):
                self.assertEqual(self.api().fastlio_pose_rejection_reason(
                    (0.0, 0.0, 0.0), sample_us=2, previous_position=previous,
                    previous_sample_us=1), 'invalid_previous_position')

    def test_covariance_uses_floor_for_zero_negative_and_unknown_values(self):
        sanitize = self.api()._finite_covariance
        self.assertEqual(sanitize((0.0, -1.0, math.nan)), (0.01, 0.01, 0.01))
        self.assertEqual(sanitize((math.inf, -math.inf, 0.5), 0.02), (0.02, 0.02, 0.5))


if __name__ == '__main__':
    unittest.main()
