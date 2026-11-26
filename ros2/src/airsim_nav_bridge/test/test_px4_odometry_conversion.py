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

    def test_time_mapping_rejects_excess_clock_pairing_skew(self):
        map_time = self.api().sim_sample_to_px4_us
        self.assertEqual(map_time(10_050_000_000, 10_000_000_000, 30_000_000,
                                  max_clock_pairing_skew_s=0.1), 30_050_000)
        with self.assertRaisesRegex(ValueError, 'clock_pairing_skew'):
            map_time(11_000_000_000, 10_000_000_000, 31_000_000,
                     max_clock_pairing_skew_s=0.1)

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

    def test_mapped_sample_must_advance_even_when_publish_clock_advances(self):
        timestamp = self.api().monotonic_px4_output_timestamp
        for sample in (29_965_000, 30_000_000):
            with self.subTest(sample=sample):
                self.assertIsNone(timestamp(30_005_000, sample, 30_000_000,
                                            previous_sample_timestamp_us=30_000_000))
        self.assertEqual(timestamp(30_200_000, 30_050_000, 30_100_000,
                                   previous_sample_timestamp_us=30_000_000), 30_200_000)

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
        for previous in ((math.nan, 0.0, 0.0), (0.0, 0.0), (1e200, 0.0, 0.0)):
            with self.subTest(previous=previous):
                self.assertEqual(self.api().fastlio_pose_rejection_reason(
                    (0.0, 0.0, 0.0), sample_us=2, previous_position=previous,
                    previous_sample_us=1), 'invalid_previous_position')

    def test_rejects_position_outside_float32_before_speed_arithmetic(self):
        self.assertEqual(self.api().fastlio_pose_rejection_reason(
            (1e200, 0.0, 0.0), sample_us=2, previous_position=(0.0, 0.0, 0.0),
            previous_sample_us=1), 'position_out_of_float32_range')

    def test_covariance_floor_applies_to_finite_nonnegative_values(self):
        sanitize = self.api()._finite_covariance
        self.assertEqual(sanitize((0.0, 0.01, 0.5), 0.02), (0.02, 0.02, 0.5))

    def test_covariance_rejects_negative_and_non_finite_values(self):
        sanitize = self.api()._finite_covariance
        for value in (-1.0, math.nan, math.inf, -math.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                sanitize((0.01, value, 0.5))

    def test_orientation_covariance_is_rotated_from_fixed_to_body_axes(self):
        rotate = self.api()._orientation_variance_body_axes
        half = math.sqrt(0.5)
        covariance = (0.01, 0.0, 0.0,
                      0.0, 0.04, 0.0,
                      0.0, 0.0, 0.25)
        for actual, expected in zip(rotate(covariance, (half, 0.0, 0.0, half)),
                                    (0.01, 0.25, 0.04)):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_covariance_rejects_finite_float32_overflow(self):
        with self.assertRaises(ValueError):
            self.api()._finite_covariance((0.0, 1e100, 0.0))

    def test_variance_floor_must_be_positive_finite_and_usable_in_float32(self):
        for floor in (0.0, -0.01, math.nan, math.inf, 1e100, 1e-100):
            with self.subTest(floor=floor), self.assertRaisesRegex(ValueError, 'variance_floor'):
                self.api()._finite_covariance((0.0, 0.0, 0.0), floor)

    def test_px4_timestamp_must_fit_uint64(self):
        for now, sample in (((1 << 64), 1), (1, (1 << 64))):
            with self.subTest(now=now, sample=sample):
                self.assertIsNone(self.api().monotonic_px4_output_timestamp(now, sample, None))


if __name__ == '__main__':
    unittest.main()
