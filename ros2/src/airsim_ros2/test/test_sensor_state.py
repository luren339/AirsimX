import math
import unittest

from airsim_ros2.sensor_samples import ImuSampleSelector, SourceSampleFilter
from airsim_ros2.sensor_conversions import pinhole_intrinsics


class SourceSampleTests(unittest.TestCase):
    def test_checking_a_candidate_does_not_commit_its_source_stamp(self):
        gate = SourceSampleFilter()
        check = getattr(gate, 'can_accept', None)
        self.assertTrue(callable(check), 'camera preparation needs a noncommitting duplicate check')
        self.assertTrue(check(100))
        self.assertTrue(check(100))
        self.assertEqual((gate.last_timestamp, gate.accepted), (0, 0))
        self.assertTrue(gate.accept(100))
        self.assertFalse(check(100))
        self.assertEqual((gate.last_timestamp, gate.accepted, gate.rejected), (100, 1, 1))

    def test_only_new_positive_source_timestamps_are_accepted(self):
        gate = SourceSampleFilter()
        self.assertTrue(gate.accept(100))
        self.assertFalse(gate.accept(100))
        self.assertFalse(gate.accept(99))
        self.assertTrue(gate.accept(101))
        self.assertEqual((gate.accepted, gate.rejected), (2, 2))
        with self.assertRaises(ValueError):
            gate.accept(0)
        self.assertEqual(gate.last_timestamp, 101)

    def test_a_new_run_uses_a_new_filter(self):
        old = SourceSampleFilter()
        old.accept(1000)
        self.assertFalse(old.accept(1))
        self.assertTrue(SourceSampleFilter().accept(1))

    def test_unrepresentable_stamp_does_not_poison_later_valid_samples(self):
        gate = SourceSampleFilter()
        with self.assertRaises(ValueError):
            gate.accept(2**63)
        self.assertTrue(gate.accept(100))

    def test_selector_preserves_real_source_samples_without_interpolation(self):
        selector = ImuSampleSelector(200.0)
        source = range(1_000_000_000, 2_000_000_001, 1_000_000)
        chosen = [t for t in source if selector.select(t)]
        self.assertEqual(len(chosen), 201)
        self.assertTrue(all(t % 1_000_000 == 0 for t in chosen))
        self.assertEqual(chosen[0], 1_000_000_000)

    def test_selector_does_not_invent_missing_samples(self):
        selector = ImuSampleSelector(200.0)
        source = [1_000_000_000, 1_100_000_000, 1_105_000_000]
        self.assertEqual([t for t in source if selector.select(t)], source)
        self.assertFalse(selector.select(source[-1]))
        passthrough = ImuSampleSelector(0.0)
        self.assertTrue(passthrough.select(1))
        self.assertTrue(passthrough.select(2))

    def test_invalid_target_rates_are_rejected(self):
        for value in (-1.0, math.nan, math.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                ImuSampleSelector(value)


class CameraParametersTests(unittest.TestCase):
    def test_existing_pinhole_and_pixel_centre_convention_is_preserved(self):
        fx, fy, cx, cy = pinhole_intrinsics(640, 480, 90.0)
        self.assertAlmostEqual(fx, 320.0)
        self.assertAlmostEqual(fy, 320.0)
        self.assertEqual((cx, cy), (319.5, 239.5))

    def test_invalid_dimensions_and_fov_are_rejected(self):
        for args in ((0, 480, 90), (640, -1, 90), (640, 480, 0),
                     (640, 480, 180), (640, 480, math.nan)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                pinhole_intrinsics(*args)


if __name__ == '__main__':
    unittest.main()
