import math
import unittest

from aps_bag_localization.trajectory import TrajectorySummary
from aps_bag_localization.validation import FreshnessGuard, diagonal_covariance


class FreshnessTests(unittest.TestCase):
    def populated(self):
        guard = FreshnessGuard(timeout=0.2)
        for stream in guard.STREAMS:
            guard.observe(stream, 10.0)
        return guard

    def test_waits_for_all_inputs_and_clock(self):
        guard = FreshnessGuard()
        self.assertFalse(guard.activate(0.0))
        guard.observe('imu', 10.0)
        guard.observe('wheel', 10.0)
        self.assertFalse(guard.activate(10.0))
        self.assertEqual(guard.problem(10.0), 'waiting_for_gyro')
        guard.observe('gyro', 10.0)
        self.assertTrue(guard.activate(10.0))

    def test_stale_imu_latches_stop_even_after_new_messages(self):
        guard = self.populated()
        self.assertTrue(guard.activate(10.0))
        guard.observe('wheel', 10.21)
        guard.observe('gyro', 10.21)
        self.assertEqual(guard.problem(10.21), 'stale_imu')
        guard.observe('imu', 10.22)
        self.assertEqual(guard.problem(10.22), 'stale_imu')
        self.assertFalse(guard.activate(10.22))

    def test_clock_reversal_stops_active_run(self):
        guard = self.populated()
        guard.activate(10.0)
        self.assertEqual(guard.problem(9.0), 'clock_moved_backwards')

    def test_out_of_order_sensor_stops_active_run(self):
        guard = self.populated()
        guard.activate(10.0)
        self.assertFalse(guard.observe('imu', 9.9))
        self.assertEqual(guard.problem(10.0), 'nonmonotonic_imu')

    def test_future_input_blocks_activation(self):
        guard = self.populated()
        guard.observe('wheel', 10.5)
        self.assertFalse(guard.activate(10.0))
        self.assertFalse(guard.active)

    def test_zero_and_nan_covariance_have_positive_floor(self):
        covariance = [0.0] * 9
        covariance[4] = math.nan
        covariance[8] = 0.2
        self.assertEqual(diagonal_covariance(covariance, 3, 0.01),
                         [0.01, 0., 0., 0., 0.01, 0., 0., 0., 0.2])


class TrajectoryTests(unittest.TestCase):
    def test_numeric_sample_acceptance_is_not_accuracy_verification(self):
        summary = TrajectorySummary(min_samples=2, min_span=1.0)
        self.assertTrue(summary.add(10., 'map', (0., 0., 0.), (0., 0., 0., 1.), [0.] * 36))
        self.assertTrue(summary.add(11., 'map', (3., 4., 0.), (0., 0., 0., 1.), [0.] * 36))
        report = summary.report()
        self.assertTrue(report['sample_checks_passed'])
        self.assertFalse(report['accuracy_verified'])
        self.assertEqual(report['path_length_m'], 5.)

    def test_rejects_frame_switch_nan_and_duplicate_timestamp(self):
        summary = TrajectorySummary(min_samples=1, min_span=0.)
        summary.add(10., 'map', (0., 0., 0.), (0., 0., 0., 1.), [0.] * 36)
        self.assertFalse(summary.add(10., 'map', (0., 0., 0.), (0., 0., 0., 1.), [0.] * 36))
        self.assertFalse(summary.add(11., 'odom', (0., 0., 0.), (0., 0., 0., 1.), [0.] * 36))
        self.assertFalse(summary.add(11., 'map', (math.nan, 0., 0.), (0., 0., 0., 1.), [0.] * 36))
        self.assertFalse(summary.report()['sample_checks_passed'])


if __name__ == '__main__':
    unittest.main()
