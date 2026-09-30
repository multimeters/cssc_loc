import math
from types import SimpleNamespace as NS
import unittest

import numpy as np

from aps_bag_localization.geometry import quaternion_rpy
from aps_bag_localization.livox_deskew import (
    IncompleteMotion, Scan, deskew_scan, fallback_due, parse_scan, quaternion_matrix, valid_mid360_tags,
)


ORIGIN = 1_790_735_606_000_000_000
SETTINGS = dict(raw_frame='livox_frame', header_tolerance_s=.001, max_scan_duration_s=.2,
                min_range_m=.5, max_range_m=100., reject_invalid_tags=True)


def sample_scan(xyz, times):
    times = np.asarray(times, dtype=np.int64)
    return Scan(np.asarray(xyz, dtype=np.float64), np.ones(len(xyz)), times, int(times.min()),
                int(times.max()), len(xyz), 0)


def message(points):
    return NS(header=NS(frame_id='livox_frame', stamp=NS(sec=ORIGIN//10**9, nanosec=ORIGIN % 10**9)),
              timebase=ORIGIN, point_num=len(points), points=points)


def point(x=2., y=0., z=0., t=0, tag=0, reflectivity=99):
    return NS(x=x, y=y, z=z, offset_time=t, tag=tag, reflectivity=reflectivity)


class LivoxDeskewTests(unittest.TestCase):
    def test_fallback_deadline_uses_simulation_time_and_never_emits_future_cloud(self):
        end = ORIGIN+100_000_000
        # Wall time/replay rate are deliberately absent from this decision.
        self.assertFalse(fallback_due(end, end-1, .3, end+1, end+1))
        self.assertFalse(fallback_due(end, end+299_999_999, .3, None, None))
        self.assertTrue(fallback_due(end, end+300_000_000, .3, None, None))
        self.assertTrue(fallback_due(end, end, .3, end+1, end+1))
        self.assertFalse(fallback_due(end, end, .3, end+1, end-1))

    def test_translation_uses_end_reference_without_lidar_or_imu_translation_error(self):
        times = ORIGIN + np.array([0, 100_000_000, 50_000_000], dtype=np.int64)
        scan = sample_scan([[10., 0., 0.], [9.8, 0., 0.], [9.9, 0., 0.]], times)
        imu = [(ORIGIN, 0., 0., 0.), (ORIGIN+100_000_000, 0., 0., 0.)]
        wheel = [(ORIGIN, 2.), (ORIGIN+100_000_000, 2.)]
        result = deskew_scan(scan, imu, wheel, np.eye(3), [.43, .1, .36], .101, .101)
        np.testing.assert_allclose(result, [[9.8, 0., 0.]]*3, atol=1e-12)

    def test_rotation_accounts_for_lidar_lever_arm_and_tilt(self):
        dt = np.array([0., .03, .1, .05])  # Deliberately unordered Livox point times.
        times = ORIGIN + np.rint(dt*1e9).astype(np.int64)
        mount = quaternion_matrix(quaternion_rpy(.1, .4, -.2))
        lever = np.array([.432445, 0., .368458])
        stationary_world = np.array([8., 4., 2.])
        raw = []
        for t in dt:
            rotation = quaternion_matrix(quaternion_rpy(0., 0., t))
            raw.append((rotation.T @ stationary_world - lever) @ mount)
        scan = sample_scan(raw, times)
        imu = [(ORIGIN, 0., 0., 1.), (ORIGIN+100_000_000, 0., 0., 1.)]
        wheel = [(ORIGIN, 0.), (ORIGIN+100_000_000, 0.)]
        result = deskew_scan(scan, imu, wheel, mount, lever, .101, .101)
        expected = (quaternion_matrix(quaternion_rpy(0., 0., .1)).T @ stationary_world - lever) @ mount
        np.testing.assert_allclose(result, np.tile(expected, (4, 1)), atol=1e-12)

    def test_combined_turning_and_forward_speed_matches_circle(self):
        dt = np.array([0., .037, .1])
        times = ORIGIN + np.rint(dt*1e9).astype(np.int64)
        world = np.array([7., -2., 3.])
        lever = np.array([1., .2, .3])
        raw = []
        for t in dt:
            rotation = quaternion_matrix(quaternion_rpy(0., 0., 2*t))
            rear = np.array([1.5*math.sin(2*t), 1.5*(1-math.cos(2*t)), 0.])
            raw.append(rotation.T @ (world - rear) - lever)
        scan = sample_scan(raw, times)
        imu = [(ORIGIN, 0., 0., 2.), (ORIGIN+100_000_000, 0., 0., 2.)]
        wheel = [(ORIGIN, 3.), (ORIGIN+100_000_000, 3.)]
        actual = deskew_scan(scan, imu, wheel, np.eye(3), lever, .101, .101)
        np.testing.assert_allclose(actual, np.tile(raw[-1], (3, 1)), atol=1e-12)

    def test_full_three_axis_angular_motion_is_not_yaw_only(self):
        axis = np.array([1., 2., -3.]) / np.sqrt(14.)
        dt = np.array([0., .04, .1])
        times = ORIGIN + np.rint(dt*1e9).astype(np.int64)
        lever = np.array([.5, 0., .4])
        world = np.array([2., 3., 4.])
        raw = []
        for t in dt:
            quaternion = [*(axis*math.sin(t/2)), math.cos(t/2)]
            raw.append(quaternion_matrix(quaternion).T @ world - lever)
        scan = sample_scan(raw, times)
        imu = [(ORIGIN, *axis), (ORIGIN+100_000_000, *axis)]
        wheel = [(ORIGIN, 0.), (ORIGIN+100_000_000, 0.)]
        actual = deskew_scan(scan, imu, wheel, np.eye(3), lever, .101, .101)
        np.testing.assert_allclose(actual, np.tile(raw[-1], (3, 1)), atol=1e-12)

    def test_large_imu_gap_requires_whole_scan_fallback(self):
        scan = sample_scan([[2., 0., 0.], [2., 0., 0.]], [ORIGIN, ORIGIN+100_000_000])
        imu = [(ORIGIN, 0., 0., 1.), (ORIGIN+100_000_000, 0., 0., 1.)]
        wheel = [(ORIGIN, 0.), (ORIGIN+100_000_000, 0.)]
        with self.assertRaisesRegex(IncompleteMotion, 'imu: gap'):
            deskew_scan(scan, imu, wheel, np.eye(3), [0., 0., 0.], .05, .101)
        np.testing.assert_array_equal(scan.xyz, [[2., 0., 0.], [2., 0., 0.]])

    def test_wheel_coverage_cannot_be_replaced_by_imu(self):
        scan = sample_scan([[2., 0., 0.], [2., 0., 0.]], [ORIGIN, ORIGIN+100_000_000])
        imu = [(ORIGIN, 0., 0., 0.), (ORIGIN+100_000_000, 0., 0., 0.)]
        with self.assertRaisesRegex(IncompleteMotion, 'wheel: no bracketing'):
            deskew_scan(scan, imu, [(ORIGIN+1, 1.), (ORIGIN+100_000_000, 1.)],
                        np.eye(3), [0., 0., 0.], .101, .101)

    def test_scan_boundaries_survive_filtering_and_integer_timestamps(self):
        scan = parse_scan(message([point(t=99_999_999, tag=2), point(t=17), point(t=0, x=float('nan'))]), SETTINGS)
        self.assertEqual(scan.start_ns, ORIGIN)
        self.assertEqual(scan.end_ns, ORIGIN+99_999_999)
        self.assertEqual(scan.times_ns.tolist(), [ORIGIN+17])
        self.assertEqual(scan.rejected_points, 2)
        self.assertEqual(scan.intensity.tolist(), [99.])

    def test_mid360_confidence_tags_are_not_other_model_echo_bits(self):
        # 0x10 is medium confidence 'other', 0x20 low, 0x04 medium dust,
        # 0x08 low dust, 0x02 low glue, upper two bits are reserved.
        actual = valid_mid360_tags([0, 0x10, 0x20, 0x04, 0x08, 0x02, 0x15, 0xc0])
        self.assertEqual(actual.tolist(), [True, True, False, True, False, False, True, True])

    def test_bad_timebase_offsets_frame_and_count_fail_explicitly(self):
        for mutate in (
            lambda m: setattr(m, 'timebase', ORIGIN+10_000_000),
            lambda m: setattr(m.points[0], 'offset_time', 1_000_000_000),
            lambda m: setattr(m.header, 'frame_id', 'map'),
            lambda m: setattr(m, 'point_num', 99),
        ):
            msg = message([point()])
            mutate(msg)
            with self.assertRaises(ValueError):
                parse_scan(msg, SETTINGS)

    def test_zero_rotation_has_no_nans(self):
        scan = sample_scan([[1., 2., 3.], [1., 2., 3.]], [ORIGIN, ORIGIN+100_000_000])
        result = deskew_scan(scan, [(ORIGIN, 0., 0., 0.), (ORIGIN+100_000_000, 0., 0., 0.)],
            [(ORIGIN, 0.), (ORIGIN+100_000_000, 0.)], np.eye(3), [.4, 0., .3], .101, .101)
        np.testing.assert_allclose(result, scan.xyz)


if __name__ == '__main__':
    unittest.main()
