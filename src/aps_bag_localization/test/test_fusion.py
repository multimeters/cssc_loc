import math
import unittest

from aps_bag_localization.geometry import (
    angular_velocity_to_base, body_pose_to_base, fusion_mode, quaternion_multiply, quaternion_rpy, rotate,
)


class FrameTests(unittest.TestCase):
    def assert_vector_close(self, first, second):
        for left, right in zip(first, second):
            self.assertAlmostEqual(left, right, places=10)

    def test_known_translation_and_yaw(self):
        xyz, q = body_pose_to_base((3., 5., 0.), (0., 0., math.pi / 2),
                                  (2., 0., 0.), (0., 0., 0.))
        self.assert_vector_close(xyz, (3., 3., 0.))
        self.assert_vector_close(q, quaternion_rpy(0., 0., math.pi / 2))

    def test_real_seed_recomposes_exact_body_pose(self):
        body_xyz = (3.569397, -2.387311, .471739)
        body_rpy = (-.017401, .357730, .111514)
        mount_xyz = (.432445, 0., .368458)
        mount_rpy = (0., math.radians(25.06895), 0.)
        base_xyz, base_q = body_pose_to_base(body_xyz, body_rpy, mount_xyz, mount_rpy)
        offset = rotate(base_q, mount_xyz)
        result_xyz = tuple(base_xyz[i] + offset[i] for i in range(3))
        result_q = quaternion_multiply(base_q, quaternion_rpy(*mount_rpy))
        self.assert_vector_close(result_xyz, body_xyz)
        self.assert_vector_close(result_q, quaternion_rpy(*body_rpy))
        self.assertGreater(math.dist(base_xyz, body_xyz), .5)

    def test_rejects_invalid_extrinsic(self):
        with self.assertRaises(ValueError):
            body_pose_to_base((0., 0., 0.), (0., 0., 0.),
                              (0., math.nan, 0.), (0., 0., 0.))


class FusionModeTests(unittest.TestCase):
    @staticmethod
    def mode(now, latest):
        return fusion_mode(now, latest, .25, .5, 1., .1)

    def test_imu_gap_uses_live_ndt_and_can_recover(self):
        latest = {'gyro': 10., 'ndt': 10.5}
        self.assertEqual(self.mode(10.5, latest), ('NDT_ONLY', True))
        latest['gyro'] = 10.51
        self.assertEqual(self.mode(10.51, latest), ('FUSED', True))

    def test_only_short_predictions_are_public(self):
        latest = {'gyro': 10., 'ndt': 10.}
        self.assertEqual(self.mode(10.6, latest), ('PREDICTING', True))
        self.assertEqual(self.mode(11.1, latest), ('STALE', False))
        latest['ndt'] = 11.11
        self.assertEqual(self.mode(11.11, latest), ('NDT_ONLY', True))

    def test_future_observation_does_not_enable_output(self):
        self.assertEqual(self.mode(10., {'gyro': 11., 'ndt': 11.}), ('STALE', False))


class ImuTransformTests(unittest.TestCase):
    def test_positive_pitch_recovers_rear_center_pure_yaw_rate(self):
        pitch = math.radians(25.06895)
        raw_imu_rates = (-math.sin(pitch), 0., math.cos(pitch))
        rates, covariance = angular_velocity_to_base(raw_imu_rates, [0.] * 9,
                                                     quaternion_rpy(0., pitch, 0.), .0004)
        for actual, expected in zip(rates, (0., 0., 1.)):
            self.assertAlmostEqual(actual, expected, places=12)
        for index in (0, 4, 8):
            self.assertAlmostEqual(covariance[index], .0004, places=12)

    def test_full_covariance_rotates_correlations(self):
        rates, covariance = angular_velocity_to_base(
            (1., 0., 0.), [2., .5, 0., .5, 1., 0., 0., 0., 3.],
            quaternion_rpy(0., 0., math.pi / 2), .0004)
        expected = [1., -.5, 0., -.5, 2., 0., 0., 0., 3.]
        for actual, target in zip(covariance, expected):
            self.assertAlmostEqual(actual, target, places=12)
        self.assertAlmostEqual(rates[1], 1., places=12)

    def test_mount_and_imu_rotation_compose_in_correct_order(self):
        mount = quaternion_rpy(.1, .4, -.2)
        imu = quaternion_rpy(.2, -.1, .3)
        combined = quaternion_multiply(mount, imu)
        raw = (.2, -.3, 1.1)
        result, _ = angular_velocity_to_base(raw, [0.] * 9, combined, .0004)
        expected = rotate(mount, rotate(imu, raw))
        for actual, target in zip(result, expected):
            self.assertAlmostEqual(actual, target, places=12)


if __name__ == '__main__':
    unittest.main()
