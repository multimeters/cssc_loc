import copy
import math
from pathlib import Path
import tempfile
import unittest

import yaml

from aps_bag_localization.configuration import adapter_parameters, load_config
from aps_bag_localization.geometry import quaternion_multiply, quaternion_rpy, rotate


MASTER = Path(__file__).resolve().parents[3] / 'config' / 'localization.yaml'


class ConfigurationTests(unittest.TestCase):
    def changed_config(self, change):
        config = load_config(MASTER)
        change(config)
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / 'localization.yaml'
            target.write_text(yaml.safe_dump(config), encoding='utf-8')
            return load_config(target)

    def test_full01_parameters_and_relative_paths_are_preserved(self):
        config = load_config(MASTER)
        self.assertTrue(Path(config['paths']['map']).is_absolute())
        self.assertEqual(Path(config['paths']['map']).name, 'GlobalMap_loc.pcd')
        self.assertEqual(config['_native_parameters']['ndt']['ndt.num_threads'], 4)
        self.assertEqual(config['_native_parameters']['ndt']['sensor_points.timeout_sec'], .5)
        self.assertTrue(config['_native_parameters']['ekf']['node.enable_yaw_bias_estimation'])
        self.assertEqual(config['_native_parameters']['gyro']['message_timeout_sec'], .25)
        self.assertEqual(config['replay']['drain_seconds'], 1.)

    def test_one_mount_edit_propagates_to_tf_initial_pose_and_imu_rotation(self):
        original = load_config(MASTER)
        changed = self.changed_config(lambda config: config['extrinsics']['rear_to_lidar'].update(
            xyz_m=[.6, .1, .4], rpy_deg=[1., 17., 3.]))
        transform = changed['_derived']['transforms'][1]
        parameters = adapter_parameters(changed)
        self.assertEqual(transform['xyz_m'], parameters['mount_xyz'])
        self.assertEqual(transform['rpy_rad'], parameters['mount_rpy'])
        self.assertNotEqual(changed['_derived']['initial_base_xyz'], original['_derived']['initial_base_xyz'])
        self.assertNotEqual(changed['_derived']['imu_to_base_quaternion'], original['_derived']['imu_to_base_quaternion'])
        offset = rotate(changed['_derived']['initial_base_quaternion'], transform['xyz_m'])
        recovered = [changed['_derived']['initial_base_xyz'][i] + offset[i] for i in range(3)]
        for actual, expected in zip(recovered, changed['initial_pose']['xyz_m']):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_imu_rotation_edit_is_composed_with_mount_once(self):
        changed = self.changed_config(lambda config: config['extrinsics']['lidar_to_imu'].update(
            rpy_deg=[3., 4., 5.]))
        transforms = changed['_derived']['transforms']
        expected = quaternion_multiply(quaternion_rpy(*transforms[1]['rpy_rad']),
                                       quaternion_rpy(*transforms[3]['rpy_rad']))
        for actual, target in zip(changed['_derived']['imu_to_base_quaternion'], expected):
            self.assertAlmostEqual(actual, target, places=12)

    def test_custom_frames_and_topics_reach_adapter_and_native_nodes(self):
        def change(config):
            config['frames'].update(map='site_map', cloud='laser_body', imu='builtin_imu', wheel='rear_wheel')
            config['topics'].update(wheel='/sensors/wheels', imu='/sensors/imu', points='/sensors/cloud')
        config = self.changed_config(change)
        parameters = adapter_parameters(config)
        self.assertEqual(parameters['imu_frame'], 'builtin_imu')
        self.assertEqual(parameters['cloud_frame'], 'laser_body')
        self.assertEqual(parameters['wheel_frame'], 'rear_wheel')
        self.assertEqual(parameters['wheel_topic'], '/sensors/wheels')
        self.assertEqual(config['_native_parameters']['ndt']['frame.map_frame'], 'site_map')
        self.assertEqual(config['_native_parameters']['ekf']['misc.pose_frame_id'], 'site_map')

    def test_initial_covariance_has_single_yaml_source(self):
        config = self.changed_config(lambda config: config['initial_pose'].update(
            covariance_diagonal=[1., 2., 3., .1, .2, .3]))
        self.assertEqual(adapter_parameters(config)['initial_covariance_diagonal'], [1., 2., 3., .1, .2, .3])

    def test_native_base_constraint_and_identity_aliases_are_validated(self):
        with self.assertRaisesRegex(ValueError, 'base_link'):
            self.changed_config(lambda config: config['frames'].update(base='another_base'))
        with self.assertRaisesRegex(ValueError, 'identity'):
            self.changed_config(lambda config: config['extrinsics']['rear_to_base'].update(xyz_m=[1., 0., 0.]))
        with self.assertRaisesRegex(ValueError, 'identity'):
            self.changed_config(lambda config: config['extrinsics']['lidar_to_cloud'].update(rpy_deg=[0., 2., 0.]))

    def test_resolved_snapshot_recomputes_derived_values(self):
        config = self.changed_config(lambda config: config['_derived'].update(initial_base_xyz=[999., 999., 999.]))
        self.assertLess(config['_derived']['initial_base_xyz'][0], 4.)

    def test_historical_acquisition_replay_requires_simulated_clock(self):
        with self.assertRaisesRegex(ValueError, 'use_sim_time'):
            self.changed_config(lambda config: config['runtime'].update(use_sim_time=False))


if __name__ == '__main__':
    unittest.main()
