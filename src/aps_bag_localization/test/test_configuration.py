import copy
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import time
import unittest

import yaml

from aps_bag_localization.configuration import adapter_parameters, load_config
from aps_bag_localization.geometry import quaternion_multiply, quaternion_rpy, rotate

try:
    from launch import LaunchContext, LaunchDescription, LaunchService
    from launch.actions import ExecuteProcess, RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node
    from aps_bag_localization.fusion_launch import nodes, shutdown_on_required_process_exit
    LAUNCH_AVAILABLE = True
except ImportError:
    LAUNCH_AVAILABLE = False


MASTER = Path(__file__).resolve().parents[3] / 'config' / 'localization.yaml'


class ConfigurationTests(unittest.TestCase):
    def changed_config(self, change, mode=None):
        config = load_config(MASTER)
        change(config)
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / 'localization.yaml'
            target.write_text(yaml.safe_dump(config), encoding='utf-8')
            return load_config(target, mode=mode)

    def test_full01_parameters_and_relative_paths_are_preserved(self):
        config = load_config(MASTER, mode='replay')
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
        self.assertEqual(changed['_derived']['base_to_lidar_xyz'], transform['xyz_m'])
        self.assertEqual(changed['_derived']['base_to_lidar_quaternion'], list(quaternion_rpy(*transform['rpy_rad'])))
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
            config['topics'].update(wheel='/sensors/wheels', imu='/sensors/imu',
                                     points='/sensors/raw_livox', processed_points='/sensors/deskewed')
        config = self.changed_config(change)
        parameters = adapter_parameters(config)
        self.assertEqual(parameters['imu_frame'], 'builtin_imu')
        self.assertEqual(parameters['cloud_frame'], 'laser_body')
        self.assertEqual(parameters['wheel_frame'], 'rear_wheel')
        self.assertEqual(parameters['wheel_topic'], '/sensors/wheels')
        self.assertEqual(parameters['raw_points_topic'], '/sensors/raw_livox')
        self.assertEqual(parameters['points_topic'], '/sensors/deskewed')
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
        config = load_config(MASTER, mode='replay')
        self.assertEqual(config['runtime'], {'mode': 'replay', 'use_sim_time': True})
        self.assertTrue(all(params['use_sim_time'] for params in config['_native_parameters'].values()))
        with self.assertRaisesRegex(ValueError, 'use_sim_time'):
            self.changed_config(lambda config: config['runtime'].update(use_sim_time=True))

    def test_live_defaults_wait_for_topic_and_need_no_bag(self):
        config = self.changed_config(lambda config: config['paths'].update(bag=None))
        self.assertIsNone(config['paths']['bag'])
        self.assertEqual(config['runtime'], {'mode': 'live', 'use_sim_time': False})
        self.assertFalse(any(params['use_sim_time'] for params in config['_native_parameters'].values()))
        parameters = adapter_parameters(config)
        self.assertEqual(parameters['initialization'], 'topic')
        self.assertEqual(parameters['initial_pose_input_topic'], '/initialpose')
        self.assertFalse(config['live']['localhost_only'])

    def test_live_configuration_initialization_is_explicit(self):
        config = self.changed_config(lambda config: config['live'].update(initialization='config'))
        self.assertEqual(adapter_parameters(config)['initialization'], 'config')
        with self.assertRaisesRegex(ValueError, 'initialization'):
            self.changed_config(lambda config: config['live'].update(initialization='automatic'))
        with self.assertRaisesRegex(ValueError, 'initial_pose_input'):
            self.changed_config(lambda config: config['topics'].update(initial_pose_input=config['topics']['initial_pose']))

    def test_mode_override_and_serialized_roundtrip_keep_native_clocks_consistent(self):
        config = load_config(MASTER, mode='replay')
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'replay.yaml'
            path.write_text(yaml.safe_dump(config), encoding='utf-8')
            replay = load_config(path)
            live = load_config(path, mode='live')
        self.assertTrue(replay['runtime']['use_sim_time'])
        self.assertEqual(adapter_parameters(replay)['initialization'], 'config')
        self.assertFalse(live['runtime']['use_sim_time'])
        self.assertFalse(any(params['use_sim_time'] for params in live['_native_parameters'].values()))
        self.assertEqual(replay['_derived']['transforms'], live['_derived']['transforms'])
        with self.assertRaisesRegex(ValueError, 'override'):
            load_config(MASTER, mode='invalid')

    def test_live_age_and_log_settings_are_validated(self):
        for name, value in (('max_sensor_age_s', 0.), ('future_tolerance_s', -.1),
                            ('initial_pose_ack_position_m', 0.), ('initial_pose_ack_angle_rad', -.05),
                            ('ndt_idle_timeout_s', 0.), ('ndt_idle_timeout_s', .5),
                            ('motion_wait_timeout_s', 0.), ('motion_wait_timeout_s', .5),
                            ('motion_wait_timeout_s', .6),
                            ('status_period_s', float('inf')), ('log_max_bytes', 0),
                            ('log_backup_count', 1.5), ('domain_id', -1)):
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.changed_config(lambda config: config['live'].update({name: value}))

    def test_raw_livox_origin_is_independent_of_imu_chip_translation(self):
        original = load_config(MASTER)
        changed = self.changed_config(lambda config: config['extrinsics']['lidar_to_imu'].update(
            xyz_m=[.3, -.4, .2]))
        self.assertEqual(original['_derived']['base_to_lidar_xyz'], changed['_derived']['base_to_lidar_xyz'])
        self.assertEqual(original['_derived']['base_to_lidar_quaternion'],
                         changed['_derived']['base_to_lidar_quaternion'])
        self.assertEqual(original['_derived']['initial_base_xyz'], changed['_derived']['initial_base_xyz'])
        self.assertEqual(original['livox']['raw_frame'], 'livox_frame')
        self.assertEqual(adapter_parameters(original)['cloud_frame'], original['frames']['cloud'])

    def test_raw_processing_limits_and_sample_stride_are_validated(self):
        for name in ('max_range_m', 'max_scan_duration_s', 'max_imu_gap_s', 'max_wheel_gap_s',
                     'buffer_seconds', 'wait_timeout_s', 'header_tolerance_s'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.changed_config(lambda config: config['livox'].update({name: float('nan')}))
        with self.assertRaisesRegex(ValueError, 'max_range_m'):
            self.changed_config(lambda config: config['livox'].update(min_range_m=101.))
        with self.assertRaisesRegex(ValueError, 'scan_sample_stride'):
            self.changed_config(lambda config: config['validation'].update(scan_sample_stride=0))
        with self.assertRaisesRegex(ValueError, 'separate topic'):
            self.changed_config(lambda config: config['topics'].update(processed_points=config['topics']['points']))
        config = self.changed_config(lambda config: config['replay'].update(tail_seconds=.2))
        self.assertEqual(config['runtime']['mode'], 'live')
        with self.assertRaisesRegex(ValueError, 'tail_seconds'):
            self.changed_config(lambda config: config['replay'].update(tail_seconds=.2), mode='replay')

    @unittest.skipUnless(LAUNCH_AVAILABLE, 'ROS launch libraries required')
    def test_launch_has_raw_preprocessor_in_ten_node_graph(self):
        config = load_config(MASTER)
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            # Construction only: the launch checks existence but does not open PCD.
            map_path = folder / 'map.pcd'
            map_path.write_text('unused configuration test map', encoding='utf-8')
            config['paths']['map'] = str(map_path)
            path = folder / 'runtime.yaml'
            path.write_text(yaml.safe_dump(config), encoding='utf-8')
            context = LaunchContext()
            context.launch_configurations['config_file'] = str(path)
            actions = nodes(context)
            process_actions = [action for action in actions if isinstance(action, Node)]
            watchdogs = [action for action in actions if isinstance(action, RegisterEventHandler)]
            executables = [action.node_executable if isinstance(action.node_executable, str)
                           else perform_substitutions(context, action.node_executable) for action in process_actions]
            self.assertEqual(len(process_actions), 10)
            self.assertEqual(len(watchdogs), 10)
            self.assertEqual(executables.count('raw_livox'), 1)
            self.assertEqual(executables.count('fusion_adapter'), 1)
            self.assertNotIn('ndt_feedback', executables)
            with self.assertRaisesRegex(ValueError, 'requires runtime.mode=replay'):
                nodes(context, required_mode='replay')
            config['runtime'].update(mode='replay', use_sim_time=True)
            path.write_text(yaml.safe_dump(config), encoding='utf-8')
            self.assertEqual(len(nodes(context, required_mode='replay')), 20)

    @unittest.skipUnless(LAUNCH_AVAILABLE, 'ROS launch libraries required')
    def test_required_child_exit_requests_graph_shutdown(self):
        event = SimpleNamespace(process_name='native_ndt', returncode=7)
        emitted = shutdown_on_required_process_exit(event, SimpleNamespace(is_shutdown=False))
        self.assertEqual(len(emitted), 1)
        self.assertIsInstance(emitted[0].event, Shutdown)
        self.assertIn('native_ndt', emitted[0].event.reason)
        self.assertEqual(shutdown_on_required_process_exit(event, SimpleNamespace(is_shutdown=True)), [])

    @unittest.skipUnless(LAUNCH_AVAILABLE, 'ROS launch libraries required')
    def test_failed_process_stops_its_running_peer(self):
        failed = ExecuteProcess(cmd=[sys.executable, '-c', 'import time,sys; time.sleep(.2); sys.exit(7)'])
        peer = ExecuteProcess(cmd=[sys.executable, '-c', 'import time; time.sleep(30)'])
        watchdog = RegisterEventHandler(OnProcessExit(target_action=failed,
                                                      on_exit=shutdown_on_required_process_exit))
        service = LaunchService(argv=[])
        service.include_launch_description(LaunchDescription([watchdog, peer, failed]))
        started = time.monotonic()
        service.run()
        self.assertLess(time.monotonic() - started, 5.)


if __name__ == '__main__':
    unittest.main()
