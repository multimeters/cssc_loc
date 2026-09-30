"""Prevent raw localization from silently replaying the old processed cloud."""
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
from run_fusion_replay import input_types, inspect_inputs
from aps_bag_localization.configuration import load_config


class RawReplayContractTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(ROOT / 'config' / 'localization.yaml')
        self.expected = input_types(self.config)

    def test_replay_excludes_old_cloud_and_localization_tf(self):
        self.assertEqual(self.expected, {
            '/hunter_odom': 'nav_msgs/msg/Odometry',
            '/livox/imu': 'sensor_msgs/msg/Imu',
            '/livox/lidar': 'livox_ros_driver2/msg/CustomMsg',
        })

    def bag(self, directory, raw_type=None):
        with sqlite3.connect(Path(directory) / 'test.db3') as db:
            db.execute('CREATE TABLE topics (id INTEGER, name TEXT, type TEXT)')
            db.execute('CREATE TABLE messages (topic_id INTEGER, data BLOB)')
            entries = list(self.expected.items())
            entries += [('/cloud_registered_body', 'sensor_msgs/msg/PointCloud2'),
                        ('/tf', 'tf2_msgs/msg/TFMessage')]
            for index, (name, type_name) in enumerate(entries):
                if name == '/livox/lidar':
                    if raw_type is None:
                        continue
                    type_name = raw_type
                db.execute('INSERT INTO topics VALUES (?, ?, ?)', (index, name, type_name))
                db.execute('INSERT INTO messages VALUES (?, ?)', (index, b''))

    def test_processed_cloud_cannot_satisfy_missing_raw_input(self):
        with tempfile.TemporaryDirectory() as folder:
            self.bag(folder)
            with self.assertRaisesRegex(ValueError, '/livox/lidar'):
                inspect_inputs(Path(folder), self.expected)

    def test_correct_raw_type_required_even_if_topic_name_matches(self):
        with tempfile.TemporaryDirectory() as folder:
            self.bag(folder, 'sensor_msgs/msg/PointCloud2')
            with self.assertRaisesRegex(ValueError, 'CustomMsg'):
                inspect_inputs(Path(folder), self.expected)

    def test_only_three_raw_sensors_counted(self):
        with tempfile.TemporaryDirectory() as folder:
            self.bag(folder, 'livox_ros_driver2/msg/CustomMsg')
            self.assertEqual(inspect_inputs(Path(folder), self.expected),
                             {name: 1 for name in self.expected})


if __name__ == '__main__':
    unittest.main()
