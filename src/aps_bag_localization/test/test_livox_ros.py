"""ROS wire-format/adapter tests; the recorded-data check skips without its bag."""
from pathlib import Path
import sqlite3
import sys
import unittest

# Exercise source edits even when an older package exists in the sourced overlay.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time
from sensor_msgs.msg import PointField

from aps_bag_localization.configuration import load_config
from aps_bag_localization.livox_deskew import parse_scan
from aps_bag_localization.livox_node import RawLivoxNode, make_cloud


MASTER = Path(__file__).resolve().parents[3] / 'config' / 'localization.yaml'


class Collector:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def synthetic_message():
    message = CustomMsg()
    message.timebase = 1_790_000_000_000_000_000
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(message.timebase, 10**9)
    message.header.frame_id = 'livox_frame'
    message.points = [CustomPoint(x=2., y=1., z=.5, offset_time=0, reflectivity=200),
                      CustomPoint(x=3., y=1., z=.5, offset_time=100_000_000, reflectivity=255)]
    message.point_num = 2
    return message


class LivoxRosTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.config = load_config(MASTER)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_generated_custommsg_roundtrip_and_output_fields(self):
        original = synthetic_message()
        decoded = deserialize_message(serialize_message(original), CustomMsg)
        scan = parse_scan(decoded, self.config['livox'])
        cloud = make_cloud(scan, scan.xyz, self.config['frames']['cloud'])
        self.assertEqual(cloud.header.frame_id, 'body')
        self.assertEqual(cloud.header.stamp.sec*10**9+cloud.header.stamp.nanosec,
                         original.timebase+100_000_000)
        self.assertEqual([(f.name, f.offset, f.datatype, f.count) for f in cloud.fields],
                         [(name, i*4, PointField.FLOAT32, 1) for i, name in enumerate(('x', 'y', 'z', 'intensity'))])
        np.testing.assert_array_equal(np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 4),
                                      [[2., 1., .5, 200.], [3., 1., .5, 255.]])

    def test_real_node_holds_until_scan_end_and_reports_whole_scan_fallback(self):
        node = RawLivoxNode(str(MASTER))
        try:
            node.publisher, node.status_publisher = Collector(), Collector()
            raw = synthetic_message()
            node.on_cloud(raw)
            end = raw.timebase+100_000_000
            node.get_clock().set_ros_time_override(Time(nanoseconds=end-1))
            node.tick()
            self.assertEqual(node.counts['cloud_published'], 0)
            node.get_clock().set_ros_time_override(Time(nanoseconds=end+round(node.settings['wait_timeout_s']*1e9)))
            node.tick()
            self.assertEqual(node.counts['fully_deskewed'], 0)
            self.assertEqual(node.counts['uncompensated'], 1)
            self.assertEqual(node.counts['cloud_published'], 1)
            np.testing.assert_array_equal(np.frombuffer(node.publisher.messages[0].data, dtype='<f4').reshape(-1, 4),
                                          [[2., 1., .5, 200.], [3., 1., .5, 255.]])
            self.assertIn('imu', node.last_scan['fallback_reason'])
            raw.header.frame_id = 'mid360_link'
            node.on_cloud(raw)
            self.assertEqual(node.counts['cloud_rejected'], 1)
            self.assertIn('unexpected raw lidar frame', node.last_rejection)
        finally:
            node.destroy_node()

    def test_first_recorded_custommsg_preserves_point_values_and_nanosecond_times(self):
        bag = Path(self.config['paths']['bag'])
        if not bag.exists():
            self.skipTest('local recorded bag is not included in the source repository')
        matches = sorted(bag.glob('*.db3'))
        self.assertTrue(matches)
        with sqlite3.connect(matches[0].as_uri()+'?mode=ro', uri=True) as database:
            topic = database.execute('SELECT id, type FROM topics WHERE name=?',
                                      (self.config['topics']['points'],)).fetchone()
            self.assertEqual(topic[1], 'livox_ros_driver2/msg/CustomMsg')
            payload = database.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1',
                                        (topic[0],)).fetchone()[0]
        raw = deserialize_message(payload, CustomMsg)
        self.assertGreater(len(raw.points), 10_000)
        self.assertEqual(raw.point_num, len(raw.points))
        self.assertEqual(raw.timebase, raw.header.stamp.sec*10**9+raw.header.stamp.nanosec)
        scan = parse_scan(raw, self.config['livox'])
        self.assertEqual(scan.end_ns, raw.timebase+max(p.offset_time for p in raw.points))
        self.assertEqual(scan.start_ns, raw.timebase+min(p.offset_time for p in raw.points))
        expected = []
        for point in raw.points:
            if (all(((point.tag >> shift) & 3) < 2 for shift in (0, 2, 4))
                    and self.config['livox']['min_range_m']**2 <= point.x**2+point.y**2+point.z**2
                    <= self.config['livox']['max_range_m']**2):
                expected.append((point.x, point.y, point.z, float(point.reflectivity), raw.timebase+point.offset_time))
        self.assertEqual(len(scan.xyz), len(expected))
        np.testing.assert_array_equal(scan.xyz, np.asarray([p[:3] for p in expected]))
        np.testing.assert_array_equal(scan.intensity, [p[3] for p in expected])
        np.testing.assert_array_equal(scan.times_ns, np.array([p[4] for p in expected], dtype=np.int64))


if __name__ == '__main__':
    unittest.main()
