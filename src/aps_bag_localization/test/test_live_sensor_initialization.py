"""Live-time input, manual initialization and dropout checks with real ROS messages."""
import copy
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import yaml

from aps_bag_localization.configuration import load_config
from aps_bag_localization.fusion_adapter import FusionAdapter, stamp_ns
from aps_bag_localization.livox_node import RawLivoxNode


MASTER = Path(__file__).resolve().parents[3] / 'config' / 'localization.yaml'


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))

    def get_subscription_count(self):
        return 1


def stamp(message, ns):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(int(ns), 10**9)


class LiveSensorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=68)
        cls.temporary = tempfile.TemporaryDirectory()
        config = load_config(MASTER, mode='live')
        config['live']['initialization'] = 'topic'
        cls.config_path = Path(cls.temporary.name) / 'live.yaml'
        cls.config_path.write_text(yaml.safe_dump(config), encoding='utf-8')

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()
        rclpy.shutdown()

    def test_wheel_and_imu_reject_stale_future_and_recover_fresh(self):
        node = FusionAdapter(str(self.config_path))
        try:
            node.wheel_pub, node.imu_pub = Publisher(), Publisher()
            now = node.now_ns()
            wheel = Odometry()
            wheel.child_frame_id = node.p['wheel_frame']
            wheel.twist.twist.linear.x = .4
            stamp(wheel, now)
            node.on_wheel(wheel)
            self.assertEqual(len(node.wheel_pub.messages), 0)
            self.assertEqual(node.current_mode(), ('WAITING_INITIAL_POSE', False))
            node.initial_sent = True
            node.initial_acknowledged = True
            for ns, expected in ((now-2*10**9, 'stale_wheel'), (now+2*10**9, 'future_wheel')):
                stamp(wheel, ns)
                node.on_wheel(wheel)
                self.assertIn(expected, node.last_rejection)
            stamp(wheel, node.now_ns())
            node.on_wheel(wheel)
            self.assertEqual(len(node.wheel_pub.messages), 1)
            imu = Imu()
            imu.header.frame_id = node.p['imu_frame']
            imu.angular_velocity.z = .2
            stamp(imu, now-2*10**9)
            node.on_imu(imu)
            stamp(imu, now+2*10**9)
            node.on_imu(imu)
            stamp(imu, node.now_ns())
            node.on_imu(imu)
            self.assertEqual(len(node.imu_pub.messages), 1)
            self.assertEqual(node.current_mode(), ('WAITING_SENSORS', False))
        finally:
            node.destroy_node()

    def test_raw_live_rejects_historical_future_and_accepts_fresh_without_restamping(self):
        node = RawLivoxNode(str(self.config_path))
        try:
            raw = CustomMsg()
            raw.header.frame_id = node.settings['raw_frame']
            raw.points = [CustomPoint(x=2., y=0., z=.3, offset_time=0),
                          CustomPoint(x=2., y=0., z=.3, offset_time=99_000_000)]
            raw.point_num = 2
            now = node.current_time_ns()
            for ns in (now-2*10**9, now+2*10**9):
                raw.timebase = ns
                stamp(raw, ns)
                node.on_cloud(raw)
            self.assertEqual(node.counts['cloud_rejected'], 2)
            self.assertEqual(len(node.pending), 0)
            raw.timebase = node.current_time_ns()-100_000_000
            stamp(raw, raw.timebase)
            node.on_cloud(raw)
            self.assertEqual(len(node.pending), 1)
            self.assertEqual(node.pending[0].start_ns, raw.timebase)
            self.assertEqual(node.pending[0].end_ns, raw.timebase+99_000_000)
        finally:
            node.destroy_node()

    def test_manual_initial_pose_cycles_native_nodes_and_reinitialization_blocks_old_output(self):
        node = FusionAdapter(str(self.config_path))
        try:
            calls = []

            class Client:
                def __init__(self, name):
                    self.name = name

                def service_is_ready(self):
                    return True

                def call_async(self, request):
                    calls.append((self.name, request.data))
                    return SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=True))

            node.activation_clients = {name: Client(name) for name in ('ndt', 'ekf')}
            class AlignClient:
                def service_is_ready(self):
                    return True

                def call_async(self, request):
                    aligned = copy.deepcopy(request.pose_with_covariance)
                    aligned.pose.pose.position.x += .4
                    calls.append(('ndt_align', request.pose_with_covariance.header.frame_id))
                    return SimpleNamespace(
                        done=lambda: True,
                        result=lambda: SimpleNamespace(
                            success=True, reliable=True, pose_with_covariance=aligned))

            node.ndt_align_client = AlignClient()
            node.initial_pub, node.output_pub, node.public_pose_pub = Publisher(), Publisher(), Publisher()
            node.cloud_pub = Publisher()
            node.tf_pub = None
            node.tick()
            self.assertEqual(calls, [])
            pose = node.configured_initial_pose()
            pose.header.stamp.sec = 1  # RViz input is deliberately restamped only at dispatch.
            old_cov = list(pose.pose.covariance)
            node.on_initial_pose(pose)
            alignment_cloud = PointCloud2()
            stamp(alignment_cloud, node.now_ns())
            node.pending.append((alignment_cloud, None))
            for _ in range(10):
                node.tick()
            self.assertEqual(
                calls,
                [('ndt', False), ('ekf', False), ('ndt_align', 'map'), ('ndt', True), ('ekf', True)])
            self.assertEqual(node.initial_publications, 1)
            self.assertAlmostEqual(node.initial_pub.messages[0].pose.pose.position.x, pose.pose.pose.position.x + .4)
            self.assertEqual(list(node.initial_pub.messages[0].pose.covariance), old_cov)
            self.assertGreater(stamp_ns(node.initial_pub.messages[0].header.stamp), 10**9)
            self.assertEqual(node.current_mode(), ('WAITING_SENSORS', False))
            prior = copy.deepcopy(node.initial_pub.messages[0])
            stamp(prior, node.initial_epoch_ns+1)
            prior.pose.pose.position.x += 2.
            node.on_ekf_prior(prior)
            self.assertFalse(node.initial_acknowledged)
            prior.pose.pose.position.x -= 2.
            node.on_ekf_prior(prior)
            self.assertTrue(node.initial_acknowledged)
            ndt = copy.deepcopy(prior)
            stamp(ndt, node.initial_epoch_ns+2)
            node.on_ndt(ndt)
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = node.p['map_frame'], node.p['base_frame']
            odom.pose = copy.deepcopy(pose.pose)
            stamp(odom, node.initial_epoch_ns-1)
            node.on_ekf_odom(odom)
            self.assertEqual(len(node.output_pub.messages), 0)
            stamp(odom, node.initial_epoch_ns+3)
            node.on_ekf_odom(odom)
            self.assertEqual(len(node.output_pub.messages), 1)
            node.pending.append(('old_scan_marker', None))
            node.on_initial_pose(pose)
            self.assertFalse(node.initial_sent)
            self.assertFalse(node.post_initial_ndt)
            self.assertEqual(len(node.pending), 0)
            self.assertIsNone(node.ekf_last_ns)
            stamp(odom, node.now_ns())
            node.on_ekf_odom(odom)
            self.assertEqual(len(node.output_pub.messages), 1)
        finally:
            node.destroy_node()

    def test_live_motion_wait_falls_back_earlier_without_relaxing_motion_gaps_or_restamping(self):
        node = RawLivoxNode(str(self.config_path))
        try:
            node.publisher = Publisher()
            self.assertEqual(node.motion_wait_timeout_s, .1)
            self.assertEqual(node.settings['wait_timeout_s'], .3)
            self.assertEqual(node.settings['max_imu_gap_s'], .05)
            raw = CustomMsg()
            raw.header.frame_id = node.settings['raw_frame']
            raw.points = [CustomPoint(x=2., y=0., z=.3, offset_time=0),
                          CustomPoint(x=2., y=0., z=.3, offset_time=100_000_000)]
            raw.point_num = 2
            now = node.current_time_ns()
            # Scan ended .15 seconds ago: new live budget elapsed, replay .3 did not.
            raw.timebase = now-250_000_000
            stamp(raw, raw.timebase)
            node.on_cloud(raw)
            node.tick()
            self.assertEqual(node.counts['fully_deskewed'], 0)
            self.assertEqual(node.counts['uncompensated'], 1)
            self.assertEqual(stamp_ns(node.publisher.messages[0].header.stamp), raw.timebase+100_000_000)
            self.assertIn('imu', node.last_scan['fallback_reason'])
        finally:
            node.destroy_node()

    def test_invalid_map_quaternion_and_covariance_initial_pose_are_rejected(self):
        node = FusionAdapter(str(self.config_path))
        try:
            for mutate in (lambda p: setattr(p.header, 'frame_id', 'odom'),
                           lambda p: setattr(p.pose.pose.orientation, 'w', 2.),
                           lambda p: p.pose.covariance.__setitem__(0, -1.)):
                pose = node.configured_initial_pose()
                mutate(pose)
                node.on_initial_pose(pose)
                self.assertIsNone(node.pending_initial)
            self.assertEqual(node.counts['rejected'], 3)
        finally:
            node.destroy_node()

    def test_stream_dropout_recovers_but_backwards_clock_latches(self):
        node = FusionAdapter(str(self.config_path))
        try:
            now = node.now_ns()
            node.initial_sent = node.post_initial_ndt = node.initial_acknowledged = True
            node.activation = {'ndt': True, 'ekf': True}
            node.initial_epoch_ns = now-3*10**9
            node.latest = {'gyro': now*1e-9-10., 'ndt': now*1e-9-10.}
            self.assertEqual(node.current_mode(), ('STALE', False))
            ndt = node.configured_initial_pose()
            stamp(ndt, node.now_ns())
            node.on_ndt(ndt)
            self.assertEqual(node.current_mode(), ('NDT_ONLY', True))
            node.clock_last_ns = node.now_ns()+10**9
            node.tick()
            self.assertEqual(node.current_mode(), ('CLOCK_REVERSED_RESTART_REQUIRED', False))
            stamp(ndt, node.now_ns())
            node.on_ndt(ndt)
            self.assertTrue(node.clock_error)
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
