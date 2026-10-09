"""Exercise serialized native lifecycle calls without changing locked vendor code."""
import copy
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rclpy
import yaml
from sensor_msgs.msg import PointCloud2

from aps_bag_localization.configuration import load_config
from aps_bag_localization.fusion_adapter import FusionAdapter


MASTER = Path(__file__).resolve().parents[3] / 'config' / 'localization.yaml'


class Future:
    def __init__(self, ready=True, result=None):
        self.ready = ready
        self._result = result

    def done(self):
        return self.ready

    def result(self):
        return self._result or SimpleNamespace(success=True)


class Client:
    def __init__(self, name, calls):
        self.name, self.calls = name, calls
        self.ready = True

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.calls.append((self.name, request.data))
        return Future(self.ready)


class Publisher:
    def __init__(self):
        self.messages = []

    def get_subscription_count(self):
        return 1

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


class LiveNdtCacheGuardTests(unittest.TestCase):
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

    def ready_node(self):
        node = FusionAdapter(str(self.config_path))
        calls = []
        node.activation_clients = {name: Client(name, calls) for name in ('ndt', 'ekf')}
        class AlignClient:
            def service_is_ready(self):
                return True

            def call_async(self, request):
                aligned = copy.deepcopy(request.pose_with_covariance)
                aligned.pose.pose.position.x += .1
                calls.append(('ndt_align', request.pose_with_covariance.header.frame_id))
                return Future(result=SimpleNamespace(
                    success=True, reliable=True, pose_with_covariance=aligned))

        node.ndt_align_client = AlignClient()
        node.activation = {'ndt': True, 'ekf': True}
        node.initial_sent = node.initial_acknowledged = node.post_initial_ndt = True
        node.initial_epoch_ns = node.now_ns()-10**9
        node.ndt_epoch_ns = node.initial_epoch_ns
        node.ndt_active_since_monotonic = time.monotonic()-node.p['ndt_idle_timeout_s']-1.
        node.initial_pub = Publisher()
        return node, calls

    def test_idle_suspends_native_prior_collection_and_fresh_cloud_wakes_with_new_epoch(self):
        node, calls = self.ready_node()
        try:
            node.tick()
            self.assertEqual(calls, [('ndt', False)])
            node.tick()
            self.assertFalse(node.activation['ndt'])
            self.assertEqual(node.counts['ndt_idle_sleeps'], 1)
            for _ in range(5):
                node.tick()
            self.assertEqual(calls, [('ndt', False)])
            node.last_valid_cloud_ns = node.now_ns()
            node.tick()
            self.assertEqual(calls[-1], ('ndt', True))
            node.ekf_first_ns, node.ekf_last_ns = 1, 2
            node.tick()
            self.assertTrue(node.activation['ndt'])
            self.assertIsNone(node.ekf_first_ns)
            self.assertIsNone(node.ekf_last_ns)
            self.assertEqual(node.counts['ndt_wakeups'], 1)
            prior = node.configured_initial_pose()
            prior.header.stamp.sec, prior.header.stamp.nanosec = divmod(node.ndt_epoch_ns-1, 10**9)
            node.on_ekf_prior(prior)
            self.assertIsNone(node.ekf_first_ns)
            prior.header.stamp.sec, prior.header.stamp.nanosec = divmod(node.ndt_epoch_ns+1, 10**9)
            node.on_ekf_prior(prior)
            self.assertEqual(node.ekf_first_ns, node.ndt_epoch_ns+1)
        finally:
            node.destroy_node()

    def test_continuously_bad_scans_still_reset_periodically_and_good_results_reset_deadline(self):
        node, calls = self.ready_node()
        try:
            node.last_valid_cloud_ns = node.now_ns()
            node.tick()
            node.tick()
            node.tick()
            self.assertEqual(calls, [('ndt', False), ('ndt', True)])
            self.assertTrue(node.activation['ndt'])
            node.ndt_active_since_monotonic -= node.p['ndt_idle_timeout_s']+1
            node.last_valid_cloud_ns = node.now_ns()
            node.tick()
            node.tick()
            node.tick()
            self.assertEqual(calls, [('ndt', False), ('ndt', True)]*2)
            observation = node.configured_initial_pose()
            ns = max(node.now_ns(), node.ndt_epoch_ns)
            observation.header.stamp.sec, observation.header.stamp.nanosec = divmod(ns, 10**9)
            node.on_ndt(observation)
            self.assertIsNotNone(node.ndt_last_result_monotonic)
            node.ndt_active_since_monotonic -= 100.
            node.tick()
            self.assertEqual(len(calls), 4)
        finally:
            node.destroy_node()

    def test_reinitialization_waits_for_in_flight_idle_service(self):
        node, calls = self.ready_node()
        try:
            node.activation_clients['ndt'].ready = False
            node.tick()
            pending = node.initialization_future[2]
            node.on_initial_pose(node.configured_initial_pose())
            for _ in range(3):
                node.tick()
            self.assertEqual(calls, [('ndt', False)])
            self.assertFalse(node.initial_sent)
            pending.ready = True
            node.activation_clients['ndt'].ready = True
            node.cloud_pub = Publisher()
            alignment_cloud = PointCloud2()
            alignment_cloud.header.stamp.sec, alignment_cloud.header.stamp.nanosec = divmod(node.now_ns(), 10**9)
            node.pending.append((alignment_cloud, None))
            for _ in range(8):
                node.tick()
            self.assertEqual(calls, [('ndt', False), ('ekf', False), ('ndt', False),
                                     ('ndt_align', 'map'), ('ekf', True), ('ndt', True)])
            self.assertTrue(node.initial_sent)
            self.assertEqual(node.initial_publications, 1)
        finally:
            node.destroy_node()

    def test_clock_fault_waits_for_activation_then_disables_ndt(self):
        node, calls = self.ready_node()
        try:
            node.activation['ndt'] = False
            node.activation_clients['ndt'].ready = False
            node.request_native_transition('ndt', True, 'initialization')
            pending = node.initialization_future[2]
            node.clock_error = True
            node.tick()
            self.assertEqual(calls, [('ndt', True)])
            pending.ready = True
            node.activation_clients['ndt'].ready = True
            node.tick()
            self.assertEqual(calls, [('ndt', True), ('ndt', False)])
            node.tick()
            self.assertFalse(node.activation['ndt'])
            self.assertEqual(node.current_mode(), ('CLOCK_REVERSED_RESTART_REQUIRED', False))
            node.last_valid_cloud_ns = node.now_ns()
            node.tick()
            self.assertEqual(len(calls), 2)
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
