"""Execute sensor callbacks with ROS messages, without starting replay or native nodes."""
from types import SimpleNamespace
from pathlib import Path
import unittest

try:
    from aps_bag_localization.fusion_adapter import FusionAdapter
    from sensor_msgs.msg import Imu
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@unittest.skipUnless(ROS_AVAILABLE, 'ROS Humble Python messages required')
class SensorContractTests(unittest.TestCase):
    def test_real_ros_node_constructor_and_status(self):
        import rclpy
        rclpy.init(domain_id=68)
        node = None
        try:
            node = FusionAdapter(config_path=Path(__file__).resolve().parents[3] / 'config' / 'localization.yaml')
            node.publish_status()
            node.tick()
            self.assertEqual(set(node.activation_clients), {'ekf', 'ndt'})
            self.assertFalse(node.initial_sent)
            input_topics = {subscription.topic_name for subscription in node.subscriptions}
            self.assertIn(node.topic_names['processed_points'], input_topics)
            self.assertNotIn(node.topic_names['points'], input_topics)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()

    def adapter(self):
        # Bind only callback helpers; no ROS node, services, replay or map is run.
        adapter = SimpleNamespace(
            p={'wheel_frame': 'hunter_base_link', 'wheel_variance_floor': .0025,
               'imu_frame': 'livox_frame', 'angular_variance_floor': .0004,
               'imu_to_base_quaternion': (0., 0., 0., 1.), 'base_frame': 'base_link'},
            counts={'wheel': 0, 'imu': 0, 'rejected': 0}, latest={}, last_input_ns={},
            wheel_pub=Publisher(), imu_pub=Publisher(),
        )
        adapter.reject = lambda reason: FusionAdapter.reject(adapter, reason)
        adapter.observe = lambda stream, stamp: FusionAdapter.observe(adapter, stream, stamp)
        return adapter

    def test_wheel_pose_and_yaw_rate_are_never_used(self):
        from std_msgs.msg import Header
        from geometry_msgs.msg import TwistWithCovariance
        from builtin_interfaces.msg import Time

        class WheelMessage:
            header = Header(stamp=Time(sec=10, nanosec=123), frame_id='hunter_odom')
            child_frame_id = 'hunter_base_link'
            twist = TwistWithCovariance()

            @property
            def pose(self):
                raise AssertionError('Wheel pose must not be read or injected into EKF')

        message = WheelMessage()
        message.twist.twist.linear.x = .7
        message.twist.twist.angular.z = 999.
        adapter = self.adapter()
        FusionAdapter.on_wheel(adapter, message)
        output = adapter.wheel_pub.messages[0]
        self.assertEqual(output.header.stamp, message.header.stamp)
        self.assertEqual(output.header.frame_id, 'base_link')
        self.assertEqual(output.twist.twist.linear.x, .7)
        self.assertEqual(output.twist.twist.angular.z, 0.)
        self.assertEqual(output.twist.covariance[0], .0025)

    def test_imu_only_angular_velocity_and_stamp_survive(self):
        message = Imu()
        message.header.frame_id = 'livox_frame'
        message.header.stamp.sec = 10
        message.header.stamp.nanosec = 456
        message.angular_velocity.z = .2
        message.linear_acceleration.x = -0.43
        message.orientation.w = 1.
        adapter = self.adapter()
        FusionAdapter.on_imu(adapter, message)
        output = adapter.imu_pub.messages[0]
        self.assertEqual(output.header.stamp, message.header.stamp)
        self.assertEqual(output.header.frame_id, 'base_link')
        self.assertEqual(message.header.frame_id, 'livox_frame')
        self.assertEqual(output.angular_velocity, message.angular_velocity)
        self.assertEqual(output.angular_velocity_covariance[8], .0004)
        self.assertEqual(output.linear_acceleration_covariance[0], -1.)
        self.assertEqual(output.orientation_covariance[0], -1.)
        self.assertEqual(output.linear_acceleration.x, 0.)


if __name__ == '__main__':
    unittest.main()
