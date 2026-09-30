"""Adapt wheel/IMU inputs and supervise the unmodified upstream EKF.

Only the guarded public odometry is a supported output. Native EKF topics and TF
are isolated because the upstream filter predicts indefinitely on stale twist.
"""
import copy
import json
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from .validation import FreshnessGuard, diagonal_covariance


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class BagAdapter(Node):
    def __init__(self):
        super().__init__('aps_bag_adapter')
        defaults = {
            'wheel_topic': '/hunter_odom', 'imu_topic': '/livox/imu',
            'wheel_frame': 'hunter_base_link', 'wheel_axes_confirmed': False,
            'experimental_frame_assumptions': False,
            'local_frame': 'localization_odom',
            'localized_child_frame': 'localized_base_link',
            'sensor_timeout': 0.2, 'future_tolerance': 0.1,
            'wheel_variance_floor': 0.0025, 'angular_variance_floor': 0.0004,
            'publish_tf': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.guard = FreshnessGuard(self.p['sensor_timeout'], self.p['future_tolerance'])
        self.activated = False
        self.activation_pending = None
        self.deactivation_pending = None
        self.deactivated = False
        self.initial_sent = False
        self.last_output_stamp = None
        self.counts = {'wheel': 0, 'imu': 0, 'gyro': 0, 'output': 0, 'rejected': 0}
        self.last_status = None
        self.rejection_reason = ''

        # BEST_EFFORT subscriber accepts both recorded sensor QoS profiles. The
        # reliable publishers satisfy the upstream gyro's reliable subscribers.
        sensor_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.wheel_pub = self.create_publisher(
            TwistWithCovarianceStamped, '/localization/input/wheel_twist', 100)
        self.imu_pub = self.create_publisher(Imu, '/localization/input/imu', 100)
        self.odom_pub = self.create_publisher(Odometry, '/localization/kinematic_state', 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/localization/pose_with_covariance', 10)
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/localization/internal/initialpose', 1)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/localization/replay_status', status_qos)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(Odometry, self.p['wheel_topic'], self.on_wheel, sensor_qos)
        self.create_subscription(Imu, self.p['imu_topic'], self.on_imu, sensor_qos)
        self.create_subscription(TwistWithCovarianceStamped,
                                 '/localization/internal/gyro_twist', self.on_gyro, 10)
        self.create_subscription(Odometry, '/localization/internal/kinematic_state',
                                 self.on_estimate, 10)
        self.trigger = self.create_client(SetBool, '/localization/internal/trigger_node')
        self.tf_pub = TransformBroadcaster(self) if self.p['publish_tf'] else None
        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.create_timer(0.02, self.supervise, clock=self.wall_clock)
        self.create_timer(1.0, self.publish_status, clock=self.wall_clock)
        self.get_logger().info('Relative wheel/gyro mode: origin is zero at first valid activation; '
                               'no wheel pose, map pose or acceleration observations are used.')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def reject(self, reason):
        self.counts['rejected'] += 1
        self.rejection_reason = reason

    def on_wheel(self, msg):
        if self.guard.stop_reason:
            return
        if not (self.p['wheel_axes_confirmed'] or self.p['experimental_frame_assumptions']):
            self.reject('wheel_axes_unconfirmed_hunter_base_link_to_base_link')
            return
        if msg.child_frame_id != self.p['wheel_frame']:
            self.reject('unexpected_wheel_frame:' + msg.child_frame_id)
            return
        if not math.isfinite(msg.twist.twist.linear.x):
            self.reject('nonfinite_wheel_speed')
            return
        if not self.guard.observe('wheel', seconds(msg.header.stamp)):
            self.reject('invalid_wheel_timestamp')
            return
        output = TwistWithCovarianceStamped()
        output.header = copy.deepcopy(msg.header)
        # Only permitted after explicit confirmation of matching forward axes.
        output.header.frame_id = 'base_link'
        output.twist.twist.linear.x = msg.twist.twist.linear.x
        output.twist.covariance = diagonal_covariance(
            msg.twist.covariance, 6, self.p['wheel_variance_floor'])
        self.wheel_pub.publish(output)
        self.counts['wheel'] += 1

    def on_imu(self, msg):
        if self.guard.stop_reason:
            return
        values = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        if (not msg.header.frame_id or not all(math.isfinite(v) for v in values)
                or msg.angular_velocity_covariance[0] < 0):
            self.reject('missing_or_invalid_angular_velocity')
            return
        if not self.guard.observe('imu', seconds(msg.header.stamp)):
            self.reject('invalid_imu_timestamp')
            return
        output = Imu()
        output.header = copy.deepcopy(msg.header)
        output.angular_velocity = copy.deepcopy(msg.angular_velocity)
        output.angular_velocity_covariance = diagonal_covariance(
            msg.angular_velocity_covariance, 3, self.p['angular_variance_floor'])
        # The supplied bags have identity orientation and acceleration near 1 g,
        # without trustworthy units/covariance. Explicitly mark both unavailable.
        output.orientation.w = 1.0
        output.orientation_covariance[0] = -1.0
        output.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(output)
        self.counts['imu'] += 1

    def on_gyro(self, msg):
        if self.guard.stop_reason:
            return
        if msg.header.frame_id != 'base_link':
            self.reject('unexpected_gyro_frame')
            return
        if not all(math.isfinite(v) for v in (
                msg.twist.twist.linear.x, msg.twist.twist.angular.z)):
            self.reject('nonfinite_gyro_twist')
            return
        # Native gyro emits only after successfully transforming IMU to base_link.
        if self.guard.observe('gyro', seconds(msg.header.stamp)):
            self.counts['gyro'] += 1

    def supervise(self):
        reason = self.guard.problem(self.now_seconds())
        if self.guard.stop_reason:
            if not self.deactivated and self.deactivation_pending is None and self.trigger.service_is_ready():
                self.deactivation_pending = self.trigger.call_async(SetBool.Request(data=False))
                self.deactivation_pending.add_done_callback(self.on_deactivated)
            self.publish_status()
            return
        if reason or self.activated or self.activation_pending is not None:
            return
        if not self.trigger.service_is_ready() or self.initial_pub.get_subscription_count() == 0:
            return
        # Activation precedes initialpose: SetBool(false) clears initialpose state.
        self.activation_pending = self.trigger.call_async(SetBool.Request(data=True))
        self.activation_pending.add_done_callback(self.on_activated)

    def on_activated(self, future):
        self.activation_pending = None
        try:
            success = future.result().success
        except Exception as error:
            self.get_logger().error('EKF activation failed: ' + str(error))
            return
        if not success:
            self.get_logger().error('EKF activation service rejected the request')
            return
        if not self.guard.activate(self.now_seconds()):
            self.guard.stop_reason = 'inputs_stale_during_activation'
            return
        initial = PoseWithCovarianceStamped()
        initial.header.stamp = self.get_clock().now().to_msg()
        initial.header.frame_id = self.p['local_frame']
        initial.pose.pose.orientation.w = 1.0
        for index, variance in zip((0, 7, 14, 21, 28, 35), (0.01, 0.01, 0.01, 0.01, 0.01, 0.01)):
            initial.pose.covariance[index] = variance
        self.initial_pub.publish(initial)
        self.initial_sent = True
        self.activated = True
        self.publish_status()
        self.get_logger().info('EKF activated and initialized once in localization_odom.')

    def on_deactivated(self, future):
        self.deactivation_pending = None
        try:
            self.deactivated = future.result().success
        except Exception as error:
            self.get_logger().error('EKF deactivation failed: ' + str(error))
        self.publish_status()

    def on_estimate(self, msg):
        if not self.activated or self.guard.problem(self.now_seconds()):
            return
        stamp = seconds(msg.header.stamp)
        if self.last_output_stamp is not None and stamp <= self.last_output_stamp:
            return
        values = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                  msg.pose.pose.position.z, msg.pose.pose.orientation.x,
                  msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w, msg.twist.twist.linear.x,
                  msg.twist.twist.angular.z, *msg.pose.covariance)
        if (msg.header.frame_id != self.p['local_frame']
                or not all(math.isfinite(value) for value in values)):
            self.guard.stop_reason = 'invalid_ekf_estimate'
            return
        if abs(self.now_seconds() - stamp) > self.p['sensor_timeout']:
            return
        output = copy.deepcopy(msg)
        output.child_frame_id = self.p['localized_child_frame']
        self.odom_pub.publish(output)
        pose = PoseWithCovarianceStamped()
        pose.header = copy.deepcopy(output.header)
        pose.pose = copy.deepcopy(output.pose)
        self.pose_pub.publish(pose)
        if self.tf_pub:
            transform = TransformStamped()
            transform.header = copy.deepcopy(output.header)
            transform.child_frame_id = output.child_frame_id
            transform.transform.translation.x = output.pose.pose.position.x
            transform.transform.translation.y = output.pose.pose.position.y
            transform.transform.translation.z = output.pose.pose.position.z
            transform.transform.rotation = copy.deepcopy(output.pose.pose.orientation)
            self.tf_pub.sendTransform(transform)
        self.last_output_stamp = stamp
        self.counts['output'] += 1

    def publish_status(self):
        reason = self.guard.problem(self.now_seconds())
        state = 'STOPPED' if self.guard.stop_reason else ('ACTIVE' if self.activated else 'WAITING')
        payload = {
            'state': state, 'reason': reason, 'mode': 'relative_wheel_gyro',
            'map_localization': False, 'origin': 'zero_at_activation',
            'frame_id': self.p['local_frame'],
            'child_frame_id': self.p['localized_child_frame'],
            'experimental_frame_assumptions': self.p['experimental_frame_assumptions'],
            'wheel_axes_confirmed': self.p['wheel_axes_confirmed'],
            'covariance_floors_calibrated': False,
            'sensor_timeout': self.p['sensor_timeout'],
            'latest_sensor_stamps': self.guard.latest,
            'counts': self.counts, 'last_rejection': self.rejection_reason,
            'initial_pose_sent': self.initial_sent, 'ekf_deactivated': self.deactivated,
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        level = DiagnosticStatus.ERROR if state == 'STOPPED' else DiagnosticStatus.WARN
        diagnostic = DiagnosticStatus(level=level, name='aps_bag_localization:replay_guard',
                                      hardware_id='bag', message=state + ': ' + (reason or 'relative only'))
        diagnostic.values = [KeyValue(key='mode', value='relative_wheel_gyro'),
                             KeyValue(key='map_localization', value='false')]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diagnostic]
        self.diag_pub.publish(array)
        key = (state, reason)
        if key != self.last_status:
            self.get_logger().info('Replay guard ' + state + ': ' + reason)
            self.last_status = key


def main(args=None):
    rclpy.init(args=args)
    node = BagAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
