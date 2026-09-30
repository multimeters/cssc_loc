"""Sensor-only adapter and initialization for native Autoware NDT/EKF fusion.

This node never publishes continuous pose measurements or NDT priors. Those
connections run directly between native NDT and native EKF nodes in launch.
"""
from collections import deque
import copy
import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from aps_ndt_localization.cloud import filter_cloud
from .configuration import adapter_parameters, load_config
from .geometry import angular_velocity_to_base, fusion_mode
from .validation import diagonal_covariance


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class FusionAdapter(Node):
    def __init__(self, config_path=None):
        super().__init__('aps_fusion_adapter')
        self.declare_parameter('configuration_file', '')
        config_path = config_path or self.get_parameter('configuration_file').value
        if not config_path:
            raise ValueError('configuration_file is required; fusion has no hidden numeric defaults')
        config = load_config(config_path)
        self.config_path = config['_config_file']
        self.p = adapter_parameters(config)
        self.topic_names = config['topics']
        self.service_names = config['services']
        self.set_parameters([Parameter('use_sim_time', value=config['runtime']['use_sim_time'])])
        self.latest = {}
        self.counts = {key: 0 for key in ('wheel', 'imu', 'gyro', 'ndt', 'ekf', 'output',
                                         'cloud_received', 'cloud_forwarded', 'cloud_dropped', 'rejected')}
        self.last_rejection = ''
        self.last_input_ns = {}
        self.ekf_first_ns = None
        self.ekf_last_ns = None
        self.last_output_ns = None
        self.pending = deque()
        self.activation = {'ekf': False, 'ndt': False}
        self.futures = {}
        self.initial_sent = False
        self.clock_last_ns = None
        self.clock_error = False
        self.last_status_mode = None
        sensor_depth = self.p['sensor_queue_depth']
        output_depth = self.p['output_queue_depth']
        sensor_qos = QoSProfile(depth=sensor_depth, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.wheel_pub = self.create_publisher(TwistWithCovarianceStamped,
                                               self.topic_names['wheel_twist'], sensor_depth)
        self.imu_pub = self.create_publisher(Imu, self.topic_names['imu_base'], sensor_depth)
        self.cloud_pub = self.create_publisher(PointCloud2, self.topic_names['ndt_points'], self.p['cloud_queue_depth'])
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                 self.topic_names['initial_pose'], self.p['initial_pose_queue_depth'])
        self.output_pub = self.create_publisher(Odometry, self.topic_names['odometry'], output_depth)
        self.public_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                     self.topic_names['pose'], output_depth)
        self.status_pub = self.create_publisher(
            String, self.topic_names['fusion_status'],
            QoSProfile(depth=self.p['status_queue_depth'], durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, self.topic_names['diagnostics'], self.p['cloud_queue_depth'])
        self.create_subscription(Odometry, self.p['wheel_topic'], self.on_wheel, sensor_qos)
        self.create_subscription(Imu, self.p['imu_topic'], self.on_imu, sensor_qos)
        self.create_subscription(PointCloud2, self.p['points_topic'], self.on_cloud, sensor_qos)
        self.create_subscription(TwistWithCovarianceStamped,
                                 self.topic_names['gyro_twist'], self.on_gyro, sensor_depth)
        self.create_subscription(PoseWithCovarianceStamped,
                                 self.topic_names['ndt_pose'], self.on_ndt, output_depth)
        self.create_subscription(PoseWithCovarianceStamped,
                                 self.topic_names['ekf_prediction'], self.on_ekf_prior, output_depth)
        self.create_subscription(Odometry, self.topic_names['ekf_odometry'], self.on_ekf_odom, output_depth)
        self.activation_clients = {
            'ekf': self.create_client(SetBool, self.service_names['ekf_activation']),
            'ndt': self.create_client(SetBool, self.service_names['ndt_activation']),
        }
        self.tf_pub = TransformBroadcaster(self) if self.p['publish_tf'] else None
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(self.p['tick_period'], self.tick, clock=self.wall_clock)
        self.create_timer(self.p['status_period'], self.publish_status, clock=self.wall_clock)
        self.get_logger().info('Native gyro -> EKF; native NDT pose -> EKF; EKF prior -> NDT. '
                               'Cloud input is raw Livox preprocessed at scan end; '
                               'deskew coverage is reported separately by the preprocessor. '
                               'Outputs represent the rear wheel center. Mounting pitch is provisional.')

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def reject(self, reason):
        self.counts['rejected'] += 1
        self.last_rejection = reason

    def observe(self, stream, stamp):
        ns = stamp_ns(stamp)
        if ns <= 0 or ns <= self.last_input_ns.get(stream, -1):
            self.reject('non_increasing_' + stream + '_stamp')
            return False
        self.last_input_ns[stream] = ns
        self.latest[stream] = ns * 1e-9
        return True

    def on_wheel(self, msg):
        # msg.pose is intentionally never read: only rear-center forward speed
        # and its reported variance enter the upstream gyro measurement.
        if msg.child_frame_id != self.p['wheel_frame'] or not math.isfinite(msg.twist.twist.linear.x):
            self.reject('invalid_wheel_frame_or_velocity')
            return
        if not self.observe('wheel', msg.header.stamp):
            return
        output = TwistWithCovarianceStamped()
        output.header = copy.deepcopy(msg.header)
        output.header.frame_id = self.p['base_frame']
        output.twist.twist.linear.x = msg.twist.twist.linear.x
        output.twist.covariance = diagonal_covariance(msg.twist.covariance, 6, self.p['wheel_variance_floor'])
        self.wheel_pub.publish(output)
        self.counts['wheel'] += 1

    def on_imu(self, msg):
        rates = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        if (msg.header.frame_id != self.p['imu_frame']
                or msg.angular_velocity_covariance[0] < 0
                or not all(math.isfinite(value) for value in rates)):
            self.reject('invalid_imu_frame_or_angular_velocity')
            return
        if not self.observe('imu', msg.header.stamp):
            return
        output = Imu()
        output.header = copy.deepcopy(msg.header)
        # The locked upstream gyro requests inverse TF (IMU<-base). Feed it an
        # explicitly transformed base-frame IMU so its native transform is identity.
        # This changes only the derived message, never the recorded raw IMU or TF.
        try:
            base_rates, base_covariance = angular_velocity_to_base(
                rates, msg.angular_velocity_covariance, self.p['imu_to_base_quaternion'],
                self.p['angular_variance_floor'])
        except ValueError as error:
            self.reject('imu_rotation_error:' + str(error))
            return
        output.header.frame_id = self.p['base_frame']
        output.angular_velocity.x, output.angular_velocity.y, output.angular_velocity.z = base_rates
        output.angular_velocity_covariance = base_covariance
        output.orientation.w = 1.
        output.orientation_covariance[0] = -1.
        output.linear_acceleration_covariance[0] = -1.
        self.imu_pub.publish(output)
        self.counts['imu'] += 1

    def on_gyro(self, msg):
        if (msg.header.frame_id == self.p['base_frame'] and
                all(math.isfinite(v) for v in (msg.twist.twist.linear.x, msg.twist.twist.angular.z))):
            if self.observe('gyro', msg.header.stamp):
                self.counts['gyro'] += 1

    def on_ndt(self, msg):
        if msg.header.frame_id == self.p['map_frame'] and self.observe('ndt', msg.header.stamp):
            self.counts['ndt'] += 1

    def on_ekf_prior(self, msg):
        # Observation only. This adapter has no continuous prior publisher.
        if msg.header.frame_id != self.p['map_frame']:
            return
        ns = stamp_ns(msg.header.stamp)
        if self.ekf_last_ns is not None and ns <= self.ekf_last_ns:
            return
        if self.ekf_first_ns is None:
            self.ekf_first_ns = ns
        self.ekf_last_ns = ns

    def on_cloud(self, msg):
        self.counts['cloud_received'] += 1
        if msg.header.frame_id != self.p['cloud_frame'] or not self.observe('cloud', msg.header.stamp):
            self.counts['cloud_dropped'] += 1
            return
        try:
            cloud = filter_cloud(msg, self.p['voxel_size'])
        except (ValueError, TypeError, BufferError) as error:
            self.reject('invalid_cloud:' + str(error))
            self.counts['cloud_dropped'] += 1
            return
        if len(self.pending) >= self.p['max_pending_scans']:
            self.pending.popleft()
            self.counts['cloud_dropped'] += 1
        self.pending.append((cloud, None))

    def tick(self):
        now_ns = self.now_ns()
        if self.clock_last_ns is not None and now_ns < self.clock_last_ns:
            self.clock_error = True
        self.clock_last_ns = now_ns
        if now_ns <= 0 or self.clock_error:
            return
        for name, client in self.activation_clients.items():
            if name in self.futures:
                future = self.futures[name]
                if future.done():
                    try:
                        self.activation[name] = bool(future.result().success)
                    except Exception as error:
                        self.get_logger().error(name + ' activation failed: ' + str(error))
                    del self.futures[name]
            elif not self.activation[name] and client.service_is_ready():
                self.futures[name] = client.call_async(SetBool.Request(data=True))
        if (all(self.activation.values()) and not self.initial_sent
                and self.initial_pub.get_subscription_count() > 0):
            initial = PoseWithCovarianceStamped()
            initial.header.frame_id = self.p['map_frame']
            initial.header.stamp = self.get_clock().now().to_msg()
            xyz, quat = self.p['initial_base_xyz'], self.p['initial_base_quaternion']
            initial.pose.pose.position.x, initial.pose.pose.position.y, initial.pose.pose.position.z = xyz
            (initial.pose.pose.orientation.x, initial.pose.pose.orientation.y,
             initial.pose.pose.orientation.z, initial.pose.pose.orientation.w) = quat
            for index, variance in zip((0, 7, 14, 21, 28, 35), self.p['initial_covariance_diagonal']):
                initial.pose.covariance[index] = variance
            self.initial_pub.publish(initial)
            self.initial_sent = True
            self.get_logger().info('Initialized EKF once with map->rear-center pose converted from map->body ICP.')
        # Wait for real EKF samples to cover the original acquisition timestamp.
        # No interpolation, extrapolation or NDT self-feedback happens here.
        while self.pending:
            cloud, ready_at = self.pending[0]
            ns = stamp_ns(cloud.header.stamp)
            if ((now_ns - ns) * 1e-9 > self.p['scan_wait_timeout']
                    or (self.ekf_first_ns is not None and ns < self.ekf_first_ns)):
                self.pending.popleft()
                self.counts['cloud_dropped'] += 1
                continue
            if (self.ekf_first_ns is None or self.ekf_last_ns == self.ekf_first_ns
                    or ns > self.ekf_last_ns or not self.activation['ndt']):
                break
            if ready_at is None:
                self.pending[0] = (cloud, time.monotonic() + self.p['scan_relay_delay'])
                break
            if time.monotonic() < ready_at:
                break
            self.pending.popleft()
            self.cloud_pub.publish(cloud)
            self.counts['cloud_forwarded'] += 1

    def current_mode(self):
        if self.clock_error:
            return 'CLOCK_REVERSED_RESTART_REQUIRED', False
        if not self.initial_sent:
            return 'INITIALIZING', False
        return fusion_mode(self.now_ns() * 1e-9, self.latest, self.p['sensor_timeout'],
                           self.p['ndt_timeout'], self.p['prediction_timeout'], self.p['future_tolerance'])

    def on_ekf_odom(self, msg):
        self.counts['ekf'] += 1
        _, usable = self.current_mode()
        ns = stamp_ns(msg.header.stamp)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if (not usable or msg.header.frame_id != self.p['map_frame'] or msg.child_frame_id != self.p['base_frame']
                or (self.last_output_ns is not None and ns <= self.last_output_ns)
                or abs(self.now_ns() - ns) * 1e-9 > self.p['sensor_timeout']
                or not all(math.isfinite(v) for v in
                           (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                            msg.twist.twist.linear.x, msg.twist.twist.angular.z, *msg.pose.covariance))):
            return
        self.output_pub.publish(msg)
        pose = PoseWithCovarianceStamped()
        pose.header = copy.deepcopy(msg.header)
        pose.pose = copy.deepcopy(msg.pose)
        self.public_pose_pub.publish(pose)
        if self.tf_pub:
            transform = TransformStamped()
            transform.header = copy.deepcopy(msg.header)
            transform.child_frame_id = self.p['rear_frame']
            transform.transform.translation.x = p.x
            transform.transform.translation.y = p.y
            transform.transform.translation.z = p.z
            transform.transform.rotation = copy.deepcopy(q)
            self.tf_pub.sendTransform(transform)
        self.last_output_ns = ns
        self.counts['output'] += 1

    def publish_status(self):
        mode, usable = self.current_mode()
        now = self.now_ns() * 1e-9
        status = {
            'configuration_file': self.config_path,
            'pointcloud_source_topic': self.p['points_topic'],
            'raw_pointcloud_topic': self.p['raw_points_topic'],
            'pointcloud_timestamp_reference': 'scan_end',
            'mode': mode, 'public_output_enabled': usable,
            'chain': 'wheel_and_imu_to_native_gyro_to_native_ekf_and_native_ndt',
            'ndt_prior_source': 'native_ekf_only', 'wheel_pose_used': False,
            'imu_acceleration_used': False, 'initial_pose_publications': int(self.initial_sent),
            'map_frame': self.p['map_frame'], 'output_reference': 'rear_wheel_center',
            'odometry_child_frame': self.p['base_frame'], 'public_tf_child_frame': self.p['rear_frame'],
            'mount_xyz': self.p['mount_xyz'], 'mount_rpy': self.p['mount_rpy'],
            'mount_provenance': self.p['mount_provenance'], 'extrinsic_accuracy_verified': False,
            'imu_raw_frame': self.p['imu_frame'], 'imu_derived_frame': self.p['base_frame'],
            'imu_to_base_quaternion': self.p['imu_to_base_quaternion'],
            'imu_rotation_provenance': 'explicit_R_base_mid360_times_R_mid360_imu_with_full_R_C_R_transpose',
            'upstream_gyro_inverse_tf_workaround': True,
            'covariance_floors_calibrated': False, 'latest_sensor_stamps': self.latest,
            'ages_sec': {name: now - stamp for name, stamp in self.latest.items()},
            'sensor_timeout': self.p['sensor_timeout'], 'ndt_timeout': self.p['ndt_timeout'],
            'prediction_timeout': self.p['prediction_timeout'],
            'activation': self.activation, 'counts': self.counts,
            'pending_scans': len(self.pending), 'last_rejection': self.last_rejection,
        }
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))
        level = DiagnosticStatus.OK if mode == 'FUSED' else (
            DiagnosticStatus.WARN if usable or mode == 'INITIALIZING' else DiagnosticStatus.ERROR)
        diagnostic = DiagnosticStatus(level=level, name='aps_bag_localization:fusion',
                                      hardware_id='bag', message=mode)
        diagnostic.values = [KeyValue(key=name, value=str(value)) for name, value in (
            ('mount_provenance', self.p['mount_provenance']), ('reference', 'rear_wheel_center'),
            ('ndt_prior', 'native_ekf_only'), ('wheel_pose_used', False))]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diagnostic]
        self.diagnostic_pub.publish(array)
        if mode != self.last_status_mode:
            self.get_logger().info('Fusion mode: ' + mode)
            self.last_status_mode = mode


def main(args=None):
    rclpy.init(args=args)
    node = FusionAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
