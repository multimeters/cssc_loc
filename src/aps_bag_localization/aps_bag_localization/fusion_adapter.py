"""Sensor-only adapter and initialization for native Autoware NDT/EKF fusion.

This node never publishes continuous pose measurements or NDT priors. Those
connections run directly between native NDT and native EKF nodes in launch.
"""
from collections import deque
import copy
import json
import math
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from autoware_internal_localization_msgs.srv import PoseWithCovarianceStamped as NdtAlign
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
        self.live_mode = self.p['runtime_mode'] == 'live'
        self.topic_names = config['topics']
        self.service_names = config['services']
        self.set_parameters([Parameter('use_sim_time', value=config['runtime']['use_sim_time'])])
        self.latest = {}
        self.counts = {key: 0 for key in ('wheel', 'imu', 'gyro', 'ndt', 'ekf', 'output',
                                         'cloud_received', 'cloud_forwarded', 'cloud_dropped', 'rejected',
                                         'ndt_idle_sleeps', 'ndt_wakeups')}
        self.last_rejection = ''
        self.last_input_ns = {}
        self.ekf_first_ns = None
        self.ekf_last_ns = None
        self.last_output_ns = None
        self.pending = deque()
        self.activation = {'ekf': False, 'ndt': False}
        self.futures = {}
        self.initial_sent = False
        self.initial_publications = 0
        self.pending_initial = None
        self.initialization_steps = deque()
        self.initialization_future = None
        self.initialization_future_owner = None
        self.initialization_restart = False
        self.initial_epoch_ns = None
        self.initial_acknowledged = False
        self.initial_target = None
        self.post_initial_ndt = False
        # Live initialization follows Autoware's pose-initializer order:
        # obtain a fresh cloud, run NDT Monte Carlo alignment, then seed EKF
        # with the reliable aligned pose.  The generation guards late replies
        # from an alignment request superseded by a newer RViz pose.
        self.monte_carlo_future = None
        self.monte_carlo_generation = 0
        self.monte_carlo_cloud_sent = False
        self.monte_carlo_aligned = False
        self.monte_carlo_failed = False
        self.monte_carlo_attempts = 0
        self.ndt_epoch_ns = None
        self.ndt_active_since_monotonic = None
        self.ndt_last_result_monotonic = None
        self.last_valid_cloud_ns = None
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
        if self.live_mode:
            self.create_subscription(PoseWithCovarianceStamped, self.p['initial_pose_input_topic'],
                                     self.on_initial_pose, self.p['initial_pose_queue_depth'])
        self.activation_clients = {
            'ekf': self.create_client(SetBool, self.service_names['ekf_activation']),
            'ndt': self.create_client(SetBool, self.service_names['ndt_activation']),
        }
        self.ndt_align_client = self.create_client(NdtAlign, self.service_names['ndt_align'])
        self.tf_pub = TransformBroadcaster(self) if self.p['publish_tf'] else None
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(self.p['tick_period'], self.tick, clock=self.wall_clock)
        self.create_timer(self.p['status_period'], self.publish_status, clock=self.wall_clock)
        self.get_logger().info('Native gyro -> EKF; native NDT pose -> EKF; EKF prior -> NDT. '
                               'Cloud input is raw Livox preprocessed at scan end; '
                               'deskew coverage is reported separately by the preprocessor. '
                               'Outputs represent the rear wheel center. Mounting pitch is provisional.')

    def now_ns(self):
        now = self.get_clock().now().nanoseconds
        if self.clock_last_ns is not None and now < self.clock_last_ns:
            self.clock_error = True
        self.clock_last_ns = now
        return now

    def reject(self, reason):
        self.counts['rejected'] += 1
        self.last_rejection = reason

    def observe(self, stream, stamp):
        ns = stamp_ns(stamp)
        if getattr(self, 'live_mode', False):
            now = self.now_ns()
            if self.clock_error:
                self.reject('clock_reversed_restart_required')
                return False
            age = (now - ns) * 1e-9
            if age > self.p['max_sensor_age_s'] or age < -self.p['live_future_tolerance_s']:
                self.reject(('stale_' if age > 0 else 'future_') + stream + '_stamp')
                return False
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
        if getattr(self, 'live_mode', False) and not (self.initial_sent and self.initial_acknowledged):
            # Native EKF queues twists even while inactive. Do not feed that
            # unbounded queue while waiting indefinitely for an operator seed.
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
        if getattr(self, 'live_mode', False) and not (self.initial_sent and self.initial_acknowledged):
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
        if self.live_mode and (not self.initial_acknowledged or not self.activation['ndt']
                               or self.initial_epoch_ns is None
                               or stamp_ns(msg.header.stamp) < max(self.initial_epoch_ns, self.ndt_epoch_ns or 0)):
            return
        if msg.header.frame_id == self.p['map_frame'] and self.observe('ndt', msg.header.stamp):
            self.counts['ndt'] += 1
            self.post_initial_ndt = True
            self.ndt_last_result_monotonic = time.monotonic()

    def on_ekf_prior(self, msg):
        # Observation only. This adapter has no continuous prior publisher.
        if msg.header.frame_id != self.p['map_frame']:
            return
        ns = stamp_ns(msg.header.stamp)
        if self.live_mode:
            if (not self.initial_sent or self.initial_epoch_ns is None or ns < self.initial_epoch_ns
                    or not self.activation['ndt'] or ns < (self.ndt_epoch_ns or 0) or self.clock_error):
                return
            age = (self.now_ns() - ns) * 1e-9
            if not -self.p['live_future_tolerance_s'] <= age <= self.p['max_sensor_age_s']:
                return
            if not self.initial_acknowledged:
                # The initial-pose topic has no service acknowledgement. Confirm
                # its effect on the EKF state before releasing any new scans.
                target, actual = self.initial_target.pose.pose, msg.pose.pose
                distance = math.sqrt(sum((getattr(actual.position, axis)-getattr(target.position, axis))**2
                                         for axis in ('x', 'y', 'z')))
                dot = abs(sum(getattr(actual.orientation, axis)*getattr(target.orientation, axis)
                              for axis in ('x', 'y', 'z', 'w')))
                if (not math.isfinite(distance) or distance > self.p['initial_pose_ack_position_m']
                        or not math.isfinite(dot) or dot < math.cos(self.p['initial_pose_ack_angle_rad']/2)):
                    return
                self.initial_acknowledged = True
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
        if self.live_mode and not self.initial_sent:
            # Before EKF is seeded, retain only the first phase's source cloud
            # for Autoware NDT Monte Carlo initialization.  Once that source
            # has been sent (or alignment has failed), do not accumulate scans
            # while the service is doing its potentially long search.
            if (self.pending_initial is None or self.monte_carlo_aligned
                    or self.monte_carlo_failed or self.monte_carlo_cloud_sent):
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
        self.last_valid_cloud_ns = stamp_ns(cloud.header.stamp)

    def configured_initial_pose(self):
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = self.p['map_frame']
        xyz, quat = self.p['initial_base_xyz'], self.p['initial_base_quaternion']
        initial.pose.pose.position.x, initial.pose.pose.position.y, initial.pose.pose.position.z = xyz
        (initial.pose.pose.orientation.x, initial.pose.pose.orientation.y,
         initial.pose.pose.orientation.z, initial.pose.pose.orientation.w) = quat
        for index, variance in zip((0, 7, 14, 21, 28, 35), self.p['initial_covariance_diagonal']):
            initial.pose.covariance[index] = variance
        return initial

    def on_initial_pose(self, msg):
        if not self.live_mode or self.clock_error:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        covariance = np.asarray(msg.pose.covariance).reshape(6, 6)
        if (msg.header.frame_id != self.p['map_frame']
                or not all(math.isfinite(value) for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > 1e-3
                or not np.all(np.isfinite(covariance))
                or not np.allclose(covariance, covariance.T, atol=1e-8, rtol=1e-6)
                or np.any(np.diag(covariance)[[0, 1, 5]] <= 0)
                or np.linalg.eigvalsh(covariance).min() < -1e-9):
            self.reject('invalid_initial_pose_frame_pose_or_covariance')
            return
        self.pending_initial = copy.deepcopy(msg)
        self.monte_carlo_generation += 1
        self.monte_carlo_future = None
        self.monte_carlo_cloud_sent = False
        self.monte_carlo_aligned = False
        self.monte_carlo_failed = False
        self.initialization_restart = True
        self.initialization_steps.clear()
        self.initial_sent = False
        self.initial_epoch_ns = None
        self.initial_acknowledged = False
        self.post_initial_ndt = False
        self.last_valid_cloud_ns = None
        self.pending.clear()
        self.ekf_first_ns = self.ekf_last_ns = None
        for name in ('gyro', 'ndt'):
            self.latest.pop(name, None)
            self.last_input_ns.pop(name, None)

    def forward_monte_carlo_source_cloud(self):
        """Give native NDT one fresh cloud before calling ndt_align_srv.

        The native sensor callback stores its input source before checking its
        activation state.  This is the same ordering used by Autoware's pose
        initializer: both NDT and EKF remain stopped while the source cloud is
        captured, then the align service is called on the stopped NDT node.
        Wait one executor tick before issuing the service request so the DDS
        delivery can complete.
        """
        if (not self.live_mode or self.monte_carlo_cloud_sent
                or not self.pending):
            return False
        cloud, _ = self.pending.pop()
        self.pending.clear()
        self.cloud_pub.publish(cloud)
        self.counts['cloud_forwarded'] += 1
        self.monte_carlo_cloud_sent = True
        return True

    def request_monte_carlo_alignment(self):
        """Start Autoware NDT's Monte Carlo initial-pose service once."""
        if (not self.live_mode or self.pending_initial is None
                or self.monte_carlo_aligned or self.monte_carlo_failed
                or not self.monte_carlo_cloud_sent or self.monte_carlo_future is not None
                or not self.ndt_align_client.service_is_ready()):
            return False
        request = NdtAlign.Request()
        request.pose_with_covariance = copy.deepcopy(self.pending_initial)
        request.pose_with_covariance.header.stamp = self.get_clock().now().to_msg()
        self.monte_carlo_future = (
            self.monte_carlo_generation,
            self.ndt_align_client.call_async(request),
        )
        self.monte_carlo_attempts += 1
        return True

    def poll_monte_carlo_alignment(self):
        """Consume the NDT result and promote only a reliable alignment."""
        if self.monte_carlo_future is None:
            return False
        generation, future = self.monte_carlo_future
        if not future.done():
            return False
        self.monte_carlo_future = None
        if generation != self.monte_carlo_generation:
            return False
        try:
            response = future.result()
            success = bool(response.success)
            reliable = bool(response.reliable)
        except Exception as error:
            self.monte_carlo_failed = True
            self.last_rejection = 'ndt_align_service_error:' + str(error)
            self.reject(self.last_rejection)
            return False
        if not success:
            self.monte_carlo_failed = True
            self.last_rejection = 'ndt_align_failed'
            self.reject(self.last_rejection)
            return False
        if not reliable:
            self.monte_carlo_failed = True
            self.last_rejection = 'ndt_align_unreliable'
            self.reject(self.last_rejection)
            return False
        aligned = copy.deepcopy(response.pose_with_covariance)
        if aligned.header.frame_id != self.p['map_frame']:
            self.monte_carlo_failed = True
            self.last_rejection = 'ndt_align_wrong_result_frame'
            self.reject(self.last_rejection)
            return False
        aligned.header.stamp = self.get_clock().now().to_msg()
        self.pending_initial = aligned
        self.monte_carlo_aligned = True
        self.pending.clear()
        return True

    def poll_native_transition(self, now_ns):
        """One service request at a time for initialization, idle recovery and clock faults."""
        if self.initialization_future is not None:
            name, desired, future = self.initialization_future
            if not future.done():
                return False
            owner = self.initialization_future_owner
            self.initialization_future = None
            self.initialization_future_owner = None
            try:
                success = bool(future.result().success)
            except Exception as error:
                success = False
                self.reject('initialization_service_error:' + str(error))
            if not success:
                if owner == 'initialization' and not self.clock_error:
                    self.initialization_steps.appendleft((name, desired))
                return False
            self.activation[name] = desired
            if name == 'ndt':
                # NDT true clears its native prior buffer. Coverage recorded by
                # the adapter must restart at the same epoch, including after a
                # prolonged outage; no stale scans may use the previous buffer.
                self.ekf_first_ns = self.ekf_last_ns = None
                self.latest.pop('ndt', None)
                if desired:
                    self.ndt_epoch_ns = now_ns
                    self.ndt_active_since_monotonic = time.monotonic()
                    self.ndt_last_result_monotonic = None
                    if owner == 'idle':
                        self.counts['ndt_wakeups'] += 1
                else:
                    self.ndt_active_since_monotonic = None
                    if owner == 'idle':
                        self.counts['ndt_idle_sleeps'] += 1
        return True

    def request_native_transition(self, name, desired, owner):
        client = self.activation_clients[name]
        if self.initialization_future is None and client.service_is_ready():
            self.initialization_future = (name, desired, client.call_async(SetBool.Request(data=desired)))
            self.initialization_future_owner = owner
            return True
        return False

    def live_initialization_tick(self):
        if self.pending_initial is None and not self.initial_sent and self.p['initialization'] == 'config':
            self.on_initial_pose(self.configured_initial_pose())
        self.poll_monte_carlo_alignment()
        if self.initialization_restart:
            # NDT deactivation acknowledgement waits for its in-flight scan
            # callback. Keep EKF inactive until NDT has returned a reliable
            # Monte Carlo alignment, then seed EKF with that refined pose.
            self.initialization_steps = deque((('ndt', False), ('ekf', False)))
            self.initialization_restart = False
        if self.initialization_steps:
            name, desired = self.initialization_steps[0]
            if self.request_native_transition(name, desired, 'initialization'):
                self.initialization_steps.popleft()
            return
        if (self.live_mode and self.pending_initial is not None
                and not self.monte_carlo_aligned):
            if self.monte_carlo_failed:
                return
            # NDT stores its source cloud even while stopped. The following
            # tick starts the service only after the cloud has been delivered
            # to the native NDT callback group.
            if self.forward_monte_carlo_source_cloud():
                return
            self.request_monte_carlo_alignment()
            return
        if (self.live_mode and self.monte_carlo_aligned
                and self.pending_initial is not None
                and not self.initial_sent
                and self.initial_pub.get_subscription_count() > 0):
            initial = self.pending_initial
            initial.header.stamp = self.get_clock().now().to_msg()
            self.initial_epoch_ns = stamp_ns(initial.header.stamp)
            self.initial_target = copy.deepcopy(initial)
            self.initial_pub.publish(initial)
            self.initial_sent = True
            self.initial_publications += 1
            self.pending_initial = None
            # Autoware publishes the aligned reset while EKF/NDT are stopped,
            # then re-enables both native nodes.  EKF keeps the reset received
            # while inactive and its trigger only clears queued measurements.
            self.initialization_steps = deque((('ndt', True), ('ekf', True)))
            self.get_logger().info('Published map->rear-center initial pose; waiting for EKF acknowledgement and fresh NDT.')
            return
        if (self.pending_initial is not None and all(self.activation.values())
                and self.initial_pub.get_subscription_count() > 0):
            initial = self.pending_initial
            initial.header.stamp = self.get_clock().now().to_msg()
            self.initial_epoch_ns = stamp_ns(initial.header.stamp)
            self.initial_target = copy.deepcopy(initial)
            self.initial_pub.publish(initial)
            self.initial_sent = True
            self.initial_publications += 1
            self.pending_initial = None
            self.get_logger().info('Published map->rear-center initial pose; waiting for EKF acknowledgement and fresh NDT.')

    def ndt_idle_tick(self, now_ns):
        """Bound the locked native NDT prior cache while point clouds are absent/bad.

        Native NDT prunes prior history only in successful scan interpolation.
        After a bounded interval without a valid result, suspend it. Fresh valid
        clouds wake it through true activation, which clears the native cache.
        Continuous failing scans therefore also trigger bounded periodic resets.
        EKF<->NDT remains a direct native feedback connection.
        """
        if (self.initialization_future is not None or self.initialization_steps
                or self.initialization_restart or self.pending_initial is not None or not self.initial_sent):
            return
        if self.activation['ndt']:
            last_result = self.ndt_last_result_monotonic
            anchor = last_result if last_result is not None else self.ndt_active_since_monotonic
            if anchor is not None and time.monotonic()-anchor >= self.p['ndt_idle_timeout_s']:
                self.request_native_transition('ndt', False, 'idle')
        elif (self.last_valid_cloud_ns is not None
              and 0 <= (now_ns-self.last_valid_cloud_ns)*1e-9 <= self.p['max_sensor_age_s']):
            self.request_native_transition('ndt', True, 'idle')

    def tick(self):
        now_ns = self.now_ns()
        if self.clock_last_ns is not None and now_ns < self.clock_last_ns:
            self.clock_error = True
        self.clock_last_ns = now_ns
        transition_ready = self.poll_native_transition(now_ns) if self.live_mode else True
        if now_ns <= 0 or self.clock_error:
            if self.clock_error:
                self.pending.clear()
                self.initialization_steps.clear()
                self.initialization_restart = False
                # A clock fault stops sensor forwarding permanently, but the
                # native EKF process can still publish priors. Explicitly stop
                # NDT too, after any in-flight service request has completed.
                if self.live_mode and transition_ready and self.activation['ndt']:
                    self.request_native_transition('ndt', False, 'clock_fault')
            return
        if self.live_mode and transition_ready:
            self.live_initialization_tick()
            self.ndt_idle_tick(now_ns)
        for name, client in (() if self.live_mode else self.activation_clients.items()):
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
        if (not self.live_mode and all(self.activation.values()) and not self.initial_sent
                and self.initial_pub.get_subscription_count() > 0):
            initial = self.configured_initial_pose()
            initial.header.stamp = self.get_clock().now().to_msg()
            self.initial_pub.publish(initial)
            self.initial_sent = True
            self.initial_publications += 1
            self.get_logger().info('Initialized EKF once with map->rear-center pose converted from map->body ICP.')
        # Wait for real EKF samples to cover the original acquisition timestamp.
        # No interpolation, extrapolation or NDT self-feedback happens here.
        while self.pending:
            cloud, ready_at = self.pending[0]
            ns = stamp_ns(cloud.header.stamp)
            max_age = min(self.p['scan_wait_timeout'], self.p['max_sensor_age_s']) if self.live_mode else self.p['scan_wait_timeout']
            if ((now_ns - ns) * 1e-9 > max_age
                    or (self.ekf_first_ns is not None and ns < self.ekf_first_ns)):
                self.pending.popleft()
                self.counts['cloud_dropped'] += 1
                continue
            if (self.ekf_first_ns is None or self.ekf_last_ns == self.ekf_first_ns
                    or ns > self.ekf_last_ns or not self.activation['ndt']
                    or (self.live_mode and (not self.initial_acknowledged or self.initialization_future is not None))):
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
        now = self.now_ns()
        if self.clock_error:
            return 'CLOCK_REVERSED_RESTART_REQUIRED', False
        if not self.initial_sent:
            if self.live_mode and self.pending_initial is None:
                return 'WAITING_INITIAL_POSE', False
            return 'INITIALIZING', False
        if self.live_mode and not self.post_initial_ndt:
            return 'WAITING_SENSORS', False
        return fusion_mode(now * 1e-9, self.latest, self.p['sensor_timeout'],
                           self.p['ndt_timeout'], self.p['prediction_timeout'], self.p['future_tolerance'])

    def on_ekf_odom(self, msg):
        self.counts['ekf'] += 1
        _, usable = self.current_mode()
        ns = stamp_ns(msg.header.stamp)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if (not usable or msg.header.frame_id != self.p['map_frame'] or msg.child_frame_id != self.p['base_frame']
                or (self.live_mode and (self.initial_epoch_ns is None or ns < self.initial_epoch_ns))
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
            'runtime_mode': self.p['runtime_mode'], 'initialization_source': self.p['initialization'],
            'initial_pose_acknowledged': self.initial_acknowledged,
            'imu_acceleration_used': False, 'initial_pose_publications': self.initial_publications,
            'monte_carlo': {
                'aligned': self.monte_carlo_aligned,
                'failed': self.monte_carlo_failed,
                'attempts': self.monte_carlo_attempts,
                'source_cloud_sent': self.monte_carlo_cloud_sent,
                'service_pending': self.monte_carlo_future is not None,
            },
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
            'ndt_idle_timeout_s': self.p['ndt_idle_timeout_s'],
            'ndt_prior_epoch_ns': self.ndt_epoch_ns,
            'native_transition_owner': self.initialization_future_owner,
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
