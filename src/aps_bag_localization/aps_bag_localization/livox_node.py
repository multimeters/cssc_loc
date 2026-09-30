"""Raw MID-360 CustomMsg -> quality-filtered scan-end PointCloud2 for NDT."""
from collections import Counter, deque
import json
import math

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import String

from .configuration import load_config
from .livox_deskew import IncompleteMotion, deskew_scan, fallback_due, parse_scan, quaternion_matrix


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def make_cloud(scan, xyz, frame):
    cloud = PointCloud2()
    cloud.header.frame_id = frame
    cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(scan.end_ns, 1_000_000_000)
    cloud.height, cloud.width = 1, len(xyz)
    cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    cloud.is_bigendian, cloud.point_step, cloud.row_step = False, 16, 16 * len(xyz)
    cloud.is_dense = True
    values = np.empty((len(xyz), 4), dtype='<f4')
    values[:, :3], values[:, 3] = xyz, scan.intensity
    cloud.data = values.tobytes()
    return cloud


class RawLivoxNode(Node):
    def __init__(self, config_path=None):
        super().__init__('aps_raw_livox')
        self.declare_parameter('configuration_file', '')
        config_path = config_path or self.get_parameter('configuration_file').value
        if not config_path:
            raise ValueError('configuration_file is required')
        self.config = load_config(config_path)
        self.settings = self.config['livox']
        self.live_mode = self.config['runtime']['mode'] == 'live'
        self.live_settings = self.config['live']
        self.motion_wait_timeout_s = (self.live_settings['motion_wait_timeout_s'] if self.live_mode
                                      else self.settings['wait_timeout_s'])
        self.clock_last_ns = None
        self.clock_error = False
        self.topics, self.frames = self.config['topics'], self.config['frames']
        self.set_parameters([Parameter('use_sim_time', value=self.config['runtime']['use_sim_time'])])
        self.rotation_imu_base = quaternion_matrix(self.config['_derived']['imu_to_base_quaternion'])
        self.rotation_lidar_base = quaternion_matrix(self.config['_derived']['base_to_lidar_quaternion'])
        self.lidar_origin_base = np.asarray(self.config['_derived']['base_to_lidar_xyz'], dtype=np.float64)
        self.imu, self.wheel, self.pending = deque(), deque(), deque()
        self.last_raw_end = None
        self.counts = {key: 0 for key in ('cloud_received', 'cloud_published', 'fully_deskewed',
            'uncompensated', 'cloud_rejected', 'imu_received', 'wheel_received', 'motion_rejected',
            'points_input', 'points_rejected', 'points_published')}
        self.fallback_reasons = Counter()
        self.last_rejection = ''
        self.last_scan = {}
        self.depth = self.config['adapter']['cloud_queue_depth']
        sensor_qos = QoSProfile(depth=self.config['adapter']['sensor_queue_depth'],
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(PointCloud2, self.topics['processed_points'], self.depth)
        self.status_publisher = self.create_publisher(String, self.topics['preprocessing_status'],
            QoSProfile(depth=self.config['adapter']['status_queue_depth'], durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(CustomMsg, self.topics['points'], self.on_cloud, sensor_qos)
        self.create_subscription(Imu, self.topics['imu'], self.on_imu, sensor_qos)
        self.create_subscription(Odometry, self.topics['wheel'], self.on_wheel, sensor_qos)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(self.config['adapter']['tick_period'], self.tick, clock=self.wall_clock)
        self.create_timer(self.config['adapter']['status_period'], self.publish_status, clock=self.wall_clock)
        self.get_logger().info('Raw Livox points use the lidar origin; scan-end deskew uses measured '
                               '3-D IMU rates and wheel forward speed. Sensor gaps produce explicitly '
                               'reported uncorrected whole scans.')

    def _append_motion(self, buffer, stamp, values):
        if self.live_mode and not self.fresh_live_stamp(stamp):
            self.counts['motion_rejected'] += 1
            return
        if stamp <= 0 or not all(math.isfinite(v) for v in values) or (buffer and stamp <= buffer[-1][0]):
            self.counts['motion_rejected'] += 1
            return
        buffer.append((stamp, *values))
        cutoff = stamp - round(self.settings['buffer_seconds'] * 1e9)
        # Retain one predecessor for boundary interpolation.
        while len(buffer) > 2 and buffer[1][0] < cutoff:
            buffer.popleft()

    def on_imu(self, message):
        self.counts['imu_received'] += 1
        if message.header.frame_id != self.frames['imu'] or message.angular_velocity_covariance[0] < 0:
            self.counts['motion_rejected'] += 1
            return
        rates = np.array([message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z])
        self._append_motion(self.imu, stamp_ns(message.header.stamp), self.rotation_imu_base @ rates)

    def on_wheel(self, message):
        self.counts['wheel_received'] += 1
        if message.child_frame_id != self.frames['wheel']:
            self.counts['motion_rejected'] += 1
            return
        # Do not read wheel orientation, pose, yaw rate, lateral or vertical speed.
        self._append_motion(self.wheel, stamp_ns(message.header.stamp), (message.twist.twist.linear.x,))

    def on_cloud(self, message):
        self.counts['cloud_received'] += 1
        try:
            scan = parse_scan(message, self.settings)
            if self.live_mode and (not self.fresh_live_stamp(scan.start_ns)
                                   or not self.fresh_live_stamp(scan.end_ns)):
                raise ValueError(self.last_rejection)
            if self.last_raw_end is not None and scan.end_ns <= self.last_raw_end:
                raise ValueError('scan end timestamps are not strictly increasing')
            self.last_raw_end = scan.end_ns
        except ValueError as error:
            self.counts['cloud_rejected'] += 1
            self.last_rejection = str(error)
            self.get_logger().warning('Rejected raw scan: ' + str(error))
            return
        self.counts['points_input'] += scan.input_points
        self.counts['points_rejected'] += scan.rejected_points
        self.pending.append(scan)
        if len(self.pending) > self.config['adapter']['max_pending_scans']:
            self.pending.popleft()
            self.counts['cloud_rejected'] += 1
            self.last_rejection = 'pending_queue_limit'

    def tick(self):
        now_ns = self.current_time_ns()
        if self.clock_error:
            self.pending.clear()
            return
        while self.pending:
            scan = self.pending[0]
            if self.live_mode and (now_ns-scan.end_ns)*1e-9 > self.live_settings['max_sensor_age_s']:
                self.pending.popleft()
                self.counts['cloud_rejected'] += 1
                self.last_rejection = 'stale_pending_cloud_stamp'
                continue
            if now_ns < scan.end_ns:
                return
            try:
                xyz = deskew_scan(scan, list(self.imu), list(self.wheel), self.rotation_lidar_base,
                    self.lidar_origin_base, self.settings['max_imu_gap_s'], self.settings['max_wheel_gap_s'])
                reason = ''
            except IncompleteMotion as error:
                if not fallback_due(scan.end_ns, now_ns, self.motion_wait_timeout_s,
                                    self.imu[-1][0] if self.imu else None,
                                    self.wheel[-1][0] if self.wheel else None):
                    return
                xyz, reason = scan.xyz, str(error)
            except ValueError as error:
                xyz, reason = scan.xyz, 'invalid_motion:' + str(error)
            self.pending.popleft()
            self._publish(scan, xyz, reason)

    def current_time_ns(self):
        now = self.get_clock().now().nanoseconds
        if self.clock_last_ns is not None and now < self.clock_last_ns:
            self.clock_error = True
            self.last_rejection = 'clock_reversed_restart_required'
        self.clock_last_ns = now
        return now

    def fresh_live_stamp(self, stamp):
        now = self.current_time_ns()
        if self.clock_error:
            return False
        age = (now-stamp)*1e-9
        if age > self.live_settings['max_sensor_age_s']:
            self.last_rejection = 'stale_sensor_stamp'
            return False
        if age < -self.live_settings['future_tolerance_s']:
            self.last_rejection = 'future_sensor_stamp'
            return False
        return True

    def _publish(self, scan, xyz, fallback_reason):
        self.publisher.publish(make_cloud(scan, xyz, self.frames['cloud']))
        self.counts['cloud_published'] += 1
        self.counts['points_published'] += len(xyz)
        mode = 'uncompensated' if fallback_reason else 'fully_deskewed'
        self.counts[mode] += 1
        if fallback_reason:
            self.fallback_reasons[fallback_reason] += 1
        self.last_scan = {'start_ns': scan.start_ns, 'end_ns': scan.end_ns, 'mode': mode,
                         'fallback_reason': fallback_reason, 'output_points': len(xyz)}

    def publish_status(self):
        data = {'node': 'aps_raw_livox', 'raw_topic': self.topics['points'],
                'runtime_mode': self.config['runtime']['mode'],
                'mode': 'CLOCK_REVERSED_RESTART_REQUIRED' if self.clock_error else 'RUNNING',
                'output_topic': self.topics['processed_points'], 'output_frame': self.frames['cloud'],
                'reference': 'scan_end', 'fallback_policy': 'whole_scan_uncorrected_with_nominal_end_header',
                'motion_wait_timeout_s': self.motion_wait_timeout_s,
                'counts': self.counts, 'pending': len(self.pending),
                'fallback_reasons': dict(self.fallback_reasons), 'last_scan': self.last_scan,
                'last_rejection': self.last_rejection}
        self.status_publisher.publish(String(data=json.dumps(data, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = RawLivoxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
