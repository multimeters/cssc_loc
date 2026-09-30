"""Record a finite, monotonic trajectory and an explicit acceptance summary."""
import csv
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from .trajectory import TrajectorySummary


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('aps_trajectory_recorder')
        self.declare_parameter('output_dir', 'artifacts/replay')
        self.declare_parameter('source_topic', '/localization/kinematic_state')
        self.declare_parameter('source_type', 'odometry')
        self.declare_parameter('minimum_samples', 100)
        self.declare_parameter('minimum_span', 5.0)
        self.output_dir = Path(self.get_parameter('output_dir').value).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.topic = self.get_parameter('source_topic').value
        self.source_type = self.get_parameter('source_type').value
        self.summary = TrajectorySummary(self.get_parameter('minimum_samples').value,
                                         self.get_parameter('minimum_span').value)
        self.status = None
        self.csv_path = self.output_dir / 'trajectory.csv'
        # Refuse to silently destroy an earlier measurement run.
        self.file = self.csv_path.open('x', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['stamp_sec', 'stamp_nanosec', 'frame_id', 'child_frame_id',
                              'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
                              'vx', 'wz', 'var_x', 'var_y', 'var_yaw'])
        if self.source_type == 'odometry':
            self.create_subscription(Odometry, self.topic, self.on_odometry, 100)
        elif self.source_type == 'pose':
            self.create_subscription(PoseWithCovarianceStamped, self.topic, self.on_pose, 100)
        else:
            self.file.close()
            raise ValueError('source_type must be odometry or pose')
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/localization/replay_status', self.on_status, status_qos)
        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.create_timer(1.0, self.save_summary, clock=self.wall_clock)
        self.save_summary()
        self.get_logger().info('Recording ' + self.topic + ' to ' + str(self.csv_path))

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Malformed replay_status JSON ignored')

    def on_odometry(self, msg):
        self.record(msg.header, msg.pose, msg.child_frame_id,
                    msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_pose(self, msg):
        self.record(msg.header, msg.pose, '', '', '')

    def record(self, header, pose, child, vx, wz):
        p = pose.pose.position
        q = pose.pose.orientation
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        if self.summary.add(stamp, header.frame_id, (p.x, p.y, p.z),
                            (q.x, q.y, q.z, q.w), list(pose.covariance)):
            self.writer.writerow([header.stamp.sec, header.stamp.nanosec,
                                  header.frame_id, child, p.x, p.y, p.z,
                                  q.x, q.y, q.z, q.w, vx, wz,
                                  pose.covariance[0], pose.covariance[7], pose.covariance[35]])

    def save_summary(self):
        self.file.flush()
        report = self.summary.report()
        report.update({'source_topic': self.topic, 'source_type': self.source_type,
                       'trajectory_csv': str(self.csv_path), 'replay_guard': self.status})
        target = self.output_dir / 'trajectory_summary.json'
        temporary = target.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        temporary.replace(target)

    def close(self):
        self.save_summary()
        self.file.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
