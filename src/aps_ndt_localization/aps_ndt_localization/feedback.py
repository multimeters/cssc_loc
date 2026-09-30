"""Supply NDT seeds from its own accepted poses; never consume recorded poses or TF."""
from collections import deque
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool

from .prediction import normalized, predict, rpy_quaternion, seed_times
from .cloud import filter_cloud


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class NdtFeedback(Node):
    def __init__(self):
        super().__init__("ndt_feedback")
        defaults = {
            "map_frame": "map", "base_frame": "body",
            "initial_x": 0.0, "initial_y": 0.0, "initial_z": 0.0,
            "initial_roll": 0.0, "initial_pitch": 0.0, "initial_yaw": 0.0,
            "max_extrapolation_sec": 0.5, "relay_delay_sec": 0.05,
            "max_pending_scans": 20, "pose_variance": 1.0, "yaw_variance": 0.25,
            "voxel_size": 0.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.params = {key: self.get_parameter(key).value for key in defaults}
        if (self.params["max_pending_scans"] < 1 or self.params["relay_delay_sec"] < 0
                or self.params["max_extrapolation_sec"] < 0 or self.params["voxel_size"] < 0
                or not all(math.isfinite(value) for value in self.params.values()
                           if isinstance(value, (float, int)))):
            raise ValueError("Invalid scan relay configuration")
        self.initial = (0.0,
                        tuple(self.params["initial_"+key] for key in ("x", "y", "z")),
                        rpy_quaternion(*(self.params["initial_"+key]
                                         for key in ("roll", "pitch", "yaw"))))
        self.latest = self.initial
        self.previous = None
        self.accepted = 0
        self.received = 0
        self.forwarded = 0
        self.dropped = 0
        self.last_scan_stamp = None
        self.last_prior_ns = None
        self.pending = deque()
        self.active = False
        self.activation_future = None
        self.prior_pub = self.create_publisher(PoseWithCovarianceStamped, "prior", 100)
        self.points_pub = self.create_publisher(PointCloud2, "points", 10)
        self.odom_pub = self.create_publisher(Odometry, "odometry", 10)
        self.create_subscription(PointCloud2, "input_points", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "ndt_pose", self.on_pose, 100)
        self.activation = self.create_client(SetBool, "trigger_node")
        # A wall timer permits activation before rosbag starts producing /clock.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.01, self.tick, clock=self.wall_clock)
        self.create_timer(10.0, self.status, clock=self.wall_clock)
        self.get_logger().info(
            "NDT prior uses configured initial pose and accepted NDT outputs only. "
            "Point cloud frame must be %s; output pose represents that frame in %s."
            % (self.params["base_frame"], self.params["map_frame"]))

    def tick(self):
        if not self.active:
            if self.activation_future is None and self.activation.service_is_ready():
                request = SetBool.Request()
                request.data = True
                self.activation_future = self.activation.call_async(request)
            elif self.activation_future is not None and self.activation_future.done():
                try:
                    self.active = bool(self.activation_future.result().success)
                except Exception as exc:
                    self.get_logger().error("Activation failed: %s" % exc)
                self.activation_future = None
                if self.active:
                    self.get_logger().info("Native NDT activated; ready for point clouds.")
        now = time.monotonic()
        while self.pending and self.pending[0][0] <= now:
            _, scan = self.pending.popleft()
            self.points_pub.publish(scan)
            self.forwarded += 1

    def on_scan(self, scan):
        self.received += 1
        if not self.active:
            self.dropped += 1
            return
        if scan.header.frame_id != self.params["base_frame"]:
            self.get_logger().error("Reject cloud frame '%s'; expected '%s', no extrinsic assumed."
                                    % (scan.header.frame_id, self.params["base_frame"]))
            self.dropped += 1
            return
        try:
            scan = filter_cloud(scan, self.params["voxel_size"])
        except (ValueError, TypeError, BufferError) as exc:
            self.get_logger().error("Reject invalid cloud: %s" % exc)
            self.dropped += 1
            return
        stamp = seconds(scan.header.stamp)
        if self.last_scan_stamp is not None and stamp <= self.last_scan_stamp:
            self.get_logger().error("Non-increasing scan time. Restart localization before replaying bag.")
            self.dropped += 1
            return
        self.last_scan_stamp = stamp
        # Two adjacent samples satisfy the upstream pose interpolation contract even at startup.
        # Both are predictions from the same accepted NDT history, never recorded localization.
        ns = scan.header.stamp.sec*1_000_000_000+scan.header.stamp.nanosec
        for seed_ns in seed_times(ns, self.last_prior_ns):
            prior = PoseWithCovarianceStamped()
            prior.header.frame_id = self.params["map_frame"]
            prior.header.stamp.sec, prior.header.stamp.nanosec = divmod(seed_ns, 1_000_000_000)
            xyz, quat = predict(self.previous, self.latest, seed_ns*1e-9,
                                self.params["max_extrapolation_sec"])
            prior.pose.pose.position.x, prior.pose.pose.position.y, prior.pose.pose.position.z = xyz
            (prior.pose.pose.orientation.x, prior.pose.pose.orientation.y,
             prior.pose.pose.orientation.z, prior.pose.pose.orientation.w) = quat
            for index in (0, 7, 14):
                prior.pose.covariance[index] = self.params["pose_variance"]
            for index in (21, 28, 35):
                prior.pose.covariance[index] = self.params["yaw_variance"]
            self.prior_pub.publish(prior)
            self.last_prior_ns = seed_ns
        if len(self.pending) >= self.params["max_pending_scans"]:
            self.pending.popleft()
            self.dropped += 1
        self.pending.append((time.monotonic()+self.params["relay_delay_sec"], scan))

    def on_pose(self, message):
        if message.header.frame_id != self.params["map_frame"]:
            self.get_logger().error("Reject NDT pose from unexpected map frame.")
            return
        pose = message.pose.pose
        xyz = (pose.position.x, pose.position.y, pose.position.z)
        if not all(math.isfinite(value) for value in xyz):
            return
        try:
            quat = normalized((pose.orientation.x, pose.orientation.y,
                               pose.orientation.z, pose.orientation.w))
        except ValueError:
            return
        stamp = seconds(message.header.stamp)
        if self.accepted and stamp <= self.latest[0]:
            return
        self.previous = self.latest if self.accepted else None
        self.latest = (stamp, xyz, quat)
        self.accepted += 1
        odom = Odometry()
        odom.header = message.header
        odom.child_frame_id = self.params["base_frame"]
        odom.pose = message.pose
        # NDT estimates pose. Do not invent a measured twist from finite differences here.
        for index in (0, 7, 14, 21, 28, 35):
            odom.twist.covariance[index] = 1e6
        self.odom_pub.publish(odom)

    def status(self):
        self.get_logger().info("clouds received=%d forwarded=%d dropped=%d accepted_ndt=%d"
                               % (self.received, self.forwarded, self.dropped, self.accepted))


def main(args=None):
    rclpy.init(args=args)
    node = NdtFeedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.status()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
