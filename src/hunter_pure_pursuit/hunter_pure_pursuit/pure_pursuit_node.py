#!/usr/bin/env python3
"""Pure Pursuit controller for the fixed Hunter route."""

import json
import math
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("hunter_pure_pursuit")
        default_config = get_package_share_directory("hunter_pure_pursuit") + "/config/route.yaml"
        self.declare_parameter("config_file", default_config)
        config_path = self.get_parameter("config_file").value
        with open(config_path, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        self.params = config["/**"]["ros__parameters"]

        self.odom_topic = self.params["odom_topic"]
        self.cmd_topic = self.params["cmd_vel_topic"]
        self.status_topic = self.params["fusion_status_topic"]
        self.initial_pose_topic = self.params.get("initial_pose_topic", "/initialpose")
        self.frame_id = self.params["map_frame"]
        self.child_frame_id = self.params["base_frame"]
        xs = [float(value) for value in self.params["waypoints_x_m"]]
        ys = [float(value) for value in self.params["waypoints_y_m"]]
        if len(xs) != len(ys) or not xs:
            raise ValueError("waypoints_x_m and waypoints_y_m must have the same nonzero length")
        if not all(math.isfinite(value) for value in xs + ys):
            raise ValueError("waypoints must contain only finite coordinates")
        self.waypoints = list(zip(xs, ys))

        self.pose = None
        self.pose_wall_time = None
        self.fusion_mode = None
        self.status_wall_time = None
        self.route_index = 0
        self.route_initialized = False
        self.completed = False
        self.drive_direction = 1
        self.last_command = (0.0, 0.0)
        self.last_control_time = time.monotonic()
        self.last_warning = {}
        self.last_reported_target_index = None

        self.publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.on_odometry, 10)
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.initial_pose_topic, self.on_initial_pose, 10
        )
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_sub = self.create_subscription(String, self.status_topic, self.on_fusion_status, status_qos)
        rate = float(self.params["control_rate_hz"])
        if rate <= 0.0:
            raise ValueError("control_rate_hz must be positive")
        self.timer = self.create_timer(1.0 / rate, self.control)

        self.get_logger().info(
            "Ready with %d map-frame waypoints; waiting for fresh localization and FUSED status."
            % len(self.waypoints)
        )
        for index, point in enumerate(self.waypoints, start=1):
            self.get_logger().info("waypoint %d: map (%.6f, %.6f)" % (index, point[0], point[1]))

    def on_fusion_status(self, message):
        try:
            mode = json.loads(message.data).get("mode")
            if isinstance(mode, str):
                self.fusion_mode = mode
                self.status_wall_time = time.monotonic()
        except (TypeError, ValueError):
            self.fusion_mode = None
            self.status_wall_time = None

    def on_initial_pose(self, message):
        if message.header.frame_id != self.frame_id:
            self.warn_throttled(
                "initial_pose_frame",
                "Ignoring initial pose in frame %s; expected %s."
                % (message.header.frame_id, self.frame_id),
            )
            return
        position = message.pose.pose.position
        if not all(math.isfinite(value) for value in (position.x, position.y)):
            self.warn_throttled("initial_pose", "Ignoring invalid initial pose.")
            return

        # RViz can reinitialize localization after tracking starts. Keep route
        # progress aligned with that pose instead of continuing a stale target.
        self.route_initialized = False
        self.completed = False
        self.drive_direction = 1
        self.last_command = (0.0, 0.0)
        self.last_reported_target_index = None
        self.initialize_route(float(position.x), float(position.y))
        self.get_logger().info("Route progress synchronized from /initialpose.")

    def on_odometry(self, message):
        if message.header.frame_id != self.frame_id or message.child_frame_id != self.child_frame_id:
            self.pose = None
            self.pose_wall_time = None
            self.warn_throttled(
                "frame",
                "Ignoring localization: expected %s -> %s, received %s -> %s"
                % (self.frame_id, self.child_frame_id,
                   message.header.frame_id, message.child_frame_id),
            )
            return
        position = message.pose.pose.position
        q = message.pose.pose.orientation
        qnorm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not all(math.isfinite(value) for value in (position.x, position.y, qnorm)) or qnorm < 1e-6:
            self.pose = None
            self.pose_wall_time = None
            self.warn_throttled("pose", "Ignoring invalid localization pose.")
            return
        qx, qy, qz, qw = q.x / qnorm, q.y / qnorm, q.z / qnorm, q.w / qnorm
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        self.pose = (float(position.x), float(position.y), yaw)
        self.pose_wall_time = time.monotonic()
        if not self.route_initialized:
            self.initialize_route(self.pose[0], self.pose[1])

    def initialize_route(self, x, y):
        radius = float(self.params["resume_near_waypoint_m"])
        nearest = min(
            range(len(self.waypoints)),
            key=lambda index: math.hypot(self.waypoints[index][0] - x, self.waypoints[index][1] - y),
        )
        distance = math.hypot(self.waypoints[nearest][0] - x, self.waypoints[nearest][1] - y)
        if distance <= radius:
            self.route_index = nearest + 1
            self.get_logger().info(
                "Resuming after waypoint %d (current pose is %.2f m from it)." % (nearest + 1, distance)
            )
        else:
            self.route_index = 0
            self.get_logger().info("Starting at waypoint 1 (nearest waypoint is %.2f m away)." % distance)
        self.route_initialized = True

    def warn_throttled(self, key, message):
        now = time.monotonic()
        if now - self.last_warning.get(key, 0.0) >= 3.0:
            self.get_logger().warning(message)
            self.last_warning[key] = now

    def reconcile_missed_waypoints(self, x, y, yaw):
        """Advance past a missed waypoint when a later route point is behind us."""
        if not self.waypoints or self.route_index >= len(self.waypoints):
            return False

        radius = float(self.params.get("waypoint_recovery_radius_m", 6.0))
        behind_threshold = float(self.params.get("waypoint_passed_behind_m", 0.75))
        if radius <= 0.0:
            return False

        # Only check the next point in route order. Searching all later points
        # can jump over intermediate waypoints, and after wrapping from the
        # final point it can repeatedly select that same final point forever.
        if self.route_index < len(self.waypoints) - 1:
            candidate = self.route_index + 1
        elif bool(self.params["loop_route"]):
            candidate = 0
        else:
            return False

        distance = math.hypot(self.waypoints[candidate][0] - x,
                              self.waypoints[candidate][1] - y)
        current = self.waypoints[self.route_index]
        current_distance = math.hypot(current[0] - x, current[1] - y)
        local_x, _ = self.local_coordinates(self.waypoints[candidate], (x, y, yaw))
        if (distance > radius or local_x > -behind_threshold
                or distance + 0.5 >= current_distance):
            return False

        next_index = candidate + 1
        if next_index >= len(self.waypoints) and bool(self.params["loop_route"]):
            next_index = 0
        self.route_index = next_index
        self.drive_direction = 1
        target_number = next_index + 1 if next_index < len(self.waypoints) else None
        if target_number is None:
            self.get_logger().warning(
                "Route progress recovered past waypoint %d; route is complete."
                % (candidate + 1)
            )
        else:
            self.get_logger().warning(
                "Route progress recovered at passed waypoint %d (%.2f m away); "
                "tracking waypoint %d."
                % (candidate + 1, distance, target_number)
            )
        return True

    def publish_command(self, speed, yaw_rate):
        command = Twist()
        command.linear.x = float(speed)
        command.angular.z = float(yaw_rate)
        self.publisher.publish(command)

    def stop(self, reason=None):
        if reason:
            self.warn_throttled("stop:" + reason, reason)
        self.last_command = (0.0, 0.0)
        self.publish_command(0.0, 0.0)

    @staticmethod
    def local_coordinates(point, pose):
        dx, dy = point[0] - pose[0], point[1] - pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return c * dx + s * dy, -s * dx + c * dy

    def select_lookahead(self, pose, waypoints, lookahead):
        """Interpolate the first forward circle/path intersection."""
        path = [(pose[0], pose[1])] + list(waypoints)
        radius_sq = lookahead * lookahead
        for start, end in zip(path[:-1], path[1:]):
            dx, dy = end[0] - start[0], end[1] - start[1]
            length = math.hypot(dx, dy)
            if length < 1e-6:
                continue
            ox, oy = start[0] - pose[0], start[1] - pose[1]
            a = dx * dx + dy * dy
            b = 2.0 * (ox * dx + oy * dy)
            c = ox * ox + oy * oy - radius_sq
            disc = b * b - 4.0 * a * c
            if disc >= 0.0:
                root = math.sqrt(disc)
                roots = sorted(((-b - root) / (2.0 * a), (-b + root) / (2.0 * a)))
                for fraction in roots:
                    if -1e-9 <= fraction <= 1.0 + 1e-9:
                        fraction = min(1.0, max(0.0, fraction))
                        point = (start[0] + fraction * dx, start[1] + fraction * dy)
                        if self.local_coordinates(point, pose)[0] > 0.02:
                            return point
        return waypoints[-1]

    def control(self):
        now = time.monotonic()
        dt = max(1e-3, min(0.2, now - self.last_control_time))
        self.last_control_time = now

        if self.pose is None or self.pose_wall_time is None:
            self.stop("Waiting for a valid map -> base_link localization pose.")
            return
        if now - self.pose_wall_time > float(self.params["pose_timeout_s"]):
            self.stop("Localization pose timed out; commanding zero velocity.")
            return
        if bool(self.params["require_fused_status"]):
            status_age = float("inf") if self.status_wall_time is None else now - self.status_wall_time
            if status_age > float(self.params["status_timeout_s"]) or self.fusion_mode != "FUSED":
                self.stop("Fusion is not healthy (%s); commanding zero velocity."
                          % (self.fusion_mode or "no status"))
                return
        if self.completed:
            self.publish_command(0.0, 0.0)
            return

        x, y, yaw = self.pose
        self.reconcile_missed_waypoints(x, y, yaw)
        reach = float(self.params["waypoint_reach_m"])
        while self.route_index < len(self.waypoints):
            goal = self.waypoints[self.route_index]
            if math.hypot(goal[0] - x, goal[1] - y) > reach:
                break
            self.get_logger().info("Reached waypoint %d." % (self.route_index + 1))
            self.route_index += 1
        if self.route_index >= len(self.waypoints):
            if bool(self.params["loop_route"]):
                self.route_index = 0
                self.drive_direction = 1
                self.get_logger().info("All waypoints reached; restarting route at waypoint 1.")
            else:
                self.completed = True
                self.publish_command(0.0, 0.0)
                self.get_logger().info("All waypoints reached; route complete.")
                return

        if self.last_reported_target_index != self.route_index:
            self.get_logger().info("Tracking waypoint %d." % (self.route_index + 1))
            self.last_reported_target_index = self.route_index

        remaining = self.waypoints[self.route_index:]
        # Slow toward the next waypoint, including waypoint 1 after route wrap.
        goal = remaining[0]
        goal_distance = math.hypot(goal[0] - x, goal[1] - y)
        lookahead = max(
            float(self.params["lookahead_min_m"]),
            min(float(self.params["lookahead_max_m"]),
                float(self.params["lookahead_distance_m"])
                + float(self.params["lookahead_gain"]) * abs(float(self.params["max_speed_mps"]))),
        )
        target = self.select_lookahead(self.pose, remaining, lookahead)
        target_x, target_y = self.local_coordinates(target, self.pose)
        target_sq = target_x * target_x + target_y * target_y
        if target_sq < 1e-6:
            self.stop("Lookahead target is too close; commanding zero velocity.")
            return

        curvature = 2.0 * target_y / target_sq
        steering_limit = abs(float(self.params["max_steering_angle_rad"]))
        wheelbase = float(self.params["wheelbase_m"])
        curvature_limit = min(
            math.tan(steering_limit) / wheelbase,
            1.0 / float(self.params["min_turning_radius_m"]),
        )
        curvature = max(-curvature_limit, min(curvature_limit, curvature))

        hysteresis = float(self.params["direction_switch_hysteresis_m"])
        if target_x < -hysteresis:
            self.drive_direction = -1
        elif target_x > hysteresis:
            self.drive_direction = 1
        forward_limit = float(self.params["max_speed_mps"])
        reverse_limit = float(self.params["max_reverse_speed_mps"])
        speed_limit = forward_limit if self.drive_direction > 0 else reverse_limit
        lateral_accel = float(self.params["max_lateral_accel_mps2"])
        speed = min(speed_limit, math.sqrt(lateral_accel / max(abs(curvature), 1e-6)))
        slowdown = float(self.params["slowdown_distance_m"])
        if slowdown > 0.0:
            speed = min(speed, speed_limit * min(1.0, goal_distance / slowdown))
        approach_distance = float(self.params["waypoint_approach_distance_m"])
        approach_speed = float(self.params["waypoint_approach_speed_mps"])
        if approach_distance > 0.0:
            # Begin braking early enough to meet the approach speed at the zone edge.
            accel_limit = float(self.params["max_linear_accel_mps2"])
            distance_before_zone = max(0.0, goal_distance - approach_distance)
            braking_speed = math.sqrt(
                approach_speed * approach_speed
                + 2.0 * accel_limit * distance_before_zone
            )
            speed = min(speed, braking_speed)
        speed *= self.drive_direction
        yaw_rate = speed * curvature
        yaw_limit = float(self.params["max_yaw_rate_radps"])
        yaw_rate = max(-yaw_limit, min(yaw_limit, yaw_rate))

        max_linear_accel = float(self.params["max_linear_accel_mps2"])
        max_angular_accel = float(self.params["max_angular_accel_radps2"])
        previous_speed, previous_yaw = self.last_command
        speed = max(previous_speed - max_linear_accel * dt,
                    min(previous_speed + max_linear_accel * dt, speed))
        yaw_rate = max(previous_yaw - max_angular_accel * dt,
                       min(previous_yaw + max_angular_accel * dt, yaw_rate))
        self.last_command = (speed, yaw_rate)
        self.publish_command(speed, yaw_rate)

    def destroy_node(self):
        for _ in range(3):
            self.publish_command(0.0, 0.0)
            time.sleep(0.03)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PurePursuitNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
