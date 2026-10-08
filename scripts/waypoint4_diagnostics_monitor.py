#!/usr/bin/env python3
"""Record time-aligned Hunter route, localization, command, and controller logs."""

import argparse
import csv
import json
import math
import re
import socket
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from hunter_msgs.msg import HunterStatus
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import String

ROOT_DIR = Path(__file__).resolve().parent.parent


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_quaternion(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-9:
        return float("nan")
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_route():
    try:
        route_path = Path(get_package_share_directory("hunter_pure_pursuit")) / "config" / "route.yaml"
    except Exception:
        route_path = ROOT_DIR / "src" / "hunter_pure_pursuit" / "config" / "route.yaml"
    with route_path.open("r", encoding="utf-8") as stream:
        params = yaml.safe_load(stream)["/**"]["ros__parameters"]
    points = list(zip(params["waypoints_x_m"], params["waypoints_y_m"]))
    return route_path, [(float(x), float(y)) for x, y in points]


class Waypoint4Diagnostics(Node):
    def __init__(self, output_dir, sample_rate_hz):
        super().__init__("hunter_waypoint4_diagnostics")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.route_path, self.waypoints = load_route()
        self.latest = {}
        self.target_waypoint = ""
        self.last_controller_log = ""
        self.last_fusion_mode = None
        self.sample_count = 0

        self.csv_file = (self.output_dir / "trajectory.csv").open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self.events_file = (self.output_dir / "events.jsonl").open(
            "w", encoding="utf-8", buffering=1
        )
        self.controller_file = (self.output_dir / "controller.log").open(
            "w", encoding="utf-8", buffering=1
        )
        self.columns = [
            "sample_utc", "localization_stamp_ns", "localization_age_s",
            "frame_id", "child_frame_id", "map_x_m", "map_y_m", "map_z_m",
            "map_yaw_rad", "localization_vx_mps", "localization_wz_radps",
            "nearest_waypoint", "nearest_waypoint_distance_m",
        ]
        for index in range(1, len(self.waypoints) + 1):
            self.columns.append("distance_wp%d_m" % index)
        self.columns += [
            "controller_target_waypoint", "controller_last_log",
            "cmd_age_s", "cmd_vx_mps", "cmd_wz_radps",
            "fusion_age_s", "fusion_mode", "fusion_cloud_age_s",
            "fusion_gyro_age_s", "fusion_imu_age_s", "fusion_ndt_age_s",
            "fusion_wheel_age_s", "hunter_odom_age_s", "hunter_odom_x_m",
            "hunter_odom_y_m", "hunter_odom_yaw_rad", "hunter_odom_vx_mps",
            "hunter_odom_wz_radps", "hunter_status_age_s",
            "hunter_linear_velocity_mps", "hunter_steering_angle_rad",
            "hunter_vehicle_state", "hunter_control_mode", "hunter_error_code",
            "hunter_battery_voltage_v", "motor_rpm", "motor_current_a",
            "motor_driver_state",
        ]
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.columns)
        self.csv_writer.writeheader()

        topics = {
            "localization": "/localization/kinematic_state",
            "fusion": "/localization/fusion_status",
            "cmd_vel": "/cmd_vel",
            "hunter_odom": "/hunter_odom",
            "hunter_status": "/hunter_status",
            "rviz_initial_pose": "/initialpose",
            "localization_initial_pose": "/localization/initialpose",
            "controller_logs": "/rosout",
        }
        metadata = {
            "started_utc": utc_now(),
            "hostname": socket.gethostname(),
            "ros_domain_id": __import__("os").environ.get("ROS_DOMAIN_ID", ""),
            "route_config": str(self.route_path),
            "waypoints_map_xy_m": [
                {"waypoint": i + 1, "x": x, "y": y}
                for i, (x, y) in enumerate(self.waypoints)
            ],
            "sample_rate_hz": sample_rate_hz,
            "topics": topics,
            "bag_topics": [
                "/localization/kinematic_state", "/localization/fusion_status",
                "/localization/pose_estimator/pose_with_covariance",
                "/localization/pose_estimator/transform_probability",
                "/cmd_vel", "/hunter_odom", "/hunter_status", "/initialpose",
                "/localization/initialpose", "/rosout", "/tf", "/tf_static",
            ],
        }
        (self.output_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        self.create_subscription(Odometry, topics["localization"], self.on_localization, 50)
        self.create_subscription(String, topics["fusion"], self.on_fusion, 10)
        self.create_subscription(Twist, topics["cmd_vel"], self.on_cmd_vel, 50)
        self.create_subscription(Odometry, topics["hunter_odom"], self.on_hunter_odom, 50)
        self.create_subscription(HunterStatus, topics["hunter_status"], self.on_hunter_status, 20)
        self.create_subscription(PoseWithCovarianceStamped, topics["rviz_initial_pose"],
                                 lambda msg: self.on_initial_pose(msg, topics["rviz_initial_pose"]), 10)
        self.create_subscription(PoseWithCovarianceStamped, topics["localization_initial_pose"],
                                 lambda msg: self.on_initial_pose(msg, topics["localization_initial_pose"]), 10)
        self.create_subscription(Log, topics["controller_logs"], self.on_rosout, 200)
        self.timer = self.create_timer(1.0 / sample_rate_hz, self.write_sample)
        self.record_event("recorder_started", {"output_dir": str(self.output_dir)})

    def record_event(self, event_type, fields):
        event = {"time_utc": utc_now(), "event": event_type}
        event.update(fields)
        self.events_file.write(json.dumps(event, ensure_ascii=False, default=str) + "\n")

    def set_latest(self, name, values):
        values["received_monotonic"] = time.monotonic()
        self.latest[name] = values

    def on_localization(self, msg):
        p = msg.pose.pose.position
        t = msg.twist.twist
        self.set_latest("localization", {
            "stamp_ns": stamp_ns(msg.header.stamp), "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id, "x": p.x, "y": p.y, "z": p.z,
            "yaw": yaw_from_quaternion(msg.pose.pose.orientation),
            "vx": t.linear.x, "wz": t.angular.z,
        })

    def on_fusion(self, msg):
        try:
            data = json.loads(msg.data)
            mode = data.get("mode", "")
            ages = data.get("ages_sec", {})
        except (TypeError, ValueError):
            mode, ages = "INVALID", {}
        self.set_latest("fusion", {"mode": mode, "ages": ages})
        if mode != self.last_fusion_mode:
            self.record_event("fusion_mode", {"mode": mode, "ages_sec": ages})
            self.last_fusion_mode = mode

    def on_cmd_vel(self, msg):
        self.set_latest("cmd_vel", {"vx": msg.linear.x, "wz": msg.angular.z})

    def on_hunter_odom(self, msg):
        p = msg.pose.pose.position
        t = msg.twist.twist
        self.set_latest("hunter_odom", {
            "stamp_ns": stamp_ns(msg.header.stamp), "x": p.x, "y": p.y,
            "yaw": yaw_from_quaternion(msg.pose.pose.orientation),
            "vx": t.linear.x, "wz": t.angular.z,
        })

    def on_hunter_status(self, msg):
        actuators = list(msg.actuator_states)
        self.set_latest("hunter_status", {
            "linear_velocity": msg.linear_velocity,
            "steering_angle": msg.steering_angle,
            "vehicle_state": msg.vehicle_state,
            "control_mode": msg.control_mode,
            "error_code": msg.error_code,
            "battery_voltage": msg.battery_voltage,
            "motor_rpm": [item.rpm for item in actuators],
            "motor_current": [item.current for item in actuators],
            "driver_state": [item.driver_state for item in actuators],
        })

    def on_initial_pose(self, msg, topic):
        p = msg.pose.pose.position
        self.record_event("initial_pose", {
            "topic": topic, "frame_id": msg.header.frame_id,
            "stamp_ns": stamp_ns(msg.header.stamp), "x": p.x, "y": p.y,
        })

    def on_rosout(self, msg):
        name = msg.name or ""
        text = msg.msg or ""
        if "hunter_pure_pursuit" not in name and "waypoint" not in text.lower():
            return
        self.last_controller_log = text
        self.controller_file.write(
            "[%s] [ROS %s] [%s] [level=%d] %s\n"
            % (utc_now(), stamp_ns(msg.stamp), name, msg.level, text)
        )
        target_match = re.search(r"tracking waypoint\s+(\d+)", text, re.IGNORECASE)
        if target_match:
            self.target_waypoint = target_match.group(1)
        event_type = "controller_log"
        recovery = re.search(
            r"passed waypoint\s+(\d+)\s*\(([0-9.eE+-]+)\s*m away\).*?tracking waypoint\s+(\d+)",
            text, re.IGNORECASE,
        )
        reached = re.search(r"reached waypoint\s+(\d+)", text, re.IGNORECASE)
        if recovery:
            event_type = "route_progress_recovery"
        elif reached:
            event_type = "waypoint_reached"
        self.record_event(event_type, {
            "ros_stamp_ns": stamp_ns(msg.stamp), "logger": name,
            "level": msg.level, "message": text,
            "passed_waypoint": int(recovery.group(1)) if recovery else None,
            "distance_m": float(recovery.group(2)) if recovery else None,
            "tracking_waypoint": int(recovery.group(3)) if recovery else
                (int(target_match.group(1)) if target_match else None),
        })

    @staticmethod
    def age_of(item):
        if not item:
            return ""
        return round(max(0.0, time.monotonic() - item["received_monotonic"]), 4)

    def write_sample(self):
        row = {column: "" for column in self.columns}
        row["sample_utc"] = utc_now()
        loc = self.latest.get("localization")
        if loc:
            row.update({
                "localization_stamp_ns": loc["stamp_ns"],
                "localization_age_s": self.age_of(loc),
                "frame_id": loc["frame_id"], "child_frame_id": loc["child_frame_id"],
                "map_x_m": loc["x"], "map_y_m": loc["y"], "map_z_m": loc["z"],
                "map_yaw_rad": loc["yaw"], "localization_vx_mps": loc["vx"],
                "localization_wz_radps": loc["wz"],
            })
            distances = [math.hypot(x - loc["x"], y - loc["y"]) for x, y in self.waypoints]
            if distances:
                nearest = min(range(len(distances)), key=distances.__getitem__)
                row["nearest_waypoint"] = nearest + 1
                row["nearest_waypoint_distance_m"] = distances[nearest]
                for index, distance in enumerate(distances, start=1):
                    row["distance_wp%d_m" % index] = distance
        row["controller_target_waypoint"] = self.target_waypoint
        row["controller_last_log"] = self.last_controller_log

        cmd = self.latest.get("cmd_vel")
        if cmd:
            row.update({"cmd_age_s": self.age_of(cmd), "cmd_vx_mps": cmd["vx"],
                        "cmd_wz_radps": cmd["wz"]})
        fusion = self.latest.get("fusion")
        if fusion:
            ages = fusion.get("ages", {})
            row.update({
                "fusion_age_s": self.age_of(fusion), "fusion_mode": fusion.get("mode", ""),
                "fusion_cloud_age_s": ages.get("cloud", ""),
                "fusion_gyro_age_s": ages.get("gyro", ""),
                "fusion_imu_age_s": ages.get("imu", ""),
                "fusion_ndt_age_s": ages.get("ndt", ""),
                "fusion_wheel_age_s": ages.get("wheel", ""),
            })
        odom = self.latest.get("hunter_odom")
        if odom:
            row.update({
                "hunter_odom_age_s": self.age_of(odom), "hunter_odom_x_m": odom["x"],
                "hunter_odom_y_m": odom["y"], "hunter_odom_yaw_rad": odom["yaw"],
                "hunter_odom_vx_mps": odom["vx"], "hunter_odom_wz_radps": odom["wz"],
            })
        status = self.latest.get("hunter_status")
        if status:
            row.update({
                "hunter_status_age_s": self.age_of(status),
                "hunter_linear_velocity_mps": status["linear_velocity"],
                "hunter_steering_angle_rad": status["steering_angle"],
                "hunter_vehicle_state": status["vehicle_state"],
                "hunter_control_mode": status["control_mode"],
                "hunter_error_code": status["error_code"],
                "hunter_battery_voltage_v": status["battery_voltage"],
                "motor_rpm": ";".join(map(str, status["motor_rpm"])),
                "motor_current_a": ";".join(map(str, status["motor_current"])),
                "motor_driver_state": ";".join(map(str, status["driver_state"])),
            })
        self.csv_writer.writerow(row)
        self.sample_count += 1
        if self.sample_count % 20 == 0:
            self.csv_file.flush()
            self.events_file.flush()
        if self.sample_count % 100 == 0:
            pose_text = "waiting for localization"
            if loc:
                pose_text = "map=(%.2f, %.2f) nearest=WP%d %.2fm" % (
                    loc["x"], loc["y"], row["nearest_waypoint"],
                    row["nearest_waypoint_distance_m"],
                )
            print("[monitor] %d samples; %s; target=WP%s; fusion=%s" % (
                self.sample_count, pose_text, self.target_waypoint or "?",
                fusion.get("mode", "?") if fusion else "?",
            ), flush=True)

    def close(self):
        self.record_event("recorder_stopped", {"samples": self.sample_count})
        self.csv_file.flush()
        self.events_file.flush()
        self.controller_file.flush()
        self.csv_file.close()
        self.events_file.close()
        self.controller_file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--sample-rate-hz", type=float, default=20.0)
    args = parser.parse_args()
    if args.sample_rate_hz <= 0.0:
        parser.error("--sample-rate-hz must be positive")

    rclpy.init()
    node = Waypoint4Diagnostics(args.output_dir, args.sample_rate_hz)
    print("[monitor] Recording locally to %s" % node.output_dir, flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
        print("[monitor] ROS context shut down; finishing recorder cleanup.", flush=True)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
