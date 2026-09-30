#!/usr/bin/env python3
"""Read-only ROS 2 sqlite3/PCD audit using only Python's standard library.

Exports the recorded wheel-odometry trajectory; this is NOT map localization.
The CDR reader supports the standard XCDR1 messages used in these bags.
"""
from __future__ import annotations

import argparse
import collections
import csv
import datetime
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import statistics
import struct
import zipfile


class Cdr:
    def __init__(self, data):
        if data[:2] not in (b"\x00\x00", b"\x00\x01"):
            raise ValueError("Only XCDR1 CDR encapsulation is supported")
        self.data, self.pos = data, 4
        self.endian = "<" if data[1] == 1 else ">"

    def scalar(self, fmt, alignment):
        self.pos += (-(self.pos - 4)) % alignment
        value = struct.unpack_from(self.endian + fmt, self.data, self.pos)[0]
        self.pos += struct.calcsize(fmt)
        return value

    def string(self):
        size = self.scalar("I", 4)
        value = self.data[self.pos:self.pos + size]
        self.pos += size
        if not value or value[-1] != 0:
            raise ValueError("Invalid CDR string")
        return value[:-1].decode("utf8")

    def doubles(self, size):
        return [self.scalar("d", 8) for _ in range(size)]

    def header(self):
        return {"stamp_ns": self.scalar("i", 4) * 1_000_000_000 + self.scalar("I", 4),
                "frame_id": self.string()}


def decode(data, msgtype):
    r = Cdr(data)
    if msgtype == "tf2_msgs/msg/TFMessage":
        result = []
        for _ in range(r.scalar("I", 4)):
            result.append(dict(r.header(), child_frame_id=r.string(),
                               translation=r.doubles(3), rotation=r.doubles(4)))
    elif msgtype == "nav_msgs/msg/Odometry":
        result = dict(r.header(), child_frame_id=r.string(), position=r.doubles(3),
                      orientation=r.doubles(4), pose_covariance=r.doubles(36),
                      linear_velocity=r.doubles(3), angular_velocity=r.doubles(3),
                      twist_covariance=r.doubles(36))
    elif msgtype == "sensor_msgs/msg/Imu":
        result = dict(r.header(), orientation=r.doubles(4), orientation_covariance=r.doubles(9),
                      angular_velocity=r.doubles(3), angular_velocity_covariance=r.doubles(9),
                      linear_acceleration=r.doubles(3), linear_acceleration_covariance=r.doubles(9))
    else:
        return None
    if r.pos != len(data):
        raise ValueError(f"{msgtype}: decoded {r.pos} of {len(data)} bytes")
    return result


def norm(values):
    return math.sqrt(sum(v * v for v in values))


def summary(values):
    if not values:
        return None
    return {"min": min(values), "max": max(values), "mean": statistics.mean(values),
            "median": statistics.median(values), "stddev": statistics.pstdev(values)}


def vector_summary(rows):
    return {axis: summary(list(values)) for axis, values in zip("xyz", zip(*rows))}


def timing(stamps):
    deltas = [(b-a)/1e9 for a, b in zip(stamps, stamps[1:])]
    duration = (stamps[-1] - stamps[0])/1e9 if stamps else 0
    return {"first_ns": stamps[0] if stamps else None, "last_ns": stamps[-1] if stamps else None,
            "duration_s": duration, "rate_hz": (len(stamps)-1)/duration if duration > 0 else None,
            "delta_s": summary(deltas), "backward_steps": sum(d < 0 for d in deltas),
            "gaps_over_100ms": sum(d > .1 for d in deltas),
            "gaps_over_1s": sum(d > 1 for d in deltas)}


def sha256(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def audit_db(path, output):
    connection = sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True)
    result = {"name": path.parent.name, "database": str(path.resolve()), "bytes": path.stat().st_size,
              "sha256": sha256(path), "sqlite_integrity": connection.execute("PRAGMA quick_check").fetchone()[0],
              "topics": [], "tf_edges": []}
    tf = collections.defaultdict(list)
    counts = connection.execute("SELECT COUNT(*),MIN(timestamp),MAX(timestamp) FROM messages").fetchone()
    result.update(message_count=counts[0], start_ns=counts[1], end_ns=counts[2], duration_s=(counts[2]-counts[1])/1e9)
    for topic_id, name, msgtype, serialization in connection.execute("SELECT id,name,type,serialization_format FROM topics ORDER BY name"):
        rows = connection.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id", (topic_id,)).fetchall()
        info = {"name": name, "type": msgtype, "serialization": serialization, "count": len(rows),
                "recording_timing": timing([row[0] for row in rows])}
        messages = [decode(data, msgtype) for _, data in rows]
        if msgtype == "tf2_msgs/msg/TFMessage":
            for (received, _), transforms in zip(rows, messages):
                for transform in transforms:
                    tf[(name, transform["frame_id"], transform["child_frame_id"])].append(dict(transform, received_ns=received))
        elif messages and messages[0] is not None:
            info["header_timing"] = timing([m["stamp_ns"] for m in messages])
            info["recording_minus_header_s"] = summary([(row[0]-m["stamp_ns"])/1e9 for row, m in zip(rows, messages)])
            info["frame_ids"] = sorted({m["frame_id"] for m in messages})
            info["orientation_norm"] = summary([norm(m["orientation"]) for m in messages])
            info["angular_velocity"] = vector_summary([m["angular_velocity"] for m in messages])
            info["first_message"] = messages[0]
            info["last_message"] = messages[-1]
            if msgtype == "sensor_msgs/msg/Imu":
                info["linear_acceleration"] = vector_summary([m["linear_acceleration"] for m in messages])
                info["linear_acceleration_norm"] = summary([norm(m["linear_acceleration"]) for m in messages])
                for key in ("orientation_covariance", "angular_velocity_covariance", "linear_acceleration_covariance"):
                    info[key + "_all_zero"] = all(all(v == 0 for v in m[key]) for m in messages)
            else:
                info["child_frame_ids"] = sorted({m["child_frame_id"] for m in messages})
                info["linear_velocity"] = vector_summary([m["linear_velocity"] for m in messages])
                info["position"] = vector_summary([m["position"] for m in messages])
                info["path_length_m"] = sum(math.dist(a["position"], b["position"]) for a, b in zip(messages, messages[1:]))
                info["endpoint_displacement_m"] = math.dist(messages[0]["position"], messages[-1]["position"])
                for key in ("pose_covariance", "twist_covariance"):
                    info[key + "_all_zero"] = all(all(v == 0 for v in m[key]) for m in messages)
                csv_path = output / (path.parent.name + "_recorded_wheel_odom.csv")
                with csv_path.open("w", newline="", encoding="utf8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(["stamp_ns", "frame_id", "child_frame_id", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "vx_mps", "vy_mps", "vz_mps", "wx_radps", "wy_radps", "wz_radps"])
                    for m in messages:
                        writer.writerow([m["stamp_ns"], m["frame_id"], m["child_frame_id"], *m["position"], *m["orientation"], *m["linear_velocity"], *m["angular_velocity"]])
                info["recorded_wheel_odom_csv"] = csv_path.name
        result["topics"].append(info)
    for (topic, parent, child), values in sorted(tf.items()):
        result["tf_edges"].append({"topic": topic, "parent": parent, "child": child, "count": len(values),
                                   "header_timing": timing([v["stamp_ns"] for v in values]),
                                   "first": values[0], "last": values[-1],
                                   "distinct_values": len({tuple(v["translation"] + v["rotation"]) for v in values})})
    result["has_pointcloud"] = any(t["type"] in ("sensor_msgs/msg/PointCloud2", "livox_ros_driver2/msg/CustomMsg", "livox_ros_driver/msg/CustomMsg") for t in result["topics"])
    connection.close()
    return result


def audit_pcd(path):
    header = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline().decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            header[parts[0]] = parts[1:]
            if parts[0] == "DATA":
                break
        offset = stream.tell()
        if header["DATA"] != ["binary"] or any(t != "F" for t in header["TYPE"]) or any(s != "4" for s in header["SIZE"]) or any(c != "1" for c in header["COUNT"]):
            return {"name": path.name, "header": header, "bounds_status": "Unsupported PCD encoding"}
        fields = header["FIELDS"]
        point_format = struct.Struct("<" + "f" * len(fields))
        mins, maxs, sums, valid = [math.inf] * 3, [-math.inf] * 3, [0.] * 3, 0
        indices = [fields.index(a) for a in "xyz"]
        for point in struct.iter_unpack(point_format.format, stream.read()):
            xyz = [point[i] for i in indices]
            if not all(math.isfinite(v) for v in xyz):
                continue
            valid += 1
            for i, value in enumerate(xyz):
                mins[i], maxs[i], sums[i] = min(mins[i], value), max(maxs[i], value), sums[i] + value
    return {"name": path.name, "sha256": sha256(path), "bytes": path.stat().st_size,
            "header": header, "payload_bytes": path.stat().st_size-offset,
            "expected_payload_bytes": int(header["POINTS"][0])*point_format.size, "finite_xyz_points": valid,
            "xyz_min_m": mins, "xyz_max_m": maxs, "xyz_mean_m": [v / valid for v in sums]}


def audit_zip(path, bags):
    if not path.exists():
        return None
    with zipfile.ZipFile(path) as archive:
        entries = []
        for item in archive.infolist():
            if item.is_dir():
                continue
            relative = Path(item.filename)
            local = bags.parent / relative
            h = hashlib.sha256()
            with archive.open(item) as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    h.update(chunk)
            digest = h.hexdigest()
            entries.append({"name": item.filename, "bytes": item.file_size, "sha256": digest,
                            "local_exists": local.exists(),
                            "matches_extracted_file": local.exists() and sha256(local) == digest})
    return {"name": path.name, "entries": entries,
            "all_entries_match_extracted_files": all(e["matches_extracted_file"] for e in entries)}


def write_markdown(report, path):
    lines = ["# Bag 数据审计（直接读取 SQLite）", "", "## 结论", "",
             "提供的两个录包都没有 PointCloud2、Livox CustomMsg、LaserScan 或 GNSS 观测，只有 IMU、轮式里程计与 TF。",
             "不能用这些数据验证 NDT/ICP 点云地图定位或在 PCD 地图中的绝对位置。可验证消息适配、轮式里程计轨迹与相对航位推算。",
             "导出的 CSV 是原始轮式里程计记录，不是定位算法或地图匹配结果。", "",
             "## 录包", "", "| 录包 | 消息数 | 录制时长(s) | topic | 消息数 | 首尾平均频率(Hz) | 最大录制间隔(s) |", "|---|---:|---:|---|---:|---:|---:|"]
    for bag in report["bags"]:
        for t in bag["topics"]:
            tm = t["recording_timing"]
            lines.append(f'| {bag["name"]} | {bag["message_count"]} | {bag["duration_s"]:.3f} | {t["name"]} | {t["count"]} | {tm["rate_hz"] or 0:.3f} | {(tm["delta_s"] or {}).get("max",0):.6f} |')
        lines.extend(["", f'### {bag["name"]} 帧与传感器', ""])
        for t in bag["topics"]:
            if t["type"] == "sensor_msgs/msg/Imu":
                lines.extend([f'- IMU frame: `{t["frame_ids"]}`; 四元数模长范围 {t["orientation_norm"]["min"]:.6g}–{t["orientation_norm"]["max"]:.6g}。',
                              f'- IMU 加速度模长均值 {t["linear_acceleration_norm"]["mean"]:.6g}，最大 {t["linear_acceleration_norm"]["max"]:.6g}；需确认驱动单位是否为 g 并转换为 m/s²，不能仅凭数值自动认定。',
                              f'- IMU header 最大间隔 {t["header_timing"]["delta_s"]["max"]:.6f}s，超过 1s 的间隔 {t["header_timing"]["gaps_over_1s"]} 个；全零协方差表示未知，不能当作零噪声。'])
            elif t["type"] == "nav_msgs/msg/Odometry":
                lines.extend([f'- Odom `{t["frame_ids"]}` → `{t["child_frame_ids"]}`; 路程 {t["path_length_m"]:.3f}m，首尾距离 {t["endpoint_displacement_m"]:.3f}m。',
                              f'- 轮式里程计 CSV: `{t["recorded_wheel_odom_csv"]}`。'])
        lines += ["", "| TF topic | parent | child | 数量 | 不同变换数量 | 首条平移 | 首条四元数(xyzw) |", "|---|---|---|---:|---:|---|---|"]
        for tf in bag["tf_edges"]:
            lines.append(f'| {tf["topic"]} | {tf["parent"]} | {tf["child"]} | {tf["count"]} | {tf["distinct_values"]} | {tf["first"]["translation"]} | {tf["first"]["rotation"]} |')
        lines += [""]
    lines.extend(["## PCD 地图", "", "| 文件 | 点数 | 字段 | xyz 最小(m) | xyz 最大(m) |", "|---|---:|---|---|---|"])
    for p in report["pcd_maps"]:
        lines.append(f'| {p["name"]} | {p["header"]["POINTS"][0]} | {", ".join(p["header"]["FIELDS"])} | {p.get("xyz_min_m")} | {p.get("xyz_max_m")} |')
    lines += ["", "PCD VIEWPOINT 不是 `map` 与 `odom` 的已知标定，地图文件中也没有 ROS frame_id。原图与 level 图的坐标系需分别确认；不要通过平移地图高度假装完成标定。", "",
              "## 补录与回放契约", "", "1. 必须补录与 IMU/里程计同步的实际雷达扫描：优先 `sensor_msgs/msg/PointCloud2`；如果是 Livox CustomMsg，需保留对应消息定义并转换。点字段至少包含 x/y/z，NDT 使用强度字段时显式补齐或适配。",
              "2. 确认雷达和 IMU 到 base_link 的真实外参，统一点云、IMU、里程计 frame_id；不能仅因为名称相似而复用外参。",
              "3. 使用同一 ROS 时间域和 `/clock`，检查传感器中断与时间戳单调性。第二段 IMU 尤其需要排查长间断。",
              "4. 地图定位需要可信初始位姿（该 PCD 坐标系中）或已验证的全局重定位算法，并评估匹配分数、连续性和误差。没有地面真值不能报告定位精度。",
              "5. 保留原始 TF 审计；正式启动定位器时避免多个节点发布同一个 map→odom 或 map→base_link 变换。", "", "## ZIP 完整性", ""]
    z = report["zip"]
    lines.append(f'ZIP 内 {len(z["entries"])} 个文件逐个 SHA-256 与已解压文件一致：{z["all_entries_match_extracted_files"]}。未发现额外点云录包。' if z else "未找到 ZIP。")
    lines.extend(["", "完整数值、首末消息、协方差、时间统计、文件哈希见 `bag_audit.json`。", ""])
    path.write_text("\n".join(lines), encoding="utf8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "reports")
    parser.add_argument("--zip", type=Path)
    args = parser.parse_args()
    if not args.bags.is_dir():
        parser.error(f"Bag directory does not exist: {args.bags}")
    args.output.mkdir(parents=True, exist_ok=True)
    databases = sorted(args.bags.rglob("*.db3"))
    if not databases:
        parser.error("No sqlite3 .db3 recordings found")
    report = {"generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "source": str(args.bags.resolve()), "method": "read-only SQLite + XCDR1 standard message decoding; no ROS or third-party Python requirements",
              "map_localization_verified": False,
              "bags": [audit_db(db, args.output) for db in databases],
              "pcd_maps": [audit_pcd(p) for p in sorted(args.bags.glob("*.pcd"))],
              "zip": audit_zip(args.zip or args.bags.parent / "bags.zip", args.bags)}
    (args.output / "bag_audit.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf8")
    write_markdown(report, args.output / "bag_audit.md")
    print(json.dumps({"report": str(args.output / "bag_audit.md"), "bags": len(report["bags"]),
                      "messages": sum(b["message_count"] for b in report["bags"]),
                      "has_pointcloud": any(b["has_pointcloud"] for b in report["bags"]),
                      "map_localization_verified": False}, ensure_ascii=False))


if __name__ == "__main__":
    main()
