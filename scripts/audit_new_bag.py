#!/usr/bin/env python3
"""Audit the new lidar recording and export actual scans/maps for registration.

Requires numpy; ROS is optional and used only for decoder cross-checking.
Outputs are observations and initial-pose candidates, not localization results.
"""
import argparse
import datetime
import json
from pathlib import Path
import sqlite3
import struct

import numpy as np

from audit_bags import Cdr, audit_db, decode, sha256, summary, timing


def pointcloud2(data):
    r = Cdr(data)
    result = r.header()
    result.update(height=r.scalar("I", 4), width=r.scalar("I", 4))
    result["fields"] = [dict(name=r.string(), offset=r.scalar("I", 4),
                             datatype=r.scalar("B", 1), count=r.scalar("I", 4))
                        for _ in range(r.scalar("I", 4))]
    result.update(is_bigendian=bool(r.scalar("B", 1)), point_step=r.scalar("I", 4), row_step=r.scalar("I", 4))
    size = r.scalar("I", 4)
    payload = data[r.pos:r.pos+size]
    r.pos += size
    result["is_dense"] = bool(r.scalar("B", 1))
    if r.pos != len(data):
        raise ValueError("Unexpected PointCloud2 CDR length")
    if len(payload) != result["row_step"] * result["height"]:
        raise ValueError("PointCloud2 payload size does not match dimensions")
    fmt = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    endian = ">" if result["is_bigendian"] else "<"
    dtype = np.dtype({"names": [f["name"] for f in result["fields"]],
                      "formats": [np.dtype(endian + fmt[f["datatype"]]) if f["count"] == 1
                                  else (np.dtype(endian + fmt[f["datatype"]]), (f["count"],)) for f in result["fields"]],
                      "offsets": [f["offset"] for f in result["fields"]], "itemsize": result["point_step"]})
    rows = np.ndarray((result["height"], result["width"]), dtype=dtype, buffer=payload,
                      strides=(result["row_step"], result["point_step"]))
    columns = ["x", "y", "z"] + (["intensity"] if "intensity" in dtype.names else [])
    points = np.column_stack([rows[name].reshape(-1) for name in columns]).astype(np.float32)
    result.update(point_count=len(points), columns=columns, finite_xyz_points=int(np.isfinite(points[:, :3]).all(axis=1).sum()))
    return result, points


def livox_metadata(data):
    # Standard livox_ros_driver2 CustomMsg/CustomPoint wire layout.
    r = Cdr(data)
    result = r.header()
    result.update(timebase=r.scalar("Q", 8), point_num=r.scalar("I", 4), lidar_id=r.scalar("B", 1))
    result["reserved"] = [r.scalar("B", 1) for _ in range(3)]
    count = r.scalar("I", 4)
    result["sequence_point_count"] = count
    if count != result["point_num"]:
        raise ValueError("CustomMsg point_num != sequence length")
    # Each point has 4-byte alignment and 19 bytes of fields.
    expected = count * 20 - (1 if count else 0)
    if len(data)-r.pos != expected:
        raise ValueError(f"Unexpected CustomPoint payload: {len(data)-r.pos} vs {expected}")
    return result


def lzf_decompress(data, expected):
    output = bytearray()
    cursor = 0
    while cursor < len(data):
        control = data[cursor]
        cursor += 1
        if control < 32:
            size = control + 1
            output.extend(data[cursor:cursor+size])
            cursor += size
        else:
            size = control >> 5
            reference = len(output) - ((control & 31) << 8) - 1
            if size == 7:
                size += data[cursor]
                cursor += 1
            reference -= data[cursor]
            cursor += 1
            for index in range(size + 2):
                output.append(output[reference + index])
    if len(output) != expected:
        raise ValueError(f"LZF output {len(output)} != {expected}")
    return output


def load_pcd(path):
    path = Path(path)
    with path.open("rb") as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("Missing PCD DATA header")
            if line.startswith(b"#") or not line.strip():
                continue
            key, *values = line.decode("ascii").split()
            header[key] = values
            if key == "DATA":
                break
        count = int(header["POINTS"][0])
        fields = header["FIELDS"]
        if set(header["TYPE"]) != {"F"} or set(header["SIZE"]) != {"4"} or set(header["COUNT"]) != {"1"}:
            raise ValueError("Only scalar float32 PCD fields supported")
        if header["DATA"] == ["binary_compressed"]:
            compressed, uncompressed = struct.unpack("<II", stream.read(8))
            raw = lzf_decompress(stream.read(compressed), uncompressed)
            points = np.frombuffer(raw, dtype="<f4").reshape(len(fields), count).T.copy()
        elif header["DATA"] == ["binary"]:
            points = np.frombuffer(stream.read(), dtype="<f4").reshape(count, len(fields)).copy()
        elif header["DATA"] == ["ascii"]:
            points = np.loadtxt(stream, dtype=np.float32).reshape(count, len(fields))
        else:
            raise ValueError("Unsupported PCD DATA encoding")
    if points.shape != (count, len(fields)):
        raise ValueError("PCD field/point count mismatch")
    return header, points


def save_pcd(path, points, fields):
    header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS " + " ".join(fields) +
              "\nSIZE " + " ".join(["4"]*len(fields)) + "\nTYPE " + " ".join(["F"]*len(fields)) +
              "\nCOUNT " + " ".join(["1"]*len(fields)) + f"\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n")
    with Path(path).open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(np.asarray(points, dtype="<f4").tobytes())


def transform_matrix(transform):
    x, y, z, w = transform["rotation"]
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = transform["translation"]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--bag-name", default="hunter_sensors_20260930_104649")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1]/"reports")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    assets = args.output / "new_bag_assets"
    assets.mkdir(exist_ok=True)
    database = next((args.data_root / "bags" / args.bag_name).glob("*.db3"))
    report = {"generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "bag": audit_db(database, args.output), "maps": [], "scan_samples": [],
              "map_localization_verified": False, "initial_pose_candidates": [],
              "notes": ["Point clouds are available in the new recording; old no-pointcloud conclusion applies only to the 20260929 recordings.",
                        "No calibration/configuration file accompanies GlobalMap_loc.pcd or GlobalMap_mid.pcd.",
                        "Recorded TF provides pose hypotheses, not truth; neither PCD embeds ROS frame identity.",
                        "body is connected to camera_init by recorded dynamic TF. No verified rigid body-to-base_link or livox_frame-to-body calibration was provided."]}
    c = sqlite3.connect(database.resolve().as_uri()+"?mode=ro", uri=True)
    edges = {}
    for raw, in c.execute("SELECT data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name='/tf') ORDER BY timestamp"):
        for tf in decode(raw, "tf2_msgs/msg/TFMessage"):
            edges.setdefault((tf["frame_id"], tf["child_frame_id"]), []).append(tf)
    for topic in report["bag"]["topics"]:
        if topic["type"] not in ("sensor_msgs/msg/PointCloud2", "livox_ros_driver2/msg/CustomMsg"):
            continue
        records = c.execute("SELECT timestamp,data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name=?) ORDER BY timestamp", (topic["name"],)).fetchall()
        metadata, samples = [], []
        sample_indices = set(np.linspace(0, len(records)-1, 10, dtype=int).tolist())
        for index, (received, raw) in enumerate(records):
            if topic["type"] == "sensor_msgs/msg/PointCloud2":
                info, points = pointcloud2(raw)
                if index in sample_indices:
                    stem = f"scan_{index:04d}_{info['stamp_ns']}"
                    np.save(assets / (stem+".npy"), points)
                    save_pcd(assets / (stem+".pcd"), points, info["columns"])
                    sample = dict(index=index, received_ns=received, **info, npy=stem+".npy", pcd=stem+".pcd")
                    samples.append(sample)
                    report["scan_samples"].append(sample)
                    body_tfs = edges.get(("camera_init", "body"), [])
                    map_tfs = edges.get(("map", "camera_init"), [])
                    if body_tfs:
                        body = min(body_tfs, key=lambda tf: abs(tf["stamp_ns"]-info["stamp_ns"]))
                        candidate = {"scan_index": index, "scan_stamp_ns": info["stamp_ns"], "camera_init_T_body": transform_matrix(body).tolist(),
                                     "body_tf_time_difference_s": (body["stamp_ns"]-info["stamp_ns"])/1e9, "map_correspondence_verified": False}
                        if map_tfs:
                            mt = min(map_tfs, key=lambda tf: abs(tf["stamp_ns"]-info["stamp_ns"]))
                            candidate.update(map_T_body=(transform_matrix(mt)@transform_matrix(body)).tolist(),
                                             map_tf_time_difference_s=(mt["stamp_ns"]-info["stamp_ns"])/1e9)
                        report["initial_pose_candidates"].append(candidate)
            else:
                info = livox_metadata(raw)
            metadata.append(info)
        topic.update(frame_ids=sorted({m["frame_id"] for m in metadata}), header_timing=timing([m["stamp_ns"] for m in metadata]),
                     recording_minus_header_s=summary([(row[0]-m["stamp_ns"])/1e9 for row,m in zip(records,metadata)]),
                     point_count=summary([m.get("point_count", m.get("point_num")) for m in metadata]),
                     first_message=metadata[0], last_message=metadata[-1])
    c.close()
    for path in sorted(args.data_root.glob("*.pcd")):
        header, points = load_pcd(path)
        stem = path.stem
        np.save(assets/(stem+".npy"), points)
        save_pcd(assets/(stem+"_binary.pcd"), points, header["FIELDS"])
        valid = np.isfinite(points[:, :3]).all(axis=1)
        report["maps"].append({"name": path.name, "source": str(path.resolve()), "sha256": sha256(path),
                                "header": header, "finite_xyz_points": int(valid.sum()), "xyz_min": points[valid,:3].min(axis=0).tolist(),
                                "xyz_max": points[valid,:3].max(axis=0).tolist(), "xyz_mean": points[valid,:3].mean(axis=0).tolist(),
                                "xyz_percentile_1_50_99": np.percentile(points[valid,:3], [1,50,99], axis=0).tolist(),
                                "npy": stem+".npy", "binary_pcd": stem+"_binary.pcd", "columns": header["FIELDS"]})
    (args.output/"new_bag_audit.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf8")
    lines = ["# 新定位录包审计", "", "本报告仅针对 2026-09-30 新录包，旧报告保留。新录包有真实点云，可开展 PCD 地图匹配；本审计本身不代表定位成功。", "",
             "| 话题 | 类型 | 数量 | Frame | header 平均 Hz |", "|---|---|---:|---|---:|"]
    for t in report["bag"]["topics"]:
        lines.append(f'| {t["name"]} | {t["type"]} | {t["count"]} | {", ".join(t.get("frame_ids",[]))} | {t.get("header_timing",{}).get("rate_hz",0) or 0:.3f} |')
    lines += ["", "`/cloud_registered_body` 为 body 坐标系中的 PointCloud2；这是一份已处理扫描，可直接测试 body 在地图中的位姿匹配。原始 `/livox/lidar` 是 livox_frame 中的 CustomMsg。", "",
              "## 地图", "", "| 文件 | 点数 | 字段 | xyz 最小 | xyz 最大 |", "|---|---:|---|---|---|"]
    for m in report["maps"]:
        lines.append(f'| {m["name"]} | {m["header"]["POINTS"][0]} | {", ".join(m["columns"])} | {m["xyz_min"]} | {m["xyz_max"]} |')
    lines += ["", "地图解压后的原坐标及 10 帧真实扫描已导出到 `new_bag_assets/`，每份都有 NPY 和 binary PCD；转换没有移动、旋转或裁剪点。", "",
              "## 初值与外参证据", "", "记录中有 `map → camera_init → body` 动态 TF，可以作为匹配初值候选；不能当作当前定位结果、真值或地图 frame 身份证明。", "",
              "`new_bag_audit.json` 中 initial_pose_candidates 提供每份扫描附近的 TF、时间差和组合矩阵。接近录包末尾的 body TF 滞后较大，必须检查时间差。", "",
              "随附文件没有标定文档或 GlobalMap 地图 frame 配置，无法从文件名认定 loc/mid 对应哪个 TF 世界。必须分别进行实际匹配并比较重叠率/残差。", "",
              "现有静态 TF 只包含 base_footprint 到 base_link/mid360_link/超声传感器；livox_frame 缺少 TF，hunter_base_link 和 base_link 也没有已验证刚性连接。暂时输出 body 定位，不伪造车体外参。", "",
              "完整数值、时间间断、哈希、TF 和扫描字段见 `new_bag_audit.json`。", ""]
    (args.output/"new_bag_audit.md").write_text("\n".join(lines), encoding="utf8")
    print(json.dumps({"report": str(args.output/"new_bag_audit.json"), "maps": len(report["maps"]), "scan_samples": len(report["scan_samples"])}, ensure_ascii=False))


if __name__ == "__main__":
    main()
