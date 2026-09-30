#!/usr/bin/env python3
"""Compare geometric initialization candidates for the first recorded scan.

Uses recorded TF only to generate hypotheses. This is a one-scan ICP diagnostic,
not the NDT localization result and not an independent accuracy measurement.
"""
import argparse
import json
from pathlib import Path
import sqlite3

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from audit_bags import decode
from audit_new_bag import load_pcd, pointcloud2, save_pcd, transform_matrix


def voxel(points, size):
    valid = points[np.isfinite(points).all(axis=1)]
    _, selected = np.unique(np.floor(valid / size).astype(np.int64), axis=0, return_index=True)
    return valid[np.sort(selected)]


def apply(points, matrix):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def metrics(scan, target, matrix):
    distances = cKDTree(target).query(apply(scan, matrix), workers=2)[0]
    result = {"scan_points": len(scan), "map_points": len(target),
              "distance_median_m": float(np.median(distances)), "distance_p90_m": float(np.percentile(distances, 90))}
    for threshold in (.1, .2, .5, 1.0):
        accepted = distances < threshold
        result[f"overlap_within_{threshold:.1f}m"] = float(accepted.mean())
        result[f"inlier_rmse_within_{threshold:.1f}m"] = float(np.sqrt(np.mean(distances[accepted]**2))) if accepted.any() else None
    return result


def icp(scan, target, initial):
    matrix = initial.copy()
    stages = []
    for spacing, gate, iterations in [(1., 3., 50), (.5, 1.5, 50), (.25, .75, 50), (.1, .4, 60)]:
        source = voxel(scan, spacing)
        destination = voxel(target, spacing)
        tree = cKDTree(destination)
        for iteration in range(iterations):
            transformed = apply(source, matrix)
            distance, index = tree.query(transformed, workers=2)
            valid = distance < gate
            if valid.sum() < 20:
                break
            # Reject the largest 15% residuals to limit moving-object influence.
            valid &= distance <= np.percentile(distance[valid], 85)
            a, b = transformed[valid], destination[index[valid]]
            centroid_a, centroid_b = a.mean(0), b.mean(0)
            u, _, vt = np.linalg.svd((a-centroid_a).T @ (b-centroid_b))
            rotation = vt.T @ u.T
            if np.linalg.det(rotation) < 0:
                vt[-1] *= -1
                rotation = vt.T @ u.T
            translation = centroid_b - rotation @ centroid_a
            delta = np.eye(4)
            delta[:3,:3], delta[:3,3] = rotation, translation
            matrix = delta @ matrix
            if np.linalg.norm(translation) < 1e-5 and Rotation.from_matrix(rotation).magnitude() < 1e-5:
                break
        stages.append({"voxel_m": spacing, "iterations": iteration+1, "accepted_points": int(valid.sum())})
    return matrix, stages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1]/"reports")
    args = parser.parse_args()
    assets = args.output / "new_bag_assets"
    assets.mkdir(parents=True, exist_ok=True)
    database = next((args.data_root/"bags/hunter_sensors_20260930_104649").glob("*.db3"))
    c = sqlite3.connect(database.resolve().as_uri()+"?mode=ro", uri=True)
    raw = c.execute("SELECT data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name='/cloud_registered_body') ORDER BY timestamp LIMIT 1").fetchone()[0]
    metadata, all_points = pointcloud2(raw)
    np.save(assets / "first_scan.npy", all_points)
    save_pcd(assets / "first_scan.pcd", all_points, metadata["columns"])
    (assets / "first_scan.json").write_text(json.dumps(metadata, indent=2), encoding="utf8")
    edges = {}
    for raw, in c.execute("SELECT data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name='/tf') ORDER BY timestamp"):
        for tf in decode(raw, "tf2_msgs/msg/TFMessage"):
            edges.setdefault((tf["frame_id"], tf["child_frame_id"]), []).append(tf)
    c.close()
    body = min(edges[("camera_init", "body")], key=lambda tf: abs(tf["stamp_ns"]-metadata["stamp_ns"]))
    mt = min(edges[("map", "camera_init")], key=lambda tf: abs(tf["stamp_ns"]-metadata["stamp_ns"]))
    candidates = {"identity": np.eye(4), "recorded_camera_init_T_body": transform_matrix(body),
                  "recorded_map_T_body": transform_matrix(mt) @ transform_matrix(body)}
    report = {"method": "one-scan multiscale trimmed point-to-point ICP, initialized from identity or recorded TF",
              "independent_ground_truth": False, "ndt_localization_verified": False,
              "scan": metadata, "nearest_recorded_body_tf": body, "nearest_recorded_map_tf": mt,
              "body_tf_time_difference_s": (body["stamp_ns"]-metadata["stamp_ns"])/1e9,
              "map_tf_time_difference_s": (mt["stamp_ns"]-metadata["stamp_ns"])/1e9, "comparisons": []}
    scan = voxel(all_points[:, :3], .1)
    for path in sorted(args.data_root.glob("GlobalMap*.pcd")):
        header, points = load_pcd(path)
        np.save(assets / (path.stem+".npy"), points)
        save_pcd(assets / (path.stem+"_binary.pcd"), points, header["FIELDS"])
        target = points[:, :3]
        target = target[np.isfinite(target).all(axis=1)]
        for name, initial in candidates.items():
            final, stages = icp(scan, target, initial)
            comparison = {"map": path.name, "initial_candidate": name, "initial_matrix": initial.tolist(),
                          "initial_metrics": metrics(scan, target, initial), "icp_matrix": final.tolist(),
                          "icp_pose_xyz_rpy": [*final[:3,3].tolist(), *Rotation.from_matrix(final[:3,:3]).as_euler("xyz").tolist()],
                          "icp_metrics": metrics(scan, target, final), "stages": stages}
            report["comparisons"].append(comparison)
            print(json.dumps({"map": path.name, "candidate": name, "pose": comparison["icp_pose_xyz_rpy"], "metrics": comparison["icp_metrics"]}), flush=True)
    ordered = sorted(report["comparisons"], key=lambda result: (-result["icp_metrics"]["overlap_within_0.2m"], result["icp_metrics"]["distance_median_m"]))
    report["suggested_initialization_for_ndt"] = {"map": ordered[0]["map"], "pose_xyz_rpy": ordered[0]["icp_pose_xyz_rpy"],
                                                   "matrix": ordered[0]["icp_matrix"], "source": "one-scan ICP candidate; not ground truth"}
    (args.output/"new_bag_initial_registration.json").write_text(json.dumps(report, indent=2), encoding="utf8")
    best = ordered[0]
    lines = ["# 首帧地图匹配初值验证", "", "使用首帧真实 body 点云，比较两张地图和三种初始位姿；结果用于给 NDT 提供固定初值。", "",
             "记录中的 TF 仅用于生成初值候选。本次结果为单帧多尺度 ICP 几何验证，不能作为独立真值、定位精度或完整 NDT 运行结果。", "",
             "| 地图 | 初值候选 | ICP 前 0.2m 重叠率 | ICP 后 0.2m 重叠率 | ICP 后最近邻中值(m) | 0.2m 内 RMSE(m) |",
             "|---|---|---:|---:|---:|---:|"]
    for item in report["comparisons"]:
        before, after = item["initial_metrics"], item["icp_metrics"]
        lines.append(f'| {item["map"]} | {item["initial_candidate"]} | {before["overlap_within_0.2m"]:.2%} | {after["overlap_within_0.2m"]:.2%} | {after["distance_median_m"]:.6f} | {after["inlier_rmse_within_0.2m"]:.6f} |')
    lines += ["", f'建议地图：`{best["map"]}`。', "", f'固定初值 xyz + roll/pitch/yaw（米、弧度）：`{best["icp_pose_xyz_rpy"]}`。', "",
              "后续定位只读取真实扫描和固定初值，不能再次消费旧定位 TF 来生成输出。", "",
              "重叠率的分母是首帧 0.1m 体素降采样后 4510 个点；距离是每个扫描点到地图最近邻的距离。RMSE 仅计算阈值内点，不是定位位姿误差。", "",
              "复现脚本：`scripts/verify_initial_registration.py`。全部变换矩阵和阶段参数见同名 JSON。", ""]
    (args.output/"new_bag_initial_registration.md").write_text("\n".join(lines), encoding="utf8")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        _, map_points = load_pcd(args.data_root / best["map"])
        aligned = apply(scan, np.asarray(best["icp_matrix"]))
        fig, axes = plt.subplots(1, 2, figsize=(13, 6))
        for axis, dimensions, labels in [(axes[0], (0,1), ("Map x (m)", "Map y (m)")),
                                         (axes[1], (0,2), ("Map x (m)", "Map z (m)"))]:
            x, y = dimensions
            axis.scatter(map_points[::2,x], map_points[::2,y], s=1, c="#a6aab2", alpha=.3, label="Map")
            axis.scatter(aligned[:,x], aligned[:,y], s=2, c="#008c84", alpha=.8, label="First scan after ICP")
            axis.set(xlabel=labels[0], ylabel=labels[1], aspect="equal")
            axis.grid(alpha=.15)
            axis.legend(loc="upper right", markerscale=3)
        fig.suptitle("First-scan geometric initialization | GlobalMap_loc\n95.3% within 0.2 m of map; not an independent accuracy measurement")
        fig.tight_layout()
        fig.savefig(args.output / "new_bag_first_scan_alignment.png", dpi=160)
        plt.close(fig)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
