#!/usr/bin/env python3
"""Evaluate recorded native fusion outputs against actual scan/map geometry.

Both ekf.csv and ndt.csv must contain map-to-base poses.  Body scans are moved
with map_T_base @ base_T_body.  No old TF or reference trajectory is consumed.
Geometric residuals are not ground-truth localization accuracy.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from audit_bags import Cdr
from audit_new_bag import load_pcd, pointcloud2


def read_csv(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def stamp_ns(row):
    if row.get("stamp_ns", "").strip():
        return int(row["stamp_ns"])
    if "stamp_sec" in row and "stamp_nanosec" in row:
        return int(row["stamp_sec"])*1_000_000_000 + int(row["stamp_nanosec"])
    raise ValueError("CSV requires stamp_ns or stamp_sec/stamp_nanosec")


def truth(value):
    return str(value).strip().lower() not in {"false", "0", "no", "invalid", "rejected"}


def finite_number(value):
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (ValueError, TypeError):
        return None


def distribution(values):
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if not len(array):
        return None
    return {"count": int(len(array)), "min": float(array.min()), "max": float(array.max()),
            "mean": float(array.mean()), "median": float(np.median(array)),
            "p90": float(np.percentile(array, 90)), "p95": float(np.percentile(array, 95))}


def poses(rows, expected_parent, expected_child):
    good, rejected = [], []
    for row_index, row in enumerate(rows):
        try:
            if not truth(row.get("valid_numeric", True)) or not truth(row.get("accepted", True)):
                raise ValueError("row marked invalid or rejected")
            parent = row.get("frame_id", row.get("parent_frame", ""))
            child = row.get("child_frame_id", row.get("body_frame", row.get("child_frame", "")))
            if parent and parent != expected_parent:
                raise ValueError(f"pose parent {parent!r} is not {expected_parent!r}")
            # base_link and base_footprint are identical in the supplied TF.
            allowed = {expected_child}
            if expected_child in {"base_link", "base_footprint"}:
                allowed |= {"base_link", "base_footprint"}
            if child and child not in allowed:
                raise ValueError(f"pose child {child!r} is not a confirmed base frame {sorted(allowed)!r}")
            xyz = np.asarray([float(row[name]) for name in ("x", "y", "z")])
            quat = np.asarray([float(row[name]) for name in ("qx", "qy", "qz", "qw")])
            if not np.isfinite(np.r_[xyz, quat]).all():
                raise ValueError("nonfinite pose")
            if abs(np.linalg.norm(quat)-1.) > .01:
                raise ValueError("quaternion norm differs from one by more than 0.01")
            matrix = np.eye(4)
            matrix[:3,:3], matrix[:3,3] = Rotation.from_quat(quat).as_matrix(), xyz
            good.append({"stamp_ns": stamp_ns(row), "matrix": matrix, "row": row, "row_index": row_index})
        except (KeyError, TypeError, ValueError) as error:
            rejected.append({"row_index": row_index, "reason": str(error)})
    good.sort(key=lambda item: item["stamp_ns"])
    return good, rejected


def voxel(points, size):
    points = points[np.isfinite(points).all(axis=1)]
    if size <= 0:
        return points
    _, selected = np.unique(np.floor(points/size).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(selected)]


def geometry(samples, assets, pose_data, base_T_body, tree, max_difference, spacing):
    timestamps = [pose["stamp_ns"] for pose in pose_data]
    evaluations, all_distances = [], []
    for sample in samples:
        result = {"scan_index": sample["index"], "scan_stamp_ns": sample["stamp_ns"],
                  "scan_frame": sample["frame_id"], "scan_file": sample["npy"]}
        if sample["frame_id"] != "body":
            result.update(evaluated=False, reason="scan frame is not body")
            evaluations.append(result)
            continue
        if not timestamps:
            result.update(evaluated=False, reason="no valid output poses")
            evaluations.append(result)
            continue
        position = bisect.bisect_left(timestamps, sample["stamp_ns"])
        index = min({max(position-1,0),min(position,len(timestamps)-1)},
                    key=lambda i: abs(timestamps[i]-sample["stamp_ns"]))
        difference = (timestamps[index]-sample["stamp_ns"])/1e9
        result.update(pose_stamp_ns=timestamps[index], pose_minus_scan_time_s=difference,
                      pose_csv_row=pose_data[index]["row_index"])
        if abs(difference) > max_difference:
            result.update(evaluated=False, reason="nearest pose exceeds time-match tolerance")
            evaluations.append(result)
            continue
        source = np.load(assets/sample["npy"])
        scan = voxel(source[:,:3], spacing)
        if not len(scan):
            result.update(evaluated=False, reason="scan has no finite XYZ points")
            evaluations.append(result)
            continue
        matrix = pose_data[index]["matrix"] @ base_T_body
        transformed = scan @ matrix[:3,:3].T + matrix[:3,3]
        distances = tree.query(transformed, workers=2)[0]
        all_distances.append(distances)
        result.update(evaluated=True, raw_scan_points=len(source), evaluated_scan_points=len(scan),
                      map_T_body=matrix.tolist(), nearest_neighbor_distance_m=distribution(distances),
                      all_point_rmse_m=float(np.sqrt(np.mean(distances**2))))
        for threshold in (.2,.5):
            inliers = distances <= threshold
            result[f"overlap_within_{threshold:.1f}m"] = float(inliers.mean())
            result[f"inlier_rmse_within_{threshold:.1f}m"] = float(np.sqrt(np.mean(distances[inliers]**2))) if inliers.any() else None
        evaluations.append(result)
    pooled = None
    if all_distances:
        values = np.concatenate(all_distances)
        pooled = {"weighting": "one weight per evaluated scan point; voxel selection applied independently per scan",
                  "nearest_neighbor_distance_m": distribution(values), "all_point_rmse_m": float(np.sqrt(np.mean(values**2)))}
        for threshold in (.2,.5):
            valid = values <= threshold
            pooled[f"overlap_within_{threshold:.1f}m"] = float(valid.mean())
            pooled[f"inlier_rmse_within_{threshold:.1f}m"] = float(np.sqrt(np.mean(values[valid]**2))) if valid.any() else None
    return {"samples": evaluations, "matched_samples": sum(s["evaluated"] for s in evaluations),
            "requested_samples":len(samples), "pooled_point_statistics":pooled,
            "matched_time_difference_abs_s":distribution([abs(s["pose_minus_scan_time_s"]) for s in evaluations if s["evaluated"]])}


def numeric_metrics(rows):
    if not rows:
        return {"rows":0,"numeric_columns":{}}
    if "metric" in rows[0] and "value" in rows[0]:
        grouped = {}
        for row in rows:
            value = finite_number(row.get("value"))
            if value is not None:
                grouped.setdefault(row["metric"],[]).append(value)
        return {"rows":len(rows),"format":"long table, summarized separately by metric",
                "metrics":{name:distribution(values) for name,values in grouped.items()}}
    columns = {}
    for name in rows[0]:
        if name in {"stamp_ns","stamp_sec","stamp_nanosec"}:
            continue
        values = [finite_number(row.get(name)) for row in rows]
        values = [value for value in values if value is not None]
        if values:
            columns[name] = distribution(values)
    return {"rows":len(rows),"numeric_columns":columns}


def native_update_evidence(path, interval):
    result={"file_exists":path.exists(),"source":"native localization: ekf_localizer diagnostics, deduplicated by diagnostic stamp",
            "criterion":"queue_size > 0 and no_update_count == 0; delay and Mahalanobis gates also reported passed",
            "interpretation":"Diagnostic update cycles can reuse smoothed measurement queues; these counts are not unique sensor message counts.",
            "malformed_json_lines":0,"diagnostic_cycles":0,"streams":{},"ten_second_common_input_bins":[]}
    stamps={"pose":[],"twist":[]}
    counts={kind:{"queued_cycles":0,"no_update_zero_and_queue_nonempty_cycles":0,"passed_update_cycles":0,
                  "delay_gate_rejection_cycles":0,"mahalanobis_gate_rejection_cycles":0} for kind in stamps}
    seen=set()
    gyro_timeouts=0
    if path.exists():
        with path.open(encoding="utf8") as stream:
            for line in stream:
                try:
                    item=json.loads(line)
                except json.JSONDecodeError:
                    result["malformed_json_lines"]+=1
                    continue
                if item.get("name")=="gyro_odometer: gyro_odometer_status" and "timeout" in item.get("message","").lower():
                    gyro_timeouts+=1
                if item.get("name")!="localization: ekf_localizer" or item["stamp_ns"] in seen:
                    continue
                seen.add(item["stamp_ns"])
                values=item.get("values",{})
                result["diagnostic_cycles"]+=1
                for kind in stamps:
                    if kind+"_no_update_count" not in values:
                        continue
                    queued=int(values.get(kind+"_queue_size",0))>0
                    zero=int(values[kind+"_no_update_count"])==0
                    delay=values.get(kind+"_is_passed_delay_gate","").lower()=="true"
                    maha=values.get(kind+"_is_passed_mahalanobis_gate","").lower()=="true"
                    counts[kind]["queued_cycles"]+=int(queued)
                    counts[kind]["no_update_zero_and_queue_nonempty_cycles"]+=int(queued and zero)
                    counts[kind]["delay_gate_rejection_cycles"]+=int(values.get(kind+"_is_passed_delay_gate","").lower()=="false")
                    counts[kind]["mahalanobis_gate_rejection_cycles"]+=int(values.get(kind+"_is_passed_mahalanobis_gate","").lower()=="false")
                    if queued and zero and delay and maha:
                        counts[kind]["passed_update_cycles"]+=1
                        stamps[kind].append(item["stamp_ns"])
    result["gyro_timeout_diagnostic_messages"]=gyro_timeouts
    for kind,values in stamps.items():
        values.sort()
        result["streams"][kind]={**counts[kind],"passed_update_stamp_range_ns":[values[0],values[-1]] if values else None,
                                  "passed_update_interval_s":distribution(np.diff(values)/1e9) if len(values)>1 else None}
    if interval:
        start,end=interval
        cursor=start
        while cursor<=end:
            limit=min(cursor+10_000_000_000,end+1)
            result["ten_second_common_input_bins"].append({"from_ns":cursor,"until_ns_exclusive":limit,
                **{kind+"_passed_update_cycles":sum(cursor<=t<limit for t in values) for kind,values in stamps.items()}})
            cursor=limit
    result["both_routes_reported_updates_in_every_common_input_bin"] = bool(result["ten_second_common_input_bins"]) and all(
        b["pose_passed_update_cycles"]>0 and b["twist_passed_update_cycles"]>0 for b in result["ten_second_common_input_bins"])
    return result


def recorded_run_status(directory):
    summary=directory/"summary.json"
    result={"summary_exists":summary.exists()}
    if summary.exists():
        result["summary"]=json.loads(summary.read_text(encoding="utf8"))
    status=directory/"fusion_status.jsonl"
    modes={}
    last=None
    if status.exists():
        with status.open(encoding="utf8") as stream:
            for line in stream:
                try:
                    item=json.loads(line)
                except json.JSONDecodeError:
                    continue
                mode=item.get("mode","unknown")
                modes[mode]=modes.get(mode,0)+1
                last=item
    result.update(fusion_mode_sample_counts=modes,last_fusion_status=last)
    return result


def largest_step(data):
    if len(data)<2:
        return None
    positions=np.asarray([p["matrix"][:3,3] for p in data])
    deltas=np.diff(positions,axis=0)
    sizes=np.linalg.norm(deltas,axis=1)
    index=int(np.argmax(sizes))
    first,second=data[index],data[index+1]
    dt=(second["stamp_ns"]-first["stamp_ns"])/1e9
    return {"first_pose_index":index,"second_pose_index":index+1,
            "first_stamp_ns":first["stamp_ns"],"second_stamp_ns":second["stamp_ns"],
            "first_xyz":positions[index].tolist(),"second_xyz":positions[index+1].tolist(),
            "delta_xyz_m":deltas[index].tolist(),"distance_m":float(sizes[index]),"dt_s":dt,
            "apparent_step_speed_mps":float(sizes[index]/dt) if dt>0 else None,
            "seconds_since_first_pose":(second["stamp_ns"]-data[0]["stamp_ns"])/1e9,
            "interpretation":"Largest consecutive output displacement; not independently verified vehicle motion."}


def inspect_ndt_step(streams,gyro,metrics,audit,tree,base_T_body,spacing,max_difference):
    result={"largest_steps":{name:largest_step(data) for name,data in streams.items()},"geometry_checks":[]}
    ndt_step=result["largest_steps"]["ndt"]
    if not ndt_step:
        return result
    stamps=[ndt_step["first_stamp_ns"],ndt_step["second_stamp_ns"]]
    middle=(stamps[0]+stamps[1])//2
    nearby=sorted(gyro,key=lambda row:abs(stamp_ns(row)-middle))[:6]
    result["nearest_gyro_messages"]=[{key:row[key] for key in ("stamp_ns","vx","wz") if key in row} for row in nearby]
    result["metrics_at_step"]=[row for row in metrics if stamp_ns(row) in stamps]
    typical_vx=np.median([abs(float(row["vx"])) for row in nearby]) if nearby else None
    result["rough_wheel_speed_times_step_dt_m"]=float(typical_vx*ndt_step["dt_s"]) if typical_vx is not None else None
    result["warning"]="A large NDT pose step despite nearby wheel speed and high matching scores is a registration fluctuation. Matching scores or convergence alone do not establish pose accuracy."
    db=Path(audit["bag"]["database"])
    if not db.exists():
        result["geometry_unavailable_reason"]="Source database unavailable for the two specific step scans"
        return result
    pc_topic=next(t for t in audit["bag"]["topics"] if t["name"]=="/cloud_registered_body")
    delays=pc_topic["recording_minus_header_s"]
    c=sqlite3.connect(db.resolve().as_uri()+"?mode=ro",uri=True)
    topic_id=c.execute("SELECT id FROM topics WHERE name='/cloud_registered_body'").fetchone()[0]
    for stamp in stamps:
        # Recorded receipt is delayed; query a narrow range using independently audited bounds.
        low=stamp+int((delays["min"]-.01)*1e9)
        high=stamp+int((delays["max"]+.01)*1e9)
        raw=None
        for candidate, in c.execute("SELECT data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp",(topic_id,low,high)):
            if Cdr(candidate).header()["stamp_ns"]==stamp:
                raw=candidate
                break
        if raw is None:
            result["geometry_checks"].append({"stamp_ns":stamp,"error":"Exact scan header not found"})
            continue
        info,points=pointcloud2(raw)
        scan=voxel(points[:,:3],spacing)
        item={"stamp_ns":stamp,"scan_frame":info["frame_id"],"scan_points":len(scan),"outputs":{}}
        for name,data in streams.items():
            if not data:
                continue
            pose=min(data,key=lambda p:abs(p["stamp_ns"]-stamp))
            difference=(pose["stamp_ns"]-stamp)/1e9
            if abs(difference)>max_difference:
                item["outputs"][name]={"error":"No close output pose"}
                continue
            matrix=pose["matrix"]@base_T_body
            transformed=scan@matrix[:3,:3].T+matrix[:3,3]
            distances=tree.query(transformed,workers=2)[0]
            item["outputs"][name]={"pose_minus_scan_s":difference,"distance_median_m":float(np.median(distances)),
                                    "overlap_within_0.2m":float((distances<=.2).mean()),
                                    "overlap_within_0.5m":float((distances<=.5).mean())}
        result["geometry_checks"].append(item)
    c.close()
    return result


def make_plots(output, map_points, streams, gyro_rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"ekf":"#087d9e","ndt":"#d05a29"}
    fig, axis = plt.subplots(figsize=(10,9))
    step = max(1,len(map_points)//120000)
    axis.scatter(map_points[::step,0],map_points[::step,1],s=1,c="#aeb5bf",alpha=.35,rasterized=True,label="Point-cloud map")
    for name,data in streams.items():
        if not data:
            continue
        xyz=np.asarray([p["matrix"][:3,3] for p in data])
        axis.plot(xyz[:,0],xyz[:,1],color=colors[name],linewidth=1.7,alpha=.85,label=f"{name.upper()} base trajectory")
        axis.scatter(*xyz[0,:2],marker="o",s=75,facecolors="white",edgecolors=colors[name],linewidths=2,zorder=5,label=f"{name.upper()} start")
        axis.scatter(*xyz[-1,:2],marker="X",s=90,c=colors[name],zorder=6,label=f"{name.upper()} end")
    axis.set(xlabel="Map x (m)",ylabel="Map y (m)",aspect="equal",title="Native fusion outputs in the map\nScan/map consistency only; no ground-truth trajectory")
    axis.grid(alpha=.15)
    axis.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output/"map_trajectory.png",dpi=170)
    all_positions=np.asarray([p["matrix"][:3,3] for data in streams.values() for p in data])
    if len(all_positions):
        axis.set_xlim(all_positions[:,0].min()-1.,all_positions[:,0].max()+1.)
        axis.set_ylim(all_positions[:,1].min()-1.,all_positions[:,1].max()+1.)
        axis.set_title("Rear-wheel-center trajectories: detail\nEKF solid, NDT dashed; no ground-truth trajectory")
        for line in axis.lines:
            if line.get_label().startswith("NDT"):
                line.set(linestyle="--",linewidth=1.15,alpha=.65)
            else:
                line.set(linewidth=2.)
        axis.legend(loc="best")
        fig.tight_layout()
        fig.savefig(output/"map_trajectory_detail.png",dpi=170)
    plt.close(fig)
    candidates=[]
    for data in streams.values():
        candidates += [p["stamp_ns"] for p in data]
    gyro=[]
    for row in gyro_rows:
        try:
            if truth(row.get("valid_numeric",True)):
                gyro.append((stamp_ns(row),row))
        except (KeyError,ValueError):
            continue
    candidates += [t for t,_ in gyro]
    origin=min(candidates) if candidates else 0
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
    series=[("gyro_odometer",gyro,"#7570b3"),("EKF",[(p["stamp_ns"],p["row"]) for p in streams["ekf"]],colors["ekf"])]
    for axis,column,label in [(axes[0],"vx","Forward velocity (m/s)"),(axes[1],"wz","Yaw rate (rad/s)")]:
        for name,rows,color in series:
            pairs=[((stamp-origin)/1e9,finite_number(row.get(column))) for stamp,row in rows]
            pairs=[(stamp,value) for stamp,value in pairs if value is not None]
            if pairs:
                axis.plot(*zip(*pairs),label=name,color=color,linewidth=1.15,alpha=.9)
        axis.set_ylabel(label)
        axis.grid(alpha=.2)
        if axis.lines:
            axis.legend(loc="best")
        else:
            axis.text(.5,.5,f"No {column} values available",transform=axis.transAxes,ha="center")
    axes[-1].set_xlabel("Header time since first valid output (s)")
    axes[0].set_title("Native gyro_odometer and EKF output measurements\nTemporary mounting pitch is a gravity-based estimate")
    fig.tight_layout()
    fig.savefig(output/"velocity_yaw_rate.png",dpi=170)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results",type=Path,required=True,help="Directory containing ekf.csv, ndt.csv, gyro.csv and metrics.csv")
    parser.add_argument("--audit",type=Path,default=Path(__file__).resolve().parents[1]/"reports/new_bag_audit.json")
    parser.add_argument("--map",type=Path,help="Map .npy or .pcd; default actual map path from results/summary.json")
    parser.add_argument("--output",type=Path,help="Output directory; default results/validation")
    parser.add_argument("--mount-pitch-deg",type=float,help="Optional verification value; must match the recorded run")
    parser.add_argument("--translation",type=float,nargs=3,help="Optional verification value; must match the recorded run")
    parser.add_argument("--max-time-difference",type=float,default=.15)
    parser.add_argument("--voxel-size",type=float,default=.1)
    parser.add_argument("--map-frame",default="map")
    parser.add_argument("--base-frame",default="base_footprint")
    args=parser.parse_args()
    if args.max_time_difference<0 or args.voxel_size<0:
        parser.error("Time tolerance and voxel size must be nonnegative")
    audit=json.loads(args.audit.read_text(encoding="utf8"))
    assets=args.audit.parent/"new_bag_assets"
    run_status=recorded_run_status(args.results)
    if not run_status["summary_exists"]:
        parser.error("No results/summary.json; wait for the replay to finish before evaluating")
    run_summary=run_status["summary"]
    final_status=run_status.get("last_fusion_status") or {}
    if "mount_pitch_deg" not in run_summary or "mount_rpy" not in final_status or "mount_xyz" not in final_status:
        parser.error("Run summary/status does not record mounting pitch and translation")
    actual_pitch=float(run_summary["mount_pitch_deg"])
    actual_rpy=np.asarray(final_status["mount_rpy"],dtype=float)
    actual_translation=np.asarray(final_status["mount_xyz"],dtype=float)
    if not np.isfinite(np.r_[actual_rpy,actual_translation,actual_pitch]).all():
        parser.error("Recorded mounting transform is nonfinite")
    if abs(math.degrees(actual_rpy[1])-actual_pitch)>1e-5 or abs(actual_rpy[0])>1e-7 or abs(actual_rpy[2])>1e-7:
        parser.error("Recorded mounting rotation does not match summary pitch-only transform")
    if args.mount_pitch_deg is not None and abs(args.mount_pitch_deg-actual_pitch)>1e-5:
        parser.error("Requested mounting pitch differs from the actual replay configuration")
    if args.translation is not None and not np.allclose(args.translation,actual_translation,atol=1e-7,rtol=0):
        parser.error("Requested mounting translation differs from the actual replay configuration")
    args.mount_pitch_deg=actual_pitch
    args.translation=actual_translation.tolist()
    if "map" not in run_summary:
        parser.error("Run summary does not identify the map")
    actual_map=Path(run_summary["map"])
    if not actual_map.exists():
        parser.error(f"The map recorded in the run summary does not exist: {actual_map}")
    map_path=args.map or actual_map
    points=np.load(map_path) if map_path.suffix.lower()==".npy" else load_pcd(map_path)[1]
    if map_path.resolve()!=actual_map.resolve():
        actual_points=np.load(actual_map) if actual_map.suffix.lower()==".npy" else load_pcd(actual_map)[1]
        if points.shape[0]!=actual_points.shape[0] or not np.array_equal(points[:,:3],actual_points[:,:3],equal_nan=True):
            parser.error("Requested map XYZ coordinates do not match the map used by the replay")
    if final_status.get("map_frame",args.map_frame)!=args.map_frame:
        parser.error("Requested map frame does not match the recorded run")
    map_points=points[:,:3]
    map_points=map_points[np.isfinite(map_points).all(axis=1)]
    if not len(map_points):
        parser.error("Map has no finite XYZ points")
    output=args.output or args.results/"validation"
    output.mkdir(parents=True,exist_ok=True)
    base_T_body=np.eye(4)
    base_T_body[:3,:3]=Rotation.from_euler("y",args.mount_pitch_deg,degrees=True).as_matrix()
    base_T_body[:3,3]=args.translation
    report={"results_directory":str(args.results.resolve()),"map_file":str(map_path.resolve()),
            "map_points":len(map_points),"independent_ground_truth_available":False,
            "method":"actual body scans transformed by new output map_T_base @ provisional base_T_body, compared with map nearest neighbors",
            "old_tf_consumed":False,"pose_frame_contract":{"parent":args.map_frame,"child":args.base_frame},
            "runtime_contract_verified":{"mount_pitch_and_translation_match_summary_and_final_status":True,
                                          "map_coordinates_match_actual_run":True,"actual_run_map":str(actual_map)},
            "extrinsic":{"base_T_body":base_T_body.tolist(),"translation_m":args.translation,"pitch_deg":args.mount_pitch_deg,
                         "status":"temporary forward-tilt assumption authorized by user; pitch estimated from 729 stationary IMU samples, not independently measured calibration"},
            "scan_sampling":{"requested_samples":len(audit["scan_samples"]),"source":"ten real scans from new_bag_audit.json", "voxel_size_m":args.voxel_size,
                             "maximum_absolute_header_time_difference_s":args.max_time_difference},
            "streams":{},"limitations":["Nearest-neighbor overlap and residual measure geometric consistency, not localization pose error.",
                "A wrong pose can still align with repeated or weak geometry; no independent truth trajectory is available.",
                "No temporal interpolation is applied: only the closest output pose within the configured time tolerance is used.",
                "All-point RMSE includes scan points outside map coverage; inlier RMSE excludes distances above its threshold.",
                "Temporary mounting pitch affects base/body conversion and must be replaced by measured calibration before claiming final calibration."]}
    tree=cKDTree(map_points)
    streams={}
    for name in ("ekf","ndt"):
        path=args.results/(name+".csv")
        rows=read_csv(path)
        data,rejected=poses(rows,args.map_frame,args.base_frame)
        streams[name]=data
        timestamps=[p["stamp_ns"] for p in data]
        positions=np.asarray([p["matrix"][:3,3] for p in data])
        report["streams"][name]={"file":str(path.resolve()),"file_exists":path.exists(),"csv_rows":len(rows),"valid_poses":len(data),
                                  "rejected_rows":rejected,"duplicate_header_timestamps":len(timestamps)-len(set(timestamps)),
                                  "header_coverage_ns":[timestamps[0],timestamps[-1]] if timestamps else None,
                                  "estimated_trajectory_length_m":float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()) if len(data)>1 else 0.,
                                  "geometry":geometry(audit["scan_samples"],assets,data,base_T_body,tree,args.max_time_difference,args.voxel_size)}
    gyro=read_csv(args.results/"gyro.csv")
    report["gyro_csv_summary"]=numeric_metrics(gyro)
    metric_rows=read_csv(args.results/"metrics.csv")
    report["metrics_csv_summary"]=numeric_metrics(metric_rows)
    input_topics={t["name"]:t for t in audit["bag"]["topics"]}
    common_interval=None
    if all(name in input_topics for name in ("/hunter_odom","/livox/imu")):
        intervals=[input_topics[name]["header_timing"] for name in ("/hunter_odom","/livox/imu")]
        common_interval=[max(t["first_ns"] for t in intervals),min(t["last_ns"] for t in intervals)]
    report["native_ekf_update_evidence"]=native_update_evidence(args.results/"diagnostics.jsonl",common_interval)
    report["recorded_run_status"]=run_status
    report["registration_fluctuation_check"]=inspect_ndt_step(streams,gyro,metric_rows,audit,tree,base_T_body,args.voxel_size,args.max_time_difference)
    report["evaluation_complete"]=all(s["geometry"]["matched_samples"]==s["geometry"]["requested_samples"] for s in report["streams"].values())
    (output/"validation.json").write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding="utf8")
    make_plots(output,map_points,streams,gyro)
    lines=["# 融合输出的点云几何验证", "", "仅使用本次 EKF/NDT 输出位姿、真实 body 点云和 GlobalMap 点云地图，没有使用旧 TF 作为参考真值。", "",
           f'暂定 body 安装外参：平移 `{args.translation}` m，前倾 pitch `+{args.mount_pitch_deg}°`。该角度来自静止 IMU 估计和用户授权，尚不是独立实测标定。', "",
           f"按 `T_map_body = T_map_base × T_base_body` 转换扫描，每帧选择时间差不超过 {args.max_time_difference}s 的最近位姿；未匹配帧不计算几何分数。", "",
           "| 输出 | 有效位姿 | 匹配扫描 | 0.2m重叠率 | 0.5m重叠率 | 最近邻中值(m) | 0.2m内RMSE(m) |", "|---|---:|---:|---:|---:|---:|---:|"]
    for name,stream in report["streams"].items():
        geo=stream["geometry"]
        pooled=geo["pooled_point_statistics"]
        if pooled:
            lines.append(f'| {name.upper()} | {stream["valid_poses"]} | {geo["matched_samples"]}/{geo["requested_samples"]} | {pooled["overlap_within_0.2m"]:.2%} | {pooled["overlap_within_0.5m"]:.2%} | {pooled["nearest_neighbor_distance_m"]["median"]:.6f} | {pooled["inlier_rmse_within_0.2m"] if pooled["inlier_rmse_within_0.2m"] is not None else "无内点"} |')
        else:
            lines.append(f'| {name.upper()} | {stream["valid_poses"]} | 0/{geo["requested_samples"]} | — | — | — | — |')
    evidence=report["native_ekf_update_evidence"]
    lines += ["", "## 原生 EKF 两路更新证据", "",
              "读取原生 EKF 诊断，去除重复时间戳，要求测量队列非空、no_update_count=0，且 delay / Mahalanobis 检查通过。统计是更新诊断周期数，可能复用平滑队列中的测量，不是独立传感器观测条数。", "",
              "| 更新输入 | 有测量队列的周期 | 通过的更新周期 | 延迟拒绝周期 | Mahalanobis拒绝周期 |", "|---|---:|---:|---:|---:|"]
    for kind,s in evidence["streams"].items():
        lines.append(f'| {kind} | {s["queued_cycles"]} | {s["passed_update_cycles"]} | {s["delay_gate_rejection_cycles"]} | {s["mahalanobis_gate_rejection_cycles"]} |')
    lines += ["", f'在 IMU 与轮速共同覆盖的时间段，以 10s 分段检查，两路每段均存在原生更新：{evidence["both_routes_reported_updates_in_every_common_input_bin"]}。gyro_odometer 超时诊断消息数：{evidence["gyro_timeout_diagnostic_messages"]}。', "",
              "## NDT 相邻帧波动", ""]
    jumps=report["registration_fluctuation_check"]
    if jumps["largest_steps"]["ndt"]:
        j=jumps["largest_steps"]["ndt"]
        e=jumps["largest_steps"]["ekf"]
        lines += [f'NDT 最大相邻位移 **{j["distance_m"]:.6f}m / {j["dt_s"]:.6f}s**，出现在首帧后 {j["seconds_since_first_pose"]:.3f}s（header `{j["first_stamp_ns"]}` → `{j["second_stamp_ns"]}`）；xyz 变化 `{j["delta_xyz_m"]}` m。', "",
                  f'附近轮速×该时间间隔约 {jumps["rough_wheel_speed_times_step_dt_m"]:.3f}m，明显小于 NDT 跳动；匹配分数仍较高，说明“全部收敛”不能代替连续性和精度评估。EKF 全程最大相邻位移为 {e["distance_m"]:.6f}m / {e["dt_s"]:.6f}s，但这是 50Hz 输出，不能只凭步长与 10Hz NDT 直接比较精度。', "",
                  "波动根因尚未确定；这两帧的地图重合度仍较高，不能据此判定 NDT 或 EKF 的真实位姿误差。", "",
                  "| 波动附近扫描 header | 位姿输出 | 0.2m重叠率 | 0.5m重叠率 | 最近邻中值(m) |", "|---|---|---:|---:|---:|"]
        for item in jumps["geometry_checks"]:
            for name,v in item.get("outputs",{}).items():
                if "distance_median_m" in v:
                    lines.append(f'| {item["stamp_ns"]} | {name.upper()} | {v["overlap_within_0.2m"]:.2%} | {v["overlap_within_0.5m"]:.2%} | {v["distance_median_m"]:.6f} |')
    lines += ["", "## 限制与时序", "",
              f'运行状态样本计数：`{run_status["fusion_mode_sample_counts"]}`；最后模式为 `{(run_status.get("last_fusion_status") or {}).get("mode","unknown")}`，因此不能声称全包每一时刻都持续 FUSED。', "",
              "点云/IMU 比轮速早约 1.313 秒结束，另有 0.2 秒回放尾部时钟，尾部进入 STALE 并停止公开输出。内部 IMU 缺口也会触发较严格的 gyro 超时；上面的 10s 覆盖检查只证明两路持续参与，不代表无缺口。", "",
              f'回放时间定义：`{run_summary.get("input_time_basis","unknown")}`；复现原记录接收延迟：`{run_summary.get("recording_delay_reproduced","unknown")}`。按 header 排序验证采集数据融合，不验证原实机到达延迟。', "",
              "轨迹长度是估计路径的累加长度，不能视为实车真实里程；地图和外参没有独立地面真值。临时安装角、噪声协方差与超时参数仍需实测标定和实机验证。", "",
              "这些数值表示点云与地图的几何一致性，不表示定位位姿精度；没有独立地面真值。0.2m 内 RMSE 仅使用阈值内的匹配点，不能忽略重叠率来解释。", "",
              "`validation.json` 包含逐帧时间匹配、拒绝原因、重合度和残差；`map_trajectory.png` 为全图轨迹，`map_trajectory_detail.png` 为局部放大，`velocity_yaw_rate.png` 为 gyro_odometer 与 EKF 车速/角速。", ""]
    (output/"validation.md").write_text("\n".join(lines),encoding="utf8")
    print(json.dumps({"output":str(output.resolve()),"evaluation_complete":report["evaluation_complete"],
                      "matched_samples":{name:s["geometry"]["matched_samples"] for name,s in report["streams"].items()}},ensure_ascii=False))


if __name__=="__main__":
    main()
