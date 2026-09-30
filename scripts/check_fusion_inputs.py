#!/usr/bin/env python3
"""Read-only frame/gravity and timestamp checks for the confirmed fusion inputs.

Does not change calibration, message timestamps, measurements, or launch files.
"""
import argparse
import bisect
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
from scipy.spatial.transform import Rotation

from audit_bags import decode, timing


def distribution(values):
    values = np.asarray(values, dtype=float)
    return {"count": len(values), "min": float(values.min()), "max": float(values.max()),
            "mean": float(values.mean()), "stddev": float(values.std()),
            "p50": float(np.percentile(values,50)), "p90": float(np.percentile(values,90)),
            "p95": float(np.percentile(values,95)), "p99": float(np.percentile(values,99))}


def nearest_index(stamps, stamp):
    pos = bisect.bisect_left(stamps, stamp)
    return min({max(0,pos-1),min(len(stamps)-1,pos)}, key=lambda index: abs(stamps[index]-stamp))


def angle_to_up(vector):
    return math.degrees(math.acos(float(np.clip(vector[2] / np.linalg.norm(vector),-1,1))))


def stationary_stats(messages, rotation, ndt_rotation):
    acc = np.asarray([m["linear_acceleration"] for m in messages])
    angular = np.asarray([m["angular_velocity"] for m in messages])
    mean = acc.mean(0)
    unit = mean / np.linalg.norm(mean)
    return {"count": len(messages), "first_stamp_ns": messages[0]["stamp_ns"], "last_stamp_ns": messages[-1]["stamp_ns"],
            "acceleration_mean_as_recorded": mean.tolist(), "acceleration_stddev_as_recorded": acc.std(0).tolist(),
            "acceleration_norm_as_recorded": distribution(np.linalg.norm(acc,axis=1)),
            "angular_velocity_mean_radps": angular.mean(0).tolist(), "angular_velocity_stddev_radps": angular.std(0).tolist(),
            "normalized_acceleration_direction_in_livox_frame": unit.tolist(),
            "assuming_livox_frame_equals_body_acceleration_in_base_footprint": (rotation @ mean).tolist(),
            "assuming_livox_frame_equals_body_base_vertical_error_deg": angle_to_up(rotation @ mean),
            "assuming_livox_frame_equals_body_acceleration_in_initial_ndt_map": (ndt_rotation @ mean).tolist(),
            "assuming_livox_frame_equals_body_ndt_map_vertical_error_deg": angle_to_up(ndt_rotation @ mean),
            "pitch_that_levels_acceleration_if_no_roll_deg": math.degrees(math.atan2(-mean[0],mean[2]))}


def simulate_callbacks(records, order, timeout):
    # Reproduces upstream gyro_odometer's queue, timeout and publish conditions,
    # assuming perfect callback delivery and a valid TF. It does not simulate EKF.
    events = sorted(records, key=lambda e: (e[order],e["received_ns"],e["kind"]))
    latest = {}
    queues = {"imu": [], "wheel": []}
    counts = {"events":len(events), "startup_checks":0, "timeout_checks":0,
              "publish_batches":0, "imu_samples_consumed":0, "wheel_samples_consumed":0}
    ages, mismatches = [], []
    for event in events:
        now = event[order]
        latest[event["kind"]] = event["stamp_ns"]
        queues[event["kind"]].append(event["stamp_ns"])
        if len(latest) < 2:
            counts["startup_checks"] += 1
            queues = {"imu": [], "wheel": []}
            continue
        age = max(abs(now-latest["imu"]),abs(now-latest["wheel"]))/1e9
        ages.append(age)
        if age > timeout:
            counts["timeout_checks"] += 1
            queues = {"imu": [], "wheel": []}
        elif queues["imu"] and queues["wheel"]:
            counts["publish_batches"] += 1
            counts["imu_samples_consumed"] += len(queues["imu"])
            counts["wheel_samples_consumed"] += len(queues["wheel"])
            mismatches.append(abs(queues["imu"][-1]-queues["wheel"][-1])/1e9)
            queues = {"imu": [], "wheel": []}
    counts["latest_input_age_at_callback_s"] = distribution(ages)
    counts["published_batch_latest_header_difference_s"] = distribution(mismatches) if mismatches else None
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag",type=Path,required=True)
    parser.add_argument("--output",type=Path,default=Path(__file__).resolve().parents[1]/"reports")
    args=parser.parse_args()
    db = args.bag if args.bag.suffix==".db3" else next(args.bag.glob("*.db3"))
    args.output.mkdir(parents=True,exist_ok=True)
    c = sqlite3.connect(db.resolve().as_uri()+"?mode=ro",uri=True)
    topics, records = {}, []
    for name,kind in [("/hunter_odom","wheel"),("/livox/imu","imu")]:
        topic_id,msgtype = c.execute("SELECT id,type FROM topics WHERE name=?",(name,)).fetchone()
        messages=[]
        for received,raw in c.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id",(topic_id,)):
            m=decode(raw,msgtype)
            m["received_ns"]=received
            messages.append(m)
            records.append({"kind":kind,"stamp_ns":m["stamp_ns"],"received_ns":received})
        topics[kind]=messages
    static=[]
    for raw, in c.execute("SELECT data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name='/tf_static')"):
        static += decode(raw,"tf2_msgs/msg/TFMessage")
    c.close()
    extrinsic = next(t for t in static if t["frame_id"]=="base_footprint" and t["child_frame_id"]=="mid360_link")
    base_R_body = Rotation.from_quat(extrinsic["rotation"]).as_matrix()
    initial=json.loads((args.output/"new_bag_initial_registration.json").read_text())["suggested_initialization_for_ndt"]
    map_T_body=np.asarray(initial["matrix"])
    body_T_base=np.eye(4)
    body_T_base[:3,:3]=base_R_body.T
    body_T_base[:3,3]=-base_R_body.T @ np.asarray(extrinsic["translation"])
    map_T_base=map_T_body @ body_T_base
    wheels=topics["wheel"]
    stamps=[m["stamp_ns"] for m in wheels]
    selected=[]
    for m in topics["imu"]:
        wheel=wheels[nearest_index(stamps,m["stamp_ns"])]
        if abs(wheel["stamp_ns"]-m["stamp_ns"]) <= 50_000_000 and abs(wheel["linear_velocity"][0])<.01 and np.linalg.norm(m["angular_velocity"])<.03:
            selected.append(m)
    initial_stationary=[m for m in selected if m["stamp_ns"] <= topics["imu"][0]["stamp_ns"]+3_000_000_000]
    final_stationary=[m for m in selected if m["stamp_ns"] >= topics["imu"][-1]["stamp_ns"]-5_000_000_000]
    report={"source_database":str(db.resolve()),"mutation_performed":False,
            "user_confirmed":{"base_footprint_origin":"rear axle center","body_equals_mid360_link":True,"wheel_measurement":"hunter_odom.twist.twist.linear.x only"},
            "unconfirmed":{"livox_frame_to_body_rotation":"No recorded TF establishes this relationship; gravity comparisons below state the identity assumption explicitly."},
            "recorded_base_footprint_T_mid360_link":extrinsic,
            "recorded_extrinsic_rpy_deg":Rotation.from_quat(extrinsic["rotation"]).as_euler("xyz",degrees=True).tolist(),
            "expected_sensor_specific_force_direction_if_base_level_and_livox_equals_body":(base_R_body.T @ np.array([0.,0.,1.])).tolist(),
            "stationary_selection":{"wheel_speed_abs_below_mps":.01,"imu_angular_norm_below_radps":.03,"wheel_imu_header_nearest_within_s":.05},
            "stationary_subsets":{},"timing":{},"gyro_odometer_timeout_model":{},
            "initial_map_T_body_source":"one-scan ICP refined initialization, not ground truth",
            "initial_map_T_body_rpy_deg":Rotation.from_matrix(map_T_body[:3,:3]).as_euler("xyz",degrees=True).tolist(),
            "initial_map_T_base_using_recorded_extrinsic":map_T_base.tolist(),
            "initial_map_T_base_rpy_deg":Rotation.from_matrix(map_T_base[:3,:3]).as_euler("xyz",degrees=True).tolist(),
            "needs_user_calibration_clarification":True}
    for name,messages in [("all_wheel_and_gyro_stationary_candidates",selected),("first_3s_stationary_candidates",initial_stationary),("last_5s_stationary_candidates",final_stationary)]:
        if messages:
            report["stationary_subsets"][name]=stationary_stats(messages,base_R_body,map_T_body[:3,:3])
    for kind,messages in topics.items():
        hs=[m["stamp_ns"] for m in messages]
        gaps=np.diff(hs)/1e9
        report["timing"][kind]={"count":len(messages),"header":timing(hs),"header_gap_s":distribution(gaps),
                                 "recording_minus_header_s":distribution([(m["received_ns"]-m["stamp_ns"])/1e9 for m in messages]),
                                 "header_gap_counts_above_s":{str(t):int((gaps>t).sum()) for t in [.1,.2,.5,.8,1.]}}
    overlap_start=max(m[0]["stamp_ns"] for m in topics.values())
    overlap_end=min(m[-1]["stamp_ns"] for m in topics.values())
    report["common_header_interval_ns"]=[overlap_start,overlap_end]
    report["wheel_tail_after_last_imu_s"]=(topics["wheel"][-1]["stamp_ns"]-topics["imu"][-1]["stamp_ns"])/1e9
    for order in ["received_ns","stamp_ns"]:
        report["gyro_odometer_timeout_model"][order]={str(t):simulate_callbacks(records,order,t) for t in [.2,.5,.8,1.,1.5,2.]}
    report["recommendations"]={"header_ordered_offline_replay_message_timeout_sec":.8,
                                "basis":"0.8 s exceeds both observed maximum internal header gaps (IMU 0.7067 s, wheel 0.7600 s); scheduling margin is only ~40 ms for wheel, so this is a bag-test configuration, not a live safety setting.",
                                "replay_semantics":"Sort by original acquisition header and drive /clock on that timeline without changing serialized measurements/header stamps. This removes recorded arrival jitter/latency from the replay experiment and does not verify original live behavior.",
                                "tail":"No new gyro data exists during the final 1.31685276 s of wheel messages; increasing timeout cannot create gyro observations. Report/trim common input coverage explicitly.",
                                "live":"Default 0.2 s timeout is exceeded by the observed recording-minus-header age for most IMU messages. Diagnose clocks, driver stamping and delivery delays before a live choice; do not simply raise timeout to conceal it.",
                                "calibration":"Do not flip pitch or equate livox_frame and body without confirming extrinsic direction and IMU axes. Gravity establishes a discrepancy conditional on identity axes and approximately level base; it does not identify the correct calibration uniquely."}
    (args.output/"fusion_input_checks.json").write_text(json.dumps(report,indent=2),encoding="utf8")
    first=report["stationary_subsets"]["first_3s_stationary_candidates"]
    lines=["# 标准 Autoware 融合输入检查", "", "只读检查；未修改消息、外参或融合配置。", "",
           "## 需要确认的外参矛盾", "", "已采用用户确认：base_footprint 位于后轮中心，body 与 mid360_link 同原点同方向，轮速只用 hunter_odom.linear.x。livox_frame 到 body 的方向关系仍未由录包 TF 给出。", "",
           "录包 base_footprint→mid360_link 的 pitch 为 -30°。若车辆近水平、IMU 的 livox_frame 与 body 同轴，则静止加速度方向应约为 `[+0.5,0,+0.866]`。", "",
           f'实际首 3 秒筛得 {first["count"]} 个静止候选，均值 `{first["acceleration_mean_as_recorded"]}`，标准差 `{first["acceleration_stddev_as_recorded"]}`。归一化方向为 `{first["normalized_acceleration_direction_in_livox_frame"]}`，x 符号与上述预期相反。', "",
           f'按当前外参旋转后，与 base_footprint 的 +z 相差 **{first["assuming_livox_frame_equals_body_base_vertical_error_deg"]:.2f}°**。仅根据重力估算、忽略 roll 的调平 pitch 约 **+{first["pitch_that_levels_acceleration_if_no_roll_deg"]:.2f}°**，这只是诊断结果，不能自动写入标定。', "",
           f'NDT 初始化 body→map 的 rpy 为 `{report["initial_map_T_body_rpy_deg"]}` 度；组合现有外参会得到 base_footprint→map rpy `{report["initial_map_T_base_rpy_deg"]}` 度。地图 z 是否严格竖直尚无独立证明，因此此项是第二个一致性线索，不能单独确定外参。', "",
           "需要确认：静态 TF 的方向/俯仰符号是否正确，以及 livox_frame（IMU 轴）是否与 body/mid360_link 同方向。不能用调 timeout 或修改轮速解决轴向矛盾。", "",
           "## 时间数据", "", "| 输入 | 条数 | header平均Hz | header最大间隔(s) | >0.2s间隔数 | 记录时间减header中位(s) | 最大(s) |", "|---|---:|---:|---:|---:|---:|---:|"]
    for kind,t in report["timing"].items():
        lines.append(f'| {kind} | {t["count"]} | {t["header"]["rate_hz"]:.3f} | {t["header_gap_s"]["max"]:.6f} | {t["header_gap_counts_above_s"]["0.2"]} | {t["recording_minus_header_s"]["p50"]:.6f} | {t["recording_minus_header_s"]["max"]:.6f} |')
    lines += ["", "记录时间减 header 的差值可能包含时钟偏差、驱动时间定义、传输/处理和记录延迟，不能仅据此把全部差值归因于网络。", "",
              "上游 gyro_odometer 在回调中比较 `/clock` 与最近 IMU/轮速 header 的年龄，默认 message_timeout_sec=0.2；两路队列都有数据才输出并清空队列。", "",
              "| 时间顺序/时钟 | timeout(s) | 超时检查次数 | 理想回调融合批次数 | 消耗IMU条数 |", "|---|---:|---:|---:|---:|"]
    for order in ["received_ns","stamp_ns"]:
        for threshold in ["0.2","0.8","1.5"]:
            v=report["gyro_odometer_timeout_model"][order][threshold]
            lines.append(f'| {order} | {threshold} | {v["timeout_checks"]} | {v["publish_batches"]} | {v["imu_samples_consumed"]} |')
    lines += ["", "模型假定 TF 正确、DDS 不丢消息，只复现上游队列与超时判定，不是实际融合运行结果。", "",
              "建议对这份录包按原 header 排序并驱动同一采集时间 `/clock`，保留消息内容和原 header，离线试验 timeout 可从 0.8s 开始。它超过实际内部采样最大缺口，但不是可直接用于车辆实机的超时标定。", "",
              f'两路 header 公共区间为 `{overlap_start}` 到 `{overlap_end}` ns。轮速尾部额外 {report["wheel_tail_after_last_imu_s"]:.6f}s 没有新的 IMU，增大 timeout 不能恢复缺失观测。', "",
              "按 header 排序改变了录包的到达时序，只验证采集数据融合；原到达时序下的持续延迟问题仍须单独排查。", ""]
    (args.output/"fusion_input_checks.md").write_text("\n".join(lines),encoding="utf8")
    print(json.dumps({"report":str(args.output/"fusion_input_checks.json"),"first_stationary":first,"initial_base_rpy_deg":report["initial_map_T_base_rpy_deg"]},indent=2))


if __name__=="__main__":
    main()
