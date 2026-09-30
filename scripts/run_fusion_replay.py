#!/usr/bin/env python3
"""Exercise native gyro -> EKF <-> NDT using unchanged recorded sensor messages.

This offline estimator test orders inputs by their original acquisition header.
It does not reproduce original recording/network delay and is not a latency test.
Only this script's own ROS launch process group is stopped on exit.
"""
import argparse
from collections import Counter
import csv
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import shutil
import subprocess
import sys
import time
import traceback
from datetime import datetime

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'aps_bag_localization'))
from aps_bag_localization.configuration import load_config

from audit_bags import Cdr
from run_ndt_replay import finite_stats, stop_owned_group

METRICS = ['nearest_voxel_transformation_likelihood', 'transform_probability',
           'exe_time_ms', 'initial_to_result_distance', 'iteration_num']


def input_types(cfg):
    topics = cfg['topics']
    return {topics['wheel']: 'nav_msgs/msg/Odometry', topics['imu']: 'sensor_msgs/msg/Imu',
            topics['points']: 'livox_ros_driver2/msg/CustomMsg'}


def inspect_inputs(bag, expected):
    """Read only metadata and counts so bad topic settings fail before a build."""
    counts = Counter()
    for path in sorted(bag.glob('*.db3')):
        with sqlite3.connect(path.resolve().as_uri()+'?mode=ro', uri=True) as db:
            for tid, name, typ in db.execute('SELECT id,name,type FROM topics'):
                if name not in expected:
                    continue
                if typ != expected[name]:
                    raise ValueError(f'{name} 类型为 {typ}，配置要求 {expected[name]}')
                counts[name] += db.execute('SELECT COUNT(*) FROM messages WHERE topic_id=?', (tid,)).fetchone()[0]
    missing = [name for name in expected if counts[name] == 0]
    if missing:
        raise ValueError('录包缺少配置指定的话题或话题为空：'+', '.join(missing))
    return dict(counts)


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT/'config'/'localization.yaml')
    p.add_argument('--output', type=Path)
    p.add_argument('--rate', type=float, help='本次回放倍速覆盖；实际值写入配置快照')
    p.add_argument('--max-bag-seconds', type=float)
    p.add_argument('--check', action='store_true', help='只校验配置与数据，不启动ROS或回放')
    args = p.parse_args()
    try:
        cfg = load_config(args.config, mode='replay')
    except (ValueError, KeyError, OSError, TypeError, yaml.YAMLError) as error:
        p.error('配置读取失败：'+str(error))
    if not cfg['paths']['bag']:
        p.error('回放模式需要 paths.bag；实时模式不需要录包')
    args.bag, args.map = Path(cfg['paths']['bag']), Path(cfg['paths']['map'])
    if args.output is None:
        args.output = Path(cfg['paths']['output_root'])/('fusion-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    if args.rate is not None:
        cfg['replay']['rate'] = args.rate
    if args.max_bag_seconds is not None:
        cfg['replay']['max_bag_seconds'] = args.max_bag_seconds
    args.rate = cfg['replay']['rate']
    args.max_bag_seconds = cfg['replay']['max_bag_seconds']
    args.domain_id = cfg['replay']['domain_id']
    args.warmup_seconds = cfg['replay']['warmup_seconds']
    args.initial_body_pose = cfg['initial_pose']['xyz_m']+cfg['initial_pose']['rpy_rad']
    args.mount_pitch_deg = cfg['extrinsics']['rear_to_lidar']['rpy_deg'][1]
    args.configuration = cfg
    if not cfg['runtime']['use_sim_time']:
        p.error('录包回放必须设置 runtime.use_sim_time: true，避免历史采集时间与墙钟混用')
    if not args.bag.is_dir() or not args.map.is_file():
        p.error('录包目录或地图文件不存在，请修改主YAML的paths：'+str(args.bag)+' / '+str(args.map))
    if not list(args.bag.glob('*.db3')):
        p.error('当前回放需要包含SQLite .db3文件的ROS2 bag目录')
    try:
        args.available_input_counts = inspect_inputs(args.bag, input_types(cfg))
    except (sqlite3.Error, ValueError) as error:
        p.error('录包预检失败：'+str(error))
    if args.output.exists():
        p.error('Output must be a new directory; previous runs are preserved')
    if not math.isfinite(args.rate) or args.rate <= 0:
        p.error('rate must be finite and positive')
    if args.max_bag_seconds is not None and (not math.isfinite(args.max_bag_seconds) or args.max_bag_seconds <= 0):
        p.error('max-bag-seconds must be finite and positive')
    return args


def load_inputs(bag, inputs):
    """No message header, coordinate, or payload modifications."""
    events = []
    for path in sorted(bag.glob('*.db3')):
        with sqlite3.connect(path.resolve().as_uri()+'?mode=ro', uri=True) as db:
            topics = {row[0]: row[1:] for row in db.execute('SELECT id,name,type FROM topics')}
            ids = [i for i, (name, typ) in topics.items() if name in inputs]
            for i in ids:
                name, typ = topics[i]
                if typ != inputs[name]:
                    raise ValueError(f'Unexpected type for {name}: {typ}')
            query = 'SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ('
            query += ','.join('?' for _ in ids)+') ORDER BY timestamp,id'
            for tid, recorded, data in db.execute(query, ids):
                name, _ = topics[tid]
                h = Cdr(data).header()
                events.append((h['stamp_ns'], recorded, name, data))
    if not events:
        raise ValueError('No usable sensor messages')
    events.sort(key=lambda e: (e[0], e[1]))
    missing = set(inputs)-{e[2] for e in events}
    if missing:
        raise ValueError(f'Missing required sensor streams: {missing}')
    return events


def main():
    args = arguments()
    cfg = args.configuration
    topics, frames, replay = cfg['topics'], cfg['frames'], cfg['replay']
    limits = cfg['validation']
    if args.check:
        print(json.dumps({'配置校验': '通过', '配置': str(args.config.resolve()),
                          '录包': str(args.bag), '地图': str(args.map),
                          '输入条数': args.available_input_counts,
                          '外参': cfg['extrinsics'], '回放': replay}, ensure_ascii=False, indent=2))
        return 0
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1' if replay['localhost_only'] else '0'
    import rclpy
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2
    from livox_ros_driver2.msg import CustomMsg
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
    from rosgraph_msgs.msg import Clock
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import String
    from autoware_internal_debug_msgs.msg import Float32Stamped, Int32Stamped

    inputs = input_types(cfg)
    events = load_inputs(args.bag, inputs)
    t0, source_end = events[0][0], events[-1][0]
    if args.max_bag_seconds:
        events = [e for e in events if e[0] <= t0+int(args.max_bag_seconds*1e9)]
    args.output.mkdir(parents=True)
    # A self-contained effective configuration: ROS launch consumes exactly this
    # snapshot, so editing the source YAML during a run cannot change its TF.
    snapshot_cfg = {k:v for k,v in cfg.items() if not k.startswith('_')}
    snapshot_cfg['native_parameters'] = {}
    native_dir = args.output/'config'/'native'
    native_dir.mkdir(parents=True)
    for name, source in cfg['native_parameters'].items():
        target = native_dir/(name+'.yaml')
        shutil.copy2(source, target)
        snapshot_cfg['native_parameters'][name] = str(target.resolve())
    config_snapshot = args.output/'config'/'localization.yaml'
    config_snapshot.write_text(yaml.safe_dump(snapshot_cfg, allow_unicode=True, sort_keys=False), encoding='utf8')
    files, writers, rows = {}, {}, {'ekf': [], 'ndt': [], 'gyro': []}
    metric_values = {name: [] for name in METRICS}
    diag_messages = Counter()
    update_evidence = {kind: Counter() for kind in ('pose', 'twist')}
    update_stamps = {kind: [] for kind in update_evidence}
    diagnostic_stamps = set()
    fusion_status = {}
    preprocessing_status = {}
    scan_samples = []
    processed_stamps = []
    sample_directory = args.output/'scan_samples'
    sample_directory.mkdir()
    counts = Counter()
    report = {
        'status': 'starting', 'source_bag': str(args.bag.resolve()), 'map': str(args.map.resolve()),
        'initial_body_pose_xyz_rpy': args.initial_body_pose, 'mount_pitch_deg': args.mount_pitch_deg,
        'mount_source': cfg['extrinsics']['rear_to_lidar']['provenance'],
        'source_configuration': str(args.config.resolve()), 'effective_configuration': str(config_snapshot.resolve()),
        'input_time_basis': 'original acquisition header timestamps, sorted; payloads unchanged',
        'recording_delay_reproduced': False, 'source_input_counts': dict(Counter(e[2] for e in events)),
        'input_header_ranges_ns': {name: [min(e[0] for e in events if e[2] == name),
                                            max(e[0] for e in events if e[2] == name)]
                                   for name in inputs},
        'pointcloud_input_topic': topics['points'],
        'pointcloud_input_type': inputs[topics['points']],
        'processed_pointcloud_topic': topics['processed_points'],
        'processed_cloud_reference_time': 'scan end = timebase + max(offset_time)',
        'recorded_processed_pointcloud_used': False,
        'ros_domain_id': args.domain_id, 'rate': args.rate, 'max_bag_seconds': args.max_bag_seconds,
        'source_header_span_s': (source_end-t0)/1e9, 'accuracy_verified': False,
        'native_pipeline': 'wheel speed + IMU -> gyro_odometer -> EKF; NDT -> EKF; EKF -> NDT',
        'recorded_localization_tf_used': False, 'full_bag_completed': False,
        'warmup_clock_seconds': args.warmup_seconds, 'tail_clock_seconds': replay['tail_seconds'],
        'warmup_contains_synthetic_sensor_measurements': False,
    }

    def csv_file(key, headers):
        files[key] = (args.output/(key+'.csv')).open('x', newline='')
        writers[key] = csv.writer(files[key])
        writers[key].writerow(headers)
    pose_headers = ['stamp_ns', 'frame_id', 'child_frame_id', 'x', 'y', 'z',
                    'qx', 'qy', 'qz', 'qw', 'vx', 'wz', 'var_x', 'var_y', 'var_yaw']
    csv_file('ekf', pose_headers)
    csv_file('ndt', pose_headers)
    csv_file('gyro', ['stamp_ns', 'frame_id', 'vx', 'wz', 'var_vx', 'var_wz'])
    csv_file('metrics', ['metric', 'stamp_ns', 'value'])
    csv_file('inputs', ['topic', 'stamp_ns', 'recording_ns'])
    files['diagnostics'] = (args.output/'diagnostics.jsonl').open('x')
    files['fusion_status'] = (args.output/'fusion_status.jsonl').open('x')
    files['preprocessing_status'] = (args.output/'preprocessing_status.jsonl').open('x')
    rclpy.init()
    node = Node('aps_fusion_test_driver')
    types = {topics['wheel']: Odometry, topics['imu']: Imu, topics['points']: CustomMsg}
    qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
    pubs = {topic: node.create_publisher(typ, topic, qos) for topic, typ in types.items()}
    clock_pub = node.create_publisher(Clock, topics['clock'], 10)
    subscriptions = []

    def stamp_ns(stamp):
        return stamp.sec*1_000_000_000+stamp.nanosec

    def on_pose(msg, key):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        vx = msg.twist.twist.linear.x if key == 'ekf' else float('nan')
        wz = msg.twist.twist.angular.z if key == 'ekf' else float('nan')
        child = msg.child_frame_id if key == 'ekf' else frames['base']
        row = [stamp_ns(msg.header.stamp), msg.header.frame_id, child,
               p.x, p.y, p.z, q.x, q.y, q.z, q.w, vx, wz,
               msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[35]]
        rows[key].append(row)
        writers[key].writerow(row)

    def on_gyro(msg):
        row = [stamp_ns(msg.header.stamp), msg.header.frame_id,
               msg.twist.twist.linear.x, msg.twist.twist.angular.z,
               msg.twist.covariance[0], msg.twist.covariance[35]]
        rows['gyro'].append(row)
        writers['gyro'].writerow(row)

    def on_metric(msg, name):
        metric_values[name].append(float(msg.data))
        writers['metrics'].writerow([name, stamp_ns(msg.stamp), msg.data])

    def on_diagnostics(msg):
        for status in msg.status:
            level = status.level[0] if isinstance(status.level, bytes) else int(status.level)
            diag_messages[(status.name, level, status.message)] += 1
            values = {v.key: v.value for v in status.values}
            files['diagnostics'].write(json.dumps({
                'stamp_ns': stamp_ns(msg.header.stamp), 'name': status.name,
                'level': level, 'message': status.message, 'values': values})+'\n')
            ts = stamp_ns(msg.header.stamp)
            if status.name != 'localization: ekf_localizer' or ts in diagnostic_stamps:
                continue
            diagnostic_stamps.add(ts)
            for kind in update_evidence:
                if kind+'_no_update_count' not in values:
                    continue
                stats = update_evidence[kind]
                stats['diagnostic_samples'] += 1
                eligible = int(values.get(kind+'_queue_size', 0)) > 0
                stats['cycles_with_queued_measurements'] += int(eligible)
                if int(values[kind+'_no_update_count']) == 0 and eligible:
                    stats['native_reported_update_cycles'] += 1
                    update_stamps[kind].append(ts)
                for gate in ('delay', 'mahalanobis'):
                    if values.get(kind+'_is_passed_'+gate+'_gate', '').lower() == 'false':
                        stats[gate+'_gate_rejection_samples'] += 1

    def on_status(msg):
        fusion_status.clear()
        fusion_status.update(json.loads(msg.data))
        files['fusion_status'].write(msg.data+'\n')

    def on_preprocessing_status(msg):
        preprocessing_status.clear()
        preprocessing_status.update(json.loads(msg.data))
        files['preprocessing_status'].write(msg.data+'\n')

    def on_processed_cloud(msg):
        # Geometry evidence comes from this run's raw-input processing output,
        # never from the recorded /cloud_registered_body or old localization TF.
        import numpy as np
        index = len(processed_stamps)
        processed_stamps.append(stamp_ns(msg.header.stamp))
        if index % limits['scan_sample_stride']:
            return
        names = ('x', 'y', 'z', 'intensity')
        fields = {field.name: field for field in msg.fields}
        dtype = np.dtype({'names': names,
                          'formats': [('>f4' if msg.is_bigendian else '<f4')]*4,
                          'offsets': [fields[name].offset for name in names],
                          'itemsize': msg.point_step})
        points = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                            strides=(msg.row_step, msg.point_step))
        values = np.column_stack([points[name].ravel() for name in names])
        filename = f'scan-{index:06d}.npy'
        np.save(sample_directory/filename, values)
        scan_samples.append({'index': index, 'stamp_ns': stamp_ns(msg.header.stamp),
                             'frame_id': msg.header.frame_id, 'npy': filename})

    subscriptions += [
        node.create_subscription(Odometry, topics['odometry'],
                                 lambda msg: on_pose(msg, 'ekf'), qos),
        node.create_subscription(PoseWithCovarianceStamped,
                                 topics['ndt_pose'],
                                 lambda msg: on_pose(msg, 'ndt'), qos),
        node.create_subscription(TwistWithCovarianceStamped,
                                 topics['gyro_twist'], on_gyro, qos),
        node.create_subscription(DiagnosticArray, topics['diagnostics'], on_diagnostics, qos),
        node.create_subscription(String, topics['fusion_status'], on_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(String, topics['preprocessing_status'], on_preprocessing_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(PointCloud2, topics['processed_points'], on_processed_cloud, qos),
    ]
    for name in METRICS:
        typ = Int32Stamped if name == 'iteration_num' else Float32Stamped
        subscriptions.append(node.create_subscription(typ, topics['metrics_prefix'].rstrip('/')+'/'+name,
                              lambda msg, name=name: on_metric(msg, name), qos))

    def clock(ns):
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(int(ns), 1_000_000_000)
        clock_pub.publish(msg)

    launch = ['ros2', 'launch', 'aps_bag_localization', 'fusion_replay.launch.py',
              'config_file:='+str(config_snapshot.resolve())]
    report['launch_command'] = launch
    process = None
    started = time.monotonic()

    def snapshot():
        out = {}
        for key in ('ekf', 'ndt'):
            data = rows[key]
            gaps = [(b[0]-a[0])/1e9 for a,b in zip(data, data[1:])]
            jumps = [math.dist(a[3:6], b[3:6]) for a,b in zip(data, data[1:])]
            out[key] = {
                'count': len(data), 'span_s': (data[-1][0]-data[0][0])/1e9 if data else 0,
                'first_ns': data[0][0] if data else None, 'last_ns': data[-1][0] if data else None,
                'max_gap_s': max(gaps, default=0), 'non_increasing_stamps': sum(x<=0 for x in gaps),
                'max_step_m': max(jumps, default=0), 'path_length_m': sum(jumps),
                'invalid_poses': sum(not all(math.isfinite(x) for x in r[3:10]) or
                                     abs(sum(x*x for x in r[6:10])-1)>.01 for r in data),
                'invalid_xy_yaw_variances': sum(any(not math.isfinite(v) or v <= 0 for v in r[12:15])
                                               for r in data),
                'frames': sorted({(r[1],r[2]) for r in data}),
                'first_xyz': data[0][3:6] if data else None, 'last_xyz': data[-1][3:6] if data else None,
            }
        out.update(gyro_messages=len(rows['gyro']), input_messages_published=dict(counts),
                   ekf_update_evidence={k:{**dict(v),
                       'successful_cycle_span_s': (update_stamps[k][-1]-update_stamps[k][0])/1e9 if update_stamps[k] else 0,
                       'max_successful_cycle_gap_s': max(((b-a)/1e9 for a,b in zip(update_stamps[k],update_stamps[k][1:])),default=0),
                       'note': 'Native update path, with repeated smoothing cycles; not unique message count.'}
                       for k,v in update_evidence.items()},
                   last_fusion_status=dict(fusion_status),
                   last_preprocessing_status=dict(preprocessing_status),
                   processed_clouds_received=len(processed_stamps),
                   processed_cloud_non_increasing_stamps=sum(b <= a for a,b in zip(processed_stamps, processed_stamps[1:])),
                   geometry_scan_samples=len(scan_samples),
                   metric_stats={k:finite_stats(v) for k,v in metric_values.items()},
                   diagnostics=[{'name':k[0], 'level':k[1], 'message':k[2], 'count':v}
                                for k,v in diag_messages.most_common(150)])
        ratio = len(rows['ndt'])/max(1,counts[topics['points']])
        out['ndt_accepted_to_published_ratio'] = ratio
        out['estimator_checks_passed'] = (
            len(processed_stamps) == counts[topics['points']] and
            out['processed_cloud_non_increasing_stamps'] == 0 and
            len(rows['gyro']) >= limits['min_gyro_messages'] and ratio >= limits['min_ndt_acceptance_ratio'] and
            out['ekf']['count'] >= limits['min_ekf_messages'] and
            out['ekf']['invalid_poses'] == 0 and out['ndt']['invalid_poses'] == 0 and
            out['ekf']['invalid_xy_yaw_variances'] == 0 and out['ndt']['invalid_xy_yaw_variances'] == 0 and
            out['ekf']['non_increasing_stamps'] == 0 and out['ndt']['non_increasing_stamps'] == 0 and
            all(out[k]['max_step_m'] < limits['max_pose_step_m'] for k in ('ekf','ndt')) and
            all(out[k]['max_gap_s'] < limits['max_pose_gap_s'] for k in ('ekf','ndt')) and
            out['ekf']['frames'] == [(frames['map'], frames['base'])] and
            out['ndt']['frames'] == [(frames['map'], frames['base'])] and
            all(metric_values[k] for k in METRICS) and
            all(v['native_reported_update_cycles'] > 0 for v in update_evidence.values()) and
            all(out['ekf_update_evidence'][k]['successful_cycle_span_s'] > limits['min_update_span_ratio']*(events[-1][0]-t0)/1e9
                for k in update_evidence))
        return out

    def save():
        report.update(snapshot())
        report['wall_elapsed_s'] = time.monotonic()-started
        for stream in files.values():
            stream.flush()
        (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
        (args.output/'scan_samples.json').write_text(json.dumps({
            'source_topic': topics['processed_points'], 'raw_source_topic': topics['points'],
            'scan_samples': scan_samples}, indent=2)+'\n')

    def spin_for(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.01, max(0,end-time.monotonic())))

    try:
        with (args.output/'launch.log').open('x') as log:
            process = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            ready_end = time.monotonic()+replay['ready_timeout']
            while True:
                spin_for(.1)
                if process.poll() is not None:
                    raise RuntimeError('Launch exited before sensors were ready; see launch.log')
                services = {name for name,_ in node.get_service_names_and_types()}
                if all(pub.get_subscription_count() for pub in pubs.values()) and \
                        cfg['services']['map_loader'] in services:
                    break
                if time.monotonic() > ready_end:
                    raise TimeoutError('Fusion subscribers / map service not discovered')
            # Synthetic clock warmup only; no synthetic sensor observations.
            warm_start = t0-int(args.warmup_seconds*1e9)
            clock(warm_start)
            spin_for(.5)
            wall0 = time.monotonic()
            while time.monotonic()-wall0 < args.warmup_seconds:
                clock(min(t0-1_000_000, warm_start+int((time.monotonic()-wall0)*1e9)))
                spin_for(.005)
            report['status'] = 'running'
            wall0, next_status, cursor = time.monotonic(), time.monotonic()+10, 0
            final_ns = events[-1][0]
            while cursor < len(events):
                now = min(final_ns, t0+int((time.monotonic()-wall0)*args.rate*1e9))
                clock(now)
                while cursor < len(events) and events[cursor][0] <= now:
                    ts, recorded, name, data = events[cursor]
                    pubs[name].publish(deserialize_message(data, types[name]))
                    counts[name] += 1
                    writers['inputs'].writerow([name,ts,recorded])
                    cursor += 1
                spin_for(.003)
                if process.poll() is not None:
                    raise RuntimeError('Fusion launch exited during replay')
                if time.monotonic() > next_status:
                    save()
                    print(json.dumps({'seconds':round((now-t0)/1e9,1),
                                      'inputs':dict(counts), 'gyro':len(rows['gyro']),
                                      'ndt':len(rows['ndt']), 'ekf':len(rows['ekf'])}), flush=True)
                    next_status = time.monotonic()+10
            # Drain pending NDT work, then advance the configured clock tail.
            # This short tail is recorded as extrapolation outside sensor coverage.
            spin_for(replay['drain_seconds'])
            tail_ns = round(replay['tail_seconds']*1e9)
            for offset in range(1, math.ceil(tail_ns/10_000_000)+1):
                clock(final_ns+min(tail_ns,offset*10_000_000))
                spin_for(.02)
            spin_for(replay['drain_seconds'])
            report['full_bag_completed'] = args.max_bag_seconds is None
            report['status'] = 'completed' if report['full_bag_completed'] else 'partial'
    except (KeyboardInterrupt, Exception) as exc:
        report['status'] = 'interrupted' if isinstance(exc, KeyboardInterrupt) else 'error'
        report['error'] = str(exc)
        (args.output/'error.txt').write_text(traceback.format_exc())
    finally:
        report['launch_shutdown_exit_code'] = stop_owned_group(process)
        save()
        for stream in files.values():
            stream.close()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({k:report[k] for k in ('status','full_bag_completed','estimator_checks_passed',
                                           'ndt_accepted_to_published_ratio','gyro_messages')},indent=2))
    return 0 if report['status'] in ('completed','partial') and report['estimator_checks_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
