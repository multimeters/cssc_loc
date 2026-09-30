#!/usr/bin/env python3
"""Exercise the live entry point with external wall-time sensor publishers.

This is an integration TEST, not the live launcher. Only this test publisher
reads a fixture bag and translates acquisition timestamps to the current clock.
Point coordinates, per-point offsets, IMU measurements and wheel measurements
remain the recorded values. No /clock or old localization topic is published.
Fixture clips and dropout/reset scenarios are not an accuracy ground truth.

Run after sourcing ROS Humble and the built workspace install/setup.bash:
  python3 scripts/test_live_localization.py --bag /path/to/bag --output /new/test
"""
from __future__ import annotations

import argparse
from collections import Counter
import copy
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import time
import traceback

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'aps_bag_localization'))
from aps_bag_localization.configuration import load_config
from audit_bags import Cdr
from run_ndt_replay import stop_owned_group


def distribution(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return None
    return {'count': len(values), 'min': float(values.min()), 'max': float(values.max()),
            'median': float(np.median(values)), 'p95': float(np.percentile(values, 95))}


def fixture_events(bag, types, duration):
    """Return original bytes, acquisition stamp and realistic availability time."""
    events = []
    for path in sorted(bag.glob('*.db3')):
        with sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True) as db:
            for tid, topic, typ in db.execute('SELECT id,name,type FROM topics'):
                if topic not in types:
                    continue
                if typ != types[topic]:
                    raise ValueError(f'{topic}: expected {types[topic]}, recorded {typ}')
                first = None
                for raw, in db.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id', (tid,)):
                    reader = Cdr(raw)
                    header = reader.header()['stamp_ns']
                    first = min(first, header) if first is not None else header
                    # Extra seconds cover small inter-topic start differences.
                    if header > first + round((duration + 3.) * 1e9):
                        continue
                    available = header
                    if typ == 'livox_ros_driver2/msg/CustomMsg':
                        base = reader.scalar('Q', 8)
                        count = reader.scalar('I', 4)
                        for _ in range(4):
                            reader.scalar('B', 1)
                        if reader.scalar('I', 4) != count or not count:
                            raise ValueError('Invalid fixture CustomMsg point count')
                        offsets = np.ndarray((count,), dtype=reader.endian+'u4',
                                             buffer=raw, offset=reader.pos, strides=(20,))
                        available = base + int(offsets.max())
                    events.append((header, available, topic, raw))
    if not events:
        raise ValueError('No fixture sensors found')
    start = min(e[0] for e in events)
    events = [e for e in events if e[0] <= start + round(duration * 1e9)]
    events.sort(key=lambda event: (event[1], event[0], event[2]))
    if set(e[2] for e in events) != set(types):
        raise ValueError('Fixture must contain raw Livox, wheel and IMU topics')
    return start, events


def rejection_counts(status):
    return {key: value for key, value in status.get('counts', {}).items()
            if 'reject' in key or 'stale' in key or 'future' in key}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/localization.yaml')
    parser.add_argument('--bag', type=Path, help='Fixture bag used only by this external test publisher')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, default=67)
    parser.add_argument('--duration', type=float, default=35., help='Main initialized sensor interval, wall seconds at 1x')
    parser.add_argument('--resume-duration', type=float, default=8.)
    parser.add_argument('--ready-timeout', type=float, default=90.)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('--output must be a new directory')
    if not 0 <= args.domain_id <= 232 or args.duration < 5. or args.resume_duration < 3.:
        parser.error('Require valid ROS domain, duration >=5 and resume-duration >=3')
    cfg = load_config(args.config, mode='live')
    fixture_bag = args.bag or (Path(cfg['paths']['bag']) if cfg['paths']['bag'] else None)
    if fixture_bag is None or not fixture_bag.is_dir():
        parser.error('Supply --bag for this test fixture; the live process itself receives paths.bag: null')
    topics = cfg['topics']
    input_types = {topics['wheel']: 'nav_msgs/msg/Odometry', topics['imu']: 'sensor_msgs/msg/Imu',
                   topics['points']: 'livox_ros_driver2/msg/CustomMsg'}
    fixture_duration = 3. + args.duration + 5. + args.resume_duration + 1.
    source_start, events = fixture_events(fixture_bag, input_types, fixture_duration)
    if max(e[0] for e in events) - source_start < round((fixture_duration-2.) * 1e9):
        parser.error('Fixture is too short for the configured test phases')
    args.output.mkdir(parents=True)
    clean = {key: copy.deepcopy(value) for key, value in cfg.items() if not key.startswith('_')}
    clean['runtime'].update(mode='live', use_sim_time=False)
    clean['paths']['bag'] = None
    clean['paths']['output_root'] = str(args.output.resolve())
    clean['live'].update(domain_id=args.domain_id, localhost_only=True, initialization='topic')
    clean['topics']['initial_pose_input'] = '/initialpose'
    test_config = args.output/'live-test.yaml'
    test_config.write_text(yaml.safe_dump(clean, allow_unicode=True, sort_keys=False), encoding='utf8')
    # Deliberately retain a YAML seed. Topic initialization must not auto-use it.
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from rcl_interfaces.srv import GetParameters
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2
    from livox_ros_driver2.msg import CustomMsg
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import String

    rclpy.init()
    node = Node('cssc_live_external_test_publisher')
    ros_types = {topics['wheel']: Odometry, topics['imu']: Imu, topics['points']: CustomMsg}
    pubs = {topic: node.create_publisher(typ, topic, QoSProfile(depth=1000))
            for topic, typ in ros_types.items()}
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    receive_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
    status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    phase = 'startup'
    counts, phase_counts, sent = Counter(), {}, Counter()
    latest = {'fusion': {}, 'preprocessing': {}}
    modes, ages, send_ages, lateness = [], [], [], []
    timing_examples = []
    epoch_mapping_shifts = []
    updates = Counter()
    seen_diagnostics = set()
    output_latest = {}
    subscriptions = []
    checks = {}
    report = {'status': 'running', 'ros_domain_id': args.domain_id,
              'fixture_bag': str(fixture_bag.resolve()), 'live_config_bag': None,
              'test_publisher_only_rebases_timestamps': True,
              'fixture_point_coordinates_offsets_and_sensor_values_unchanged': True,
              'synthetic_clock_published': False, 'ground_truth_accuracy_test': False,
              'fixture_time_basis': 'original acquisition timing at 1x monotonic scheduling; raw cloud delivered at scan end; TEST publisher maps each event with the current wall-minus-monotonic epoch offset and shifts header/timebase together',
              'fixture_uses_fixed_epoch_delta': False,
              'clock_mapping_note': 'WSL can step CLOCK_REALTIME relative to monotonic time. This TEST publisher records and follows that epoch offset without changing the host clock or live freshness thresholds. Hardware clock stability and original network latency are not verified.',
              'checks': checks, 'phases': {}, 'started_wall_ns': time.time_ns()}
    process = None
    log = (args.output/'live-process.log').open('x', encoding='utf8')

    def stamp(stamp_msg):
        return int(stamp_msg.sec)*1_000_000_000 + int(stamp_msg.nanosec)

    def record(message, kind):
        counts[kind] += 1
        phase_counts.setdefault(phase, Counter())[kind] += 1
        if hasattr(message, 'header'):
            age = (time.time_ns()-stamp(message.header.stamp))*1e-9
            if kind in ('ekf', 'ndt'):
                ages.append(age)
                position = message.pose.pose.position
                output_latest[kind] = {'stamp_ns': stamp(message.header.stamp), 'receive_wall_ns': time.time_ns(),
                                       'xyz': [position.x, position.y, position.z]}

    def on_status(message, kind):
        latest[kind] = json.loads(message.data)
        if kind == 'fusion':
            modes.append({'phase': phase, 'wall_ns': time.time_ns(), 'mode': latest[kind].get('mode'),
                          'public_output_enabled': latest[kind].get('public_output_enabled')})

    def on_diagnostics(message):
        for status in message.status:
            if status.name != 'localization: ekf_localizer':
                continue
            ts = stamp(message.header.stamp)
            if ts in seen_diagnostics:
                continue
            seen_diagnostics.add(ts)
            values = {item.key: item.value for item in status.values}
            for kind in ('pose', 'twist'):
                if (int(values.get(kind+'_queue_size', 0)) > 0
                        and int(values.get(kind+'_no_update_count', -1)) == 0
                        and values.get(kind+'_is_passed_delay_gate', '').lower() == 'true'
                        and values.get(kind+'_is_passed_mahalanobis_gate', '').lower() == 'true'):
                    updates[kind] += 1

    for typ, topic, kind in ((Odometry, topics['odometry'], 'ekf'),
                            (PoseWithCovarianceStamped, topics['ndt_pose'], 'ndt'),
                            (TwistWithCovarianceStamped, topics['gyro_twist'], 'gyro'),
                            (PointCloud2, topics['processed_points'], 'processed')):
        subscriptions.append(node.create_subscription(typ, topic,
                             lambda message, key=kind: record(message, key), receive_qos))
    for topic, kind in ((topics['fusion_status'], 'fusion'), (topics['preprocessing_status'], 'preprocessing')):
        subscriptions.append(node.create_subscription(String, topic,
                             lambda message, key=kind: on_status(message, key), status_qos))
    subscriptions.append(node.create_subscription(DiagnosticArray, topics['diagnostics'], on_diagnostics, receive_qos))

    def check_clock_failure():
        failed_clocks = {kind: status for kind, status in latest.items()
                         if status.get('mode') == 'CLOCK_REVERSED_RESTART_REQUIRED'}
        if failed_clocks:
            report['clock_failure_detection'] = {
                'phase': phase, 'observed_wall_ns': time.time_ns(),
                'observed_monotonic_s': time.monotonic(),
                'node_statuses': copy.deepcopy(failed_clocks),
                'epoch_mapping_shifts_so_far': copy.deepcopy(epoch_mapping_shifts),
                'action': 'fail immediately; do not publish remaining fixture; stop only owned live process'}
            raise RuntimeError('Live nodes detected ROS system-clock reversal and require restart; '
                               'the integration test stops immediately and preserves clock evidence')

    def spin(seconds):
        check_clock_failure()
        until = time.monotonic() + max(0., seconds)
        while time.monotonic() < until:
            if process is not None and process.poll() is not None:
                raise RuntimeError(f'Live process exited without requested stop: {process.returncode}')
            rclpy.spin_once(node, timeout_sec=min(.01, max(0., until-time.monotonic())))
            check_clock_failure()

    def check(name, passed, evidence=None):
        checks[name] = {'passed': bool(passed), 'evidence': evidence}
        if not passed:
            raise AssertionError(name + ': ' + repr(evidence))

    def phase_snapshot():
        return {'counts': dict(counts), 'fusion': copy.deepcopy(latest['fusion']),
                'preprocessing': copy.deepcopy(latest['preprocessing']), 'live_process_alive': process.poll() is None}

    def publish_initial():
        message = PoseWithCovarianceStamped()
        message.header.frame_id = cfg['frames']['map']
        message.header.stamp = node.get_clock().now().to_msg()
        for key, value in zip(('x', 'y', 'z'), cfg['_derived']['initial_base_xyz']):
            setattr(message.pose.pose.position, key, value)
        for key, value in zip(('x', 'y', 'z', 'w'), cfg['_derived']['initial_base_quaternion']):
            setattr(message.pose.pose.orientation, key, value)
        for index, value in enumerate(cfg['initial_pose']['covariance_diagonal']):
            message.pose.covariance[index*7] = value
        initial_pub.publish(message)
        sent['/initialpose'] += 1

    def publish_event(event, delta=None, deliberate=False, mapping=None):
        header, _, topic, data = event
        message = deserialize_message(data, ros_types[topic])
        if mapping is not None:
            source_origin, monotonic_origin, original_delta, previous_delta = mapping
            current_mono = time.monotonic()
            current_wall = time.time_ns()
            delta = current_wall-round((current_mono-monotonic_origin)*1e9)-source_origin
            if abs(delta-previous_delta[0]) > 20_000_000:
                epoch_mapping_shifts.append({'phase': phase, 'wall_ns': current_wall,
                    'change_since_previous_s': (delta-previous_delta[0])*1e-9,
                    'change_since_phase_start_s': (delta-original_delta)*1e-9})
                previous_delta[0] = delta
        rebased = header + delta
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(rebased, 1_000_000_000)
        if topic == topics['points']:
            message.timebase = int(message.timebase)+delta
        pubs[topic].publish(message)
        sent[topic] += 1
        if not deliberate:
            age = (time.time_ns()-rebased)*1e-9
            send_ages.append(age)
            if len(timing_examples) < 20 or (age > .25 and sum(item['phase'] == phase for item in timing_examples) < 30):
                timing_examples.append({'phase': phase, 'topic': topic, 'source_stamp_ns': header,
                    'rebased_stamp_ns': rebased, 'publish_wall_ns': time.time_ns(), 'age_s': age})

    def feed(start_seconds, duration, name):
        nonlocal phase
        phase = name
        low = source_start + round(start_seconds*1e9)
        high = low + round(duration*1e9)
        selected = [event for event in events if low <= event[0] < high]
        if not selected:
            raise ValueError('Empty fixture clip: ' + name)
        wall_start = time.time_ns() + 100_000_000
        monotonic_start = time.monotonic() + .1
        delta = wall_start-low
        mapping = (low, monotonic_start, delta, [delta])
        before_counts, before_sent = counts.copy(), sent.copy()
        age_begin = len(send_ages)
        for event in selected:
            due = monotonic_start+(event[1]-low)*1e-9
            spin(due-time.monotonic())
            lateness.append(max(0., time.monotonic()-due))
            publish_event(event, mapping=mapping)
            rclpy.spin_once(node, timeout_sec=0.)
        spin(max(0., monotonic_start+duration-time.monotonic()))
        report['phases'][name] = phase_snapshot()
        report['phases'][name].update(
            new_counts=dict(counts-before_counts), inputs_published=dict(sent-before_sent),
            header_age_at_publish_s=distribution(send_ages[age_begin:]),
            wall_elapsed_s=(time.time_ns()-wall_start)*1e-9,
            monotonic_elapsed_s=time.monotonic()-monotonic_start,
            wall_minus_monotonic_elapsed_s=(time.time_ns()-wall_start)*1e-9-(time.monotonic()-monotonic_start))
        print(json.dumps({'phase': name, 'counts': dict(counts),
                          'fusion_mode': latest['fusion'].get('mode')}, ensure_ascii=False), flush=True)

    try:
        command = ['bash', str(ROOT/'start.sh'), '--mode', 'live', '--no-build', '--config', str(test_config.resolve()),
                   '--output', str((args.output/'live-session').resolve())]
        report['command'] = command
        process = subprocess.Popen(command, cwd=str(ROOT), stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=os.environ.copy())
        deadline = time.monotonic()+args.ready_timeout
        while time.monotonic() < deadline:
            spin(.05)
            if (all(pubs[topic].get_subscription_count() > 0 for topic in pubs)
                    and initial_pub.get_subscription_count() > 0 and latest['fusion'] and latest['preprocessing']):
                break
        else:
            raise TimeoutError('Live input subscriptions/status/initialpose did not become ready')
        report['startup'] = phase_snapshot()
        print(json.dumps({'phase': 'live_graph_ready', 'nodes': len(node.get_node_names_and_namespaces())}), flush=True)
        check('live_started_with_no_bag', clean['paths']['bag'] is None and process.poll() is None)
        check('no_clock_publisher_at_start', node.count_publishers(topics['clock']) == 0)
        check('no_old_processed_cloud_subscription', node.count_subscribers('/cloud_registered_body') == 0)

        # Verify every discovered graph node rather than trusting the launch YAML.
        clock_parameters = {}
        private_tf_helpers = []
        for name, namespace in sorted(set(node.get_node_names_and_namespaces())):
            full_name = namespace.rstrip('/')+'/'+name
            if name.startswith('transform_listener_impl_'):
                # tf2 creates private listener helpers without parameter services.
                # Their parent estimator's clock is checked below; also verify
                # that these helpers have no simulated-clock subscription.
                private_tf_helpers.append(full_name)
                continue
            client = node.create_client(GetParameters, full_name+'/get_parameters')
            request = GetParameters.Request(names=['use_sim_time'])
            until = time.monotonic()+3.
            while not client.service_is_ready() and time.monotonic() < until:
                spin(.03)
            if not client.service_is_ready():
                clock_parameters[full_name] = 'parameter service unavailable'
                node.destroy_client(client)
                continue
            future = client.call_async(request)
            while not future.done() and time.monotonic() < until:
                spin(.01)
            response = future.result() if future.done() else None
            clock_parameters[full_name] = (response.values[0].bool_value
                if response and len(response.values) == 1 and response.values[0].type == 1 else 'unavailable or nonboolean')
            node.destroy_client(client)
        report['graph_use_sim_time'] = clock_parameters
        clock_subscribers = [info.node_namespace.rstrip('/')+'/'+info.node_name
                             for info in node.get_subscriptions_info_by_topic(topics['clock'])]
        report['private_tf_helpers_without_parameter_services'] = private_tf_helpers
        report['clock_subscriber_nodes'] = clock_subscribers
        check('private_tf_helpers_do_not_subscribe_clock', not set(private_tf_helpers).intersection(clock_subscribers), clock_subscribers)
        check('every_graph_node_uses_wall_clock', bool(clock_parameters) and all(value is False for value in clock_parameters.values()), clock_parameters)
        graph = dict(node.get_topic_names_and_types())
        report['input_topic_types'] = {topic: graph.get(topic) for topic in input_types}
        check('raw_imu_wheel_topic_types', all(graph.get(topic) == [typ] for topic, typ in input_types.items()), report['input_topic_types'])
        check('external_publisher_is_only_sensor_source', all(node.count_publishers(topic) == 1 for topic in input_types))

        # Join after graph startup to verify the one-shot map is retained for RViz.
        visible_map = []
        subscriptions.append(node.create_subscription(
            PointCloud2, '/map/output/debug/downsampled_pointcloud_map',
            lambda message: visible_map.append((message.header.frame_id, message.width*message.height)),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        map_deadline = time.monotonic()+5.
        while not visible_map and time.monotonic() < map_deadline:
            spin(.05)
        check('rviz_map_received_by_late_joiner', bool(visible_map) and visible_map[-1][0] == 'map'
              and visible_map[-1][1] > 0, visible_map)

        feed(0., 3., 'before_initial_pose')
        spin(.5)
        report['phases']['before_initial_pose'] = phase_snapshot()
        check('sensors_received_before_initial_pose', counts['processed'] > 0 and latest['preprocessing'].get('counts', {}).get('imu_received', 0) > 0)
        check('no_public_or_ndt_pose_before_initial_pose', counts['ekf'] == 0 and counts['ndt'] == 0, dict(counts))
        publish_initial()
        feed(3., args.duration, 'initialized')
        spin(.7)
        main_phase = report['phases']['initialized']
        raw_main_count = main_phase['inputs_published'][topics['points']]
        check('main_live_processed_cloud_coverage', main_phase['new_counts'].get('processed', 0) >= .9*raw_main_count, main_phase)
        check('main_live_ndt_coverage', main_phase['new_counts'].get('ndt', 0) >= .8*raw_main_count, main_phase)
        check('raw_ndt_gyro_and_public_ekf_active', counts['ndt'] >= 10 and counts['gyro'] >= 10 and counts['ekf'] >= 50, dict(counts))
        check('both_native_ekf_update_routes', updates['pose'] > 0 and updates['twist'] > 0, dict(updates))

        phase = 'bad_timestamp_injection'
        bad_before = phase_snapshot()
        processed_before = counts['processed']
        representatives = {topic: next(event for event in events if event[2] == topic) for topic in input_types}
        for skew in (-5., 5.):
            for topic, event in representatives.items():
                delta = time.time_ns()+round(skew*1e9)-event[0]
                publish_event(event, delta, deliberate=True)
            spin(.1)
        spin(.8)
        bad_after = phase_snapshot()
        report['timestamp_injection'] = {'skews_s': [-5., 5.], 'before': bad_before, 'after': bad_after}
        check('bad_raw_timestamps_do_not_produce_clouds', counts['processed'] == processed_before, [processed_before, counts['processed']])
        for name in ('fusion', 'preprocessing'):
            before_counts = rejection_counts(bad_before[name])
            after_counts = rejection_counts(bad_after[name])
            delta = sum(after_counts.get(key, 0)-before_counts.get(key, 0) for key in after_counts)
            check(name+'_reports_timestamp_rejections', delta >= 2, {'before': before_counts, 'after': after_counts})

        cursor = 3.+args.duration
        before = counts['ekf']
        feed(cursor, 5., 'timestamp_recovery')
        cursor += 5.
        check('recovers_after_bad_timestamps_without_reset', counts['ekf'] > before+10, {'before': before, 'after': counts['ekf']})

        phase = 'sensor_dropout'
        dropout_seconds = max(2.5, cfg['adapter']['prediction_timeout']+1.2,
                              cfg['live']['ndt_idle_timeout_s']+1.2)
        spin(dropout_seconds)
        count_at_stale = counts['ekf']
        status_at_stale = copy.deepcopy(latest['fusion'])
        spin(.8)
        report['dropout'] = {'duration_before_observation_s': dropout_seconds,
            'status': status_at_stale, 'public_count_before_quiet_window': count_at_stale,
            'public_count_after_quiet_window': counts['ekf'], 'quiet_window_s': .8}
        check('sensor_dropout_reports_stale', status_at_stale.get('mode') == 'STALE', status_at_stale)
        check('public_pose_stops_after_sensor_dropout', counts['ekf'] == count_at_stale)
        check('live_process_persists_during_dropout', process.poll() is None)
        check('ndt_sleeps_during_long_dropout', status_at_stale.get('activation', {}).get('ndt') is False
              and status_at_stale.get('counts', {}).get('ndt_idle_sleeps', 0) > 0, status_at_stale)
        before = counts['ekf']
        ndt_before = counts['ndt']
        feed(cursor, args.resume_duration, 'dropout_recovery')
        check('public_pose_resumes_after_fresh_sensors', counts['ekf'] > before+10, {'before': before, 'after': counts['ekf']})
        check('ndt_wakes_and_matches_after_long_dropout', counts['ndt'] > ndt_before
              and latest['fusion'].get('counts', {}).get('ndt_wakeups', 0) > 0,
              {'before': ndt_before, 'after': counts['ndt'], 'fusion': latest['fusion']})

        phase = 'second_initial_pose'
        before_reset = copy.deepcopy(latest['fusion'])
        before = counts['ekf']
        publish_initial()
        feed(0., 5., 'after_second_initial_pose')
        spin(.2)
        report['second_initial_pose'] = {'before': before_reset, 'after': copy.deepcopy(latest['fusion']),
                                        'messages_published': sent['/initialpose']}
        check('outputs_after_second_initial_pose', counts['ekf'] > before+10)
        # The new first-clip geometry is near the original seed; confirm the reset
        # was acknowledged, not merely that old EKF prediction kept publishing.
        init_before = before_reset.get('initial_pose_publications', 0)
        init_after = latest['fusion'].get('initial_pose_publications', 0)
        accepted_before = before_reset.get('counts', {}).get('initial_pose_accepted', 0)
        accepted_after = latest['fusion'].get('counts', {}).get('initial_pose_accepted', 0)
        check('second_initial_pose_acknowledged', init_after > init_before or accepted_after > accepted_before,
              {'publications_before': init_before, 'publications_after': init_after,
               'accepted_before': accepted_before, 'accepted_after': accepted_after})
        reset_distance = float(np.linalg.norm(np.asarray(output_latest['ekf']['xyz'])
                                              - np.asarray(cfg['_derived']['initial_base_xyz'])))
        check('reset_output_returns_near_requested_seed', reset_distance < 1., {'distance_m': reset_distance})
        check('no_clock_publisher_at_end', node.count_publishers(topics['clock']) == 0)
        check('output_epoch_matches_current_wall_clock', bool(ages) and max(abs(value) for value in ages) < 5., distribution(ages))
        check('live_runs_until_explicit_stop', process.poll() is None)
        session_summary = json.loads((args.output/'live-session'/'summary.json').read_text(encoding='utf8'))
        report['live_session_summary_before_stop'] = session_summary
        check('live_runner_confirms_no_bag_or_clock', session_summary.get('bag_opened') is False
              and session_summary.get('clock_published') is False
              and session_summary.get('use_sim_time') is False and session_summary.get('status') == 'running',
              {key: session_summary.get(key) for key in ('bag_opened', 'clock_published', 'use_sim_time', 'status')})
        report['status'] = 'passed'
    except Exception:
        report['status'] = 'failed'
        report['error'] = traceback.format_exc()
    finally:
        report.update(counts=dict(counts), phase_counts={key: dict(value) for key, value in phase_counts.items()},
            input_messages_published=dict(sent), native_ekf_passed_update_cycles=dict(updates),
            final_status=copy.deepcopy(latest), mode_samples=modes, output_header_age_s=distribution(ages),
            test_input_header_age_at_publish_s=distribution(send_ages), scheduling_lateness_s=distribution(lateness),
            test_input_timing_examples=timing_examples,
            test_publisher_epoch_mapping_shifts=epoch_mapping_shifts,
            latest_output=output_latest, explicit_stop_wall_ns=time.time_ns())
        report['owned_live_process_returncode'] = stop_owned_group(process)
        log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        (args.output/'live_integration_report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf8')
    print(json.dumps({'status': report['status'], 'output': str(args.output.resolve()),
                      'checks': len(checks), 'counts': dict(counts), 'error': report.get('error')}, ensure_ascii=False))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
