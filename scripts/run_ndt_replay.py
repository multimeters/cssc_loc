#!/usr/bin/env python3
"""Run isolated NDT replay and record actual ROS callbacks, not CLI snapshots.

Run after sourcing ROS Humble and this workspace's install/setup.bash. Outputs
measure pipeline execution and registration diagnostics; they are not ground
truth accuracy validation. Only subprocess groups created here are terminated.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback


METRIC_TYPES = {
    'nearest_voxel_transformation_likelihood': 'float',
    'transform_probability': 'float',
    'exe_time_ms': 'float',
    'initial_to_result_distance': 'float',
    'iteration_num': 'int',
}


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--map', required=True, type=Path)
    parser.add_argument('--map-metadata', type=Path)
    parser.add_argument('--initial-pose', required=True, nargs=6, type=float,
                        metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'))
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--rate', default=0.5, type=float)
    parser.add_argument('--max-bag-seconds', type=float)
    parser.add_argument('--domain-id', default=57, type=int)
    parser.add_argument('--ready-timeout', default=90.0, type=float)
    parser.add_argument('--drain-seconds', default=5.0, type=float)
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='body')
    parser.add_argument('--ndt-threads', default=2, type=int)
    parser.add_argument('--jump-threshold', default=2.0, type=float,
                        help='Report adjacent accepted position changes above this distance (m).')
    args = parser.parse_args()
    if args.rate <= 0 or not math.isfinite(args.rate):
        parser.error('--rate must be finite and positive')
    if not all(math.isfinite(value) for value in args.initial_pose):
        parser.error('--initial-pose must contain finite numbers')
    if args.max_bag_seconds is not None and args.max_bag_seconds <= 0:
        parser.error('--max-bag-seconds must be positive')
    if not args.bag.is_dir() or not (args.bag / 'metadata.yaml').is_file():
        parser.error('--bag must be a ROS 2 bag directory containing metadata.yaml')
    if not args.map.is_file():
        parser.error('--map does not exist')
    if args.map_metadata is not None and not args.map_metadata.is_file():
        parser.error('--map-metadata does not exist')
    if args.output.exists():
        parser.error('--output must be a new directory; existing results are preserved')
    return args


def bag_metadata(path):
    import yaml
    metadata = yaml.safe_load((path / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    input_count = sum(item['message_count'] for item in metadata.get('topics_with_message_count', [])
                      if item['topic_metadata']['name'] == '/cloud_registered_body')
    return {'expected_pointcloud_messages': input_count,
            'bag_duration_sec': metadata['duration']['nanoseconds'] * 1e-9,
            'bag_start_sec': metadata['starting_time']['nanoseconds_since_epoch'] * 1e-9}


def stop_owned_group(process, timeout=10.0):
    """Terminate only a process group created using start_new_session=True."""
    if process is None:
        return None
    for action in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        # The leader may exit while its children remain. This owned PGID remains
        # safe to address for the duration of this short cleanup.
        try:
            os.killpg(process.pid, action)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout if action == signal.SIGINT else 3.0)
        except subprocess.TimeoutExpired:
            continue
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
    return process.poll()


def finite_stats(values):
    valid = [value for value in values if math.isfinite(value)]
    ordered = sorted(valid)
    return {'count': len(values), 'finite_count': len(valid),
            'nonfinite_count': len(values) - len(valid),
            'minimum': min(valid) if valid else None,
            'maximum': max(valid) if valid else None,
            'mean': sum(valid) / len(valid) if valid else None,
            'median': (ordered[(len(ordered) - 1) // 2] + ordered[len(ordered) // 2]) / 2 if ordered else None}


def make_recorder(args, output):
    import rclpy
    from autoware_internal_debug_msgs.msg import Float32Stamped, Int32Stamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2

    class Recorder(Node):
        def __init__(self):
            super().__init__('aps_ndt_replay_evidence')
            self.input_count = 0
            self.input_points = []
            self.input_frames = set()
            self.input_first = None
            self.input_last = None
            self.input_max_gap = 0.0
            self.input_nonmonotonic = 0
            self.clock_first = None
            self.clock_last = None
            self.accepted_count = 0
            self.accepted_valid_count = 0
            self.accepted_nonfinite = 0
            self.accepted_bad_quaternion = 0
            self.accepted_nonmonotonic = 0
            self.accepted_first = None
            self.accepted_last = None
            self.accepted_frames = set()
            self.accepted_max_gap = 0.0
            self.last_position = None
            self.last_quaternion = None
            self.path_length = 0.0
            self.position_jumps = []
            self.metrics = {name: [] for name in METRIC_TYPES}
            self.files = []
            self.pose_writer = self.open_csv(output / 'trajectory.csv', [
                'stamp_sec', 'stamp_nanosec', 'frame_id', 'body_frame',
                'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'var_x', 'var_y', 'var_yaw', 'valid_numeric'])
            self.input_writer = self.open_csv(output / 'inputs.csv', [
                'stamp_sec', 'stamp_nanosec', 'frame_id', 'point_count', 'clock_sec'])
            self.metric_writer = self.open_csv(output / 'metrics.csv', [
                'metric', 'stamp_sec', 'stamp_nanosec', 'value', 'finite'])
            sensor_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(PointCloud2, '/cloud_registered_body', self.on_input, sensor_qos)
            self.create_subscription(Clock, '/clock', self.on_clock, sensor_qos)
            self.create_subscription(PoseWithCovarianceStamped,
                                     '/localization/ndt/pose_with_covariance', self.on_pose, 100)
            for name, kind in METRIC_TYPES.items():
                message_type = Float32Stamped if kind == 'float' else Int32Stamped
                self.create_subscription(message_type, '/localization/ndt/' + name,
                                         lambda msg, metric=name: self.on_metric(metric, msg), 100)

        def open_csv(self, path, headers):
            handle = path.open('x', newline='', encoding='utf-8')
            self.files.append(handle)
            writer = csv.writer(handle)
            writer.writerow(headers)
            return writer

        def on_clock(self, msg):
            stamp = msg.clock.sec + msg.clock.nanosec * 1e-9
            if self.clock_first is None:
                self.clock_first = stamp
            self.clock_last = stamp

        def on_input(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            points = msg.width * msg.height
            if self.input_first is None:
                self.input_first = stamp
            if self.input_last is not None:
                self.input_max_gap = max(self.input_max_gap, stamp - self.input_last)
                self.input_nonmonotonic += int(stamp <= self.input_last)
            self.input_last = stamp
            self.input_count += 1
            self.input_points.append(points)
            self.input_frames.add(msg.header.frame_id)
            self.input_writer.writerow([msg.header.stamp.sec, msg.header.stamp.nanosec,
                                        msg.header.frame_id, points, self.clock_last])

        def on_pose(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            xyz = [p.x, p.y, p.z]
            quaternion = [q.x, q.y, q.z, q.w]
            finite = all(math.isfinite(value) for value in [stamp, *xyz, *quaternion, *msg.pose.covariance])
            unit = finite and abs(sum(value * value for value in quaternion) - 1.) < 0.01
            self.accepted_count += 1
            self.accepted_nonfinite += int(not finite)
            self.accepted_bad_quaternion += int(finite and not unit)
            self.accepted_frames.add(msg.header.frame_id)
            self.pose_writer.writerow([msg.header.stamp.sec, msg.header.stamp.nanosec,
                                       msg.header.frame_id, args.base_frame, *xyz, *quaternion,
                                       msg.pose.covariance[0], msg.pose.covariance[7],
                                       msg.pose.covariance[35], finite and unit])
            if not finite or not unit:
                return
            if self.accepted_last is not None and stamp <= self.accepted_last:
                self.accepted_nonmonotonic += 1
                return
            if self.accepted_first is None:
                self.accepted_first = stamp
            if self.accepted_last is not None:
                dt = stamp - self.accepted_last
                distance = math.dist(xyz, self.last_position)
                self.path_length += distance
                self.accepted_max_gap = max(self.accepted_max_gap, dt)
                if distance > args.jump_threshold:
                    self.position_jumps.append({'stamp_sec': stamp, 'delta_m': distance,
                                                'delta_time_sec': dt, 'implied_speed_mps': distance / dt})
            self.accepted_last = stamp
            self.last_position = xyz
            self.last_quaternion = quaternion
            self.accepted_valid_count += 1

        def on_metric(self, name, msg):
            value = float(msg.data)
            self.metrics[name].append(value)
            self.metric_writer.writerow([name, msg.stamp.sec, msg.stamp.nanosec, value, math.isfinite(value)])

        def ready(self):
            services = {name for name, _ in self.get_service_names_and_types()}
            subscribers = self.get_subscriptions_info_by_topic('/cloud_registered_body')
            external_input = any(item.node_name != self.get_name() for item in subscribers)
            # Pose publisher + point-cloud subscriber + map service avoid playing
            # before the native stack is discovered. Actual map loading can only
            # start after the bag's simulated clock begins; startup skips are counted.
            return ('/map/get_differential_pointcloud_map' in services and external_input
                    and self.count_publishers('/localization/ndt/pose_with_covariance') > 0)

        def snapshot(self):
            span = 0. if self.accepted_first is None else self.accepted_last - self.accepted_first
            return {
                'input_messages_received': self.input_count,
                'input_point_count_stats': finite_stats(self.input_points),
                'input_frames': sorted(self.input_frames),
                'input_header_first_sec': self.input_first, 'input_header_last_sec': self.input_last,
                'input_header_max_gap_sec': self.input_max_gap,
                'input_nonmonotonic_timestamps': self.input_nonmonotonic,
                'clock_first_sec': self.clock_first, 'clock_last_sec': self.clock_last,
                'accepted_pose_messages': self.accepted_count,
                'valid_monotonic_pose_messages': self.accepted_valid_count,
                'accepted_pose_frames': sorted(self.accepted_frames),
                'accepted_first_sec': self.accepted_first, 'accepted_last_sec': self.accepted_last,
                'accepted_span_sec': span, 'accepted_max_gap_sec': self.accepted_max_gap,
                'accepted_nonfinite': self.accepted_nonfinite,
                'accepted_bad_quaternion': self.accepted_bad_quaternion,
                'accepted_nonmonotonic_timestamps': self.accepted_nonmonotonic,
                'accepted_to_received_input_ratio': self.accepted_count / self.input_count if self.input_count else None,
                'path_length_m': self.path_length, 'last_position_xyz': self.last_position,
                'last_quaternion_xyzw': self.last_quaternion,
                'position_jump_threshold_m': args.jump_threshold,
                'position_jumps': self.position_jumps,
                'registration_attempts': len(self.metrics['iteration_num']),
                'metric_stats': {name: finite_stats(values) for name, values in self.metrics.items()},
                'missing_metrics': [name for name, values in self.metrics.items() if not values],
                'output_sample_checks_passed': (self.accepted_valid_count >= 100 and span >= 5.
                                               and self.accepted_nonfinite == 0
                                               and self.accepted_bad_quaternion == 0
                                               and self.accepted_nonmonotonic == 0),
                'accuracy_verified': False,
                'accuracy_note': 'NDT output and convergence scores are execution evidence; no independent ground-truth accuracy check is performed.',
            }

        def flush(self):
            for handle in self.files:
                handle.flush()

        def close(self):
            for handle in self.files:
                handle.close()

    return Recorder()


def main():
    args = arguments()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    # Environment must be selected before initializing any RMW context.
    import rclpy

    output = args.output.resolve()
    output.mkdir(parents=True)
    metadata = bag_metadata(args.bag)
    report = {
        'status': 'starting', 'bag': str(args.bag.resolve()), 'map': str(args.map.resolve()),
        'map_frame': args.map_frame, 'base_frame': args.base_frame,
        'initial_pose_xyz_rpy': args.initial_pose, 'rate': args.rate,
        'max_bag_seconds': args.max_bag_seconds,
        'ros_domain_id': args.domain_id, 'ros_localhost_only': True,
        'replayed_topics': ['/cloud_registered_body'], 'metadata': metadata,
        'full_bag_completed': False, 'error': None,
    }
    launch_command = [
        'ros2', 'launch', 'aps_ndt_localization', 'ndt_replay.launch.py',
        'map_path:=' + str(args.map.resolve()), 'map_frame:=' + args.map_frame,
        'base_frame:=' + args.base_frame, 'use_sim_time:=true',
        'ndt_threads:=' + str(args.ndt_threads),
    ]
    for name, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), args.initial_pose):
        launch_command.append('initial_' + name + ':=' + str(value))
    if args.map_metadata:
        launch_command.append('map_metadata_path:=' + str(args.map_metadata.resolve()))
    playback_command = [
        'ros2', 'bag', 'play', str(args.bag.resolve()), '--clock', '100',
        '--rate', str(args.rate), '--disable-keyboard-controls',
        '--read-ahead-queue-size', '100', '--delay', '2',
        '--topics', '/cloud_registered_body',
    ]
    report.update({'launch_command': launch_command, 'playback_command': playback_command})
    launch_process = playback_process = recorder = None
    launch_log = playback_log = None
    launched_at = time.monotonic()
    exit_code = 1

    def save():
        if recorder is not None:
            recorder.flush()
            report.update(recorder.snapshot())
        report['wall_elapsed_sec'] = time.monotonic() - launched_at
        temporary = output / 'summary.json.tmp'
        temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        temporary.replace(output / 'summary.json')

    try:
        rclpy.init()
        recorder = make_recorder(args, output)
        launch_log = (output / 'launch.log').open('x', encoding='utf-8')
        playback_log = (output / 'playback.log').open('x', encoding='utf-8')
        launch_process = subprocess.Popen(launch_command, stdout=launch_log, stderr=subprocess.STDOUT,
                                          start_new_session=True)
        deadline = time.monotonic() + args.ready_timeout
        print('Waiting for native NDT point-cloud subscription, pose publisher and map service.', flush=True)
        while not recorder.ready():
            if launch_process.poll() is not None:
                raise RuntimeError('NDT launch exited during startup: ' + str(launch_process.returncode))
            if time.monotonic() > deadline:
                report['graph_services'] = recorder.get_service_names_and_types()
                report['graph_topics'] = recorder.get_topic_names_and_types()
                raise TimeoutError('NDT graph readiness timed out')
            rclpy.spin_once(recorder, timeout_sec=0.1)
        report['graph_ready'] = True
        playback_process = subprocess.Popen(playback_command, stdout=playback_log,
                                            stderr=subprocess.STDOUT, start_new_session=True)
        report['status'] = 'running'
        print('Playback started; recording point clouds, NDT poses and all five metric topics.', flush=True)
        playback_started = time.monotonic()
        planned_duration = metadata['bag_duration_sec']
        if args.max_bag_seconds is not None:
            planned_duration = min(planned_duration, args.max_bag_seconds)
        wall_deadline = playback_started + planned_duration / args.rate + 120.
        last_save = 0.
        cutoff = False
        while playback_process.poll() is None:
            rclpy.spin_once(recorder, timeout_sec=0.05)
            if launch_process.poll() is not None:
                raise RuntimeError('NDT launch exited during playback: ' + str(launch_process.returncode))
            if time.monotonic() > wall_deadline:
                raise TimeoutError('Playback exceeded expected wall duration plus 120 seconds')
            if (args.max_bag_seconds is not None and recorder.clock_last is not None
                    and recorder.clock_last - metadata['bag_start_sec'] >= args.max_bag_seconds):
                cutoff = True
                report['status'] = 'partial'
                report['partial_reason'] = 'requested_max_bag_seconds'
                stop_owned_group(playback_process)
                break
            if time.monotonic() - last_save > 1.:
                save()
                last_save = time.monotonic()
        report['playback_returncode'] = playback_process.poll()
        if not cutoff and playback_process.returncode != 0:
            raise RuntimeError('Bag playback failed: ' + str(playback_process.returncode))
        drain_until = time.monotonic() + args.drain_seconds
        while time.monotonic() < drain_until:
            if launch_process.poll() is not None:
                raise RuntimeError('NDT launch exited while draining output: ' + str(launch_process.returncode))
            rclpy.spin_once(recorder, timeout_sec=0.05)
        report['full_bag_completed'] = not cutoff
        if not cutoff:
            report['status'] = 'completed'
        report['all_expected_inputs_received'] = (
            not cutoff and recorder.input_count == metadata['expected_pointcloud_messages'])
        if recorder.accepted_count == 0:
            report['status'] = 'partial' if cutoff else 'failed'
            report['error'] = 'No accepted NDT pose messages were received'
        elif not cutoff and not report['all_expected_inputs_received']:
            report['status'] = 'partial'
            report['partial_reason'] = 'input_message_count_mismatch'
            report['error'] = 'Recorder did not receive every expected point-cloud message'
        else:
            exit_code = 0
    except KeyboardInterrupt:
        report['status'] = 'partial'
        report['partial_reason'] = 'user_interrupt'
        exit_code = 130
    except BaseException as error:
        report['status'] = 'partial' if playback_process is not None else 'failed'
        report['error'] = type(error).__name__ + ': ' + str(error)
        (output / 'error.log').write_text(traceback.format_exc(), encoding='utf-8')
        print(report['error'], file=sys.stderr, flush=True)
    finally:
        report['launch_returncode_before_cleanup'] = launch_process.poll() if launch_process else None
        report['playback_returncode'] = stop_owned_group(playback_process)
        report['launch_returncode_after_cleanup'] = stop_owned_group(launch_process)
        save()
        if recorder is not None:
            recorder.close()
            recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if launch_log:
            launch_log.close()
        if playback_log:
            playback_log.close()
    print(json.dumps({'status': report['status'], 'accepted_pose_messages': report.get('accepted_pose_messages', 0),
                      'input_messages_received': report.get('input_messages_received', 0),
                      'summary': str(output / 'summary.json'), 'accuracy_verified': False}), flush=True)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
