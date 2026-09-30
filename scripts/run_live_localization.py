#!/usr/bin/env python3
"""常驻实时定位：订阅外部传感器，保存配置和有限大小的日志，不读取录包。"""
import argparse
from datetime import datetime
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'aps_bag_localization'))
from aps_bag_localization.configuration import load_config
from run_ndt_replay import stop_owned_group
from runtime_support import save_configuration, write_json


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/localization.yaml')
    parser.add_argument('--output', type=Path, help='新的状态与滚动日志目录')
    parser.add_argument('--check', action='store_true', help='只校验配置与地图，不要求 bag 或已连接的传感器')
    args = parser.parse_args()
    try:
        args.configuration = load_config(args.config, mode='live')
    except (ValueError, KeyError, OSError, TypeError, yaml.YAMLError) as error:
        parser.error('配置读取失败：' + str(error))
    config = args.configuration
    if not Path(config['paths']['map']).is_file():
        parser.error('地图不存在，请修改 paths.map：' + config['paths']['map'])
    if config['paths']['map_metadata'] and not Path(config['paths']['map_metadata']).is_file():
        parser.error('地图元数据文件不存在：' + config['paths']['map_metadata'])
    if args.output is None:
        args.output = Path(config['paths']['output_root']) / ('live-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    args.output = args.output.expanduser().resolve()
    if args.output.exists():
        parser.error('结果目录已存在，请使用新目录；之前的运行记录不会覆盖')
    return args


def main():
    args = arguments()
    config = args.configuration
    live, topics = config['live'], config['topics']
    if args.check:
        print(json.dumps({'配置校验': '通过', '模式': 'live', '系统时钟': True,
                          '地图': config['paths']['map'], '需要录包': False,
                          '初始化': live['initialization'],
                          '初值话题': topics['initial_pose_input'],
                          '输入话题': {name: topics[name] for name in ('points', 'imu', 'wheel')},
                          'ROS_DOMAIN_ID': live['domain_id'], '仅本机通信': live['localhost_only']},
                         ensure_ascii=False, indent=2))
        return 0
    os.environ['ROS_DOMAIN_ID'] = str(live['domain_id'])
    os.environ['ROS_LOCALHOST_ONLY'] = '1' if live['localhost_only'] else '0'
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    args.output.mkdir(parents=True)
    snapshot = save_configuration(config, args.output)
    logger = logging.getLogger('cssc_live_launch')
    logger.setLevel(logging.INFO)
    logger.propagate = False
    handler = RotatingFileHandler(args.output / 'launch.log', maxBytes=live['log_max_bytes'],
                                  backupCount=live['log_backup_count'], encoding='utf8')
    handler.setFormatter(logging.Formatter('%(message)s'))
    logger.addHandler(handler)
    started = time.monotonic()
    report = {'status': 'starting', 'mode': 'live', 'use_sim_time': False,
              'source_configuration': str(args.config.resolve()), 'effective_configuration': str(snapshot),
              'map': config['paths']['map'], 'bag_opened': False, 'clock_published': False,
              'raw_sensor_timestamps_modified': False, 'ros_domain_id': live['domain_id'],
              'localhost_only': live['localhost_only'], 'initialization': live['initialization'],
              'initial_pose_input_topic': topics['initial_pose_input'],
              'input_topics': {name: topics[name] for name in ('points', 'imu', 'wheel')},
              'output_counts': {'ekf': 0, 'ndt': 0, 'gyro': 0},
              'last_fusion_status': {}, 'last_preprocessing_status': {},
              'hardware_live_verified': False}
    process, reader, node = None, None, None
    mode_last = None
    failed = False
    rclpy.init()

    def save():
        report['wall_elapsed_s'] = time.monotonic() - started
        write_json(args.output / 'summary.json', report)

    def read_log():
        for line in process.stdout:
            logger.info(line.rstrip('\r\n'))

    def receive_status(message, key):
        report[key] = json.loads(message.data)

    def count_output(message, key):
        report['output_counts'][key] += 1

    try:
        node = Node('cssc_live_monitor')
        subscriptions = []
        for name, key in (('fusion_status', 'last_fusion_status'), ('preprocessing_status', 'last_preprocessing_status')):
            subscriptions.append(node.create_subscription(String, topics[name],
                lambda message, key=key: receive_status(message, key),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        for name, typ, key in (('odometry', Odometry, 'ekf'), ('ndt_pose', PoseWithCovarianceStamped, 'ndt'),
                                ('gyro_twist', TwistWithCovarianceStamped, 'gyro')):
            subscriptions.append(node.create_subscription(typ, topics[name],
                lambda message, key=key: count_output(message, key), config['adapter']['output_queue_depth']))
        command = ['ros2', 'launch', 'aps_bag_localization', 'localization.launch.py', 'config_file:=' + str(snapshot)]
        report['launch_command'] = command
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, encoding='utf8', errors='replace', start_new_session=True)
        reader = threading.Thread(target=read_log, daemon=True)
        reader.start()
        report['status'] = 'running'
        save()
        print('实时定位已启动，等待外部雷达、IMU 和轮速话题。状态目录：' + str(args.output), flush=True)
        if live['initialization'] == 'topic':
            print('请在 RViz 中设置 Fixed Frame 为 ' + config['frames']['map'] +
                  '，使用 2D Pose Estimate 发布后轮中心初值到 ' + topics['initial_pose_input'], flush=True)
        next_save = time.monotonic() + live['status_period_s']
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if process.poll() is not None:
                raise RuntimeError('定位节点退出，详细原因见 launch.log（退出码 ' + str(process.returncode) + '）')
            if time.monotonic() >= next_save:
                save()
                current = report['last_fusion_status'].get('mode')
                if current and current != mode_last:
                    print('定位状态：' + current, flush=True)
                    mode_last = current
                next_save = time.monotonic() + live['status_period_s']
        report['status'] = 'stopped'
    except (KeyboardInterrupt, ExternalShutdownException):
        report['status'] = 'stopped'
    except Exception as error:
        report['status'], report['error'] = 'error', str(error)
        print('实时定位失败：' + str(error), file=sys.stderr, flush=True)
        failed = True
    finally:
        report['launch_shutdown_exit_code'] = stop_owned_group(process)
        if reader:
            reader.join(timeout=3.0)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        save()
        handler.close()
        logger.removeHandler(handler)
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
