"""Validate and resolve the one runtime YAML before ROS is installed or started."""
import copy
import math
from pathlib import Path
import re

import yaml

from .geometry import body_pose_to_base, quaternion_multiply, quaternion_rpy


FRAME_KEYS = ('map', 'rear', 'base', 'lidar', 'cloud', 'imu', 'wheel', 'ndt_debug')
TOPIC_KEYS = ('wheel', 'imu', 'points', 'processed_points', 'preprocessing_status',
              'wheel_twist', 'imu_base', 'ndt_points',
              'initial_pose', 'initial_pose_input', 'odometry', 'pose', 'ndt_pose', 'ndt_pose_stamped',
              'ekf_prediction', 'ekf_odometry', 'gyro_twist', 'fusion_status',
              'diagnostics', 'ekf_tf', 'ndt_tf', 'metrics_prefix', 'clock')
EXTRINSIC_FRAMES = {
    'rear_to_base': ('rear', 'base'), 'rear_to_lidar': ('rear', 'lidar'),
    'lidar_to_cloud': ('lidar', 'cloud'), 'lidar_to_imu': ('lidar', 'imu'),
}
POSITIVE_ADAPTER_KEYS = (
    'wheel_variance_floor', 'angular_variance_floor', 'sensor_timeout', 'ndt_timeout',
    'prediction_timeout', 'scan_wait_timeout', 'tick_period', 'status_period',
)
QUEUE_KEYS = ('max_pending_scans', 'sensor_queue_depth', 'cloud_queue_depth',
              'output_queue_depth', 'initial_pose_queue_depth', 'status_queue_depth')


def require_keys(value, keys, location):
    if not isinstance(value, dict):
        raise ValueError(location + ' must be a mapping')
    missing = set(keys) - value.keys()
    if missing:
        raise ValueError(location + ' missing keys: ' + ', '.join(sorted(missing)))


def number(value, location, minimum=None, strict=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(location + ' must be a finite number')
    if minimum is not None and (value <= minimum if strict else value < minimum):
        raise ValueError(location + (' must exceed ' if strict else ' must be at least ') + str(minimum))
    return float(value)


def vector(value, size, location, positive=False):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(location + ' must contain ' + str(size) + ' numbers')
    return [number(item, location, 0. if positive else None, positive) for item in value]


def boolean(value, location):
    if not isinstance(value, bool):
        raise ValueError(location + ' must be a YAML boolean')
    return value


def resolve_references(value, config):
    """Resolve exact ${section.key} values without evaluating code or strings."""
    if isinstance(value, dict):
        return {key: resolve_references(item, config) for key, item in value.items()}
    if isinstance(value, list):
        return [resolve_references(item, config) for item in value]
    if isinstance(value, str) and '${' in value:
        match = re.fullmatch(r'\$\{([a-zA-Z_][a-zA-Z0-9_.]*)\}', value)
        if not match:
            raise ValueError('References must occupy the entire YAML value: ' + value)
        resolved = config
        for key in match.group(1).split('.'):
            if not isinstance(resolved, dict) or key not in resolved:
                raise ValueError('Unknown configuration reference: ' + value)
            resolved = resolved[key]
        return copy.deepcopy(resolved)
    return value


def flatten_parameters(values, prefix=''):
    result = {}
    for key, value in values.items():
        name = prefix + key
        if isinstance(value, dict):
            result.update(flatten_parameters(value, name + '.'))
        else:
            result[name] = value
    return result


def load_config(path, mode=None):
    """Return normalized config with absolute paths and derived native parameters.

    Sensor/map path existence is checked by the runner or launch, so configuration
    can be validated before a dataset is copied. Referenced native YAML files must
    exist. `_derived` and `_native_parameters` are regenerated on every load;
    previously saved resolved configurations cannot override their calculations.
    The serialized runtime mode and clock choice must agree. An explicit mode
    override then changes both together before native parameters are resolved.
    """
    source = Path(path).expanduser().resolve()
    with source.open(encoding='utf-8') as handle:
        config = yaml.safe_load(handle)
    require_keys(config, ('schema_version', 'paths', 'runtime', 'live', 'replay', 'frames', 'topics',
                         'services', 'extrinsics', 'initial_pose', 'livox', 'adapter', 'native_parameters', 'validation'), 'config')
    config = copy.deepcopy(config)
    if type(config['schema_version']) is not int or config['schema_version'] != 1:
        raise ValueError('Only schema_version 1 is supported')
    require_keys(config['paths'], ('bag', 'map', 'map_metadata', 'output_root'), 'paths')
    for name, value in config['paths'].items():
        if name == 'bag' and value is None:
            continue
        if name == 'map_metadata' and value in (None, ''):
            config['paths'][name] = ''
            continue
        if not isinstance(value, str) or not value:
            raise ValueError('paths.' + name + ' must be a nonempty path')
        candidate = Path(value).expanduser()
        config['paths'][name] = str((source.parent / candidate).resolve())
    require_keys(config['runtime'], ('mode', 'use_sim_time'), 'runtime')
    boolean(config['runtime']['use_sim_time'], 'runtime.use_sim_time')
    serialized_mode = config['runtime']['mode']
    if serialized_mode not in ('live', 'replay'):
        raise ValueError('runtime.mode must be live or replay')
    if config['runtime']['use_sim_time'] != (serialized_mode == 'replay'):
        raise ValueError('runtime.mode/use_sim_time must agree: live/false or replay/true')
    if mode is not None and mode not in ('live', 'replay'):
        raise ValueError('mode override must be live or replay')
    selected_mode = serialized_mode if mode is None else mode
    config['runtime']['mode'] = selected_mode
    config['runtime']['use_sim_time'] = selected_mode == 'replay'
    live = config['live']
    require_keys(live, ('domain_id', 'localhost_only', 'initialization', 'max_sensor_age_s', 'ndt_idle_timeout_s',
                        'motion_wait_timeout_s',
                        'future_tolerance_s', 'initial_pose_ack_position_m', 'initial_pose_ack_angle_rad',
                        'status_period_s', 'log_max_bytes', 'log_backup_count'), 'live')
    if type(live['domain_id']) is not int or not 0 <= live['domain_id'] <= 232:
        raise ValueError('live.domain_id must be an integer from 0 to 232')
    boolean(live['localhost_only'], 'live.localhost_only')
    if live['initialization'] not in ('topic', 'config'):
        raise ValueError('live.initialization must be topic or config')
    live['max_sensor_age_s'] = number(live['max_sensor_age_s'], 'live.max_sensor_age_s', 0., True)
    live['motion_wait_timeout_s'] = number(live['motion_wait_timeout_s'], 'live.motion_wait_timeout_s', 0., True)
    if live['motion_wait_timeout_s'] >= live['max_sensor_age_s']:
        raise ValueError('live.motion_wait_timeout_s must be below live.max_sensor_age_s')
    live['ndt_idle_timeout_s'] = number(live['ndt_idle_timeout_s'], 'live.ndt_idle_timeout_s',
                                        live['max_sensor_age_s'], True)
    live['future_tolerance_s'] = number(live['future_tolerance_s'], 'live.future_tolerance_s', 0.)
    live['status_period_s'] = number(live['status_period_s'], 'live.status_period_s', 0., True)
    for name in ('initial_pose_ack_position_m', 'initial_pose_ack_angle_rad'):
        live[name] = number(live[name], 'live.' + name, 0., True)
    for name in ('log_max_bytes', 'log_backup_count'):
        if type(live[name]) is not int or live[name] < 1:
            raise ValueError('live.' + name + ' must be a positive integer')
    frames = config['frames']
    require_keys(frames, FRAME_KEYS, 'frames')
    for name in FRAME_KEYS:
        if not isinstance(frames[name], str) or not frames[name] or frames[name].startswith('/'):
            raise ValueError('frames.' + name + ' must be a nonempty frame without leading slash')
    if frames['base'] != 'base_link':
        raise ValueError('frames.base must be base_link: the locked native EKF hardcodes this frame')
    if len(set(frames[name] for name in FRAME_KEYS if name != 'wheel')) != len(FRAME_KEYS) - 1:
        raise ValueError('Static-tree and map/debug frames must have distinct names')
    for section, keys in (('topics', TOPIC_KEYS),
                          ('services', ('ekf_activation', 'ndt_activation', 'map_loader'))):
        require_keys(config[section], keys, section)
        for name in keys:
            item = config[section][name]
            if not isinstance(item, str) or not item.startswith('/') or ' ' in item:
                raise ValueError(section + '.' + name + ' must be an absolute ROS name')
    sensor_topics = [config['topics'][name] for name in ('wheel', 'imu', 'points')]
    if len(set(sensor_topics)) != 3:
        raise ValueError('Wheel, IMU and cloud input topics must differ')
    if config['topics']['processed_points'] in (*sensor_topics, config['topics']['ndt_points']):
        raise ValueError('Processed PointCloud2 must have a separate topic from raw inputs and NDT relay')
    if config['topics']['ekf_prediction'] == config['topics']['ndt_pose']:
        raise ValueError('EKF prediction and NDT observation topics must differ')
    if config['topics']['initial_pose_input'] in [config['topics'][name] for name in
                                                ('initial_pose', 'pose', 'ndt_pose', 'ekf_prediction')]:
        raise ValueError('initial_pose_input must differ from internal and output pose topics')
    extrinsics = config['extrinsics']
    require_keys(extrinsics, EXTRINSIC_FRAMES, 'extrinsics')
    transforms = []
    for name, (parent, child) in EXTRINSIC_FRAMES.items():
        item = extrinsics[name]
        require_keys(item, ('xyz_m', 'rpy_deg', 'provenance'), 'extrinsics.' + name)
        item['xyz_m'] = vector(item['xyz_m'], 3, name + '.xyz_m')
        item['rpy_deg'] = vector(item['rpy_deg'], 3, name + '.rpy_deg')
        if not isinstance(item['provenance'], str) or not item['provenance']:
            raise ValueError(name + '.provenance is required')
        if name in ('rear_to_base', 'lidar_to_cloud') and any(
                abs(value) > 1e-12 for value in (*item['xyz_m'], *item['rpy_deg'])):
            raise ValueError(name + ' must be identity for the native base/confirmed cloud convention')
        transforms.append({'name': name, 'parent': frames[parent], 'child': frames[child],
                           'xyz_m': item['xyz_m'], 'rpy_rad': [math.radians(v) for v in item['rpy_deg']]})
    if extrinsics['rear_to_lidar'].get('confirmed_for_replay') is not True:
        raise ValueError('extrinsics.rear_to_lidar.confirmed_for_replay must be true')
    if extrinsics['lidar_to_imu'].get('axes_confirmed') is not True:
        raise ValueError('extrinsics.lidar_to_imu.axes_confirmed must be true')
    initial = config['initial_pose']
    require_keys(initial, ('reference', 'xyz_m', 'rpy_rad', 'covariance_diagonal', 'provenance'), 'initial_pose')
    if initial['reference'] != 'cloud':
        raise ValueError('initial_pose.reference must be cloud (map -> point-cloud frame)')
    initial['xyz_m'] = vector(initial['xyz_m'], 3, 'initial_pose.xyz_m')
    initial['rpy_rad'] = vector(initial['rpy_rad'], 3, 'initial_pose.rpy_rad')
    initial['covariance_diagonal'] = vector(initial['covariance_diagonal'], 6,
                                             'initial_pose.covariance_diagonal', positive=True)
    livox = config['livox']
    require_keys(livox, ('raw_frame', 'min_range_m', 'max_range_m', 'max_scan_duration_s',
                         'max_imu_gap_s', 'max_wheel_gap_s', 'buffer_seconds', 'wait_timeout_s',
                         'reject_invalid_tags', 'header_tolerance_s'), 'livox')
    if not isinstance(livox['raw_frame'], str) or not livox['raw_frame'] or livox['raw_frame'].startswith('/'):
        raise ValueError('livox.raw_frame must be a nonempty recorded frame without leading slash')
    for name in ('min_range_m', 'header_tolerance_s'):
        livox[name] = number(livox[name], 'livox.' + name, 0.)
    for name in ('max_range_m', 'max_scan_duration_s', 'max_imu_gap_s', 'max_wheel_gap_s',
                 'buffer_seconds', 'wait_timeout_s'):
        livox[name] = number(livox[name], 'livox.' + name, 0., True)
    boolean(livox['reject_invalid_tags'], 'livox.reject_invalid_tags')
    if livox['max_range_m'] <= livox['min_range_m']:
        raise ValueError('livox.max_range_m must exceed min_range_m')
    if livox['buffer_seconds'] <= livox['max_scan_duration_s']:
        raise ValueError('livox.buffer_seconds must exceed max_scan_duration_s')
    if livox['header_tolerance_s'] > livox['max_scan_duration_s']:
        raise ValueError('livox.header_tolerance_s must not exceed max_scan_duration_s')
    adapter = config['adapter']
    require_keys(adapter, (*POSITIVE_ADAPTER_KEYS, *QUEUE_KEYS, 'future_tolerance',
                           'scan_relay_delay', 'voxel_size', 'publish_tf'), 'adapter')
    for name in POSITIVE_ADAPTER_KEYS:
        adapter[name] = number(adapter[name], 'adapter.' + name, 0., True)
    for name in ('future_tolerance', 'scan_relay_delay', 'voxel_size'):
        adapter[name] = number(adapter[name], 'adapter.' + name, 0.)
    for name in QUEUE_KEYS:
        if type(adapter[name]) is not int or adapter[name] < 1:
            raise ValueError('adapter.' + name + ' must be a positive integer')
    boolean(adapter['publish_tf'], 'adapter.publish_tf')
    replay = config['replay']
    require_keys(replay, ('rate', 'domain_id', 'localhost_only', 'warmup_seconds', 'tail_seconds',
                          'max_bag_seconds', 'ready_timeout', 'drain_seconds'), 'replay')
    for name in ('rate', 'ready_timeout'):
        replay[name] = number(replay[name], 'replay.' + name, 0., True)
    for name in ('warmup_seconds', 'tail_seconds', 'drain_seconds'):
        replay[name] = number(replay[name], 'replay.' + name, 0.)
    if (selected_mode == 'replay'
            and replay['tail_seconds'] < livox['max_scan_duration_s'] + livox['wait_timeout_s']):
        raise ValueError('replay.tail_seconds must cover livox.max_scan_duration_s + livox.wait_timeout_s')
    if replay['max_bag_seconds'] is not None:
        replay['max_bag_seconds'] = number(replay['max_bag_seconds'], 'replay.max_bag_seconds', 0., True)
    if type(replay['domain_id']) is not int or not 0 <= replay['domain_id'] <= 232:
        raise ValueError('replay.domain_id must be an integer from 0 to 232')
    boolean(replay['localhost_only'], 'replay.localhost_only')
    validation = config['validation']
    require_keys(validation, ('scan_sample_stride', 'min_gyro_messages', 'min_ndt_acceptance_ratio', 'min_ekf_messages',
                              'max_pose_step_m', 'max_pose_gap_s', 'min_update_span_ratio'), 'validation')
    for name in ('scan_sample_stride', 'min_gyro_messages', 'min_ekf_messages'):
        if type(validation[name]) is not int or validation[name] < 1:
            raise ValueError('validation.' + name + ' must be a positive integer')
    for name in ('max_pose_step_m', 'max_pose_gap_s'):
        validation[name] = number(validation[name], 'validation.' + name, 0., True)
    for name in ('min_ndt_acceptance_ratio', 'min_update_span_ratio'):
        validation[name] = number(validation[name], 'validation.' + name, 0., True)
        if validation[name] > 1.:
            raise ValueError('validation.' + name + ' must not exceed 1')
    mount = transforms[1]
    imu = transforms[3]
    base_xyz, base_q = body_pose_to_base(initial['xyz_m'], initial['rpy_rad'],
                                        mount['xyz_m'], mount['rpy_rad'])
    imu_q = quaternion_multiply(quaternion_rpy(*mount['rpy_rad']), quaternion_rpy(*imu['rpy_rad']))
    config['_derived'] = {'transforms': transforms, 'initial_base_xyz': list(base_xyz),
                          'initial_base_quaternion': list(base_q), 'imu_to_base_quaternion': list(imu_q),
                          'base_to_lidar_xyz': list(mount['xyz_m']),
                          'base_to_lidar_quaternion': list(quaternion_rpy(*mount['rpy_rad']))}
    config['_config_file'] = str(source)
    require_keys(config['native_parameters'], ('gyro', 'ekf', 'ndt', 'map_loader'), 'native_parameters')
    native = {}
    for name, native_path in config['native_parameters'].items():
        if not isinstance(native_path, str) or not native_path:
            raise ValueError('native_parameters.' + name + ' must be a path')
        parameter_path = (source.parent / Path(native_path).expanduser()).resolve()
        config['native_parameters'][name] = str(parameter_path)
        with parameter_path.open(encoding='utf-8') as handle:
            document = yaml.safe_load(handle)
        try:
            values = document['/**']['ros__parameters']
        except (KeyError, TypeError) as error:
            raise ValueError(str(parameter_path) + ' must contain /**: ros__parameters:') from error
        native[name] = flatten_parameters(resolve_references(values, config))
    if native['gyro'].get('output_frame') != frames['base']:
        raise ValueError('Native gyro output_frame must reference frames.base')
    if native['gyro'].get('message_timeout_sec') != adapter['sensor_timeout']:
        raise ValueError('Native gyro timeout must reference adapter.sensor_timeout')
    if native['ekf'].get('misc.pose_frame_id') != frames['map']:
        raise ValueError('Native EKF pose_frame_id must reference frames.map')
    if native['ndt'].get('frame.base_frame') != frames['base'] or native['ndt'].get('frame.map_frame') != frames['map']:
        raise ValueError('Native NDT frames must reference the main configuration')
    if native['ndt'].get('frame.ndt_base_frame') != frames['ndt_debug']:
        raise ValueError('Native NDT debug frame must reference frames.ndt_debug')
    if any(parameters.get('use_sim_time') != config['runtime']['use_sim_time'] for parameters in native.values()):
        raise ValueError('All native use_sim_time values must reference runtime.use_sim_time')
    if (native['map_loader'].get('pcd_paths_or_directory') != [config['paths']['map']]
            or native['map_loader'].get('pcd_metadata_path') != config['paths']['map_metadata']):
        raise ValueError('Native map loader paths must reference the main paths section')
    config['_native_parameters'] = native
    return config


def adapter_parameters(config):
    """Derived adapter values; mounting numbers and sensor frames are never duplicated."""
    frames, topics = config['frames'], config['topics']
    mount = config['extrinsics']['rear_to_lidar']
    result = copy.deepcopy(config['adapter'])
    result.update({
        'runtime_mode': config['runtime']['mode'],
        'initialization': config['live']['initialization'] if config['runtime']['mode'] == 'live' else 'config',
        'initial_pose_input_topic': topics['initial_pose_input'],
        'max_sensor_age_s': config['live']['max_sensor_age_s'],
        'ndt_idle_timeout_s': config['live']['ndt_idle_timeout_s'],
        'live_future_tolerance_s': config['live']['future_tolerance_s'],
        'initial_pose_ack_position_m': config['live']['initial_pose_ack_position_m'],
        'initial_pose_ack_angle_rad': config['live']['initial_pose_ack_angle_rad'],
        'wheel_topic': topics['wheel'], 'imu_topic': topics['imu'],
        'points_topic': topics['processed_points'], 'raw_points_topic': topics['points'],
        'wheel_frame': frames['wheel'], 'imu_frame': frames['imu'], 'cloud_frame': frames['cloud'],
        'map_frame': frames['map'], 'rear_frame': frames['rear'], 'base_frame': frames['base'],
        'initial_base_xyz': config['_derived']['initial_base_xyz'],
        'initial_base_quaternion': config['_derived']['initial_base_quaternion'],
        'initial_covariance_diagonal': config['initial_pose']['covariance_diagonal'],
        'mount_xyz': mount['xyz_m'], 'mount_rpy': [math.radians(value) for value in mount['rpy_deg']],
        'imu_to_base_quaternion': config['_derived']['imu_to_base_quaternion'],
        'mount_provenance': mount['provenance'],
    })
    return result
