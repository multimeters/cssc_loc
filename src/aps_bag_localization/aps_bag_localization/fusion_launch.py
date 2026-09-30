"""Build the native ROS graph exclusively from the validated runtime YAML."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from .configuration import load_config


def nodes(context):
    path = LaunchConfiguration('config_file').perform(context)
    if not Path(path).is_absolute():
        raise ValueError('config_file must be an absolute path to the runtime YAML')
    config = load_config(path)
    paths, topics, services = config['paths'], config['topics'], config['services']
    native = config['_native_parameters']
    sim_time = config['runtime']['use_sim_time']
    if not Path(paths['map']).exists():
        raise ValueError('Configured map does not exist: ' + paths['map'])
    if paths['map_metadata'] and not Path(paths['map_metadata']).is_file():
        raise ValueError('Configured map metadata does not exist: ' + paths['map_metadata'])
    common_clock = [('/clock', topics['clock'])]
    common_diagnostics = [('/diagnostics', topics['diagnostics'])]
    actions = []
    for transform in config['_derived']['transforms']:
        arguments = ['--frame-id', transform['parent'], '--child-frame-id', transform['child']]
        for axis, item in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'),
                              (*transform['xyz_m'], *transform['rpy_rad'])):
            arguments += ['--' + axis, str(item)]
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name=transform['name'], arguments=arguments,
                            parameters=[{'use_sim_time': sim_time}], remappings=common_clock))
    actions += [
        Node(package='autoware_map_loader', executable='autoware_pointcloud_map_loader',
             name='pointcloud_map_loader', namespace='map', output='screen',
             parameters=[native['map_loader']], remappings=common_clock + common_diagnostics + [
                 ('service/get_differential_pcd_map', services['map_loader'])]),
        Node(package='aps_bag_localization', executable='fusion_adapter', output='screen',
             parameters=[{'use_sim_time': sim_time, 'configuration_file': config['_config_file']}],
             remappings=common_clock),
        Node(package='autoware_gyro_odometer', executable='autoware_gyro_odometer_node',
             name='gyro_odometer', namespace=topics['gyro_twist'].rsplit('/', 1)[0], output='screen',
             parameters=[native['gyro']], remappings=common_clock + common_diagnostics + [
                 ('vehicle/twist_with_covariance', topics['wheel_twist']),
                 ('imu', topics['imu_base']), ('twist_with_covariance', topics['gyro_twist']),
             ]),
        Node(package='autoware_ekf_localizer', executable='autoware_ekf_localizer_node',
             name='ekf_localizer', namespace=topics['ekf_prediction'].rsplit('/', 1)[0], output='screen',
             parameters=[native['ekf']], remappings=common_clock + common_diagnostics + [
                 ('initialpose', topics['initial_pose']), ('trigger_node_srv', services['ekf_activation']),
                 ('in_pose_with_covariance', topics['ndt_pose']),
                 ('in_twist_with_covariance', topics['gyro_twist']),
                 ('ekf_pose_with_covariance', topics['ekf_prediction']),
                 ('ekf_odom', topics['ekf_odometry']), ('/tf', topics['ekf_tf']),
             ]),
        Node(package='autoware_ndt_scan_matcher', executable='autoware_ndt_scan_matcher_node',
             name='ndt_scan_matcher', namespace=topics['metrics_prefix'], output='screen',
             parameters=[native['ndt']], remappings=common_clock + common_diagnostics + [
                 ('points_raw', topics['ndt_points']),
                 ('ekf_pose_with_covariance', topics['ekf_prediction']),
                 ('ndt_pose', topics['ndt_pose_stamped']),
                 ('ndt_pose_with_covariance', topics['ndt_pose']),
                 ('trigger_node_srv', services['ndt_activation']),
                 ('pcd_loader_service', services['map_loader']),
                 # Keep tf_static global: both gyro and NDT need actual extrinsics.
                 ('/tf', topics['ndt_tf']),
             ]),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', description='Absolute path to config/localization.yaml or a resolved run snapshot'),
        OpaqueFunction(function=nodes),
    ])
