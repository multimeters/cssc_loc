"""Isolated native gyro/EKF replay, blocked until frame assumptions are confirmed."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory('aps_bag_localization')) / 'config'
    sim = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    timeout = ParameterValue(LaunchConfiguration('sensor_timeout'), value_type=float)
    experiment = ParameterValue(LaunchConfiguration('experimental_frame_assumptions'), value_type=bool)
    confirmed = ParameterValue(LaunchConfiguration('wheel_axes_confirmed'), value_type=bool)
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('wheel_topic', default_value='/hunter_odom'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('wheel_frame', default_value='hunter_base_link'),
        DeclareLaunchArgument('wheel_axes_confirmed', default_value='false',
                              description='Confirm wheel forward x axis equals base_link forward x.'),
        DeclareLaunchArgument('experimental_frame_assumptions', default_value='false',
                              description='UNVERIFIED smoke test: assume wheel axes and livox_frame=mid360_link.'),
        DeclareLaunchArgument('sensor_timeout', default_value='0.2'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('output_dir', default_value='artifacts/relative_replay'),
    ]
    nodes = [
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='experimental_livox_frame_alias',
             arguments=['--frame-id', 'mid360_link', '--child-frame-id', 'livox_frame'],
             parameters=[{'use_sim_time': sim}],
             condition=IfCondition(LaunchConfiguration('experimental_frame_assumptions'))),
        Node(package='aps_bag_localization', executable='bag_adapter', output='screen',
             parameters=[str(config / 'adapter.yaml'), {
                 'use_sim_time': sim, 'sensor_timeout': timeout,
                 'wheel_topic': LaunchConfiguration('wheel_topic'),
                 'imu_topic': LaunchConfiguration('imu_topic'),
                 'wheel_frame': LaunchConfiguration('wheel_frame'),
                 'wheel_axes_confirmed': confirmed,
                 'experimental_frame_assumptions': experiment,
             }]),
        Node(package='autoware_gyro_odometer', executable='autoware_gyro_odometer_node',
             name='gyro_odometer', namespace='localization/internal', output='screen',
             parameters=[str(config / 'gyro.yaml'), {'use_sim_time': sim, 'message_timeout_sec': timeout}],
             remappings=[('vehicle/twist_with_covariance', '/localization/input/wheel_twist'),
                         ('imu', '/localization/input/imu'),
                         ('twist_with_covariance', '/localization/internal/gyro_twist')]),
        Node(package='autoware_ekf_localizer', executable='autoware_ekf_localizer_node',
             name='ekf_localizer', namespace='localization/internal', output='screen',
             parameters=[str(config / 'ekf.yaml'), {'use_sim_time': sim}],
             remappings=[('in_twist_with_covariance', '/localization/internal/gyro_twist'),
                         ('in_pose_with_covariance', '/localization/internal/unused_pose_observation'),
                         ('initialpose', '/localization/internal/initialpose'),
                         ('trigger_node_srv', '/localization/internal/trigger_node'),
                         ('ekf_odom', '/localization/internal/kinematic_state'),
                         ('/tf', '/localization/internal/tf')]),
        Node(package='aps_bag_localization', executable='trajectory_recorder', output='screen',
             condition=IfCondition(LaunchConfiguration('record')),
             parameters=[{'use_sim_time': sim, 'output_dir': LaunchConfiguration('output_dir')}]),
    ]
    return LaunchDescription(arguments + nodes)
