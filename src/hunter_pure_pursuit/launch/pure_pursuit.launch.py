from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("hunter_pure_pursuit"), "config", "route.yaml"
    )
    route_config = LaunchConfiguration("route_config")
    return LaunchDescription([
        DeclareLaunchArgument(
            "route_config",
            default_value=default_config,
            description="YAML file with Pure Pursuit settings and map-frame waypoints",
        ),
        Node(
            package="hunter_pure_pursuit",
            executable="pure_pursuit",
            name="hunter_pure_pursuit",
            output="screen",
            parameters=[route_config],
        ),
    ])
