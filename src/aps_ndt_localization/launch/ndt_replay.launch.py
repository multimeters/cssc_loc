"""Standalone map localization; PCD + XYZ PointCloud2 + explicit initial pose required."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    map_path = Path(value("map_path")).expanduser()
    if not map_path.exists():
        raise RuntimeError("PCD map_path does not exist: %s" % map_path)
    metadata = value("map_metadata_path")
    if metadata and not Path(metadata).is_file():
        raise RuntimeError("PCD metadata does not exist: %s" % metadata)
    sim_time = value("use_sim_time").lower() == "true"
    ndt_params = str(Path(get_package_share_directory("autoware_ndt_scan_matcher"))
                     / "config" / "ndt_scan_matcher.param.yaml")
    feedback_params = {
        "use_sim_time": sim_time,
        "map_frame": value("map_frame"), "base_frame": value("base_frame"),
        **{name: float(value(name)) for name in (
            "initial_x", "initial_y", "initial_z", "initial_roll", "initial_pitch", "initial_yaw",
            "max_extrapolation_sec", "relay_delay_sec", "voxel_size")},
    }
    return [
        Node(package="autoware_map_loader", executable="autoware_pointcloud_map_loader",
             name="pointcloud_map_loader", namespace="map", output="screen",
             parameters=[{
                 "use_sim_time": sim_time, "pcd_paths_or_directory": [str(map_path)],
                 "pcd_metadata_path": metadata, "enable_whole_load": False,
                 "enable_downsampled_whole_load": False, "enable_partial_load": False,
                 "enable_selected_load": False, "leaf_size": 1.0,
             }], remappings=[("service/get_differential_pcd_map",
                              "/map/get_differential_pointcloud_map")]),
        Node(package="autoware_ndt_scan_matcher", executable="autoware_ndt_scan_matcher_node",
             name="ndt_scan_matcher", namespace="localization/ndt", output="screen",
             parameters=[ndt_params, {
                 "use_sim_time": sim_time, "frame.base_frame": value("base_frame"),
                 "frame.ndt_base_frame": "ndt_"+value("base_frame"),
                 "frame.map_frame": value("map_frame"),
                 "ndt.resolution": float(value("ndt_resolution")),
                 "ndt.num_threads": int(value("ndt_threads")),
                 "sensor_points.required_distance": float(value("required_distance")),
                 "sensor_points.timeout_sec": float(value("sensor_timeout_sec")),
             }], remappings=[
                 ("points_raw", "/localization/ndt/input_points"),
                 ("ekf_pose_with_covariance", "/localization/ndt/prior"),
                 ("ndt_pose", "/localization/ndt/pose"),
                 ("ndt_pose_with_covariance", "/localization/ndt/pose_with_covariance"),
                 ("trigger_node_srv", "/localization/ndt/trigger_node"),
                 ("pcd_loader_service", "/map/get_differential_pointcloud_map"),
                 # Upstream publishes this TF even for rejected matches. Isolate it.
                 ("/tf", "/localization/ndt/debug_tf"),
                 ("/tf_static", "/localization/ndt/debug_tf_static"),
             ]),
        Node(package="aps_ndt_localization", executable="ndt_feedback", name="ndt_feedback",
             namespace="localization/ndt", output="screen", parameters=[feedback_params],
             remappings=[
                 ("input_points", value("points_topic")),
                 ("points", "/localization/ndt/input_points"),
                 ("prior", "/localization/ndt/prior"),
                 ("ndt_pose", "/localization/ndt/pose_with_covariance"),
                 ("odometry", "/localization/ndt/odometry"),
                 ("trigger_node", "/localization/ndt/trigger_node"),
             ]),
    ]


def generate_launch_description():
    defaults = {
        "map_metadata_path": "", "points_topic": "/cloud_registered_body",
        "map_frame": "map", "base_frame": "body", "use_sim_time": "true",
        "initial_x": "0.0", "initial_y": "0.0", "initial_z": "0.0",
        "initial_roll": "0.0", "initial_pitch": "0.0", "initial_yaw": "0.0",
        "ndt_resolution": "2.0", "ndt_threads": "4", "required_distance": "10.0",
        "max_extrapolation_sec": "0.5", "relay_delay_sec": "0.05",
        "sensor_timeout_sec": "2.0", "voxel_size": "0.0",
    }
    return LaunchDescription([
        DeclareLaunchArgument("map_path", description="PCD file, or tiled PCD directory with metadata"),
        *(DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()),
        OpaqueFunction(function=nodes),
    ])
