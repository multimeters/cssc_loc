# Updated scope: 2026-09-30 pointcloud dataset

The older `bags` directory supported only relative odometry replay. A second dataset now supplies `/cloud_registered_body` (`sensor_msgs/msg/PointCloud2`, body frame, 969 frames at approximately 10 Hz) and PCD maps. It enables a real native NDT map-matching test. The older-data limitations below apply only to the original bag.

`aps_ndt_localization` now starts the pinned native CPU NDT matcher and PCD map loader. It uses a configured one-time initial pose and feedback from accepted NDT matches; it never subscribes to recorded localization TF or poses. Body-frame localization remains separate from the base_link EKF until an actual body/base_link calibration exists. The detailed initial-registration evidence is in `new_bag_initial_registration.json`; it is geometric initialization evidence, not independent localization ground truth.

The full four-root closure (`autoware_gyro_odometer`, `autoware_ekf_localizer`, `autoware_ndt_scan_matcher`, `autoware_map_loader`) is 28 source packages: 16 baseline + 11 NDT/map additions listed below + `autoware_lanelet2_extension`. Lanelet's manifest introduces no additional Autoware source packages; it adds system Lanelet2 core/io/maps/projection/routing/traffic_rules/validation, GeographicLib, pugixml-dev and range-v3. The delivered workspace also includes optional stop_filter, twist2accel and signal_processing, giving 31 vendor packages. See `sources.lock.json` for the actual extraction profile lists and exact source commits.

The new cloud has float32 XYZ, intensity, normals, curvature and a 48-byte point step. The adapter preserves original fields, rejects invalid layouts, removes nonfinite XYZ, and optionally voxel-reduces clouds. The default does not downsample. Input stamps are preserved exactly. Its original record time runs approximately 1.39 seconds after point acquisition; a 2.0 second NDT latency diagnostic threshold accommodates that measured recording delay without lowering the native convergence threshold.

Native NDT publishes its TF before checking convergence. The launch isolates that stream on `/localization/ndt/debug_tf`. Only native accepted pose topics drive the feedback loop and accepted Odometry output. Validation must count those pose messages and join per-alignment metrics by acquisition stamp; a TF or a metric message alone is not a successful localization.

---
# Localization extraction audit

Audit date: 2026-09-29. Source of truth is the actual `autoware.repos` file in `libpet-co/autoware.APS`, branch `hmi_container_dev`, commit `1b29946659d716b06c1712fb39f269566297332e`.

## Resolved implementation repositories

- `libpet-co/autoware_core.APS`, requested `airy_test`, resolved `d54caaf87d94dd2f584677e871031f5050c1d9d8`.
- `libpet-co/autoware_universe.APS`, requested `fleet-target`, resolved `1363f65bbad4089ceec40357ff571b81c006e6ce`.
- Checkout locations: `E:\zhongchuandingwei\upstream-repos\autoware_core.APS` and `E:\zhongchuandingwei\upstream-repos\autoware_universe.APS`.
- HTTPS transport was used for the same GitHub repositories after SSH host key verification failed; no authentication/host verification setting was disabled.
- Universe needed repository-local `core.longpaths=true` for a planning header on Windows. Checkout then succeeded.

## Recommended extraction roots

1. Baseline replay: `autoware_gyro_odometer`, `autoware_ekf_localizer`.
2. Future pointcloud/map localization: add `autoware_ndt_scan_matcher`, `autoware_map_loader`.
3. Optional small processing/diagnostics: `autoware_stop_filter`, `autoware_twist2accel`, `autoware_localization_error_monitor`, `autoware_pose_instability_detector`.

Do not use the entire `autoware_core_localization` launch unchanged. It assumes GNSS pose input, relays vehicle twist instead of starting the gyro odometer, and its stop filter input/output remappings are not suitable for a standalone minimal replay as-is.

`autoware_pose_initializer` is optional. A dedicated startup node can publish one initial pose and call the EKF/NDT activation services without pulling in the large ADAPI/motion/map-height initialization chain. Keeping the unmodified upstream initializer as an optional profile is reasonable if the full ADAPI interface is wanted later.

## Exact baseline source closure (tests excluded)

16 ROS packages, preserving upstream package contents:

- autoware_cmake repository: `autoware_cmake`.
- autoware_core.APS: `autoware_gyro_odometer`, `autoware_ekf_localizer`, `autoware_localization_util`, `autoware_kalman_filter`.
- autoware_utils: `autoware_utils_diagnostics`, `autoware_utils_geometry`, `autoware_utils_logging`, `autoware_utils_math`, `autoware_utils_system`, `autoware_utils_tf`.
- autoware_internal_msgs: `autoware_internal_debug_msgs`, `autoware_internal_planning_msgs`.
- autoware_msgs: `autoware_common_msgs`, `autoware_perception_msgs`, `autoware_planning_msgs`.

Planning/perception message packages are transitive data-type dependencies from `autoware_utils_geometry` -> `autoware_internal_planning_msgs`, not planning/perception nodes.

Manifest versions: autoware_cmake `1.0.2`, autoware_utils `1.4.2`, autoware_internal_msgs `1.12.0`, autoware_msgs `1.9.0`. Extraction should lock their resolved commits, not mutable labels alone.

System dependencies include ament_cmake_auto, ament_lint_auto, eigen/eigen3_cmake_module, fmt, libboost-system-dev, ROS geometry/nav/sensor/std/diagnostic/visualization messages, TF2 packages, rclcpp/components, ROSIDL build/runtime, unique_identifier_msgs and logging_demo. The `logging_demo` dependency is an ordinary `<depend>` in autoware_utils_logging and must be available even with tests disabled.

## Baseline runtime contracts

- gyro odometer input is `geometry_msgs/msg/TwistWithCovarianceStamped`, not `nav_msgs/msg/Odometry`. Extract the wheel velocity from `/odom`; retain its acquisition stamp and child frame; assign explicit nonzero covariance if upstream data left covariance unknown/zero.
- gyro consumes angular velocity from IMU, not IMU orientation or acceleration. It transforms IMU angular velocity into `base_link`; a valid static transform is required. It computes a fused twist and covariance from IMU angular velocity and wheel linear.x.
- gyro subscriptions are reliable (`rclcpp::QoS{100}`), so rosbag playback QoS should be reliable for IMU and odometry adapter output.
- gyro default timeout is 0.2 seconds against node time. Enable `use_sim_time` everywhere and provide `/clock` from bag playback.
- EKF starts inactive and without an initial pose. Publish `PoseWithCovarianceStamped` to its `initialpose` remap and call `trigger_node_srv` (`std_srvs/srv/SetBool`, data=true). Activating clears queued pose/twist measurements.
- `misc.pose_frame_id` is configurable, but output Odometry child and outgoing TF child are hardcoded `base_link` in upstream EKF. A bag publishing `odom -> base_link` must not fight a second TF publisher. Isolate recorded TF or remap output TF deliberately.
- Use the first wheel pose only to initialize if the goal is independent dead reckoning from wheel speed and IMU yaw rate. Feeding wheel pose and the same wheel speed continuously into the EKF double-counts correlated information unless modeled deliberately.
- Without subsequent absolute pose observations, the EKF still predicts from twist, but its pose stale/uncertainty diagnostics are expected. Disabling yaw-bias estimation is advisable for pure twist-only replay because yaw bias is not observable without repeated heading/pose correction.
- Data containing only IMU, wheel odometry and TF supports relative dead reckoning, not validated global map localization. Comparing output to wheel odometry is a consistency check, not localization ground truth.

## NDT + PCD map chain

`autoware_ndt_scan_matcher` lives in Universe. Its CPU multi-grid NDT/OpenMP implementation is bundled in the package; no separate ndt_omp repository and no GPU/TensorRT are required.

NDT inputs are PointCloud2 sensor points, EKF predicted pose-with-covariance, a differential PCD map service and the sensor-to-base_link TF. NDT outputs its pose and pose-with-covariance for EKF correction. Its activation service is SetBool and it offers the `ndt_align_srv` initial alignment service.

`autoware_map_loader` lives in Core. Its package builds both pointcloud and lanelet loaders; preserving the package as upstream adds `autoware_lanelet2_extension` from `libpet-co/autoware_lanelet2_extension.APS` at `aps_dev`, `autoware_geography_utils`, and `autoware_component_interface_specs`. These in turn require map/control/localization/vehicle message packages and Lanelet2/GeographicLib. This is a build dependency cost; the standalone launch need only start the PCD loader, not lanelet loading.

Before Lanelet extension recursion, the four extraction roots produce the 16 baseline packages plus these 11 packages: `autoware_component_interface_specs`, `autoware_control_msgs`, `autoware_geography_utils`, `autoware_internal_localization_msgs`, `autoware_localization_msgs`, `autoware_map_loader`, `autoware_map_msgs`, `autoware_ndt_scan_matcher`, `autoware_utils_pcl`, `autoware_utils_visualization`, `autoware_vehicle_msgs`. Resolve and append `autoware_lanelet2_extension` and its dependencies from its actual manifest.

### Upstream map launch defects to handle in dedicated launch

The Core `pointcloud_map_loader.launch.xml` declares `pointcloud_map_path`/`pointcloud_map_metadata_path`, but its default YAML substitutes `pcd_paths_or_directory`/`pcd_metadata_path`. Avoid that mismatched pair by supplying direct Node parameters:

- `pcd_paths_or_directory`: list of one PCD file or directory;
- `pcd_metadata_path`: metadata YAML path, or empty string for a single PCD (loader computes bounds);
- `enable_whole_load`, `enable_downsampled_whole_load`, `enable_partial_load`, `enable_selected_load`: bool values;
- `leaf_size`: positive downsampling size.

Differential loader is always constructed; there is no enable_differential_load parameter. Remap `service/get_differential_pcd_map` explicitly to `/map/get_differential_pointcloud_map`, and remap NDT's `pcd_loader_service` to the same endpoint. The upstream launch currently remaps only partial/selected services.

Multiple PCD files require valid metadata. Pointcloud sensor messages are required; `/tf` cannot reconstruct laser returns, and a `.db3` file with no PointCloud2 cannot be turned into an NDT localization test. NDT defaults also expect max point distance >= 10 m, input age <= 1 s, initial pose age <= 1 s, and sensor frame transformation availability; tune only from actual sensor/map data.

## Localization packages considered but not needed for supplied bag

Core localization package names: `autoware_core_localization`, `autoware_ekf_localizer`, `autoware_gyro_odometer`, `autoware_localization_util`, `autoware_stop_filter`, `autoware_twist2accel`.

Universe localization package names: `autoware_geo_pose_projector`, `autoware_ar_tag_based_localizer`, `autoware_landmark_manager`, `autoware_lidar_marker_localizer`, `autoware_localization_error_monitor`, `autoware_localization_reliability_monitor`, `autoware_ndt_scan_matcher`, `autoware_pose2twist`, `autoware_pose_covariance_modifier`, `autoware_pose_estimator_arbiter`, `autoware_pose_initializer`, `autoware_pose_instability_detector`, `yabloc_common`, `yabloc_image_processing`, `yabloc_monitor`, `yabloc_particle_filter`, `yabloc_pose_initializer`.

Yabloc/AR need camera inputs and suitable maps/landmarks; lidar markers/NDT need point clouds; Eagleye needs GNSS-related observations. Their source repositories being present in autoware.repos does not make them runnable on an IMU/odometry-only bag. Hardware drivers, planning/control/perception nodes, fleet/HMI/ADAPI orchestration, TensorRT/CUDA, Eagleye/RTK/GNSS converters are not required by the recommended replay roots.
