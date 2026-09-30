# Standalone NDT localization

This package launches the original Autoware CPU NDT scan matcher and PCD map loader. Its small feedback node supplies a configured initial pose, then predicts each scan's initial guess using the last two **accepted NDT poses**. It does not subscribe to recorded TF, recorded localization poses, odometry, or IMU. It therefore exercises map matching rather than replaying an existing localization trajectory.

```bash
ros2 launch aps_ndt_localization ndt_replay.launch.py \
  map_path:=/absolute/path/GlobalMap_loc.pcd \
  points_topic:=/cloud_registered_body base_frame:=body map_frame:=map \
  initial_x:=0.0 initial_y:=0.0 initial_z:=0.0 \
  initial_roll:=0.0 initial_pitch:=0.0 initial_yaw:=0.0
```

The initial pose is the pose of `body` in the PCD map's coordinate system. Defaults are the origin, not an automatic global initialization result. Supply a geometrically checked initial pose before evaluating localization. `map_frame` names the numeric coordinate system of the PCD; the PCD format itself does not prove that coordinate system's relationship to a recorded TF frame.

Replay PointCloud2 and `/clock` with simulated time. Do not replay `/tf` as an output of this localization package. The raw point cloud must have XYZ fields and `frame_id` equal to `base_frame`. This deliberately avoids assuming an unavailable sensor/body/base_link extrinsic transform. The existing `/livox/lidar` CustomMsg is not needed when `/cloud_registered_body` is available. These registered-body clouds may already have undergone deskewing in the source system; this run validates localization from those point clouds, not the upstream raw Livox processing.

The first map load occurs when the original NDT node receives its first pose prior and its one-second map timer fires. Early scans may be skipped during startup. Run playback slowly enough that compute keeps up. The feedback node reports received, forwarded, locally dropped, and accepted counts; the upstream NDT sensor subscription uses a depth-one queue and may independently drop clouds under load. A forwarded scan is not evidence of a successful match.

Outputs:

- `/localization/ndt/pose_with_covariance`: accepted pose of `body` in `map_frame`.
- `/localization/ndt/odometry`: the same accepted pose, explicit `child_frame_id=body`; twist is unknown with high covariance.
- `/localization/ndt/nearest_voxel_transformation_likelihood`, `transform_probability`, `iteration_num`, `exe_time_ms`: native per-alignment metrics, including rejected attempts.
- `/diagnostics`: native input, map loading, and scan matching diagnostics.
- `/localization/ndt/debug_tf`: native diagnostic transform stream. Upstream publishes this even for rejected matches, so it is isolated from normal `/tf`.

The native defaults accept nearest-voxel likelihood strictly greater than 2.3 and a converged iteration condition. No acceptance threshold is lowered by this package. Forwarding a pose to an EKF that represents `base_link` requires the actual calibrated `body` to `base_link` transform; these two frames are not silently equated.

`map_metadata_path` is optional for a single PCD file. Multiple map tiles require metadata. `ndt_resolution`, `ndt_threads`, and `required_distance` expose the native settings. `max_extrapolation_sec` caps feedback prediction; `relay_delay_sec` lets the two timestamped prior samples reach the native interpolation buffer before forwarding a cloud. A bag time reset requires restarting localization.

The adapter removes nonfinite XYZ points while preserving intensity, normals, and every other original point field. `voxel_size:=0.3` optionally keeps one original point per 0.3 m cell; the default `0.0` does not downsample. `sensor_timeout_sec` defaults to 2.0 seconds because this dataset's recorded clock is about 1.39 seconds ahead of its point acquisition stamps. Acquisition stamps remain unchanged. This setting changes a latency warning, not NDT's acceptance score.
