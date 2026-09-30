# APS native Autoware fusion

This package connects the extracted native gyro odometer, EKF, NDT and map loader.
The active entry point is `fusion_replay.launch.py`; all runtime settings come
from one main YAML and its referenced native parameter YAML files.

## Configuration and startup

Edit repository `config/localization.yaml`, then launch with its absolute path:

```bash
ros2 launch aps_bag_localization fusion_replay.launch.py \
  config_file:=/absolute/path/to/aps-localization/config/localization.yaml
```

`config_file` is the only launch argument. The repository's replay script uses
that same file and saves a resolved configuration/native-parameter snapshot for
each run. No separate command-line mounting angle, sensor offset, initial pose,
or implicit experiment parameter can override it.

The main file contains:

- `paths`: bag, PCD map, optional map metadata and output root.
- `frames`, `topics`, `services`: graph names shared by adapters and native nodes.
- `extrinsics`: the four physical/identity transforms, with provenance.
- `initial_pose`: map-to-cloud-frame seed, covariance and provenance.
- `livox`: raw CustomMsg label, range/tag filters, scan timing and motion-coverage limits.
- `adapter`: covariance floors, observation/relay timeouts, queue depths and timers.
- `runtime`, `replay`: simulated time, domain, replay rate and timing.
- `native_parameters`: paths to the complete gyro, EKF, NDT and map-loader YAML files.
- `validation`: output/coverage checks, not physical accuracy criteria.

Relative paths resolve against the main YAML's directory. Exact references such
as `${frames.base}` or `${paths.map}` in native YAML are resolved with their
original YAML types before being passed to ROS. They are not shell expressions.
The native parameter files must be used through this loader, rather than handed
directly to `ros2 run --params-file` with unresolved references.

`aps_bag_localization.configuration.load_config(path)` works without ROS and
returns absolute paths, `_derived` transforms/initial pose/IMU rotation, and
`_native_parameters` containing resolved flat ROS parameters. A saved resolved
configuration is revalidated and its derived values are recomputed when loaded.
The authoritative configuration files are included in the installed package
share directory as well as the repository.

## Estimator connections

```text
wheel forward speed ------> native gyro_odometer ---> native EKF twist
IMU angular velocity ---------------^
/livox/lidar CustomMsg --> timed filtering + measured-motion deskew
                          --> body PointCloud2 --> native NDT --> native EKF pose
native EKF predicted pose --------------------------> native NDT prior
```

No NDT self-feedback node runs in this launch. The adapter has no continuous
prior or pose-observation publisher. It publishes the configured initial pose
once, after both native activation services succeed. The cloud relay waits until
actual EKF predictions cover each derived scan-end timestamp and then forwards it.
Wheel pose and wheel angular velocity are not consumed. IMU acceleration and
orientation are marked unavailable.

The preprocessor handles unordered per-point offsets using integer nanosecond
timestamps. It compensates 3-D rotation and forward translation, including the
lidar mounting lever arm. A scan without complete IMU/wheel coverage is published
whole and uncorrected, explicitly counted as `uncompensated`; its nominal end
header does not imply successful deskew. Timeouts use the simulated clock, so
slow replay cannot publish scans before acquisition ends. The bag's historical
`/cloud_registered_body` is never an input to the main replay.

Replay with the repository's acquisition-time runner. It orders unchanged
recorded sensor messages by their original header timestamp and supplies
configured simulated time. Its configured clock warmup allows activation, EKF
prediction and native map loading before the first scan. Do not additionally
replay recorded localization poses, `/tf`, or the bag's old `/tf_static`.

## Coordinate conventions and calibration status

Both NDT and EKF estimate the rear wheel center. Frame names come from YAML:

```text
rear -> base           identity; native EKF base must be base_link
rear -> lidar          one configurable mounting transform
lidar -> cloud         confirmed identity alias
lidar -> IMU           physical internal IMU offset and axis orientation
```

Raw Livox messages reuse the `livox_frame` label for point coordinates at the
lidar origin. They are interpreted as lidar points and emitted in the confirmed
identity cloud frame; the IMU chip translation is never applied to raw points.

The loader rejects a non-identity rear-to-base transform, a non-identity
lidar-to-cloud alias, or a base frame other than `base_link`, because those would
violate this native pipeline's assumptions. Other frame/topic names propagate
from the same configuration into both the adapter and native nodes.

The current dataset's forward mounting tilt is explicitly a user-authorized
estimate from static gravity, not a surveyed calibration. Its provenance remains
in the YAML and fusion status. The MID360 manual supplies the IMU axis convention
and physical offset. There is no second active copy of either transform in
Python defaults.

The seed is expressed as `map -> cloud`, not `map -> rear`. Using the same
mounting transform that supplies TF, the loader calculates:

```text
T_map_base = T_map_cloud * inverse(T_base_cloud)
R_base_imu = R_base_lidar * R_lidar_imu
```

Native EKF and NDT dynamic TF are isolated on configured internal topics. The
adapter publishes `map -> rear`, followed by the static identity `rear -> base`,
so the TF tree has only one parent for each frame. Public odometry still has
child frame `base_link` and represents the rear wheel center.

## IMU rotation in the locked vendor version

The locked native gyro calls `get_latest_transform(imu_frame, base_link)`, whose
implementation passes those arguments directly to TF2
`lookupTransform(target, source)`. Static review and a ROS TF2 numerical check
confirmed that this applies the inverse rotation to an IMU-frame vector.

The fusion adapter works around this without changing vendor code: it transforms
angular velocity with the configured `R_base_imu`, transforms the full covariance
as `R C R^T`, and publishes the derived IMU in `base_link`. The native gyro then
uses an identity transform, avoiding double rotation. Original raw messages,
timestamps and physical TF are preserved. IMU translation does not affect rigid
body angular velocity.

Unknown diagonal covariance receives the configured floor; valid cross terms
are retained. Invalid covariance is symmetrized and conservatively regularized.
The native gyro's existing covariance averaging/isotropic approximation still
follows. Fusion status records the applied quaternion and
`upstream_gyro_inverse_tf_workaround: true`. Floors are numerical assumptions,
not calibrated noise measurements.

## Outputs and gaps

Output topic names are defined under `topics` in the YAML:

| Configuration key | Meaning |
|---|---|
| `odometry`, `pose` | Guarded fused rear-center estimates |
| `ndt_pose` | Native accepted NDT rear-center observation |
| `gyro_twist` | Native wheel/gyro twist |
| `ekf_prediction` | Native EKF prediction supplied directly to NDT |
| `fusion_status` | Mode, observation ages, counts and calibration provenance |
| `metrics_prefix` | Prefix for native NDT convergence/iteration/time diagnostics |

A short IMU gap does not permanently stop fusion. Fresh NDT observations continue
correcting EKF with mode `NDT_ONLY`; fresh gyro without NDT gives `GYRO_ONLY`.
When both are fresh the mode is `FUSED`. A configurable short interval with
neither source fresh is labeled `PREDICTING`; later public output is suppressed
as `STALE`. Real observations allow recovery. No fake IMU, wheel or stationary
pose observation is generated. A reversed clock requires restarting the process.

Native diagnostics must be inspected to verify EKF measurement gates and update
counts, not merely topic arrival. NDT convergence, finite poses and complete
replay coverage do not establish independent physical accuracy.

## Tests

```bash
cd src/aps_bag_localization
python3 -m unittest discover -s test -v
```

Configuration/frame/covariance tests run without ROS. Sensor callback and real
node-construction tests additionally require ROS Humble and the sibling
`aps_ndt_localization` package. Tests cover one-source transform propagation,
frame constraints, initialization covariance, full covariance rotation,
nonzero mounting/IMU rotation composition, absence of wheel-pose injection, and
recovery after sensor gaps. Legacy relative/standalone experiment files are not
part of this active fusion launch or its acceptance results.
