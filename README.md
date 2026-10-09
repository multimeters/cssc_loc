# CSSC 独立定位程序

本程序从 `libpet-co/autoware.APS` 默认分支 `hmi_container_dev` 提取定位所需组件，使用原生 Autoware **轮速＋IMU → gyro_odometer → EKF，NDT 位姿 → EKF，EKF 预测 → NDT** 链路。输出参考点是车辆后轮中心。

**默认直接使用 Livox MID-360 原始 `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`）做定位。** 原始点云经过本程序的逐点时间解码、距离与置信度过滤、轮速/IMU 运动补偿，再与 `GlobalMap_loc.pcd` 做 NDT 匹配；一键回放不读取录包中的 `/cloud_registered_body`。

**默认是常驻实时定位：直接订阅外部传感话题，不需要 bag 文件。** 所有参数从 YAML 读取，外参只在 `config/localization.yaml` 定义一次；录包验证保留为独立的 `replay` 模式。

## 一键启动

### 当前 Windows 电脑

在仓库目录中，**双击 `start.cmd`** 即可启动实时定位。它自动调用 WSL 的 `Ubuntu-22.04`；雷达和底盘驱动需要提前或随后单独启动。

也可以在终端运行：

```powershell
.\start.cmd
```

本机现有工作目录是 `E:\zhongchuandingwei\aps-localization`。从 GitHub 新克隆的目录可命名为 `cssc_loc`，启动脚本不依赖仓库文件夹名称。

### Ubuntu / WSL 终端

```bash
cd /你的路径/cssc_loc
bash start.sh
```

默认读取 `config/localization.yaml`，按 `runtime.mode: live` 使用系统时钟，常驻等待话题及初始位姿。第一次需要编译，后续增量编译；按 `Ctrl+C` 停止本次定位节点。构建、配置或节点运行失败时返回非零退出码，Windows 双击窗口保留错误信息。

录包验证改为显式启动：

```bash
bash start.sh --mode replay
```

Windows 对应 `.\start.cmd --mode replay`。此模式自动生成 `use_sim_time: true` 的配置快照，以 YAML 中的倍速回放，并在完成后退出。

## 实车接入步骤

1. 修改 `paths.map` 指向 PCD 地图，核查 YAML 中的外参、话题和 frame。实时模式允许 `paths.bag: null`，不会访问录包。
2. 启动 MID-360 和底盘驱动，确保下表的三个输入持续发布；本包不负责启动硬件驱动。
3. 运行 `bash start.sh --mode live` 或双击 `start.cmd`。
4. RViz 的 Fixed Frame 设置为 `map`，用 **2D Pose Estimate** 在地图上给出车辆**后轮中心**位置和朝向，发布到 `/initialpose`。程序会停用 NDT/EKF，调用 Autoware NDT 的 Monte Carlo 初始搜索，再把可靠的对齐结果交给 EKF，最后重新激活 NDT/EKF；确认并完成新一轮 NDT 匹配后才公开定位结果。候选箭头显示在 `/localization/pose_estimator/monte_carlo_initial_pose_marker`。

在 RViz 添加 **PointCloud2**，话题选择 `/map/output/debug/downsampled_pointcloud_map`，Durability 设为 **Transient Local**、Reliability 设为 **Reliable**，即可看到用于选取初值的地图。显示地图只发布一次并保留供后加入的订阅者读取；`config/native/map_loader.yaml` 中的 `leaf_size` 只控制这个显示副本，不改变 NDT 地图。此原生显示模块固定使用 `map` frame；若自定义 `frames.map`，需关闭该显示副本并另行提供正确 frame 的可视化地图。

| 输入话题 | ROS 消息类型 | 使用内容与 frame |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | MID-360 原始点云，驱动标签 `livox_frame`，坐标按雷达原点解释 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 三轴角速度，`header.frame_id: livox_frame` |
| `/hunter_odom` | `nav_msgs/msg/Odometry` | 仅 `twist.twist.linear.x`，`child_frame_id: hunter_base_link` |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 初始/重新定位请求，`header.frame_id: map`，pose 表示后轮中心 |

实时默认 `live.initialization: topic`，不会自动套用录包的旧起点。再次发布 `/initialpose` 可重新初始化，期间暂停公开位姿并清除旧扫描，等待 Monte Carlo 对齐、新 EKF 初值确认和后续 NDT 观测。若起点固定且已核实，可改为 `live.initialization: config`，启动时使用 `initial_pose` 中的 **map → 雷达** 初值；两种初值的参考点不同，不能直接混填。

`live.domain_id` 默认 `0`，`live.localhost_only` 默认 `false`。雷达驱动、底盘、RViz 与定位进程应使用相同的 `ROS_DOMAIN_ID`；不同 Domain 无法互相发现，参见 [ROS 2 官方说明](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Domain-ID.rst)。跨机时还需确保 DDS 网络可达；Windows/WSL 启动器不会自动配置网络、防火墙或传感器。

三路传感器的时间戳必须与定位电脑的系统时间处于同一时间基准。程序不改写真实消息的时间戳：超过 `live.max_sensor_age_s` 的旧消息或超过 `live.future_tolerance_s` 的未来消息会被拒绝并计数。输入采用 best-effort、volatile 订阅，可接收兼容的 reliable/best-effort 传感发布器，见 [ROS 2 QoS 兼容规则](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)。

实时模式用 `live.motion_wait_timeout_s: 0.1` 限制等待 IMU/轮速补齐的时间，为后续匹配留出延迟余量；回放继续用 `livox.wait_timeout_s: 0.3`。这只改变等待时间，完整补偿仍必须满足同样的采样覆盖及间隔要求，否则整帧以明确标记的未补偿点云输出。

短暂断流会进入降级或 `STALE` 状态，超过预测时限后停止公开位姿/TF；新鲜数据恢复后继续尝试定位。若停机期间车辆移动很远、时间发生倒退或不能重新匹配，应校准时钟并重新启动/给出初值。实时模式不发布 `/clock`，也不加载旧定位 TF；已有驱动不得同时发布另一条冲突的 `map → base_footprint` 定位链。

连续 `live.ndt_idle_timeout_s` 秒没有有效 NDT 结果时，程序暂停 NDT，防止上游在没有扫描时无限积累 EKF 预测缓存；收到新的有效点云后会重新启用 NDT、清理其缓存，并等待新的预测覆盖后恢复匹配。时钟倒退时也会暂停 NDT，需在时钟稳定后重启程序。

## 首次准备

支持 **Ubuntu 22.04 + ROS 2 Humble**；Windows 需要已安装名为 `Ubuntu-22.04` 的 WSL 发行版，并在该发行版内安装 ROS 2 Humble。启动脚本不安装操作系统或 ROS 发行版。

下载程序：

```bash
git clone https://github.com/multimeters/cssc_loc.git
cd cssc_loc
```

首次安装本工作区依赖并启动：

```bash
bash start.sh --install-deps
```

Windows 对应命令：

```powershell
.\start.cmd --install-deps
```

该选项安装编译工具、Python 依赖及 rosdep 解析出的 ROS 依赖，然后继续编译并启动选定模式；需要联网，可能要求输入 Ubuntu 的 sudo 密码。已完成依赖安装后直接使用普通的一键入口。

**录包和 PCD 地图不包含在 Git 仓库中。** 实时定位只需要地图；仅回放验证需要录包。原始数据默认目录如下，位置不同时修改主 YAML 的 `paths.map`，回放时再设置 `paths.bag`：

```text
同一个父目录/
├── cssc_loc/                         # 也可以是 aps-localization
│   ├── start.cmd
│   ├── start.sh
│   └── config/localization.yaml
└── zhongchuanbag-20260930T031102Z-1-001/
    └── zhongchuanbag/
        ├── GlobalMap_loc.pcd
        ├── GlobalMap_mid.pcd
        └── bags/
            └── hunter_sensors_20260930_104649/
                ├── metadata.yaml
                └── hunter_sensors_20260930_104649_0.db3
```

YAML 中的相对路径**以该 YAML 文件所在目录为基准**，不是以终端当前目录为基准。在 WSL 使用 `/mnt/e/...` 路径，不能在 YAML 中直接写 `E:\...`。Windows 启动器会转换命令行 `--config` 的 Windows 路径，但不会擅自改写 YAML 内容。

## 参数在哪里修改

| 文件 | 内容 |
|---|---|
| [`config/localization.yaml`](config/localization.yaml) | 数据路径、全部外参、坐标系、话题、初始位姿、适配参数、回放与运行检查参数 |
| [`config/native/gyro.yaml`](config/native/gyro.yaml) | 原生轮速/IMU 融合参数 |
| [`config/native/ekf.yaml`](config/native/ekf.yaml) | EKF 频率、噪声、测量门限及诊断参数 |
| [`config/native/ndt.yaml`](config/native/ndt.yaml) | NDT 分辨率、线程数、收敛判据及协方差参数 |
| [`config/native/map_loader.yaml`](config/native/map_loader.yaml) | 地图加载参数 |

原生节点 YAML 中的 `${frames.base}` 等是本项目配置加载器的引用写法，启动时由主配置展开，避免坐标系和超时值出现多处独立设置。不要直接把含引用的原生 YAML 传给其他 ROS 启动文件。

修改 YAML 后重新一键启动即可，无需修改 Python 或 launch 文件。程序在启动前检查有限数值、单位向量长度、必要坐标关系和文件路径；回放还会检查 bag 中的输入话题类型。实时输入的消息类型必须与上表一致，frame 和时间戳在接收时检查。

### 外参的方向与单位

`xyz_m` 单位是**米**，`rpy_deg` 顺序为 **roll、pitch、yaw，单位是度**；旋转采用 ROS 固定轴约定 `R = Rz(yaw) × Ry(pitch) × Rx(roll)`。下面每个变换表示“子坐标系在父坐标系中的位姿”，点坐标满足 `p_parent = R × p_child + t`。

| YAML 项 | 父坐标系 → 子坐标系 | 平移 xyz（米） | 旋转 rpy（度） |
|---|---|---|---|
| `extrinsics.rear_to_base` | `base_footprint` → `base_link` | `[0, 0, 0]` | `[0, 0, 0]` |
| `extrinsics.rear_to_lidar` | `base_footprint` → `mid360_link` | `[0.432445, 0, 0.368458]` | `[0, 25.06895, 0]` |
| `extrinsics.lidar_to_cloud` | `mid360_link` → `body` | `[0, 0, 0]` | `[0, 0, 0]` |
| `extrinsics.lidar_to_imu` | `mid360_link` → `livox_frame` | `[0.011, 0.02329, -0.04412]` | `[0, 0, 0]` |

MID360 官方手册确认内部 IMU 与点云输出三轴同向，原点有上述偏移。`body` 与 `mid360_link` 同原点同方向由用户确认；原生 EKF 的 `base_link` 在本项目中与后轮中心 `base_footprint` 重合。这两组恒等关系由配置校验器检查，不能只改 frame 名称来替代实际外参。

**+25.06895° 是车辆接近水平时根据静止重力估计、经用户授权用于回放的前下倾角，尚未完成独立实测标定。** `confirmed_for_replay: true` 表示已同意以此试跑，`calibration_verified: false` 明确保留标定状态。修改 `rear_to_lidar` 后，同一组外参会同时用于静态 TF、初始位姿逆变换以及 IMU 到车体的角速度旋转，不存在另一套隐藏安装角。

锁定版本的原生 gyro 存在坐标旋转方向问题，适配器先按正确外参变换角速度及协方差，再交给原生 gyro；原始录包和上游源码没有改写。依据见 [`reports/gyro_transform_review.md`](reports/gyro_transform_review.md)。

### 原始 Livox 点云处理

输入与计算链路为：

```text
/livox/lidar (CustomMsg) ── 解码、过滤、逐点运动补偿 ── /localization/pointcloud/deskewed
                                  ↑                              │
/hunter_odom 的前向速度 ────────────┤                          可选下采样
/livox/imu 的三轴角速度 ────────────┘                              │
                                                              NDT ← PCD 地图
轮速 + IMU → 原生 gyro_odometer → 原生 EKF ←───────────────────────┘
                                      └── 预测位姿送回 NDT
```

`livox` 下的 YAML 参数控制原始消息 frame 校验、距离范围、标签过滤、允许的扫描时长、IMU/轮速采样间隔和等待时间。逐点时刻为 `timebase + offset_time`（纳秒），补偿到该扫描最后一个点的时刻；派生 PointCloud2 的时间戳采用这个帧末时刻。完整三维旋转与前向平移使用同一组安装外参，并计入后轮中心到雷达原点的杠杆臂。

`adapter.voxel_size` 控制可选体素下采样，当前为 `0.0`，保留经过前述过滤的所有点。`adapter.scan_relay_delay: 0.08` 给原生 NDT 留出转发间隔（墙钟秒）；其输入队列只保留一帧，过小的间隔可能在缺测等待后集中转发时覆盖尚未处理的扫描。

**原始点云的 `header.frame_id` 虽然也是 `livox_frame`，其 XYZ 表示雷达点云原点，不是 IMU 芯片原点。** 本程序检查这个驱动标签后，将点云按已确认同原点同方向的 `body` 输出；不会对原始点云误加 `lidar_to_imu` 的平移。IMU 角速度仍按 IMU 到车体的旋转变换。

录包存在 IMU 缺口。只有覆盖整段扫描且相邻样本间隔满足 YAML 门限时，才进行完整去畸变；覆盖不足则整帧保留未补偿坐标供 NDT 匹配，并明确记录 `uncompensated`，不使用轮速里程计的姿态/角速度或旧定位 TF 填补。此时帧末时间戳仅表示选定的扫描参考时刻，并不表示该帧已经完成运动补偿。

仓库提供与官方消息定义一致的 `livox_ros_driver2` **消息接口包**，用于反序列化和订阅 CustomMsg，不包含硬件驱动或 Livox SDK。连接实体雷达时，在独立驱动工作区运行官方驱动并保持消息定义一致；不要把两个同名 `livox_ros_driver2` 包放进同一源码工作区。

### 初值、地图和回放

`initial_pose.reference: cloud` 表示 YAML 初值是 **map → body**；`xyz_m` 为米，**这里的 `rpy_rad` 为弧度**。仅回放或显式 `live.initialization: config` 使用它，并用外参换算成后轮中心位姿。保存的初值来自 `GlobalMap_loc.pcd` 首帧几何配准，不是未知起点的全局重定位；换场景时应使用 `/initialpose` 或重新核查配置初值。

默认回放参数位于 `replay`：倍速 `rate: 0.5`、隔离域 `domain_id: 58`、仅本机通信 `localhost_only: true`。点云输入为 `/livox/lidar`；运动输入为 `/hunter_odom` 的前向速度和 `/livox/imu` 的角速度。不消费轮速累计 pose、轮速角速度、IMU orientation 或 IMU 加速度。

录包回放要求 `runtime.use_sim_time: true`。输入按原始 header 采集时间排序，原始消息内容和时间戳不改，不复现原系统的消息记录延迟。原始扫描先进入缓存，待运动观测覆盖帧末后处理；派生点云使用上文所述帧末时间。首端仅预热时钟 3 秒，末端推进 0.6 秒用于处理尾部扫描和缺测等待，不添加虚构传感器观测；末端时长必须至少覆盖 `max_scan_duration_s + wait_timeout_s`。

## 常用命令

```bash
# 仅检查配置、地图路径和环境，不启动节点，也不要求传感器在线
bash start.sh --check

# 使用另一份配置（外参仍只读该 YAML）
bash start.sh --config /绝对路径/localization.yaml

# 快速验证录包前 30 秒
bash start.sh --mode replay --max-bag-seconds 30

# 已确认编译过当前代码时，跳过增量编译
bash start.sh --no-build

# 指定新的结果目录，已存在的目录不会覆盖
bash start.sh --output /绝对路径/新结果目录
```

Windows 将上述 `bash start.sh` 替换为 `.\start.cmd` 即可，例如：

```powershell
.\start.cmd --check
.\start.cmd --mode replay --config "E:\配置目录\localization.yaml" --max-bag-seconds 30
```

`--mode live|replay` 可显式选择模式，并同步设置该模式的时钟；YAML 自身的 `runtime.mode` 与 `use_sim_time` 必须一致。`--rate` 和 `--max-bag-seconds` 仅用于回放；在实时模式传入会明确报错。有效配置写入运行快照。外参不提供第二套命令行覆盖。旧 `scripts/replay_hunter.sh` 仍明确启动回放模式。

## 输出和检查

实时状态目录为 `artifacts/fusion/live-日期-时间/`（可改 `paths.output_root`），包括完整配置快照、原子更新的 `summary.json` 和滚动 `launch.log`。状态摘要包含最近的融合/去畸变状态、输出计数和退出原因；日志大小与备份数由 `live.log_max_bytes`、`live.log_backup_count` 控制。实时模式不会持续保存全量点云或无限增长的 CSV。

录包验证目录为 `artifacts/fusion/fusion-日期-时间/`，每次使用新目录：

| 文件 | 内容 |
|---|---|
| `config/localization.yaml`、`config/native/*.yaml` | 本次实际使用的完整配置快照；ROS 节点直接读取该快照 |
| `summary.json` | 是否完整回放、输入计数、NDT 收敛比例、EKF 两路更新和连续性检查 |
| `ekf.csv`、`ndt.csv`、`gyro.csv` | 位姿与速度结果 |
| `diagnostics.jsonl`、`fusion_status.jsonl` | 原生诊断、降级状态及配置来源 |
| `preprocessing_status.jsonl` | 原始 Livox 收发计数、完整补偿和未补偿扫描数量及原因 |
| `scan_samples.json`、`scan_samples/*.npy` | 从本次原始点云处理输出抽样保存的实际点云，用于几何评估 |
| `metrics.csv`、`inputs.csv`、`launch.log` | 匹配指标、实际发布记录与节点日志 |

`summary.json` 中 `status: completed`、`full_bag_completed: true` 和 `estimator_checks_passed: true` 表示完整回放及当前运行检查通过；这些字段**不表示绝对定位精度通过**。短段回放会标为 `partial`。

主要 ROS 输出：

| 话题 | 含义 |
|---|---|
| `/localization/kinematic_state` | EKF 后轮中心位姿和速度，map → base_link |
| `/localization/pose_with_covariance` | EKF 融合位姿 |
| `/localization/pose_estimator/pose_with_covariance` | 通过原生收敛判据的 NDT 位姿 |
| `/localization/twist_estimator/twist_with_covariance` | 原生 gyro 融合速度 |
| `/localization/fusion_status` | 输入新鲜度、接收数量与外参来源 |
| `/localization/pointcloud/deskewed` | 原始 Livox 派生 PointCloud2；是否完整补偿需结合状态话题判断 |
| `/localization/livox/status` | 去畸变覆盖情况及过滤统计 |

公开 TF 为 `map → base_footprint`。RViz 的 Fixed Frame 使用 `map`；原生 NDT 的调试 TF 已隔离，不作为正式定位结果。入口不自动打开 RViz。

录包验证结束后可生成点云/地图几何评估图（不适用于实时状态目录）：

```bash
python3 scripts/evaluate_fusion_result.py --results artifacts/fusion/某次结果目录
```

评估器会检查实际运行配置。原始 Livox 运行只使用本次保存的派生点云样本，不使用旧录包中的处理后点云替代。当前几何评估工具仅支持本次纯 pitch 安装；若改为非零 roll/yaw，它会明确拒绝。NumPy、SciPy、Matplotlib 已包含在依赖安装脚本中。

## 已完成的验证与限制

**本次实时版本通过 75 项自动测试和 31 项外部话题集成检查**，工作区 34 个包构建通过，最终适配包已重新构建。35 秒测试发布阶段共发送 350 帧原始 Livox 点云，观察到 345 帧预处理输出及 341 条 NDT 位姿；整轮测试包含等待初值、时间戳异常、长断流休眠/唤醒及二次初始化，共观察到 514 条 NDT、986 条 gyro 和 2,699 条公开 EKF 输出。原生诊断确认位姿和速度两路都进入 EKF 更新路径。

保留的 30 秒录包回归也通过：298/298 帧 NDT、537 条 gyro、1,210 条 EKF 输出。上游 31 个包的源码锁校验无差异。结果与验证边界见 [实时定位验证报告](reports/live_localization_validation.json)。

实时集成测试使用独立的测试发布器，见 `scripts/test_live_localization.py`。该脚本读取测试录包并将时间映射到当前系统时钟，定位进程自身仍只订阅话题，配置中 `paths.bag: null`。测试发布器会显式记录时钟映射变化；它不能证明真实雷达时钟同步、网络延迟或定位精度。生产实时入口 `start.sh --mode live` 不包含这些时间改写行为。

此前在这台 WSL 上观察到数秒的系统时钟跳变，相关轮次因消息过期或时钟倒退而失败，未计为验证通过。最终一轮完成全部集成检查，但**尚未连接真实车辆/MID-360，也未做长时间连续运行验证**；实车部署仍需确认稳定的时钟、网络和安装标定。

**此前原始 Livox 回放版本**已通过 55 项测试、34 包编译和 Windows 一键完整回放：967 帧原始 CustomMsg 全部完成转换和转发，967/967 帧输出通过原生收敛检查的 NDT 位姿；同时获得 1,747 条 gyro 融合速度及 3,661 条公开 EKF 位姿。原生 EKF 诊断确认点云位姿和轮速/IMU 速度都持续进入更新路径。10 帧本次派生点云抽样的 EKF 结果在 0.2 米内地图重合率约 96.08%，这不是独立真值定位精度。

当前严格采样间隔门限下，**48 帧完成完整去畸变，919 帧因 IMU/轮速覆盖不足而整帧未补偿**，无扫描拒收。完整回放成功不意味着每帧都有足够 IMU 数据；状态与报告保留这一差异。详见 [原始 Livox 运行验证](reports/raw_livox_release_validation.json)、[点云与地图几何报告](reports/raw_livox_fusion_validation/validation.md)、[原始输入审计](reports/raw_livox_input_audit.json) 和 [原始首帧初值检查](reports/raw_livox_initial_check.json)。

此前使用 `/cloud_registered_body` 的版本通过了 28 项测试、33 包编译及 Windows 一键整包回放；这些是历史记录，不能作为当前原始 `/livox/lidar` 链路的验证结果。详细记录见 [`reports/yaml_release_validation.json`](reports/yaml_release_validation.json) 和 [`reports/windows_launcher_validation.json`](reports/windows_launcher_validation.json)。

历史处理后点云完整融合验证得到 969/969 帧 NDT 位姿、1,679 条 gyro 融合速度和 4,889 条公开 EKF 位姿，见 [`reports/final_fusion_validation/validation.md`](reports/final_fusion_validation/validation.md)。

已知限制保留在本次版本中：

- 安装角与协方差下限仍是试验参数，没有独立真值轨迹，几何重合度不等于定位位姿精度。
- IMU 录包存在间断，期间会出现 `NDT_ONLY` 等状态。观测结束后仅允许配置时限内的预测；本次回放在有限尾部时钟后退出，最终状态以实际记录为准。
- 历史处理后点云版本在第 62.7 秒附近出现过约 0.64 米的 NDT 跳动；新链路也需独立检查连续性。全帧收敛不能替代质量验证。
- 原始扫描只有在 IMU 和轮速覆盖充分时才能完整去畸变，未补偿帧在运动中仍可能存在畸变。

## 工作区与源码来源

主配置在根目录 `config/`；启动程序在 `start.*`、`scripts/`；输入适配与配置校验在 `src/aps_bag_localization/`；原生依赖在 `src/vendor/`。保留 7 个 Autoware 上游仓库内的 31 个 ROS 包，加上 2 个适配包和 1 个 Livox 消息接口包，共 34 包。辅助 NDT/旧相对里程计入口属于历史诊断工具，正式一键启动使用原始 Livox 完整融合链。

构建缓存默认位于 `~/.cache/cssc_loc/<当前仓库路径哈希>/`，不同克隆目录互不混用。可通过 `APS_BUILD_ROOT` 显式指定缓存路径；不要把另一份源目录生成的 CMake 缓存搬过来复用。构建产物、大型数据、运行日志和配置快照不提交 Git。

源码基线为 `libpet-co/autoware.APS` 的 `hmi_container_dev` 提交 `1b29946659d716b06c1712fb39f269566297332e`。`localization.repos` 固定依赖版本，`sources.lock.json` 保存逐包来源及哈希，`config/upstream.autoware.repos` 保留原清单：

```bash
python3 scripts/verify_source_lock.py
```

新增适配代码使用 Apache-2.0；上游代码、内嵌依赖的 LICENSE 和 NOTICE 按原许可证保留，详见 `LICENSE`、`third_party/` 以及各包中的许可证文件。
