# APS 原生 Autoware 定位包

本包连接提取的 Autoware 原生 `gyro_odometer`、EKF、NDT 和地图加载器，支持实时订阅与录包回放。输入为原始 Livox 点云、IMU 角速度和车轮前向速度，公开定位结果统一表示后轮中心。

实时和回放使用同一套配置、外参和估计器。当前软件测试与录包验证不等同于真实车辆在线验证；安装角仍是已获授权的暂定估计，定位精度需要独立基准验证。

## 启动与模式

在仓库根目录编辑 `config/localization.yaml`，使用统一入口：

```bash
# 默认遵循 YAML 的 runtime.mode；仓库默认是 live。
bash start.sh

# 实时订阅，不读取 bag。
bash start.sh --mode live

# 仅检查配置、地图和环境，不要求传感器在线。
bash start.sh --mode live --check

# 显式回放，也可先验证前 30 秒。
bash start.sh --mode replay --max-bag-seconds 30
```

实时模式需要 ROS 2 Humble、地图及外部传感器话题。`paths.bag` 可以为 `null`，不影响实时启动。推荐上述入口：它负责设置 YAML 中的 ROS domain 和通信范围、加载构建环境，并保存生效配置及原生参数快照；实时运行保存状态和滚动日志。

已经加载 ROS 与工作区环境时，也可直接调用通用 launch：

```bash
ros2 launch aps_bag_localization localization.launch.py \
  config_file:=/absolute/path/to/aps-localization/config/localization.yaml
```

`config_file` 是唯一 launch 参数，必须为绝对路径。通用入口遵循文件内的 `runtime.mode`。**直接调用 launch 前，必须手动设置与 YAML 一致的 `ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY`**，并确保传感器发布端处于可通信的环境；launch 本身不应用这两个环境变量。

`fusion_replay.launch.py` 只接受 `runtime.mode: replay`、`runtime.use_sim_time: true` 的配置快照。默认主配置为 `live/false`，不能直接交给回放入口；仓库回放脚本负责模式切换、快照及仿真时钟。任一必要节点退出都会停止整套 launch，避免缺少部分估计器后继续驻留。

## 单一配置来源

主配置及其引用的完整原生 YAML 是运行参数的唯一来源；外参、传感器偏移与初值没有另一套命令行覆盖值。

| 配置项 | 内容 |
|---|---|
| `paths` | 地图、可选地图元数据、可选 bag、结果根目录 |
| `runtime` | `live/false` 或 `replay/true` 的模式与时钟组合 |
| `live` | ROS 通信设置、初始化来源、时效与初值确认容差、状态及日志限制 |
| `replay` | 回放速率、ROS 通信设置、预热、尾部时钟和结束等待 |
| `frames`、`topics`、`services` | 适配器和原生节点共用的名称 |
| `extrinsics` | 四条物理或恒等变换、确认信息和来源说明 |
| `initial_pose` | 地图到点云帧的配置初值、协方差与来源 |
| `livox` | 原始帧标签、距离/tag 过滤、扫描时长、运动覆盖与等待限制 |
| `adapter` | 协方差下限、观测与转发超时、队列和定时器 |
| `native_parameters` | 完整 gyro、EKF、NDT、地图加载器参数文件路径 |
| `validation` | 数量、连续性和覆盖检查；不代表物理精度标准 |

相对路径以主 YAML 所在目录为基准。原生 YAML 中 `${frames.base}`、`${paths.map}` 等完整引用由加载器解析并保留 YAML 类型，不是 shell 表达式。不要将未解析的原生文件直接传给 `ros2 run --params-file`。

`aps_bag_localization.configuration.load_config(path, mode=None)` 无需 ROS 即可使用。它先校验文件内模式与时钟一致，再按显式 `mode` 覆盖同时调整二者，返回绝对路径、`_derived` 外参及初值和 `_native_parameters` 原生参数。重新加载快照时会重新校验并计算派生值，不能通过快照中的派生字段覆盖主外参。配置也随包安装到 share 目录。

## 实时初始化与时间要求

默认 `live.initialization: topic`，等待 `topics.initial_pose_input`，默认 `/initialpose`，类型为 `geometry_msgs/msg/PoseWithCovarianceStamped`。消息必须满足：

- `header.frame_id` 是配置的地图坐标系，默认 `map`。
- pose 表示**后轮中心 / `base_link` 在地图中的位置与姿态**，不是雷达位置。
- 四元数归一化、数值有限；协方差有限、对称、半正定，x、y、yaw 方差大于零。

可使用 RViz 初始位姿工具或外部程序发布符合上述约定的初值。外部初值已是地图到后轮中心的变换，适配器不会再次乘雷达安装外参。内部初始化消息使用当前时钟时间；原始传感器时间戳保持不变。

接收初值后，适配器按 Autoware pose initializer 的顺序停用 NDT/EKF，转发一帧新点云并调用 `/localization/pose_estimator/ndt_align_srv`，使用初值协方差运行原生 NDT Monte Carlo/TPE 搜索；只有服务返回 `success=true` 时，才把对齐后的位姿发布给 EKF，随后重新激活 NDT/EKF。RViz 配置会显示 `/localization/pose_estimator/monte_carlo_initial_pose_marker` 中的候选箭头。原生 EKF 初值话题没有应答，因此适配器检查初始化后的 EKF 预测是否在 `live.initial_pose_ack_position_m` 和 `live.initial_pose_ack_angle_rad` 容差内匹配**对齐后的位姿**。确认前不释放新扫描和轮速/IMU 派生观测；确认后还需新的 NDT 观测，才允许公开定位输出。

适配器同时提供与 Autoware `InitializeLocalization` 相同字段的 `/localization/initialize` 服务（消息类型为 `autoware_internal_localization_msgs/srv/InitializeLocalization`）。`AUTO` 使用请求中的地图坐标初值进入 NDT Monte Carlo，`DIRECT` 跳过 NDT 搜索而直接走 EKF/NDT 停启和复位流程；本 Hunter 配置没有 GNSS，因此不带 pose 的 `AUTO` 请求会返回错误。服务响应表示请求已接收，实际完成状态通过 `/localization/fusion_status` 的 `mode`、`initial_pose_acknowledged` 和 `monte_carlo` 字段观察。

Monte Carlo 服务返回失败时，适配器不会把粗初值直接写入 EKF；请重新用 RViz 发布 `/initialpose`。服务若返回 `reliable=false`，会沿用 Autoware 的行为继续使用成功返回的对齐位姿，同时在 `fusion_status.monte_carlo.reliable` 和 `last_rejection` 中保留告警。

再次发布外部初值可以重新初始化。重置期间暂停公开输出，丢弃旧扫描并等待新观测。等待初值时不会持续向未初始化的原生 EKF 积压速度观测。

只有显式设置 `live.initialization: config` 才会自动采用 YAML 中保存的初值。该配置初值与外部 `/initialpose` 含义不同：它是地图到点云帧的变换，需先换算为后轮中心。回放也使用这一配置初值。

实时模式使用系统时钟，`use_sim_time: false`，不发布 `/clock`。传感器采集时间应与运行机器系统时间一致；不要向实时模式发送历史 bag 时间戳。节点按 `live.max_sensor_age_s`、`live.future_tolerance_s` 拒绝过旧或超前数据，并限制等待队列。不能以修改原始采集时间绕过时效检查。系统时钟倒退会进入需要重启的状态。

## 原生估计器与点云处理

```text
轮速 linear.x ──────────────> 原生 gyro_odometer ──> 原生 EKF 速度观测
IMU 角速度 ──坐标旋转───────────────^
/livox/lidar CustomMsg ──过滤、运动补偿──> body PointCloud2
                                           │
                                           v
                                       原生 NDT ──> 原生 EKF 位姿观测
原生 EKF 预测位姿 ──────────────────────────> 原生 NDT 初始预测
```

没有 NDT 自反馈节点。适配器不持续发布位姿观测或 NDT 预测，这些连接直接发生在原生 NDT 与原生 EKF 之间。点云转发等待真实 EKF 预测时间范围覆盖扫描结束时刻，再送入 NDT。

轮速只使用前向 `linear.x` 及协方差，不使用里程计的 pose、姿态或角速度。IMU 只使用角速度；派生 IMU 的加速度和姿态被标记为不可用。

预处理器按整数纳秒处理点内偏移，支持乱序点偏移；使用测得的三维角速度与前向速度补偿旋转、平移和雷达安装杆臂，输出时间戳为扫描结束时刻。

运动覆盖不完整时，等待限制后整帧输出未经运动补偿的点云，并计为 `uncompensated`。扫描结束时间戳或默认输出话题名中的 `deskewed`，均不代表该帧成功补偿。应查看 `topics.preprocessing_status` 的 `fully_deskewed`、`uncompensated` 和原因记录。

等待扫描结束及运动样本使用所选 ROS 时钟：实时为系统时钟，回放为仿真时钟。慢速回放不会导致扫描在采集结束前提前输出。历史 `/cloud_registered_body` 不作为主定位链输入。

## 坐标约定与外参状态

NDT 和 EKF 均估计后轮中心，帧名统一从 YAML 读取：

```text
rear  -> base     恒等；原生 EKF 要求 base 命名为 base_link
rear  -> lidar    唯一一份可配置的安装变换
lidar -> cloud    已确认的恒等别名
lidar -> IMU      内部 IMU 的物理偏移与轴向
```

原始 Livox 消息复用 `livox_frame` 标签，但点坐标属于雷达原点，IMU 属于芯片测量坐标系。预处理器按雷达原点解释原始点，并输出到已确认与雷达重合的 cloud 帧；**不会将 IMU 芯片平移应用到点云**。

加载器拒绝非恒等 rear→base、非恒等 lidar→cloud 或非 `base_link` 的 base 命名，因为这些配置违反当前原生链的约束。全部外参只由 YAML 提供，同时用于静态 TF、初值换算、IMU 旋转和点云运动补偿。

当前前倾角是依据静止重力估计且经用户授权采用的暂定外参，并非测量标定值；来源保留在 YAML 与融合状态中。MID360 官方手册提供 IMU 同轴约定和内部偏移；Python 活跃路径中没有另一份实验外参数字默认值。

回放及 `live.initialization: config` 的初值为 `map -> cloud`，使用相同外参计算：

```text
T_map_base = T_map_cloud * inverse(T_base_cloud)
R_base_imu = R_base_lidar * R_lidar_imu
```

实时外部 `/initialpose` 直接提供 `T_map_base`，不执行第一条换算。

原生 EKF 和 NDT 的动态 TF 隔离在配置的内部话题。适配器公开 `map -> rear`，随后通过静态恒等 `rear -> base` 连接，避免同一帧存在两个父节点。公开 Odometry 的 `child_frame_id` 为 `base_link`，参考点为后轮中心。

## 锁定版本的 IMU 旋转兼容处理

锁定的原生 gyro 调用 `get_latest_transform(imu_frame, base_link)`，实现直接作为 TF2 `lookupTransform(target, source)` 参数。源码检查与真实 ROS TF2 数值测试确认，这会将逆方向旋转应用到 IMU 帧向量。

本包不修改 vendor。融合适配器先用正确的 `R_base_imu` 旋转角速度，完整协方差变换为 `R C R^T`，以 `base_link` 发布派生 IMU；原生 gyro 因此使用恒等变换，避免重复旋转。原始消息、时间戳和物理 TF 保持不变。刚体角速度不受 IMU 平移影响。

未知对角协方差补入 YAML 下限，保留有效交叉项；不合法协方差进行对称化和保守正则化。之后仍经过原生 gyro 的协方差平均/各向同性近似。状态记录实际旋转四元数及 `upstream_gyro_inverse_tf_workaround: true`。这些下限是数值假设，并非已标定噪声。

## 输出、断流与回放约束

| `topics` 配置键 | 内容 |
|---|---|
| `odometry`、`pose` | 经状态及时效保护的后轮中心融合结果 |
| `ndt_pose` | 原生 NDT 接受的后轮中心位姿观测 |
| `gyro_twist` | 原生轮速/陀螺仪速度结果 |
| `ekf_prediction` | 直接供 NDT 使用的原生 EKF 预测 |
| `fusion_status` | 初始化状态、观测时间、模式、计数和外参来源 |
| `preprocessing_status` | 点云处理、补偿覆盖和退化原因 |
| `metrics_prefix` | NDT 收敛分数、迭代数与耗时话题前缀 |

实时启动首先处于 `WAITING_INITIAL_POSE`，初始化期间为 `INITIALIZING`，等待初始化后 NDT 数据时为 `WAITING_SENSORS`。进入工作状态后，两路观测新鲜为 `FUSED`；只有 NDT 新鲜为 `NDT_ONLY`；只有轮速/陀螺仪速度新鲜为 `GYRO_ONLY`。两路均中断时，短时间可标为 `PREDICTING`，超过保护时限则为 `STALE` 并停止公开输出。

短时 IMU 中断不会永久锁死融合。真实新观测恢复后允许恢复输出，不会伪造 IMU、轮速或静止位姿。时钟倒退是例外，需要重启。应同时检查原生 EKF 的诊断、测量门限和更新计数；话题到达本身不能证明 EKF 接受了观测。

`live.ndt_idle_timeout_s` 限制没有有效 NDT 结果时的持续激活时间。到期后停用 NDT，收到新鲜有效点云再启用并清理先验缓存，等待新一轮 EKF 预测覆盖后转发扫描。这样长期断流或持续无效扫描也不会使上游 NDT 的预测历史无限增长；该管理逻辑与初值服务调用串行，保留原生 EKF 与 NDT 的直接连接。时钟倒退也会停用 NDT。

回放脚本按原始消息头时间排序，驱动未修改的传感器消息和仿真时钟。时钟预热供原生激活、EKF 预测及地图加载，尾部时钟覆盖末帧结束和预处理等待。不要同时播放 bag 的已有定位结果、`/tf` 或旧 `/tf_static`，以免覆盖当前估计及外参。

NDT 收敛、有限数值、回放覆盖率和连续输出只能说明对应运行检查通过，不能单独证明绝对定位精度或实车可用性。

## 验证范围

```bash
cd src/aps_bag_localization
python3 -m unittest discover -s test -v
```

纯配置、坐标和协方差测试无需 ROS。节点构造、传感器回调、初始化及进程联动测试需要 ROS Humble、Livox 消息接口和同工作区的 `aps_ndt_localization`。测试覆盖单一外参传播、帧约束、完整协方差旋转、轮速 pose 不注入、双模式时钟选择、初值与时效保护、断流恢复和必要进程退出后停止整图。

以上属于软件验证与已有录包验证，未据此宣称真实硬件在线定位已经验证。旧相对定位或独立 NDT 实验文件不属于当前融合 launch 或其验收结果。
