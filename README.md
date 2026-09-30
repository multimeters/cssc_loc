# CSSC 独立定位程序

本程序从 `libpet-co/autoware.APS` 默认分支 `hmi_container_dev` 提取定位所需组件，使用原生 Autoware **轮速＋IMU → gyro_odometer → EKF，NDT 位姿 → EKF，EKF 预测 → NDT** 链路。输出参考点是车辆后轮中心。

**所有本次定位参数均从 YAML 读取，外参只在 `config/localization.yaml` 定义一次。** 一键入口负责检查配置和数据、增量编译、启动节点、回放录包、保存结果并关闭本次启动的进程。

## 一键启动

### 当前 Windows 电脑

在仓库目录中，**双击 `start.cmd`** 即可启动。它自动调用 WSL 的 `Ubuntu-22.04`，不需要手动开启多个终端或逐个启动节点。

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

默认读取 `config/localization.yaml`，以 **0.5 倍速**回放整个指定录包。第一次需要编译，后续会增量编译；按 `Ctrl+C` 可停止。构建失败、配置错误或回放检查失败时，入口返回非零退出码，Windows 双击窗口会保留错误信息。

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

该选项安装编译工具、Python 依赖及 rosdep 解析出的 ROS 依赖，然后继续编译和回放；需要联网，可能要求输入 Ubuntu 的 sudo 密码。已完成依赖安装后直接使用普通的一键入口。

**录包和 PCD 地图不包含在 Git 仓库中。** 默认数据目录结构如下；如果位置不同，只修改主 YAML 的 `paths.bag` 和 `paths.map`：

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

修改 YAML 后重新一键启动即可，无需修改 Python 或 launch 文件。程序在启动前检查有限数值、单位向量长度、必要坐标关系、文件路径和输入话题类型；遇到错误会直接退出。

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

### 初值、地图和回放

`initial_pose.reference: cloud` 表示初值是 **map → body**；`xyz_m` 为米，**这里的 `rpy_rad` 为弧度**。启动时自动用外参换算为后轮中心位姿。当前初值来自 `GlobalMap_loc.pcd` 的首帧几何配准，不是未知起点的全局重定位；换地图或录包时必须同步核查初值。

默认回放参数位于 `replay`：倍速 `rate: 0.5`、隔离域 `domain_id: 58`、仅本机通信 `localhost_only: true`。默认只融合 `/hunter_odom` 的前向速度和 `/livox/imu` 的角速度，不消费轮速累计 pose、轮速角速度、IMU orientation 或 IMU 加速度。

录包回放要求 `runtime.use_sim_time: true`。输入按原始 header 采集时间排序，消息内容和时间戳不改，不复现原系统约 1.4 秒的点云记录延迟。首端仅预热时钟 3 秒，末端推进 0.2 秒用于处理尾部输出，不添加虚构传感器观测。

## 常用命令

```bash
# 仅检查配置、输入话题和环境，不编译、不回放
bash start.sh --check

# 使用另一份配置（外参仍只读该 YAML）
bash start.sh --config /绝对路径/localization.yaml

# 快速验证前 30 秒
bash start.sh --max-bag-seconds 30

# 已确认编译过当前代码时，跳过增量编译
bash start.sh --no-build

# 指定新的结果目录，已存在的目录不会覆盖
bash start.sh --output /绝对路径/新结果目录
```

Windows 将上述 `bash start.sh` 替换为 `.\start.cmd` 即可，例如：

```powershell
.\start.cmd --check
.\start.cmd --config "E:\配置目录\localization.yaml" --max-bag-seconds 30
```

`--rate` 和 `--max-bag-seconds` 可临时覆盖本次回放设置；有效值会写入运行快照。外参不提供第二套命令行覆盖。旧 `scripts/replay_hunter.sh` 已改为转发到同一个一键入口。

## 输出和检查

默认结果目录为 `artifacts/fusion/fusion-日期-时间/`（可改 `paths.output_root`），每次使用新目录：

| 文件 | 内容 |
|---|---|
| `config/localization.yaml`、`config/native/*.yaml` | 本次实际使用的完整配置快照；ROS 节点直接读取该快照 |
| `summary.json` | 是否完整回放、输入计数、NDT 收敛比例、EKF 两路更新和连续性检查 |
| `ekf.csv`、`ndt.csv`、`gyro.csv` | 位姿与速度结果 |
| `diagnostics.jsonl`、`fusion_status.jsonl` | 原生诊断、降级状态及配置来源 |
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

公开 TF 为 `map → base_footprint`。RViz 的 Fixed Frame 使用 `map`；原生 NDT 的调试 TF 已隔离，不作为正式定位结果。入口默认完成离线回放并保存结果，不自动打开 RViz。

可生成点云/地图几何评估图：

```bash
python3 scripts/audit_new_bag.py --data-root /你的数据路径/zhongchuanbag
python3 scripts/evaluate_fusion_result.py --results artifacts/fusion/某次结果目录
```

评估器会检查实际运行配置，当前几何评估工具仅支持本次纯 pitch 安装；若改为非零 roll/yaw，它会明确拒绝，不会默默套用旧角度。NumPy、SciPy、Matplotlib 已包含在依赖安装脚本中。

## 已完成的验证与限制

本次统一 YAML / 一键启动版本已通过 28 项测试，并从空缓存完整编译 33 个包；随后实际从 Windows `start.cmd` 启动、再次增量编译并完成整包回放，969/969 帧通过 NDT 收敛检查，程序正常退出。新 Git 克隆目录的源码校验、配置读取和启动预检也已通过。详细记录见 [`reports/yaml_release_validation.json`](reports/yaml_release_validation.json) 和 [`reports/windows_launcher_validation.json`](reports/windows_launcher_validation.json)。

2026-09-30 的原始完整融合验证得到 969/969 帧 NDT 位姿、1,679 条 gyro 融合速度和 4,889 条公开 EKF 位姿。两类测量均有持续进入 EKF 原生更新路径的证据；10 帧点云抽样中，EKF 变换结果在 0.2 米内的地图重叠率约 93.09%。详见 [`reports/final_fusion_validation/validation.md`](reports/final_fusion_validation/validation.md)。

已知限制保留在本次版本中：

- 安装角与协方差下限仍是试验参数，没有独立真值轨迹，几何重合度不等于定位位姿精度。
- IMU 录包存在间断，期间会出现 `NDT_ONLY` 等状态；尾部点云/IMU 比轮速早结束，最终进入 `STALE` 并停止公开输出。
- 第 62.7 秒附近，NDT 在约 0.1 秒内跳动约 0.64 米，超过同期轮速对应位移，根因尚未确定。全帧收敛不能替代质量验证。
- 使用的是录包已有的 `/cloud_registered_body` 处理后点云；本程序不重做 Livox 原始包转换和去畸变，也不重放旧定位 TF。

## 工作区与源码来源

主配置在根目录 `config/`；启动程序在 `start.*`、`scripts/`；输入适配与配置校验在 `src/aps_bag_localization/`；原生依赖在 `src/vendor/`。共保留 7 个上游仓库内的 31 个 ROS 包，以及 2 个适配包。辅助 NDT/旧相对里程计入口属于历史诊断工具，正式一键启动使用完整融合链。

构建缓存默认位于 `~/.cache/cssc_loc/<当前仓库路径哈希>/`，不同克隆目录互不混用。可通过 `APS_BUILD_ROOT` 显式指定缓存路径；不要把另一份源目录生成的 CMake 缓存搬过来复用。构建产物、大型数据、运行日志和配置快照不提交 Git。

源码基线为 `libpet-co/autoware.APS` 的 `hmi_container_dev` 提交 `1b29946659d716b06c1712fb39f269566297332e`。`localization.repos` 固定依赖版本，`sources.lock.json` 保存逐包来源及哈希，`config/upstream.autoware.repos` 保留原清单：

```bash
python3 scripts/verify_source_lock.py
```

新增适配代码使用 Apache-2.0；上游代码、内嵌依赖的 LICENSE 和 NOTICE 按原许可证保留，详见 `LICENSE`、`third_party/` 以及各包中的许可证文件。
