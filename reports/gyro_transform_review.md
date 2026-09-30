# 固定版本 gyro IMU 坐标转换审查

审查日期：2026-09-30。结论仅适用于本仓库锁定的源码组合。当前标准融合入口已在适配层把 IMU 角速度正确转换到 `base_link`，再交给未经修改的原生 `autoware_gyro_odometer`。这保持了轮速与 IMU → gyro → EKF ↔ NDT 的计算链。

## 固定源码和问题位置

`sources.lock.json` 以及本地上游 Git HEAD 给出：

| 源仓库 | 引用 | commit |
| --- | --- | --- |
| `libpet-co/autoware_core.APS` | `airy_test` | `d54caaf87d94dd2f584677e871031f5050c1d9d8` |
| `autowarefoundation/autoware_utils` | `1.4.2` | `10d45b89b9902382c56306cf2677b7db9c0846aa` |

原生 gyro 的 `src/vendor/autoware_gyro_odometer/src/gyro_odometer_core.cpp:178` 调用
`get_latest_transform(imu_frame, output_frame)`，随后第 203 行直接用返回值转换 IMU 的角速度。
[固定源码](https://github.com/libpet-co/autoware_core.APS/blob/d54caaf87d94dd2f584677e871031f5050c1d9d8/localization/autoware_gyro_odometer/src/gyro_odometer_core.cpp#L176-L207)

utils 的 `src/vendor/autoware_utils_tf/include/autoware_utils_tf/transform_listener.hpp:49` 把这两个参数原样传给 `lookupTransform(from, to, TimePointZero)`。tf2 这两个位置实际表示 `target_frame, source_frame`，因此上述调用得到的是 `T_imu_base`，而将 IMU 向量转换到车体系需要 `T_base_imu`。局部变量名 `tf_imu2base_ptr` 不改变实际方向。
[固定源码](https://github.com/autowarefoundation/autoware_utils/blob/10d45b89b9902382c56306cf2677b7db9c0846aa/autoware_utils_tf/include/autoware_utils_tf/transform_listener.hpp#L44-L49)

当安装旋转为单位矩阵时，两种方向相同，因而水平安装测试可能发现不了该问题。这不是要求反转真实 TF 的依据。

## ROS2 实际数值复现

使用本机 ROS2 Humble 的 `tf2_ros.Buffer.set_transform_static`、`lookup_transform` 和 `tf2_geometry_msgs.do_transform_vector3` 实测，未用本项目的旋转实现替代 tf2：

- 静态 TF：parent=`base_link`，child=`livox_frame`，绕 Y 轴 pitch=`+25.06895°`，四元数 xyzw=`[0, 0.21702701423569845, 0, 0.9761655981911768]`。
- 假设车体纯 yaw 角速度为 `[0, 0, 1] rad/s`。其 IMU 坐标分量应为 `[-0.4237086103500712, 0, 0.9057985501838759] rad/s`。
- 原生调用次序 `lookup_transform('livox_frame', 'base_link', Time())` 返回 quaternion y=`-0.2170270142356984`。用它转换上述 IMU 向量得到 `[-0.7675892899110386, 0, 0.6409420270304231]`。
- 正确次序 `lookup_transform('base_link', 'livox_frame', Time())` 得到约 `[0, 0, 1]`（x 的数值误差为 `-5.55e-17`）。
- `lookup_transform('base_link', 'base_link', Time())` 的旋转实测为单位四元数 `[0, 0, 0, 1]`。

因此这个安装角度下，原路径会把上述纯 yaw 测试的 z 分量减为约 0.641，并引入错误 x 分量。该合成例子用于验证变换方向，不代表实际车辆的角速度轨迹。

## 当前适配层处理及独立复核

`src/aps_bag_localization/launch/fusion_replay.launch.py:46` 计算
`q_base_imu = q_base_body * q_body_imu`，与静态安装链一致。用户已确认 `body=mid360_link`；官方说明内置 IMU 输出轴向与点云相同，故当前 `q_body_imu` 为单位旋转。芯片原点仍有官方给定平移，不能说二者同点，参见 `mid360_imu_frames.md`。

`src/aps_bag_localization/aps_bag_localization/fusion_adapter.py:149` 复制原始 header，再对角速度应用 `R_base_imu`，只把派生消息 frame 改为 `base_link`。原始 bag、原始 IMU 话题数据、采集时间戳和真实安装 TF 均未改写。原生 gyro 随后的同 frame 查询为单位变换，因此不会再反旋转。角速度属于同一刚体，其坐标旋转不需要加入平移项；加速度和位置观测不能据此省略杆臂处理，但当前 gyro 链不消费加速度或 IMU 姿态。

`src/aps_bag_localization/aps_bag_localization/geometry.py:40` 的角速度协方差采用完整 `R C Rᵀ`。未知/零对角项加配置下限，相关项保留并对称化；若不正定则增加对角量。这些下限是运行参数，未声称已标定。适配器把未使用的姿态和加速度协方差首项设为 `-1`。

独立复核包括：

1. 阅读实际 launch、适配器、几何函数与原生 gyro 源码，确认乘法次序、header 保留和输出 frame 相符，没有重放旧定位 TF，也没有向 NDT 注入自反馈 pose。
2. 用独立展开的 NumPy 四元数旋转矩阵，对 100 组随机单位四元数、角速度及正定完整协方差验证 `R v` 和 `R C Rᵀ`。最大绝对差 `4.218847493575595e-15`。
3. 上述 ROS2 tf2 数值复现同时确认同 frame 变换确实为单位变换。项目测试另覆盖纯 yaw、相关协方差旋转和两个不交换安装旋转的组合。

适配层保留完整协方差，不代表原生 gyro 的输出仍保留它。锁定的 `gyro_odometer_core.cpp:32` 中 `transform_covariance` 会取三个对角项的最大值，把协方差改为等向对角阵；随后还按队列数量缩放。此原生近似保持未改。更改此策略需要另行验证，不能把本次结果描述成完整协方差贯穿所有模块。

## 结果范围

安装 pitch=`+25.06895°` 是用户授权的暂定前倾估计，依赖近水平静止重力假设，不是测量标定。官方同轴信息不能独自确定传感器相对车辆的实际安装角。

`fusion-smoke-03` 使用旧变换，仅用于排查其它回放问题，不作为最终有效结果。`fusion-full-01` 的运行状态中已记录 `imu_derived_frame=base_link`、`upstream_gyro_inverse_tf_workaround=true` 和使用的旋转四元数；其完整包统计及地图几何验证以该结果目录的最终文件为准。

本次审查和修复范围是标准融合入口 `fusion_replay.launch.py`。旧 `relative_replay` 入口仍沿用此前原生 TF 流程，不属于当前带非零安装角的有效交付验证路径。

## 完整运行后的独立验收

读取 `results/fusion-full-01/summary.json`、`inputs.csv`、`ndt.csv`、`diagnostics.jsonl` 和最终 adapter 状态，确认该次运行 `completed`、`full_bag_completed=true`：

| 项目 | 独立核对结果 |
| --- | --- |
| 输入和 adapter 收到的数据 | 点云 969、轮速 4807、IMU 2304，逐类计数一致 |
| 点云转发 | 969，丢弃 0，末尾待发队列 0 |
| NDT 输出 | 969，其原始 header 时间戳逐帧与 969 个输入点云完全相等 |
| 原生 gyro 输出 | 1679；EKF `callback_twist` 也为 1679 |
| NDT → EKF | EKF `callback_pose` 969 |
| 公开 EKF 输出 | 4889，跨度 97.759 秒，最大相邻间隔 31.053 毫秒 |
| NDT → EKF 原生报告更新 | 4605 个有队列周期，全数进入更新路径，跨度 96.900 秒 |
| gyro → EKF 原生报告更新 | 2073 个有队列周期，全数进入更新路径，跨度 95.359 秒 |
| 原生 delay / Mahalanobis 门限 | 两路所有有队列周期均通过，无门限拒绝 |

更新证据按 `(diagnostic name, stamp)` 去重，只取 `localization: ekf_localizer` 合并诊断中的 `queue_size > 0` 且 `no_update_count == 0`。单独收到 callback 或无队列时默认 `True` 的门限值都不算已更新。因为 pose/twist 有平滑重复处理，更新周期数不是独立接纳消息数。进一步的源码限制是原生 `measurement_update_pose/twist` 未传播底层 `updateWithDelay` 的布尔返回值，因此这些指标严格表示原生测量更新路径报告成功；最终有限姿态、正方差和地图几何检验仍需一起看。

本次不是“全程无告警”。相对首点云采集时刻：

- 协方差椭圆告警集中在预热至 `+0.320` 秒，以及点云/IMU 结束后的 `97.440–98.304` 秒。
- 真实传感器间断期间反复出现 twist 缺测，twist 原生成功更新最大间隔 `0.860489` 秒；去重 gyro 诊断含 IMU 超时 484 个状态、轮速超时 53 个状态，阈值为 0.25 秒。
- 尾部点云和 IMU 已结束而轮速仍有记录，最后状态 `STALE`、`public_output_enabled=false`，没有无限延长预测输出。尾部有限预测期间可能先出现协方差告警，消费者应一起读取状态和协方差。

结论是该次完整输入已运行标准 Autoware 融合链，并通过当前处理完整性及原生门限验收；不等于三类观测始终同时新鲜，也不代表暂定外参已标定或绝对定位精度已经获得真值验证。回放按原始采集 header 排序并驱动时钟，不再现原始消息到达延迟。
