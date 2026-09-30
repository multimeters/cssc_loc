# MID-360 内置 IMU 坐标系核实

核实日期：2026-09-30。针对 MID-360，未借用 MID-360S 或 MID-360L 参数。

## 官方依据

1. [Livox MID-360 用户手册（2024.04）](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf)：印刷第 15 页、PDF 第 17 页 IMU Data。已经读取原始 PDF 并检查坐标示意图。
2. [官方驱动固定提交](https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/src/lddc.cpp#L480-L496)：`InitImuMsg` 直接将 gyro_x/y/z 赋给 ROS angular_velocity.x/y/z，无轴交换、旋转或单位换算；frame_id 被命名为 livox_frame。
3. [官方通信协议 IMU Data](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html)：角速度单位 rad/s，加速度单位 g。
4. 同一固定提交中，[SDK 数据回调处理](https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/src/comm/pub_handler.cpp#L97-L126) 把 `LivoxLidarEthernetPacket::data` 解释为 `RawImuPoint`，逐项复制到 `ImuData`；[IMU 队列](https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/src/comm/lidar_imu_data_queue.cpp#L29-L44) 再原样复制；[数据结构注释](https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/src/comm/lidar_imu_data_queue.h#L33-L60) 明确 `gyro` 为 rad/s、`acc` 为 g。已检查从 SDK 回调到 ROS 发布的这条实际代码路径，没有额外坐标变换或 g 到 m/s² 的转换。

官方手册来源可从 [MID-360 下载页](https://www.livoxtech.com/mid-360/downloads) 复核。硬件轴图位于印刷第 12 页/PDF 第 14 页，IMU 同轴说明和内部位置位于印刷第 15 页/PDF 第 17 页。当前检查的官方驱动提交完整 SHA 为 `21445540f0d100dc86a7e6df312dd70bbdb4afdf`；它是用于说明官方实现的源码证据，不等于已证明本次录包使用了这一提交或未经改动的发布链。

## 结论

MID-360 内置 IMU 输出轴与点云坐标轴同方向，旋转矩阵为单位矩阵。IMU 原点在雷达坐标系中的位置为 `(0.011, 0.02329, -0.04412)` 米；原点并不重合。

对于未经二次旋转的官方驱动输出，ROS TF parent=`mid360_link`、child=`livox_frame` 应使用上述平移和四元数 `(0, 0, 0, 1)`。逆向变换的平移取相反数。gyro_odometer 仅融合角速度，内部位置偏移不影响其角速度旋转。

用 `L` 表示雷达点云坐标系、`I` 表示实际 IMU 原点坐标系，则手册描述的是 `p_L = R_LI p_I + t_LI`，其中 `R_LI=I`、`t_LI=(+0.011,+0.02329,-0.04412)` m。这是 **IMU 原点在雷达坐标系中的位置**。反方向为 `p_I = p_L - t_LI`，即 parent=`livox_frame`、child=`mid360_link` 时平移为 `(-0.011,-0.02329,+0.04412)` m。不能只凭“雷达到 IMU”这种口头说法决定符号，也不能把驱动复用 `livox_frame` 标签视为两个物理原点重合的证据。

该固定版本官方驱动发布的角速度已经是 rad/s，不需要角度到弧度的转换；其加速度数值仍是 g。若后续算法真正使用线加速度，必须检查实际发布链是否已执行单位转换，避免漏乘或重复乘重力常数。本任务标准 gyro_odometer 路线读取 IMU 角速度，不利用 IMU 线加速度做积分；静止加速度在这里仅作为安装方向一致性的条件检查。

用户已确认 `body` 与 `mid360_link` 同原点、同方向；`base_footprint` 是后轮中心；`hunter_odom` 仅作为纵向速度来源，不融合其累计位姿。录包中 `base_footprint → base_link` 为单位变换，可兼容上游 EKF 的 base_link 输出。

## 当前需要核实的安装信息

录包 `base_footprint → mid360_link` 的 pitch 为 -30 度。静止 IMU 比力的 x 分量为负，在同轴且车辆近水平的条件下，与该安装角预期的正 x 分量不一致。不能用已确认的 IMU 同轴关系掩盖此差异，也不应自动翻转安装角。应先确认录包时车体姿态和该安装 TF 是否为当前有效配置，再进行标准 gyro_odometer + EKF + NDT 融合回放。

此核实没有运行融合，也没有更改录包或安装外参。

## 本次录包数值检查

按 IMU 时刻匹配最近轮速样本（时间差小于 0.1 秒），筛选 `abs(vx)<0.01 m/s` 且陀螺仪模长 `<0.02 rad/s`，得到 729 条近静止样本。加速度分量中位数为 `[-0.4209882, 0.00117235, 0.89998287] g`。使用录包 `Ry(-30°)` 旋转后为 `[-0.81457791, 0.00117235, 0.56891393] g`，与车辆 +Z 相差约 55.07 度。

若车辆处于近水平地面且输出未被二次旋转，该观测提示安装 pitch 约 +25.07 度，而不是 -30 度。这是用于发现配置矛盾的条件推断，不是已经完成的外参标定；没有据此修改 TF。需要用户确认实际车体姿态、安装角及录包静态 TF 是否有效。

这个方向推断还要求车辆坐标轴采用 x 向前、y 向左、z 向上的约定，且所读取的旋转确实是把传感器坐标转换到车辆坐标的 `R_base_sensor`。如果读取的是逆变换、坐标轴定义不同、数据被二次旋转或车辆有明显坡度，上面的数值仍可复算，但不能直接解释为真实安装角。

## 为什么 RViz 看起来正确仍不能单独确定安装角

已知录包有两条从 map 出发的链：`map → camera_init → body` 和 `map → base_footprint → mid360_link`；点云 `/cloud_registered_body` 的消息 frame 为 `body`。若 RViz 的 Fixed Frame 为 map，则该点云显示使用前一条链，静态安装角位于后一条链，点云与地图重合并没有检验后一条链的安装角。用户确认 body 与 mid360_link 物理相同，但当前 TF 拓扑没有通过同一固定边强制二者在数据里保持相同。

这不证明整个 bag TF 错误，也不能据此认定用户在 RViz 中看到的对象、Fixed Frame 或坐标轴一定为何。需要实际 RViz 配置才能把这条一般机制落实到那次显示。以下均是待核实解释，而非已经发现的事实：读取父子逆变换造成符号误读；URDF visual/mesh origin 的额外旋转让模型外观与 link 轴不同；车辆 base 坐标轴不采用上述约定；IMU 发布前存在额外旋转；真实静止地面并不接近水平。当前没有基于这些可能性修改任何 TF 或运行融合。
