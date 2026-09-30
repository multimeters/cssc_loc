# 新定位录包审计

本报告仅针对 2026-09-30 新录包，旧报告保留。新录包有真实点云，可开展 PCD 地图匹配；本审计本身不代表定位成功。

| 话题 | 类型 | 数量 | Frame | header 平均 Hz |
|---|---|---:|---|---:|
| /cloud_registered_body | sensor_msgs/msg/PointCloud2 | 969 | body | 10.000 |
| /hunter_odom | nav_msgs/msg/Odometry | 4807 | hunter_odom | 49.557 |
| /livox/imu | sensor_msgs/msg/Imu | 2304 | livox_frame | 24.150 |
| /livox/lidar | livox_ros_driver2/msg/CustomMsg | 967 | livox_frame | 10.000 |
| /tf | tf2_msgs/msg/TFMessage | 1628 |  | 0.000 |
| /tf_static | tf2_msgs/msg/TFMessage | 11 |  | 0.000 |

`/cloud_registered_body` 为 body 坐标系中的 PointCloud2；这是一份已处理扫描，可直接测试 body 在地图中的位姿匹配。原始 `/livox/lidar` 是 livox_frame 中的 CustomMsg。

## 地图

| 文件 | 点数 | 字段 | xyz 最小 | xyz 最大 |
|---|---:|---|---|---|
| GlobalMap_loc.pcd | 115816 | x, y, z, intensity | [-14.232912063598633, -35.69328689575195, -3.3651788234710693] | [19.91459083557129, 12.093025207519531, 5.833889484405518] |
| GlobalMap_mid.pcd | 45550 | x, y, z | [-14.232912063598633, -35.69328689575195, -3.3651788234710693] | [19.91459083557129, 12.093025207519531, 5.833889484405518] |

地图解压后的原坐标及 10 帧真实扫描已导出到 `new_bag_assets/`，每份都有 NPY 和 binary PCD；转换没有移动、旋转或裁剪点。

## 初值与外参证据

记录中有 `map → camera_init → body` 动态 TF，可以作为匹配初值候选；不能当作当前定位结果、真值或地图 frame 身份证明。

`new_bag_audit.json` 中 initial_pose_candidates 提供每份扫描附近的 TF、时间差和组合矩阵。接近录包末尾的 body TF 滞后较大，必须检查时间差。

随附文件没有标定文档或 GlobalMap 地图 frame 配置，无法从文件名认定 loc/mid 对应哪个 TF 世界。必须分别进行实际匹配并比较重叠率/残差。

现有静态 TF 只包含 base_footprint 到 base_link/mid360_link/超声传感器；livox_frame 缺少 TF，hunter_base_link 和 base_link 也没有已验证刚性连接。暂时输出 body 定位，不伪造车体外参。

完整数值、时间间断、哈希、TF 和扫描字段见 `new_bag_audit.json`。
