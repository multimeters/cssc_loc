# 首帧地图匹配初值验证

使用首帧真实 body 点云，比较两张地图和三种初始位姿；结果用于给 NDT 提供固定初值。

记录中的 TF 仅用于生成初值候选。本次结果为单帧多尺度 ICP 几何验证，不能作为独立真值、定位精度或完整 NDT 运行结果。

| 地图 | 初值候选 | ICP 前 0.2m 重叠率 | ICP 后 0.2m 重叠率 | ICP 后最近邻中值(m) | 0.2m 内 RMSE(m) |
|---|---|---:|---:|---:|---:|
| GlobalMap_loc.pcd | identity | 26.59% | 46.10% | 0.262887 | 0.093166 |
| GlobalMap_loc.pcd | recorded_camera_init_T_body | 28.25% | 53.26% | 0.168697 | 0.090482 |
| GlobalMap_loc.pcd | recorded_map_T_body | 95.61% | 95.28% | 0.061179 | 0.066618 |
| GlobalMap_mid.pcd | identity | 23.84% | 42.48% | 0.292256 | 0.120941 |
| GlobalMap_mid.pcd | recorded_camera_init_T_body | 25.37% | 51.33% | 0.188508 | 0.111953 |
| GlobalMap_mid.pcd | recorded_map_T_body | 94.68% | 94.48% | 0.094836 | 0.097274 |

建议地图：`GlobalMap_loc.pcd`。

固定初值 xyz + roll/pitch/yaw（米、弧度）：`[3.5693973688574903, -2.3873112776921737, 0.4717393716073427, -0.01740061514856, 0.3577300618484043, 0.11151351010888175]`。

后续定位只读取真实扫描和固定初值，不能再次消费旧定位 TF 来生成输出。

重叠率的分母是首帧 0.1m 体素降采样后 4510 个点；距离是每个扫描点到地图最近邻的距离。RMSE 仅计算阈值内点，不是定位位姿误差。

复现脚本：`scripts/verify_initial_registration.py`。全部变换矩阵和阶段参数见同名 JSON。
