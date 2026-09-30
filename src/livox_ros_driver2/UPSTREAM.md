# Livox 录包消息兼容包

此目录只生成 `livox_ros_driver2/msg/CustomMsg` 和 `CustomPoint`，用于读取已有录包，不包含硬件驱动，也不依赖 Livox SDK。

两个 `.msg` 文件和 `LICENSE.txt` 原样取自 Livox 官方仓库：

- 仓库：https://github.com/Livox-SDK/livox_ros_driver2
- 固定提交：`21445540f0d100dc86a7e6df312dd70bbdb4afdf`
- 消息：https://github.com/Livox-SDK/livox_ros_driver2/tree/21445540f0d100dc86a7e6df312dd70bbdb4afdf/msg
- 许可证：https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/LICENSE.txt

本地仅新增 CMake、包清单和此来源记录。同一工作区不能再放入另一个同名完整驱动包。将来接入实时雷达时，可用完整官方驱动替换此接口包。

点坐标单位为米，`reflectivity` 原样映射为浮点 `intensity`，范围 0–255。
`timebase` 为首点基准纳秒时间，逐点时间为 `timebase + offset_time`；源码计算见官方 `src/lddc.cpp`。
MID-360 的 tag 位组表示置信度，不沿用其他型号的回波过滤公式：
https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md#tag-information

本项目 `reject_invalid_tags: true` 时，保留三组置信度全部为 0（高）或 1（中）的点，拒绝任一组为 2（低）或 3（保留值）的点；忽略高两位保留位。这里的“无效”是明确的质量筛选策略，不意味着厂家规定所有低置信度点均无效。
