# 运行模式

系统支持以下三种互斥运行模式：

- `mapping`：启动 FASTLIO2 和 PGO，仅由 PGO 发布 `map -> odom`。
- `localization`：启动 FASTLIO2 和瓦片地图定位器，仅由定位器发布 `map -> odom`，不启动 PGO。
- `maintenance`：执行离线地图校验、瓦片构建或批量优化，不发布实时全局位姿。

使用以下命令启动建图模式：

```shell
ros2 launch pgo pgo_launch.py operational_profile:=mapping
```

使用以下命令启动定位模式：

```shell
ros2 launch localizer localizer_launch.py operational_profile:=localization
```

不受支持的模式组合会在 launch 或节点启动阶段直接失败。全局定位进入 `LOST` 状态后，
FASTLIO2 的局部里程计仍可继续工作，但 `global_pose_valid` 为 `false`，系统不会用新时间戳
继续发布已经失效的全局 TF。

地图构建分为两个阶段：

1. 调用 `/pgo/save_maps`，以事务方式创建不可变的关键帧流地图代次。
2. 运行以下命令构建瓦片地图：

```shell
ros2 run map_tools tile_builder_node MAP_ROOT SOURCE_GENERATION OUTPUT_GENERATION LEVEL_ID
```

构建完成后使用以下命令校验地图：

```shell
ros2 run map_tools map_validator GENERATION_DIR map
```

只有通过校验的瓦片地图代次才能用于定位。`current` 指针标识当前启用的地图代次，旧代次
继续保留，可在需要时回滚。
