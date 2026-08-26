# FAST_LIO_LOCALIZATION 重定位能力与 ROS 2 Jazzy 对比研究

## 1. 结论摘要

对照项目 `HViktorTsoi/FAST_LIO_LOCALIZATION` 的名称和 README 使用了 “global localization”，但从源码看，它更准确的能力分类是：

> **人工或外部系统提供粗略 6DoF 初值后的初始配准，加上以前一次 `map -> odom` 为初值的低频局部 scan-to-map 跟踪。**

它不是无初值全地图检索，也没有机器人被搬动后的自动全局恢复。其核心是两级 point-to-point ICP；初始定位失败时重新等待外部 `/initialpose`，运行期匹配失败时保留旧修正并继续周期性尝试。

因此建议如下：

1. **不直接移植该仓库。** 它是 ROS 1、catkin、Python 2、`rospy`、ROS 1 `tf` 工程，直接用于 ROS 2 Jazzy 的迁移工作量接近重写，而且算法能力不强于本仓库已有 localizer。
2. **不需要额外叠加它来获得重定位功能。** 本仓库 P3 规划已经包含 `UNINITIALIZED/LOST -> RELOCALIZING -> RECOVERING -> TRACKING`、地点候选检索、按需全局配准和 GICP 精配准，目标能力明显更完整。
3. **可复用的是架构思想，不是实现代码：** FASTLIO2 保持高频连续里程计，低频地图匹配只更新 `map -> odom`；点云只保留最新一帧；正常状态不持续运行重型全局算法。
4. **现有规划需要补一个关键实施细节：** 回环数据库与预建地图重定位数据库不是同一个问题。P3 已明确为在线关键帧建立 Scan Context 数据库，但还必须规定如何在建图/切图阶段生成并保存“地图关键帧描述子 + 对应地图位姿 + tile/submap 引用”。没有这个地图侧地点索引，仅靠一张合并 PCD 不能完成规划中声称的快速全局候选检索。
5. 所有实施、依赖和验收说明应以 **ROS 2 Jazzy、Ubuntu 24.04、C++17/Python 3** 为基线，性能约束写成通用目标，不绑定某一款硬件。

本研究锁定的对照项目版本为提交 [`2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2`](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/tree/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2)。

## 2. 对照项目到底实现了什么

### 2.1 算法链

对照项目的完整数据流如下：

1. 启动文件用 ROS 1 `pcl_ros/pcd_to_pointcloud` 将一张 PCD 作为 `/map` 发布；定位节点一次性接收整张地图，转成 Open3D `PointCloud`，再按 0.4 m 体素降采样。[启动文件](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/launch/localization_avia.launch#L19-L28)、[地图初始化](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/scripts/global_localization.py#L169-L175)
2. FAST-LIO 输出已经变换到 odom/`camera_init` 坐标系的 `/cloud_registered` 和 `/Odometry`；定位脚本保存最新扫描和里程计。[点云与里程计订阅](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/scripts/global_localization.py#L178-L198)
3. 初始化阶段等待 `/initialpose`，把完整 6DoF 位姿作为 ICP 初值。README 给出的接口是 `x y z yaw pitch roll`，也明确说明初值由 RViz 粗估或其他传感器/算法提供。[README 初值说明](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/README.md#L21-L21)、[README 使用方法](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/README.md#L123-L132)
4. 每次定位先用当前位姿估计把**整张降采样地图**变换到当前雷达视角，再按距离和 FOV 筛出子图。[FOV 裁剪](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/scripts/global_localization.py#L79-L113)
5. 对当前扫描与上述子图执行两级 Open3D point-to-point ICP：`scale=5` 粗配准和 `scale=1` 精配准，每级最多 20 次迭代；最终 `fitness > 0.95` 才更新 `map -> odom`。[ICP 实现](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/scripts/global_localization.py#L42-L50)、[粗精配准及接受条件](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/scripts/global_localization.py#L116-L157)
6. 初始化成功后，默认以 0.5 Hz 使用上一次 `map -> odom` 作为下一次 ICP 初值；独立融合节点以 50 Hz 将 `map -> odom` 与 FAST-LIO `odom -> base` 相乘并输出全局定位结果。[周期定位](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/scripts/global_localization.py#L201-L219)、[高频融合输出](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/scripts/transform_fusion.py#L27-L60)

### 2.2 能力分类

| 能力 | 是否具备 | 源码判断 |
|---|---:|---|
| 有粗初值的初始定位 | 是 | `/initialpose` 直接作为 ICP 初值 |
| 正常运行时低频地图跟踪 | 是 | 上一次 `map -> odom` 作为下一次初值，默认 0.5 Hz |
| 无初值全地图位置搜索 | 否 | 没有地点描述子、候选数据库、多位姿假设或全局特征匹配 |
| 机器人被搬动后的自动重定位 | 否 | 没有 LOST 状态和全局候选搜索 |
| 回环检测与位姿图优化 | 否 | 该仓库没有回环候选、回环因子或 PGO |
| 地图匹配抑制里程计累积漂移 | 有条件具备 | 仅当初值仍在 ICP 收敛域且匹配持续正确时成立 |

“global”在这里表示结果位于全局地图坐标系，而不是算法能够从任意未知位置进行全局搜索。更准确的命名应是“有先验初值的地图初始化和局部地图跟踪”。

### 2.3 初值依赖与失锁恢复

初始化循环在一次 ICP 失败后会再次等待新的 `/initialpose`，不会围绕全地图自行寻找其他候选。[初始化循环](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/scripts/global_localization.py#L242-L255)

运行期匹配失败时只打印 `Not match` 并返回 `False`，原来的 `T_map_to_odom` 不变；周期线程之后仍以这个旧修正为初值继续 ICP。[失败处理](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/scripts/global_localization.py#L139-L157)、[周期线程](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/scripts/global_localization.py#L201-L207)

这意味着：

- 短时失败后，如果真实误差仍位于 ICP 捕获范围，后续帧可能重新收敛；
- 漂移已经过大、航向错误较大或机器人被搬走时，没有可靠的恢复通道；
- 没有 `TRACKING/DEGRADED/LOST` 状态，没有连续失败计数、超时、重叠率、退化性、修正跳变量或多帧一致性判断；
- 单一 `fitness` 门槛无法充分拒绝重复走廊、相似楼层等假匹配。

因此不能把它描述为“失锁后自动全局重定位”。

### 2.4 地图表示与性能特征

其节省计算量的办法主要是：地图 0.4 m 体素、扫描 0.1 m 体素、定位仅 0.5 Hz、ROS 订阅队列深度为 1。README 将其概括为低频地图定位与高频 FAST-LIO 里程计融合。[README 性能说明](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/README.md#L11-L17)、[参数与队列](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/scripts/global_localization.py#L210-L240)

但其大地图扩展性较弱：

- 整张地图以一个 Open3D 点云常驻内存，没有分块、空间索引或缓存淘汰；
- 每次定位都先把全地图转成 NumPy 数组、补齐齐次坐标，并对全部地图点做矩阵变换，然后才进行 FOV 筛选；这部分时间和内存带宽随全图点数线性增长；
- 粗、精两级 ICP 又分别对扫描和裁剪地图执行体素降采样；
- 源码明确留下了线程安全 TODO，定位线程、订阅回调与融合线程共享状态但没有完整同步。[全图裁剪实现](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/scripts/global_localization.py#L79-L127)、[线程安全 TODO](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/scripts/transform_fusion.py#L27-L50)

对于至少 100,000 m² 的地图，25 m 分块、局部活动窗口和复用预处理搜索结构比这种“每周期遍历全图再裁剪”的实现更合理。

## 3. 与本仓库现有 localizer 的比较

本仓库已经实现 ROS 2 C++ localizer，其算法本质与对照项目相近：

- 服务请求给出 PCD 路径和完整 `x/y/z/yaw/roll/pitch` 初值：[localizer/src/localizer_node.cpp:206](../../localizer/src/localizer_node.cpp#L206)
- PCD 加载后生成粗、精两个降采样全图目标：[localizer/src/localizers/icp_localizer.cpp:10](../../localizer/src/localizers/icp_localizer.cpp#L10)
- 用 PCL ICP 执行粗配准再精配准，接受条件只有收敛和 fitness 阈值：[localizer/src/localizers/icp_localizer.cpp:68](../../localizer/src/localizers/icp_localizer.cpp#L68)
- 成功后更新 `map -> odom` 修正，FASTLIO2 局部里程计保持连续：[localizer/src/localizer_node.cpp:148](../../localizer/src/localizer_node.cpp#L148)

两者能力对比如下：

| 项目 | 对照项目 | 本仓库现有 localizer | P0–P3 规划 |
|---|---|---|---|
| 中间件 | ROS 1 | ROS 2 | ROS 2 Jazzy |
| 初始定位 | 人工粗初值 + 两级 Open3D ICP | 服务粗初值 + 两级 PCL ICP | 人工初值兼容 + 自动候选恢复 |
| 正常跟踪 | 0.5 Hz ICP | 默认 1 Hz ICP | 低频局部 GICP/VGICP |
| 地图 | 单个全图 Open3D 点云，每次全图裁剪 | 单个全图 PCL 点云，直接全图 ICP | 25 m tile、3×3 活动窗口、有界缓存 |
| 候选搜索 | 无 | 无 | Scan Context top-k |
| 全局配准 | 无 | 无 | 初始化/LOST 时按需运行 |
| 几何精配准 | point-to-point ICP | point-to-point ICP | GICP/VGICP |
| 质量判定 | fitness | converged + fitness | 内点、重叠、RMSE、Hessian、跳变量、运动一致性 |
| 失锁状态 | 无 | 无 | `DEGRADED/LOST/RELOCALIZING/RECOVERING` |
| 恢复确认 | 无 | 无 | 连续多帧确认后才恢复可信状态 |
| 回环/PGO | 无 | 另有基础 PGO | Scan Context + 几何验证 + 鲁棒因子 |

结论是：直接加入对照项目会形成第二套重复且更弱的 ICP localizer，同时带来 ROS 1/Python 2/Open3D 旧接口依赖。它不会补上当前真正缺少的“无可靠初值时找候选并安全恢复”能力。

### 3.1 值得吸收的设计点

以下思想值得保留，但本仓库或现有规划基本已经覆盖：

- FASTLIO2 高频输出连续 `odom`，低频地图匹配只修正 `map -> odom`；
- 地图匹配失败不反向跳变 FASTLIO2 滤波器状态；
- 点云订阅采用小队列/只保留最新扫描；
- 正常跟踪降低地图匹配频率，避免重型算法常驻；
- 可以保留 ROS 2 `/initialpose` 适配器，让 RViz 人工粗初值作为调试和兜底入口。

### 3.2 不应复用的实现

- Python 2、`rospy`、ROS 1 `tf`、catkin 和 ROS 1 XML launch；
- 每周期变换整张地图后再做 FOV 裁剪；
- 只有 point-to-point ICP 与单一 fitness 阈值；
- 用共享全局变量和裸线程处理定位与融合；
- 把周期 ICP 称为全局重定位；
- 没有失锁状态却持续输出看似有效的全局位姿。

## 4. 现有规划是否包含重定位

**包含。** 设计文档已经给出以下链路：

- 状态机包含 `UNINITIALIZED -> RELOCALIZING -> RECOVERING -> TRACKING`，以及正常跟踪失败后的 `DEGRADED -> LOST`：[设计文档状态机](../plans/2026-08-26-fastlio2-bounded-resource-p0-p3-design.md#6-localization-health-state-machine)
- `LOST` 或初始定位时，先用 Scan Context 取得候选地图关键帧及航向，再加载候选关联的同层瓦片，依次执行粗、精两级 GICP；正常 `TRACKING` 不运行这条全局恢复链路：[设计文档全局恢复](../plans/2026-08-26-fastlio2-bounded-resource-p0-p3-design.md#73-global-recovery-and-loop-closure)
- 实施任务 11 已规划自动/人工触发、可取消的有界恢复 worker、最佳/次佳歧义门限、候选几何验证和三帧运动一致性确认；只有验收数据证明召回不足时，才评估更重的全局配准后端：[实施计划任务 11](../plans/2026-08-26-fastlio2-bounded-resource-p0-p3-implementation.md#task-11-add-on-demand-global-recovery-and-robust-loop-verification-p3)

该目标能力比对照项目更接近真正的自动重定位。

### 4.1 本次规划补实的地图侧地点索引

初版实施任务 10 主要描述 PGO 对“已接纳在线关键帧”计算 Scan Context 并查询历史关键帧，这足以服务在线回环；优化后的任务 10 已把地图侧索引闭环补入计划：[实施计划任务 10](../plans/2026-08-26-fastlio2-bounded-resource-p0-p3-implementation.md#task-10-add-shared-online-and-prebuilt-map-place-recognition-p3)

机器人启动时或在预建地图中被搬动后，localizer 需要查询的是**预建地图的地点数据库**。本次优化已把以下内容加入实施计划：

1. 建图阶段保存带地图位姿的关键帧，而不只是合并后的 PCD；
2. 离线切图工具为这些地图关键帧生成 Scan Context 描述子；
3. 版本化保存 `descriptor -> map pose -> tile/submap IDs` 索引，并记录与地图 manifest 的一致性哈希；
4. 重定位 worker 查询地图索引 top-k，按候选位姿加载相邻 tiles，再执行全局/鲁棒配准和 GICP；
5. 地图只剩一张合并 PCD、没有原始关键帧时，定义明确的索引重建策略或降级为人工初值，不应假定合并 PCD 天然具备正确的扫描视角描述子；
6. 地图更新时使旧描述子索引失效，避免地图版本与地点索引错配。

这些工作安排在 P1 离线地图构建和 P3 全局恢复的交界处，并增加“任意地图区域启动”“机器人被搬动”“重复走廊/相似楼层拒绝”“地图索引版本不匹配”测试，使地图侧候选数据链形成闭环。

## 5. 是否值得直接移植

### 决策：不直接移植

原因不只是 ROS 版本不同：

1. **算法能力重复且偏弱。** 本仓库已有两级 ICP localizer；对照项目没有增加地点识别、全局特征匹配、失锁判断或鲁棒恢复。
2. **大地图性能路线不合适。** 对照项目每次遍历和变换全地图，本仓库规划的 tile 活动窗口才是可扩展路径。
3. **迁移不是简单编译适配。** 对照项目使用 `catkin`、`roscpp/rospy`、ROS 1 `tf`、`pcl_ros` 可执行程序、Python 2.7 和 Open3D 0.9。[CMake ROS 1 依赖](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/CMakeLists.txt#L42-L88)、[package.xml ROS 1 依赖](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/package.xml#L19-L40)、[README Python 2/Open3D 依赖](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/README.md#L31-L65)
4. **可靠性基础不足。** 源码自身标有时间戳、线程安全、状态融合等 TODO；README 也将时间戳和与 FAST-LIO 状态融合列为未完成项。[README TODO](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION/blob/2bc274ed0b36d14a5c34ada5e1473d52aa1db0d2/README.md#L145-L149)

推荐做法是继续增强本仓库 C++ localizer，仅将对照项目视为“低频 map correction + 高频 odometry fusion”的历史参考。

## 6. ROS 2 Jazzy 实施注意事项

### 6.1 平台与语言基线

ROS 2 Jazzy 的 Tier 1 Linux 平台是 Ubuntu 24.04 的 amd64 和 arm64；官方发布基线包括 C++17、Python 3.12.3 和 PCL 1.14.0。[ROS 2 Jazzy 发布与依赖基线](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)、[REP 2000](https://www.ros.org/reps/rep-2000.html)

因此：

- 不要继承对照项目的 Python 2.7、Open3D 0.9、PCL 1.8 固定依赖；
- 在线 registration 主路径继续使用 C++17；
- 第三方 GICP、Scan Context 和全局配准依赖必须锁定支持 C++17、arm64 与 PCL 1.14 的提交/版本，并在 Jazzy CI 中编译验证；
- 实施文档和 README 已统一为 Jazzy/Ubuntu 24.04：[实施计划技术栈与环境](../plans/2026-08-26-fastlio2-bounded-resource-p0-p3-implementation.md#execution-preconditions)。

### 6.2 构建与依赖声明

ROS 2 C++ 包使用 `ament_cmake`，依赖应同时正确写入 `package.xml`、`find_package(...)` 和 `ament_target_dependencies(...)`；官方 Jazzy 包开发文档也采用这一结构。[ROS 2 Jazzy 包开发文档](https://docs.ros.org/en/jazzy/How-To-Guides/Developing-a-ROS-2-Package.html)

本仓库 localizer 已经是 `ament_cmake`，但当前源码使用 `rclcpp`，`package.xml` 未声明 `<depend>rclcpp</depend>`，CMake 也未显式调用 `find_package(rclcpp REQUIRED)`；实施 P0 时应一并修正：[localizer/package.xml](../../localizer/package.xml)、[localizer/CMakeLists.txt](../../localizer/CMakeLists.txt#L15-L44)。

### 6.3 ROS 1 API 替换关系

若吸收任何对照项目思想，应落在现有 ROS 2 节点内，而不是兼容旧脚本：

| ROS 1 对照项目 | ROS 2 Jazzy 实现 |
|---|---|
| `rospy` / `roscpp` | `rclcpp`，非性能关键辅助工具才考虑 `rclpy` |
| `tf.TransformBroadcaster` | `tf2_ros::TransformBroadcaster` |
| `/initialpose` 的阻塞 `wait_for_message` | 普通订阅或显式异步重定位服务/action |
| catkin | `ament_cmake` + colcon |
| ROS 1 XML launch | ROS 2 Python/XML launch，使用 `launch_ros` |
| Python 2 `ros_numpy` | 在线点云主路径用 C++ `pcl_conversions`，避免大点云 Python 往返复制 |

Jazzy 的 `tf2_ros::TransformBroadcaster` 明确将 `child_frame_id` 到 `header.frame_id` 的变换发布出去，实施时应统一验证 `map -> odom` 的方向，避免照搬旧脚本中容易混淆的变量命名。[Jazzy TransformBroadcaster API](https://docs.ros.org/en/jazzy/p/tf2_ros/generated/classtf2__ros_1_1TransformBroadcaster.html)

### 6.4 QoS 与消息同步

- 点云应采用 `rclcpp::SensorDataQoS` 或与雷达发布端兼容的显式 QoS。Jazzy 的 `SensorDataQoS` 默认为 keep-last、深度 5、best-effort、volatile。[Jazzy SensorDataQoS](https://docs.ros.org/en/ros2_packages/jazzy/api/rclcpp/generated/classrclcpp_1_1SensorDataQoS.html)
- `message_filters::ApproximateTime` 的多个订阅端必须与对应发布端使用兼容、且相互一致的 QoS，否则同步器无法正确匹配消息。[Jazzy ApproximateTime 教程](https://docs.ros.org/en/ros2_packages/jazzy/api/message_filters/doc/Tutorials/Approximate-Synchronizer-Cpp.html)
- 地图元数据、地图状态或低频静态地图可考虑 transient-local；高频点云不要使用大 reliable 队列。
- latest-only 语义应由容量为 1 的工作槽和 generation ID 保证，不能只依赖 DDS 队列深度。

### 6.5 并发与生命周期

- 当前 `rclcpp::spin(node)` 是单线程 executor；耗时 ICP 若留在 timer/service 回调内，会阻塞该节点其他消息处理：[localizer/src/localizer_node.cpp:288](../../localizer/src/localizer_node.cpp#L288)。
- 推荐让 registration backend 归属独立 worker，ROS 回调只提交最新输入并读取结果；这比仅切换到 `MultiThreadedExecutor` 更容易限制并发和内存。
- 如果使用 MultiThreadedExecutor，必须显式设计 callback groups；ROS 2 默认 callback group 是 mutually exclusive，不分组时即使多线程 executor 也可能仍按单线程方式执行。[ROS 2 Callback Groups 指南](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html)
- 地图 reload、tile cache、registration result 都应带 map generation；旧地图任务完成后不得覆盖新地图状态。

### 6.6 接口语义

重定位请求接口应区分：

- 请求已接受/地图已加载；
- 正在检索候选；
- 找到候选但仍在恢复确认；
- 全局位姿已可信；
- 失败或超时。

当前 `relocalize` 服务在地图加载并保存初值后就返回 `"relocalize success"`，此时 ICP 尚未完成，Jazzy 重构时应修正这个语义：[localizer/src/localizer_node.cpp:226](../../localizer/src/localizer_node.cpp#L226)。长时间全局检索更适合 action 或“启动服务 + 状态 topic/service”，不能让一个同步 service 回调阻塞数秒。

## 7. 建议落地决策

1. 不添加对照项目代码或其 Python/Open3D 节点。
2. 保留本仓库现有 `/relocalize` 兼容入口，并可新增 ROS 2 `/initialpose` 适配器作为人工兜底。
3. 正常跟踪采用 25 m tiles、3×3 活动窗口和低频 GICP；这承担“持续压制漂移”，不是全局搜索。
4. P3 增加独立的**地图地点索引构建产物**，让 Scan Context top-k 能真正服务启动定位和 LOST 恢复，而不仅服务在线回环。
5. 全局配准只在 `UNINITIALIZED/LOST` 按需运行，候选接受后再经局部 GICP 与连续多帧质量门控。
6. 所有文档、CI、依赖和测试基线统一到 ROS 2 Jazzy；性能指标使用 CPU、RSS、P95/P99 延迟、队列上限和恢复时间等通用指标描述。

最终建议可以概括为：

> **我们的规划有重定位，而且目标能力高于 FAST_LIO_LOCALIZATION；无需直接移植。应补齐地图侧 Scan Context/地点索引，并继续按“候选检索 -> 按需全局配准 -> GICP 精配准 -> 多帧确认”的 ROS 2 Jazzy 原生链路实施。**
