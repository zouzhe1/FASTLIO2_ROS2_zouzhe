# 重定位

为兼容旧接口保留的 `relocalize` 服务只用于确认请求是否被接收。新接入的程序应使用支持
取消操作的 `interface/action/Relocalize` Action，地址为
`/localizer/relocalize_action`。

`AUTO` 模式加载轻量级地点索引，检索相似度最高的 3 个候选位置，只加载候选位置对应的
同楼层瓦片，并使用有界 GICP 对每个候选进行几何验证。当绝对质量不达标，或第一、第二
候选之间的区分度不足时，系统会拒绝本次重定位结果。

`ROUGH_POSE` 模式跳过描述子检索，根据输入的六自由度粗略位姿，加载其周围同楼层的
3×3 瓦片邻域。两种模式都必须连续通过 3 帧运动一致性检查，才会恢复全局定位有效状态
并重新发布全局 TF。

```bash
ros2 action send_goal --feedback /localizer/relocalize_action \
  interface/action/Relocalize \
  "{mode: 0, map_path: '/maps/site', initial_pose: {header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}}"
```

系统会在候选切换期间以及等待连续帧确认时检查取消请求。发生取消、超时、瓦片损坏或
缺失、楼层错误、候选存在歧义或几何验证失败时，全局定位保持无效，但局部里程计继续
运行。

可通过 `localization_status` 监控定位状态、状态原因、当前候选、地图代次、质量指标、
处理延迟以及接受/拒绝次数。
