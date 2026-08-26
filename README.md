# FASTLIO2 ROS2 zouzhe版本

## zouzhe新增优化功能（2026-08-26）
1. 增强 LiDAR/IMU 输入检查，支持 IMU 单位配置、时间戳回退、数据间断、异常值检测、静止初始化判断和缓存上限。
2. 增加 mapping、localization、maintenance 三种运行模式和定位状态机，统一 `map -> odom` 发布权，定位失效时停止发布无效的全局 TF。
3. 增加 25 m 分层瓦片地图，支持负坐标、多楼层隔离、地图版本、校验和、事务保存、离线分块和有界 LRU 缓存。
4. 使用 small_gicp 实现有界异步局部配准，通过 RMSE、重叠率、内点率、退化程度、耗时和位姿跳变判断定位质量。
5. 增加 AUTO 和 ROUGH_POSE 重定位，使用共享 Scan Context 候选检索、同层瓦片几何验证、歧义拒绝和连续多帧一致性确认。
6. 优化回环检测和位姿图调度，增加关键帧预筛选、任务覆盖、旋转/平移独立噪声、鲁棒因子、回环复核和错误候选黑名单。
7. 限制 QoS 深度、路径、点云、可视化历史和内部任务数量，降低大地图及长时间运行时的 CPU、内存和通信开销。
8. 增加地图校验、故障注入、性能测试和 ROS 2 Jazzy 持续集成。

相关文档：[运行模式](docs/operational-profiles.md) · [重定位](docs/relocalization.md) · [地图格式](docs/map-format.md) · [性能调优](docs/performance-tuning.md) · [验收条件](docs/acceptance-criteria.md)

## 运行环境
1. Ubuntu 24.04
2. ROS2 Jazzy

---
# 下面是原说明信息
## fork前主要工作
1. 重构[FASTLIO2](https://github.com/hku-mars/FAST_LIO) 适配ROS2
2. 添加回环节点，基于位置先验+ICP进行回环检测，基于GTSAM进行位姿图优化
3. 添加重定位节点，基于由粗到细两阶段ICP进行重定位
4. 增加一致性地图优化，基于[BLAM](https://github.com/hku-mars/BALM) (小场景地图) 和[HBA](https://github.com/hku-mars/HBA) (大场景地图)


## 编译依赖
```text
pcl
Eigen
sophus
gtsam
livox_ros_driver2
```

## 详细说明
### 1.编译 LIVOX-SDK2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

### 2.编译 livox_ros_driver2
```shell
mkdir -r ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/jazzy/setup.sh
./build.sh jazzy
```

### 3.编译 Sophus
```shell
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```

**新的Sophus依赖fmt，可以在CMakeLists.txt中添加add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)去除，否则会报错**


## 实例数据集
```text
链接: https://pan.baidu.com/s/1rTTUlVwxi1ZNo7ZmcpEZ7A?pwd=t6yb 提取码: t6yb 
```

## 部分脚本

### 1.激光惯性里程计 
```shell
ros2 launch fastlio2 lio_launch.py
ros2 bag play your_bag_file
```

### 2.里程计加回环
#### 启动回环节点
```shell
ros2 launch pgo pgo_launch.py
ros2 bag play your_bag_file
```
#### 保存地图
```shell
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: 'your_save_dir', save_patches: true}"
```

### 3.里程计加重定位
#### 启动重定位节点
```shell
ros2 launch localizer localizer_launch.py
ros2 bag play your_bag_file // 可选
```
#### 设置重定位初始值
```shell
ros2 service call /localizer/relocalize interface/srv/Relocalize "{"pcd_path": "your_map.pcd", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
```
#### 检查重定位结果
```shell
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
```

### 4.一致性地图优化
#### 启动一致性地图优化节点
```shell
ros2 launch hba hba_launch.py
```
#### 调用优化服务
```shell
ros2 service call /hba/refine_map interface/srv/RefineMap "{"maps_path": "your maps directory"}"
```
**如果需要调用优化服务，保存地图时需要设置save_patches为true**

## 特别感谢
1. [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
2. [BLAM](https://github.com/hku-mars/BALM)
3. [HBA](https://github.com/hku-mars/HBA)
## 性能相关的问题
该代码主要使用timerCB作为频率触发主函数，由于ROS2中的timer、subscriber以及service的回调实际上运行在同一个线程上，在电脑性能不是好的时候，会出现调用阻塞的情况，建议使用线程并发的方式将耗时的回调独立出来(如timerCB)来提升性能

