# 2026-08-26：有界资源定位、回环与重定位增强

本分支在保留原 FASTLIO2 ROS2 功能和下方原始说明的基础上，增加了面向
ROS 2 Jazzy / Ubuntu 24.04 的 P0～P3 稳定性与性能改造：

- LiDAR/IMU 单位、时间戳、间断、静止初始化和进程内缓存均显式校验并设上限；
- mapping、localization、maintenance 三种运行模式互斥，每个在线模式只有一个
  `map -> odom` 发布者，LOST 时停止发布伪新鲜的全局 TF；
- 25 m、楼层隔离的瓦片地图，带版本、generation、校验和、事务保存和数量/字节双限 LRU；
- 有界异步 small_gicp、本地质量门、共享 Scan Context top-3、几何复核、歧义拒绝和
  连续三帧恢复确认；
- 回环因子使用分离的旋转/平移方差、鲁棒核、候选复核和拒绝黑名单；
- 高频 QoS、路径、点云和可视化历史均有边界，完整地图点云默认不发布。

使用入口见 [运行模式](docs/operational-profiles.md)、[重定位](docs/relocalization.md)、
[地图格式](docs/map-format.md)、[性能调优](docs/performance-tuning.md) 和
[验收条件](docs/acceptance-criteria.md)。仓库提供自动化/合成测试框架，但不附带
10 万平方米、30 分钟的现场验收包，因此现场精度与长期性能仍须使用者按验收文档实测。

---

# FASTLIO2 ROS2
## 主要工作
1. 重构[FASTLIO2](https://github.com/hku-mars/FAST_LIO) 适配ROS2
2. 添加回环节点，基于位置先验+ICP进行回环检测，基于GTSAM进行位姿图优化
3. 添加重定位节点，基于由粗到细两阶段ICP进行重定位
4. 增加一致性地图优化，基于[BLAM](https://github.com/hku-mars/BALM) (小场景地图) 和[HBA](https://github.com/hku-mars/HBA) (大场景地图)

## 环境依赖
1. Ubuntu 24.04
2. ROS2 Jazzy

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
--来自百度网盘超级会员v7的分享
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

