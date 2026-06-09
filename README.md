# ORB-SLAM3 Dense ROS2 with OctoMap Converter

本项目提供了基于ORB-SLAM3的稠密点云生成和OctoMap转换功能，支持将SLAM生成的稠密点云转换为OctoMap八叉树地图。

## 功能特性

- **稠密点云生成**：使用ORB-SLAM3生成高精度稠密点云
- **实时OctoMap转换**：将稠密点云实时转换为OctoMap格式
- **PCD文件转换**：支持将已有的PCD文件转换为OctoMap
- **参数可配置**：可调整OctoMap的分辨率、最大范围等参数

## 系统要求

- ROS 2 Humble Hawksbill
- C++17或更高版本
- PCL 1.15.0
- OctoMap
- OpenCV 4.9.0
- Eigen 3.3.7

## 安装步骤

### 1. 安装依赖

```bash
# 安装ROS 2 Humble (如果尚未安装)
# 参考: https://docs.ros.org/en/humble/Installation.html

# 安装OctoMap和相关依赖
sudo apt install ros-humble-octomap ros-humble-octomap-msgs ros-humble-octomap-server

# 安装PCL相关依赖
sudo apt install libpcl-dev
```

### 2. 编译项目

```bash
# 进入工作空间
cd /home/ricky/WCR_ws

# 编译项目
colcon build --packages-select orbslam3_dense_ros2
```

## 运行指南

### 方法一：实时从ORB-SLAM3稠密点云生成OctoMap

1. **启动ORB-SLAM3稠密SLAM节点**：
   ```bash
   # 激活工作空间
   source /home/ricky/WCR_ws/install/setup.bash
   
   # 推荐：使用 launch 同时启动 RealSense、ORB-SLAM3 稠密节点和 OctoMap 转换
   ros2 launch orbslam3_dense_ros2 dense_mapping.launch.py
   ```

   或者只启动 ORB-SLAM3 稠密节点：
   ```bash
   ros2 run orbslam3_dense_ros2 orb_slam3_main \
     /home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt \
     /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml \
     true \
     /camera/camera/color/image_raw \
     /camera/camera/aligned_depth_to_color/image_raw \
     true
   ```

2. **启动OctoMap转换节点**：
   ```bash
   # 打开新终端，激活工作空间
   source /home/ricky/WCR_ws/install/setup.bash
   
   # 运行OctoMap转换节点
   ros2 run orbslam3_dense_ros2 octomap_converter
   ```

3. **可视化OctoMap**：
   ```bash
   # 打开新终端，激活工作空间
   source /home/ricky/WCR_ws/install/setup.bash
   
   # 启动RViz2
   ros2 run rviz2 rviz2
   ```
   在RViz2中：
   - 添加 `PointCloud2` 显示，话题设置为 `/orb_slam3/dense_points` 查看稠密点云
   - 添加 `OccupancyGrid` 显示，话题设置为 `/orb_slam3/octomap` 查看OctoMap

### 方法二：播放RGB-D数据集进行稠密建图

新节点 `rgbd_dataset_dense_node` 用于把 TUM RGB-D 或 `rgb/depth` 文件夹数据集按时间顺序送入 ORB-SLAM3。它和实时相机节点一样会初始化 `ORB_SLAM3::System(..., RGBD, use_viewer)`，所以 `use_viewer=true` 时 Pangolin 可以看到稀疏地图点、当前帧特征点和相机 frame；同时节点会发布同样格式的稠密点云：

| 话题名 | 类型 | 字段 |
|--------|------|------|
| /orb_slam3/dense_points | sensor_msgs/PointCloud2 | `x/y/z/rgb`，`frame_id=map` |

1. **TUM关联文件模式**：
   ```bash
   source /home/ricky/WCR_ws/install/setup.bash

   ros2 run orbslam3_dense_ros2 rgbd_dataset_dense_node \
     /home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt \
     /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/TUM1.yaml \
     /home/ricky/WCR_ws/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_room \
     /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/associations/fr1_room.txt \
     true \
     realtime
   ```

   等价的参数写法：
   ```bash
   ros2 run orbslam3_dense_ros2 rgbd_dataset_dense_node \
     --voc /home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt \
     --param /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/TUM1.yaml \
     --tum /home/ricky/WCR_ws/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_room:/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/associations/fr1_room.txt \
     --viewer true \
     --dense-mode realtime
   ```

2. **普通文件夹模式**：
   数据集根目录需要包含 `rgb/` 和 `depth/` 两个子目录，文件按文件名排序后逐帧配对。
   ```bash
   ros2 run orbslam3_dense_ros2 rgbd_dataset_dense_node \
     --voc /home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt \
     --param /home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/TUM1.yaml \
     --root_dir /path/to/rgbd_dataset \
     --viewer true \
     --dense-mode realtime
   ```

3. **同时查看稠密点云和OctoMap**：
   ```bash
   ros2 run orbslam3_dense_ros2 octomap_converter
   ```
   `octomap_converter` 默认订阅 `/orb_slam3/dense_points`。如果要订阅其他点云话题：
   ```bash
   ros2 run orbslam3_dense_ros2 octomap_converter --ros-args \
     -p input_pointcloud_topic:=/your/pointcloud_topic
   ```

数据集节点的稠密建图模式：

| 模式 | 说明 |
|------|------|
| realtime | 播放过程中发现新关键帧就送入 `PointCloudMappingRGBD`，适合边跑边在RViz看 `/orb_slam3/dense_points` |
| deferred | 播放结束后关闭SLAM，再按优化后的 Atlas 关键帧重新读取RGB-D图像并融合，逻辑更接近 `rgbd_tum_dense.cc` |
| both | 播放时实时发布一次，结束后再用 Atlas 关键帧重建一次最终稠密地图 |

常用参数：

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| dense_build_mode | realtime | `realtime`、`deferred` 或 `both` |
| dense_resolution | 0.01 | 稠密点云体素滤波分辨率（米） |
| dense_mean_k | 50.0 | PCL统计离群点滤波 MeanK |
| dense_std_thresh | 2.0 | PCL统计离群点滤波阈值 |
| dense_unit | 5000.0 | 深度图单位换算；TUM 16UC1 深度通常使用 5000 |
| dataset_fps | 30.0 | 文件夹模式和异常时间戳时的播放帧率 |
| playback_speed | 1.0 | 数据集播放速度；设为 `0.0` 表示不等待，尽快处理 |
| max_frames | -1 | 最多处理多少帧；`-1` 表示不限制 |
| keep_alive_after_finish | true | 数据集播完后保持节点和Pangolin/RViz查看状态 |
| dense_pointcloud_topic | /orb_slam3/dense_points | 稠密点云发布话题 |
| dense_publish_period_ms | 500 | 稠密点云发布周期（毫秒） |
| deferred_timestamp_tolerance | 1e-4 | deferred模式下关键帧时间戳匹配原始图像的容差 |

### 方法三：从PCD文件生成OctoMap

1. **准备PCD文件**：
   确保你有一个PCD格式的点云文件，例如 `/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/PointCloudMapping_RGBD.pcd`

2. **运行PCD到OctoMap转换节点**：
   ```bash
   # 激活工作空间
   source /home/ricky/WCR_ws/install/setup.bash
   
   # 运行PCD到OctoMap转换节点
   ros2 run orbslam3_dense_ros2 pcd_to_octomap_node --ros-args -p pcd_file:=/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/PointCloudMapping_RGBD.pcd
   ```
   ```bash
   # local run rviz2
   rviz2
   ```

3. **可视化OctoMap**：
   同上，在RViz2中添加 `OccupancyGrid` 显示，话题设置为 `/orb_slam3/octomap`

## 参数配置

### ORB-SLAM3稠密节点参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| dense_build_mode | deferred | 稠密建图模式：`deferred` 退出时按优化后的 Atlas 关键帧生成 PCD；`realtime` 实时插入关键帧并发布点云；`both` 两者都做 |
| dense_resolution | 0.008 | 稠密点云体素滤波分辨率（米） |
| dense_mean_k | 15.0 | PCL 统计离群点滤波 MeanK |
| dense_std_thresh | 3.0 | PCL 统计离群点滤波阈值 |
| dense_publish_period_ms | 300 | 实时点云发布周期（毫秒） |
| dense_pointcloud_topic | /orb_slam3/dense_points | 稠密点云发布话题 |

### RealSense启动参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| initial_reset | true | 启动相机节点前重置 RealSense，适合相机处于半挂起状态时恢复 |
| emitter_enabled | 1 | 打开深度模组红外投射器，RealSense ROS 中该参数是整数枚举 |
| color_profile | 848x480x30 | 彩色图像分辨率和帧率 |
| depth_profile | 848x480x30 | 深度图像分辨率和帧率 |

### OctoMap转换节点参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| input_pointcloud_topic | /orb_slam3/dense_points | 输入稠密点云话题 |
| resolution | 0.05 | OctoMap分辨率（米） |
| max_range | 5.0 | 最大点云范围（米） |
| color | true | 是否使用彩色OctoMap |

### PCD到OctoMap节点参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| pcd_file | - | PCD文件路径（必须指定） |
| resolution | 0.05 | OctoMap分辨率（米） |
| max_range | 5.0 | 最大点云范围（米） |
| color | true | 是否使用彩色OctoMap |

## 话题信息

| 话题名 | 类型 | 说明 |
|--------|------|------|
| /orb_slam3/dense_points | sensor_msgs/PointCloud2 | ORB-SLAM3生成的稠密点云 |
| /orb_slam3/octomap | octomap_msgs/Octomap | 转换后的OctoMap |

## 故障排除

1. **编译错误**：
   - 确保所有依赖已正确安装
   - 检查CMakeLists.txt中的路径配置

2. **运行时错误**：
   - 检查ROS 2环境变量是否正确设置
   - 确保传感器驱动已正确安装（如RealSense相机驱动）
   - 检查PCD文件路径是否正确

3. **OctoMap显示问题**：
   - 在RViz2中确保选择了正确的参考系（通常为`map`或`base_link`）
   - 调整OctoMap的分辨率和最大范围参数

## 示例

### 实时SLAM与OctoMap生成

1. 连接RealSense D435i相机
2. 启动ORB-SLAM3稠密SLAM节点
3. 启动OctoMap转换节点
4. 在RViz2中同时查看稠密点云和OctoMap

### PCD文件转换

1. 准备一个PCD格式的点云文件
2. 运行PCD到OctoMap转换节点，指定PCD文件路径
3. 在RViz2中查看生成的OctoMap

## 注意事项

- OctoMap的分辨率设置会影响地图精度和内存使用，根据实际需求调整
- 最大范围参数应根据环境大小设置，避免处理过多的点云数据
- 对于大型环境，建议使用较低的分辨率以减少计算负担

## 许可证

本项目基于ORB-SLAM3，遵循其原始许可证GPLv3。

## 联系方式

- 维护者：ricky
- 邮箱：1954459554@qq.com

---

如有任何问题，请联系项目维护者。
