<p align="center">
  <img src="docs/assets/demo_dense_mapping_09s_12s.gif" width="48%" alt="稠密建图预览" />
  <img src="docs/assets/demo_mapping_result_19s_24s.gif" width="48%" alt="建图结果预览" />
</p>

<h1 align="center">ORB-SLAM3 Dense ROS2</h1>

<p align="center">
  <a href="README_EN.md">English</a> ·
  <a href="README_CH.md">中文</a>
</p>

这个包是同级 [`dense_orbslam3`](../../dense_orbslam3/README.md) 仓库的 ROS 2 封装。`dense_orbslam3` 负责改过的 ORB-SLAM3 核心、稠密建图代码、词袋、相机配置和点云工具；本仓库负责把这些能力接入 ROS 2，并发布稠密点云或 OctoMap。核心仓库远端链接：[Pplll-pppl/Dense_OrbSlam3](https://github.com/Pplll-pppl/Dense_OrbSlam3)。

## 仓库位置

推荐目录：

```text
<ws>/
  dense_orbslam3/              # ORB-SLAM3 核心、libORB_SLAM3.so、配置、数据集
  src/orbslam3_dense_ros2/     # 本 ROS 2 包
```

把示例里的路径都换成自己的 `<ws>`。当前 `CMakeLists.txt` 默认 `dense_orbslam3` 在 `<ws>/dense_orbslam3`；如果位置不同，需要改 `DENSE_ORBSLAM3_ROOT`、`OpenCV_DIR` 和 `PCL_DIR`。

## `dense_orbslam3` 里的关键文件

| 路径 | 作用 |
|------|------|
| `<ws>/dense_orbslam3/lib/libORB_SLAM3.so` | 本 ROS 2 包链接的 ORB-SLAM3 库 |
| `<ws>/dense_orbslam3/Vocabulary/ORBvoc.txt` | 运行时必须提供的 ORB 词袋 |
| `<ws>/dense_orbslam3/Examples/RGB-D/*.yaml` | 相机参数，例如 `TUM1.yaml` 或 `RealSense_D435i.yaml` |
| `<ws>/dense_orbslam3/Examples/RGB-D/associations/*.txt` | TUM RGB-D 时间戳关联文件 |
| `<ws>/dense_orbslam3/dataset/TUM-RGBD/<sequence>` | 可选的 TUM 数据集目录 |
| `<ws>/dense_orbslam3/Examples/RGB-D/PointCloudMapping_RGBD.pcd` | 示例稠密点云，可用于 PCD 转 OctoMap |
| `<ws>/dense_orbslam3/3rdParty/...` | 本包使用的 OpenCV、PCL、Pangolin 构建产物 |

数据集也可以放在其他地方，运行时传绝对路径即可。

## 编译

先编译 `dense_orbslam3`：

```bash
cd <ws>/dense_orbslam3
chmod +x build.sh
./build.sh
```

再编译本 ROS 2 包：

```bash
cd <ws>
colcon build --packages-select orbslam3_dense_ros2
source install/setup.bash
```

## 节点

| 节点 | 作用 |
|------|------|
| `orb_slam3_main` | 接实时 RGB-D 相机，运行 ORB-SLAM3 稠密建图 |
| `rgbd_dataset_dense_node` | 播放 TUM RGB-D 或 `rgb/` + `depth/` 文件夹数据集 |
| `octomap_converter` | 把 `/orb_slam3/dense_points` 转成 OctoMap |
| `pcd_to_octomap_node` | 把保存好的 `.pcd` 文件转成 OctoMap |

主要话题：

| 话题 | 类型 | 含义 |
|------|------|------|
| `/orb_slam3/dense_points` | `sensor_msgs/PointCloud2` | ORB-SLAM3 输出的 RGB 稠密点云 |
| `/orb_slam3/octomap` 或 `/octomap` | `octomap_msgs/Octomap` | 转换后的 OctoMap |

## RealSense RGB-D 运行

```bash
source <ws>/install/setup.bash

ros2 run orbslam3_dense_ros2 orb_slam3_main \
  <ws>/dense_orbslam3/Vocabulary/ORBvoc.txt \
  <ws>/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml \
  true \
  /camera/camera/color/image_raw \
  /camera/camera/aligned_depth_to_color/image_raw \
  true
```

另开终端：

```bash
source <ws>/install/setup.bash
ros2 run orbslam3_dense_ros2 octomap_converter
```

在 RViz2 中查看 `/orb_slam3/dense_points` 和 OctoMap 话题。

## TUM RGB-D 数据集运行

```bash
source <ws>/install/setup.bash

ros2 run orbslam3_dense_ros2 rgbd_dataset_dense_node \
  --voc <ws>/dense_orbslam3/Vocabulary/ORBvoc.txt \
  --param <ws>/dense_orbslam3/Examples/RGB-D/TUM1.yaml \
  --tum <ws>/dense_orbslam3/dataset/TUM-RGBD/rgbd_dataset_freiburg1_room:<ws>/dense_orbslam3/Examples/RGB-D/associations/fr1_room.txt \
  --viewer true \
  --dense-mode realtime
```

普通文件夹模式要求根目录包含 `rgb/` 和 `depth/`：

```bash
ros2 run orbslam3_dense_ros2 rgbd_dataset_dense_node \
  --voc <ws>/dense_orbslam3/Vocabulary/ORBvoc.txt \
  --param <ws>/dense_orbslam3/Examples/RGB-D/TUM1.yaml \
  --root_dir /absolute/path/to/rgbd_dataset \
  --viewer true \
  --dense-mode realtime
```

稠密模式：

| 模式 | 含义 |
|------|------|
| `realtime` | 新关键帧产生时实时发布稠密点云 |
| `deferred` | 播放结束后按优化后的 Atlas 关键帧重建 |
| `both` | 实时发布一次，结束后再重建最终点云 |

## PCD 转 OctoMap

```bash
source <ws>/install/setup.bash

ros2 run orbslam3_dense_ros2 pcd_to_octomap_node --ros-args \
  -p pcd_file:=<ws>/dense_orbslam3/Examples/RGB-D/PointCloudMapping_RGBD.pcd \
  -p topic_name:=/octomap \
  -p resolution:=0.05
```

如果点云在别的位置，直接换成自己的 `.pcd` 绝对路径。

## 常用参数

| 参数 | 默认值 | 用途 |
|------|--------|------|
| `dense_build_mode` | `realtime` 或 `deferred` | 稠密建图模式 |
| `dense_resolution` | `0.01` / `0.008` | 稠密点云体素滤波 |
| `dense_unit` | `5000.0` | TUM 16UC1 深度单位 |
| `dense_pointcloud_topic` | `/orb_slam3/dense_points` | 稠密点云发布话题 |
| `input_pointcloud_topic` | `/orb_slam3/dense_points` | `octomap_converter` 输入 |
| `pcd_file` | 空 | `pcd_to_octomap_node` 输入文件 |
| `resolution` | `0.05` | OctoMap 分辨率 |

## 排查

- 先编译 `dense_orbslam3`，本包需要链接它的 `libORB_SLAM3.so`。
- 路径报错时，把 `<ws>` 换成自己的工作空间路径，并检查 `CMakeLists.txt`。
- 如果 PCL/OpenCV 冲突，优先使用 `dense_orbslam3/3rdParty` 里编译好的版本。
- RViz2 中 fixed frame 通常设为 `map`。

## 许可证

本项目基于 ORB-SLAM3，遵循其原始 GPLv3 许可证。
