cmake_minimum_required(VERSION 3.8)
project(orbslam3_dense_ros2)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR "/home/ricky/WCR_ws/dense_orbslam3/3rdParty/Opencv/build-full" CACHE PATH "OpenCV config path" FORCE)
set(PCL_DIR "/home/ricky/WCR_ws/dense_orbslam3/3rdParty/pcl/pcl_install/share/pcl-1.15" CACHE PATH "PCL config path" FORCE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(octomap REQUIRED)

# ====================== 指向你的 dense_orbslam3 ======================
set(DENSE_ORBSLAM3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../dense_orbslam3")
set(DENSE_PCL_ROOT "${DENSE_ORBSLAM3_ROOT}/3rdParty/pcl/pcl_install")
set(DENSE_PCL_INCLUDE_DIR "${DENSE_PCL_ROOT}/include/pcl-1.15")
set(DENSE_OPENCV_ROOT "${DENSE_ORBSLAM3_ROOT}/3rdParty/Opencv/build-full")
set(DENSE_PANGOLIN_ROOT "${DENSE_ORBSLAM3_ROOT}/3rdParty/Pangolin/build/src")

# 使用 dense_orbslam3 自带的 PCL（避免与系统PCL冲突）
set(PCL_INCLUDE_DIRS
    ${DENSE_PCL_INCLUDE_DIR}
    ${DENSE_PCL_ROOT}/include
    /usr/include/vtk-9.1
    /usr/include/eigen3
    /usr/include/freetype2
)

set(PCL_LIBRARY_DIRS 
    ${DENSE_PCL_ROOT}/lib
)

# 使用绝对路径固定到 3rdParty PCL，避免 ROS/系统环境里的 PCL 1.14 混进来。
set(PCL_LIBRARIES
    ${DENSE_PCL_ROOT}/lib/libpcl_common.so
    ${DENSE_PCL_ROOT}/lib/libpcl_filters.so
    ${DENSE_PCL_ROOT}/lib/libpcl_io.so
    ${DENSE_PCL_ROOT}/lib/libpcl_kdtree.so
    ${DENSE_PCL_ROOT}/lib/libpcl_search.so
    pthread
    atomic
    vtkCommonCore-9.1
    vtkCommonDataModel-9.1
    vtkRenderingCore-9.1
    vtkRenderingOpenGL2-9.1
    vtkInteractionStyle-9.1
    vtkRenderingFreeType-9.1
    vtkFiltersCore-9.1
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories(
  ${DENSE_ORBSLAM3_ROOT}
  ${DENSE_ORBSLAM3_ROOT}/include
  ${DENSE_ORBSLAM3_ROOT}/Thirdparty/Sophus
  ${DENSE_ORBSLAM3_ROOT}/include/CameraModels
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/orbslam3_dense_ros2
  ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(${DENSE_ORBSLAM3_ROOT}/lib)

# ====================== 可执行文件 ======================
add_executable(orb_slam3_main
  src/orb_slam3_main.cpp
  src/ros2_slam_publisher.cpp
)

add_executable(rgbd_dataset_dense_node
  src/rgbd_dataset_dense_node.cpp
)

add_executable(octomap_converter
  src/octomap_converter.cpp
)

add_executable(pcd_to_octomap_node
  src/pcd_to_octomap_node.cpp
)

target_include_directories(orb_slam3_main BEFORE PRIVATE
  ${DENSE_PCL_INCLUDE_DIR}
)

target_include_directories(rgbd_dataset_dense_node BEFORE PRIVATE
  ${DENSE_PCL_INCLUDE_DIR}
)

target_include_directories(octomap_converter PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(pcd_to_octomap_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(orb_slam3_main
  rclcpp
  sensor_msgs
  message_filters
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs
)

ament_target_dependencies(rgbd_dataset_dense_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(octomap_converter
  rclcpp
  sensor_msgs
  octomap_msgs
)

ament_target_dependencies(pcd_to_octomap_node
  rclcpp
  sensor_msgs
  octomap_msgs
)

target_link_libraries(orb_slam3_main
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ORB_SLAM3
)

target_compile_options(orb_slam3_main PRIVATE -march=native)

target_link_libraries(rgbd_dataset_dense_node
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ORB_SLAM3
)

target_compile_options(rgbd_dataset_dense_node PRIVATE -march=native)

# 添加 rpath 确保运行时能找到自定义库
set_target_properties(orb_slam3_main PROPERTIES
  BUILD_RPATH "${DENSE_ORBSLAM3_ROOT}/lib:${DENSE_PCL_ROOT}/lib:${DENSE_OPENCV_ROOT}/lib:${DENSE_PANGOLIN_ROOT}"
  INSTALL_RPATH "${DENSE_ORBSLAM3_ROOT}/lib:${DENSE_PCL_ROOT}/lib:${DENSE_OPENCV_ROOT}/lib:${DENSE_PANGOLIN_ROOT}"
)

set_target_properties(rgbd_dataset_dense_node PROPERTIES
  BUILD_RPATH "${DENSE_ORBSLAM3_ROOT}/lib:${DENSE_PCL_ROOT}/lib:${DENSE_OPENCV_ROOT}/lib:${DENSE_PANGOLIN_ROOT}"
  INSTALL_RPATH "${DENSE_ORBSLAM3_ROOT}/lib:${DENSE_PCL_ROOT}/lib:${DENSE_OPENCV_ROOT}/lib:${DENSE_PANGOLIN_ROOT}"
)

set_target_properties(octomap_converter pcd_to_octomap_node PROPERTIES
  BUILD_RPATH "${DENSE_PCL_ROOT}/lib"
  INSTALL_RPATH "${DENSE_PCL_ROOT}/lib"
)

target_link_libraries(octomap_converter
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

target_link_libraries(pcd_to_octomap_node
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# 安装
install(TARGETS orb_slam3_main rgbd_dataset_dense_node octomap_converter pcd_to_octomap_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 config 和 launch 文件
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
