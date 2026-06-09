#ifndef ROS2_SLAM_PUBLISHER_H
#define ROS2_SLAM_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <thread>
#include <mutex>

// Include from dense_orbslam3
#include "SlamDataProcess.h"
#include "System.h"

namespace ORB_SLAM3 {

class Ros2SlamPublisher : public rclcpp::Node
{
public:
    Ros2SlamPublisher(
        const std::string& voc_file,
        const std::string& settings_file,
        bool use_dense,
        bool use_viewer,
        const std::string& rgb_topic,
        const std::string& depth_topic);

    ~Ros2SlamPublisher();

    void Run();
    void RequestFinish();
    bool IsFinished();
    
private:
    void rgbd_callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);

    void PublishPoseData();
    void PublishPointCloudData();
    void PublishDensePointCloudData();
    void PublishFrameData();
    void PublishTrajectoryData();
    void PublishStaticTransform();

    void LoadROSParameters();
    cv::Mat ImageMsgToBgrMat(const sensor_msgs::msg::Image& msg);
    cv::Mat ImageMsgToDepthMat(const sensor_msgs::msg::Image& msg);
    sensor_msgs::msg::Image MatToImageMsg(const cv::Mat& image, const std::string& frame_id);
    geometry_msgs::msg::PoseStamped EigenMatrixToPoseStamped(const Eigen::Matrix4f& matrix, 
                                                             const std::string& frame_id);
    sensor_msgs::msg::PointCloud2 PCLToROS(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
                                          const std::string& frame_id);
    nav_msgs::msg::Path TrajectoryToPath(const std::vector<Eigen::Matrix4f>& trajectory,
                                        const std::string& frame_id);

    // ROS2 components
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr camera_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ref_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dense_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> map_to_slam_map_broadcaster_;

    // Message filters for synchronization
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // SLAM components from dense_orbslam3
    System* slam_system_;
    SlamDataProcess* slam_data_processor_;
    
    // Threading
    std::thread pose_thread_;
    std::thread pointcloud_thread_;
    std::thread dense_pointcloud_thread_;
    std::thread frame_thread_;
    std::thread trajectory_thread_;
    
    // Control
    bool finish_requested_;
    bool finished_;
    std::mutex finish_mutex_;
    
    // ROS2 Parameters
    double publish_rate_hz_;
    std::string camera_frame_;
    std::string ground_frame_;
    std::string vehicle_frame_;
};

} // namespace ORB_SLAM3

#endif // ROS2_SLAM_PUBLISHER_H
