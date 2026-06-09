#include "ros2_slam_publisher.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstring>
#include <rmw/qos_profiles.h>
#include <stdexcept>

namespace ORB_SLAM3 {

Ros2SlamPublisher::Ros2SlamPublisher(
    const std::string& voc_file,
    const std::string& settings_file,
    bool use_dense,
    bool use_viewer,
    const std::string& rgb_topic,
    const std::string& depth_topic)
    : Node("orb_slam3_dense_node"),
      finish_requested_(false),
      finished_(false),
      publish_rate_hz_(10.0),
      camera_frame_("camera"),
      ground_frame_("map"),
      vehicle_frame_("base_link")
{
    RCLCPP_INFO(this->get_logger(), "Initializing ORB-SLAM3 ROS2 node");
    RCLCPP_INFO(this->get_logger(), "Vocabulary: %s", voc_file.c_str());
    RCLCPP_INFO(this->get_logger(), "Settings: %s", settings_file.c_str());
    RCLCPP_INFO(this->get_logger(), "Use dense mapping: %s", use_dense ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Use viewer: %s", use_viewer ? "true" : "false");

    // Load ROS parameters
    LoadROSParameters();

    // Initialize ORB-SLAM3 system from dense_orbslam3
    try {
        slam_system_ = new System(voc_file, settings_file, System::RGBD, use_viewer);
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 system initialized from dense_orbslam3");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize ORB-SLAM3: %s", e.what());
        throw;
    }

    // Get system components
    FrameDrawer* frame_drawer = slam_system_->GetFrameDrawer();
    MapDrawer* map_drawer = slam_system_->GetMapDrawer();
    Tracking* tracker = slam_system_->GetTracker();
    Atlas* atlas = slam_system_->GetAtlas();

    // Initialize SLAM data processor from dense_orbslam3
    slam_data_processor_ = new SlamDataProcess(
        slam_system_, frame_drawer, map_drawer, tracker, settings_file, atlas, use_dense);
    
    // Create ROS2 publishers
    camera_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/orb_slam3/camera_pose", 10);
    vehicle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/orb_slam3/vehicle_pose", 10);
    camera_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/orb_slam3/camera_trajectory", 10);
    vehicle_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/orb_slam3/vehicle_trajectory", 10);
    all_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/orb_slam3/all_points", 10);
    ref_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/orb_slam3/ref_points", 10);
    dense_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/orb_slam3/dense_points", 10);

    frame_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/orb_slam3/tracking_frame", 10);

    // TF broadcasters
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    map_to_slam_map_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // QoS profile matching most RGB-D camera publishers (e.g., RealSense, Azure Kinect)
    // Using RELIABLE reliability and TRANSIENT_LOCAL durability for compatibility
    rclcpp::QoS image_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .transient_local();
    
    rgb_sub_.subscribe(this, rgb_topic, image_qos.get_rmw_qos_profile());
    depth_sub_.subscribe(this, depth_topic, image_qos.get_rmw_qos_profile());
    RCLCPP_INFO(this->get_logger(),
        "Subscribed to RGB-D topics with sensor_data QoS: rgb=%s depth=%s",
        rgb_topic.c_str(), depth_topic.c_str());

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(
        std::bind(&Ros2SlamPublisher::rgbd_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 ROS2 node ready. Waiting for RGB-D images...");
}

Ros2SlamPublisher::~Ros2SlamPublisher()
{
    RequestFinish();
}

void Ros2SlamPublisher::LoadROSParameters()
{
    this->declare_parameter("publish_rate_hz", 10.0);
    this->declare_parameter("camera_frame", "camera");
    this->declare_parameter("ground_frame", "map");
    this->declare_parameter("vehicle_frame", "base_link");

    this->get_parameter("publish_rate_hz", publish_rate_hz_);
    this->get_parameter("camera_frame", camera_frame_);
    this->get_parameter("ground_frame", ground_frame_);
    this->get_parameter("vehicle_frame", vehicle_frame_);

    RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f Hz", publish_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "Camera frame: %s", camera_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Ground frame: %s", ground_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Vehicle frame: %s", vehicle_frame_.c_str());
}

void Ros2SlamPublisher::Run()
{
    // Start SlamDataProcess thread from dense_orbslam3
    std::thread slam_data_thread(&SlamDataProcess::Run, slam_data_processor_);

    // Start publishing threads
    pose_thread_ = std::thread(&Ros2SlamPublisher::PublishPoseData, this);
    pointcloud_thread_ = std::thread(&Ros2SlamPublisher::PublishPointCloudData, this);
    if (slam_data_processor_->IsDenseMappingEnabled()) {
        dense_pointcloud_thread_ = std::thread(&Ros2SlamPublisher::PublishDensePointCloudData, this);
    }
    frame_thread_ = std::thread(&Ros2SlamPublisher::PublishFrameData, this);
    trajectory_thread_ = std::thread(&Ros2SlamPublisher::PublishTrajectoryData, this);

    // Publish static transform
    PublishStaticTransform();

    // Spin until shutdown
    rclcpp::spin(shared_from_this());

    // Cleanup
    slam_data_processor_->RequestFinish();
    slam_data_thread.join();

    RequestFinish();
}

void Ros2SlamPublisher::RequestFinish()
{
    std::lock_guard<std::mutex> lock(finish_mutex_);
    finish_requested_ = true;
}

bool Ros2SlamPublisher::IsFinished()
{
    std::lock_guard<std::mutex> lock(finish_mutex_);
    return finished_;
}

void Ros2SlamPublisher::rgbd_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
    try {
        static std::size_t synced_frame_count = 0;
        ++synced_frame_count;
        if (synced_frame_count == 1 || synced_frame_count % 30 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "Received synchronized RGB-D frame #%zu: rgb=%s depth=%s",
                synced_frame_count,
                rgb_msg->encoding.c_str(),
                depth_msg->encoding.c_str());
        }

        cv::Mat rgb_image = ImageMsgToBgrMat(*rgb_msg);
        cv::Mat depth_image = ImageMsgToDepthMat(*depth_msg);

        double timestamp = rgb_msg->header.stamp.sec + 
                          rgb_msg->header.stamp.nanosec * 1e-9;

        // Track RGBD frame using dense_orbslam3
        Sophus::SE3f pose = slam_system_->TrackRGBD(
            rgb_image, depth_image, timestamp);

        // Update SLAM data processor with new pose
        if (!pose.matrix().isZero()) {
            cv::Mat cv_pose = cv::Mat::eye(4, 4, CV_32F);
            Eigen::Matrix4f eigen_pose = pose.matrix();
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    cv_pose.at<float>(i, j) = eigen_pose(i, j);
                }
            }
            slam_data_processor_->SetCurrentCameraPose(cv_pose);
        }

        // Handle dense mapping: cache keyframes and insert into dense mapper
        if (slam_data_processor_->IsDenseMappingEnabled()) {
            KeyFrame* kf = slam_system_->GetTracker()->GetLastKeyFrame();
            if (kf) {
                slam_data_processor_->InsertKeyFrameForDense(kf, rgb_image, depth_image);
            }
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing RGBD: %s", e.what());
    }
}

void Ros2SlamPublisher::PublishPoseData()
{
    rclcpp::Rate rate(publish_rate_hz_);

    while (rclcpp::ok() && !finish_requested_) {
        SlamDataProcess::PoseData pose_data = slam_data_processor_->GetCurrentPoseData();
        
        if (pose_data.has_new_pose) {
            // Publish camera pose
            auto cam_pose_msg = EigenMatrixToPoseStamped(pose_data.cam_pose_to_ground, ground_frame_);
            cam_pose_msg.header.stamp = this->get_clock()->now();
            camera_pose_pub_->publish(cam_pose_msg);

            // Publish vehicle pose
            auto vehicle_pose_msg = EigenMatrixToPoseStamped(pose_data.vehicle_pose_to_ground, ground_frame_);
            vehicle_pose_msg.header.stamp = this->get_clock()->now();
            vehicle_pose_pub_->publish(vehicle_pose_msg);

            // Publish TF transforms
            geometry_msgs::msg::TransformStamped cam_tf;
            cam_tf.header.stamp = this->get_clock()->now();
            cam_tf.header.frame_id = ground_frame_;
            cam_tf.child_frame_id = camera_frame_;
            cam_tf.transform.translation.x = cam_pose_msg.pose.position.x;
            cam_tf.transform.translation.y = cam_pose_msg.pose.position.y;
            cam_tf.transform.translation.z = cam_pose_msg.pose.position.z;
            cam_tf.transform.rotation = cam_pose_msg.pose.orientation;
            tf_broadcaster_->sendTransform(cam_tf);

            geometry_msgs::msg::TransformStamped vehicle_tf;
            vehicle_tf.header.stamp = this->get_clock()->now();
            vehicle_tf.header.frame_id = ground_frame_;
            vehicle_tf.child_frame_id = vehicle_frame_;
            vehicle_tf.transform.translation.x = vehicle_pose_msg.pose.position.x;
            vehicle_tf.transform.translation.y = vehicle_pose_msg.pose.position.y;
            vehicle_tf.transform.translation.z = vehicle_pose_msg.pose.position.z;
            vehicle_tf.transform.rotation = vehicle_pose_msg.pose.orientation;
            tf_broadcaster_->sendTransform(vehicle_tf);
        }

        rate.sleep();
    }
}

void Ros2SlamPublisher::PublishPointCloudData()
{
    rclcpp::Rate rate(publish_rate_hz_);

    while (rclcpp::ok() && !finish_requested_) {
        SlamDataProcess::PointCloudData cloud_data = slam_data_processor_->GetCurrentPointCloudData();
        
        if (cloud_data.has_new_cloud) {
            if (cloud_data.all_points && !cloud_data.all_points->empty() && cloud_data.all_points->width > 0) {
                try {
                    sensor_msgs::msg::PointCloud2 cloud_msg = PCLToROS(cloud_data.all_points, ground_frame_);
                    all_points_pub_->publish(cloud_msg);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error publishing all points cloud: %s", e.what());
                }
            }

            if (cloud_data.ref_points && !cloud_data.ref_points->empty() && cloud_data.ref_points->width > 0) {
                try {
                    sensor_msgs::msg::PointCloud2 cloud_msg = PCLToROS(cloud_data.ref_points, ground_frame_);
                    ref_points_pub_->publish(cloud_msg);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error publishing ref points cloud: %s", e.what());
                }
            }
        }

        rate.sleep();
    }
}

void Ros2SlamPublisher::PublishDensePointCloudData()
{
    rclcpp::Rate rate(2.0); // Lower rate for dense map (2Hz)

    while (rclcpp::ok() && !finish_requested_) {
        SlamDataProcess::DensePointCloudData dense_data = slam_data_processor_->GetCurrentDensePointCloudData();
        
        if (dense_data.has_new_dense_cloud) {
            if (dense_data.dense_points && !dense_data.dense_points->empty() && dense_data.dense_points->width > 0) {
                try {
                    sensor_msgs::msg::PointCloud2 cloud_msg = PCLToROS(dense_data.dense_points, ground_frame_);
                    dense_points_pub_->publish(cloud_msg);
                    RCLCPP_DEBUG(this->get_logger(), 
                                "Published dense point cloud with %lu points", 
                                dense_data.dense_points->size());
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error publishing dense point cloud: %s", e.what());
                }
            } else if (dense_data.dense_points && dense_data.dense_points->empty()) {
                RCLCPP_WARN(this->get_logger(), "Dense point cloud is empty, skipping publish");
            } else if (!dense_data.dense_points) {
                RCLCPP_WARN(this->get_logger(), "Dense point cloud pointer is null, skipping publish");
            }
        }

        rate.sleep();
    }
}

void Ros2SlamPublisher::PublishFrameData()
{
    rclcpp::Rate rate(publish_rate_hz_);

    while (rclcpp::ok() && !finish_requested_) {
        SlamDataProcess::FrameData frame_data = slam_data_processor_->GetCurrentFrameData();
        
        if (frame_data.has_new_frame && !frame_data.current_frame.empty()) {
            frame_pub_->publish(MatToImageMsg(frame_data.current_frame, camera_frame_));
        }

        rate.sleep();
    }
}

void Ros2SlamPublisher::PublishTrajectoryData()
{
    rclcpp::Rate rate(publish_rate_hz_);

    while (rclcpp::ok() && !finish_requested_) {
        SlamDataProcess::TrajectoryData trajectory_data = slam_data_processor_->GetCurrentTrajectoryData();
        
        if (trajectory_data.has_new_trajectory) {
            if (!trajectory_data.camera_trajectory.empty()) {
                nav_msgs::msg::Path cam_path = TrajectoryToPath(trajectory_data.camera_trajectory, ground_frame_);
                camera_path_pub_->publish(cam_path);
            }

            if (!trajectory_data.vehicle_trajectory.empty()) {
                nav_msgs::msg::Path vehicle_path = TrajectoryToPath(trajectory_data.vehicle_trajectory, ground_frame_);
                vehicle_path_pub_->publish(vehicle_path);
            }
        }

        rate.sleep();
    }
}

void Ros2SlamPublisher::PublishStaticTransform()
{
    geometry_msgs::msg::TransformStamped static_tf;
    static_tf.header.stamp = this->get_clock()->now();
    static_tf.header.frame_id = "ros2_map";
    static_tf.child_frame_id = ground_frame_;
    static_tf.transform.translation.x = 0.0;
    static_tf.transform.translation.y = 0.0;
    static_tf.transform.translation.z = 0.0;
    static_tf.transform.rotation.w = 1.0;
    static_tf.transform.rotation.x = 0.0;
    static_tf.transform.rotation.y = 0.0;
    static_tf.transform.rotation.z = 0.0;
    map_to_slam_map_broadcaster_->sendTransform(static_tf);
}

cv::Mat Ros2SlamPublisher::ImageMsgToBgrMat(const sensor_msgs::msg::Image& msg)
{
    if (msg.height == 0 || msg.width == 0 || msg.data.empty())
        throw std::runtime_error("empty RGB image message");

    const int height = static_cast<int>(msg.height);
    const int width = static_cast<int>(msg.width);

    if (msg.encoding == sensor_msgs::image_encodings::BGR8)
    {
        return cv::Mat(height, width, CV_8UC3, const_cast<unsigned char*>(msg.data.data()),
                       msg.step)
            .clone();
    }
    if (msg.encoding == sensor_msgs::image_encodings::RGB8)
    {
        cv::Mat rgb(height, width, CV_8UC3, const_cast<unsigned char*>(msg.data.data()),
                    msg.step);
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }
    if (msg.encoding == sensor_msgs::image_encodings::BGRA8)
    {
        cv::Mat bgra(height, width, CV_8UC4, const_cast<unsigned char*>(msg.data.data()),
                     msg.step);
        cv::Mat bgr;
        cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
        return bgr;
    }
    if (msg.encoding == sensor_msgs::image_encodings::RGBA8)
    {
        cv::Mat rgba(height, width, CV_8UC4, const_cast<unsigned char*>(msg.data.data()),
                     msg.step);
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        return bgr;
    }
    if (msg.encoding == sensor_msgs::image_encodings::MONO8)
    {
        cv::Mat mono(height, width, CV_8UC1, const_cast<unsigned char*>(msg.data.data()),
                     msg.step);
        cv::Mat bgr;
        cv::cvtColor(mono, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }

    throw std::runtime_error("unsupported RGB image encoding: " + msg.encoding);
}

cv::Mat Ros2SlamPublisher::ImageMsgToDepthMat(const sensor_msgs::msg::Image& msg)
{
    if (msg.height == 0 || msg.width == 0 || msg.data.empty())
        throw std::runtime_error("empty depth image message");

    const int height = static_cast<int>(msg.height);
    const int width = static_cast<int>(msg.width);

    if (msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        msg.encoding == sensor_msgs::image_encodings::MONO16)
    {
        return cv::Mat(height, width, CV_16UC1, const_cast<unsigned char*>(msg.data.data()),
                       msg.step)
            .clone();
    }
    if (msg.encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        cv::Mat depth_m(height, width, CV_32FC1, const_cast<unsigned char*>(msg.data.data()),
                        msg.step);
        cv::Mat depth_mm;
        depth_m.convertTo(depth_mm, CV_16UC1, 1000.0);
        return depth_mm;
    }

    throw std::runtime_error("unsupported depth image encoding: " + msg.encoding);
}

sensor_msgs::msg::Image Ros2SlamPublisher::MatToImageMsg(const cv::Mat& image,
                                                         const std::string& frame_id)
{
    cv::Mat bgr;
    if (image.channels() == 1)
        cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    else if (image.channels() == 3)
        bgr = image;
    else if (image.channels() == 4)
        cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
    else
        throw std::runtime_error("unsupported frame channel count");

    if (!bgr.isContinuous())
        bgr = bgr.clone();

    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id;
    msg.height = static_cast<uint32_t>(bgr.rows);
    msg.width = static_cast<uint32_t>(bgr.cols);
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(bgr.cols * bgr.elemSize());
    msg.data.assign(bgr.datastart, bgr.dataend);
    return msg;
}

geometry_msgs::msg::PoseStamped Ros2SlamPublisher::EigenMatrixToPoseStamped(
    const Eigen::Matrix4f& matrix, const std::string& frame_id)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = frame_id;

    pose_msg.pose.position.x = matrix(0, 3);
    pose_msg.pose.position.y = matrix(1, 3);
    pose_msg.pose.position.z = matrix(2, 3);

    Eigen::Matrix3f rot = matrix.block<3,3>(0,0);
    Eigen::Quaternionf q(rot);
    pose_msg.pose.orientation.w = q.w();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();

    return pose_msg;
}

sensor_msgs::msg::PointCloud2 Ros2SlamPublisher::PCLToROS(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
    const std::string& frame_id)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.header.frame_id = frame_id;

    if (!pcl_cloud || pcl_cloud->empty() || pcl_cloud->width == 0) {
        RCLCPP_WARN(this->get_logger(), 
                    "PCLToROS: Invalid point cloud (null=%d, empty=%d, width=%d)", 
                    !pcl_cloud, pcl_cloud && pcl_cloud->empty(), 
                    pcl_cloud ? pcl_cloud->width : 0);
        return cloud_msg;
    }

    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(pcl_cloud->points.size());
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = pcl_cloud->is_dense;
    cloud_msg.point_step = 16;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

    cloud_msg.fields.resize(4);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;
    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;
    cloud_msg.fields[3].name = "rgb";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[3].count = 1;

    cloud_msg.data.resize(cloud_msg.row_step);
    for (size_t i = 0; i < pcl_cloud->points.size(); ++i) {
        const auto& point = pcl_cloud->points[i];
        uint8_t* dst = cloud_msg.data.data() + i * cloud_msg.point_step;

        const uint32_t rgb_packed =
            (static_cast<uint32_t>(point.r) << 16) |
            (static_cast<uint32_t>(point.g) << 8) |
            static_cast<uint32_t>(point.b);
        float rgb_float = 0.0f;
        std::memcpy(&rgb_float, &rgb_packed, sizeof(rgb_float));

        std::memcpy(dst + 0, &point.x, sizeof(float));
        std::memcpy(dst + 4, &point.y, sizeof(float));
        std::memcpy(dst + 8, &point.z, sizeof(float));
        std::memcpy(dst + 12, &rgb_float, sizeof(float));
    }

    return cloud_msg;
}

nav_msgs::msg::Path Ros2SlamPublisher::TrajectoryToPath(
    const std::vector<Eigen::Matrix4f>& trajectory,
    const std::string& frame_id)
{
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = frame_id;

    for (const auto& pose : trajectory) {
        geometry_msgs::msg::PoseStamped pose_stamped = EigenMatrixToPoseStamped(pose, frame_id);
        pose_stamped.header.stamp = this->get_clock()->now();
        path.poses.push_back(pose_stamped);
    }

    return path;
}

} // namespace ORB_SLAM3
