#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <limits>
#include <stdexcept>

class PCDToOctomapNode : public rclcpp::Node
{
public:
    PCDToOctomapNode() : Node("pcd_to_octomap_node")
    {
        declareParameters();
        loadParameters();
        printParameters();

        if (pcd_file_.empty()) {
            RCLCPP_FATAL(this->get_logger(), "PCD file path is empty!");
            throw std::runtime_error("pcd_file is empty");
        }

        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                           .reliable()
                           .transient_local();

        octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>(
            topic_name_, map_qos);

        if (!convertPCDToOctomap()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to convert PCD to Octomap");
            throw std::runtime_error("convert failed");
        }

        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(publish_period_)),
            std::bind(&PCDToOctomapNode::publishOctomap, this));

        publishOctomap();
    }

private:
    void declareParameters()
    {
        this->declare_parameter<std::string>("pcd_file", "");
        this->declare_parameter<std::string>("output_file", "output.bt");
        this->declare_parameter<double>("resolution", 0.05);
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<std::string>("topic_name", "/octomap");
        this->declare_parameter<double>("publish_period", 1.0);

        // 建议默认不要对 PCD 做 Tcw 变换。
        // 很多 ORB-SLAM / dense pointcloud 导出的 PCD 已经是 world/map 坐标。
        this->declare_parameter<bool>("apply_tcw", false);

        // Tcw 默认必须是单位矩阵，不能是 16 个 0。
        // 这里按照常见 YAML/launch 写法：row-major 行主序。
        this->declare_parameter<std::vector<double>>(
            "Tcw",
            std::vector<double>{
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0
            }
        );
    }

    void loadParameters()
    {
        pcd_file_ = this->get_parameter("pcd_file").as_string();
        output_file_ = this->get_parameter("output_file").as_string();
        resolution_ = this->get_parameter("resolution").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        topic_name_ = this->get_parameter("topic_name").as_string();
        publish_period_ = this->get_parameter("publish_period").as_double();
        apply_tcw_ = this->get_parameter("apply_tcw").as_bool();

        const auto tcw_param = this->get_parameter("Tcw").as_double_array();

        if (tcw_param.size() == 16) {
            using RowMajorMatrix4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
            tcw_ = Eigen::Map<const RowMajorMatrix4d>(tcw_param.data());
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Tcw parameter size is %zu, not 16. Using identity matrix.",
                tcw_param.size());
            tcw_ = Eigen::Matrix4d::Identity();
        }

        if (resolution_ <= 0.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid resolution %.6f. Resetting to 0.05.",
                resolution_);
            resolution_ = 0.05;
        }

        if (publish_period_ <= 0.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid publish_period %.6f. Resetting to 1.0.",
                publish_period_);
            publish_period_ = 1.0;
        }

        if (apply_tcw_) {
            const double det = tcw_.determinant();
            if (std::abs(det) < 1e-12 || !std::isfinite(det)) {
                RCLCPP_ERROR_STREAM(
                    this->get_logger(),
                    "Tcw is singular or invalid. Determinant = " << det
                    << "\nTcw:\n" << tcw_
                    << "\nDisabling Tcw transform.");
                apply_tcw_ = false;
                tcw_ = Eigen::Matrix4d::Identity();
            }
        }

        twc_ = tcw_.inverse();
    }

    void printParameters()
    {
        RCLCPP_INFO(this->get_logger(), "PCD file: %s", pcd_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output file: %s", output_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Resolution: %.6f", resolution_);
        RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Topic name: %s", topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish period: %.3f s", publish_period_);
        RCLCPP_INFO(this->get_logger(), "Apply Tcw: %s", apply_tcw_ ? "true" : "false");

        RCLCPP_INFO_STREAM(this->get_logger(), "Tcw:\n" << tcw_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Twc = inverse(Tcw):\n" << twc_);
    }

    bool convertPCDToOctomap()
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", pcd_file_.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded PCD with %zu points", cloud.size());

        octree_ = std::make_shared<octomap::OcTree>(resolution_);

        size_t valid_points = 0;
        size_t invalid_points = 0;

        double raw_min_x = std::numeric_limits<double>::max();
        double raw_min_y = std::numeric_limits<double>::max();
        double raw_min_z = std::numeric_limits<double>::max();
        double raw_max_x = std::numeric_limits<double>::lowest();
        double raw_max_y = std::numeric_limits<double>::lowest();
        double raw_max_z = std::numeric_limits<double>::lowest();

        double world_min_x = std::numeric_limits<double>::max();
        double world_min_y = std::numeric_limits<double>::max();
        double world_min_z = std::numeric_limits<double>::max();
        double world_max_x = std::numeric_limits<double>::lowest();
        double world_max_y = std::numeric_limits<double>::lowest();
        double world_max_z = std::numeric_limits<double>::lowest();

        for (const auto& point : cloud.points) {
            if (!std::isfinite(point.x) ||
                !std::isfinite(point.y) ||
                !std::isfinite(point.z)) {
                ++invalid_points;
                continue;
            }

            raw_min_x = std::min(raw_min_x, static_cast<double>(point.x));
            raw_min_y = std::min(raw_min_y, static_cast<double>(point.y));
            raw_min_z = std::min(raw_min_z, static_cast<double>(point.z));
            raw_max_x = std::max(raw_max_x, static_cast<double>(point.x));
            raw_max_y = std::max(raw_max_y, static_cast<double>(point.y));
            raw_max_z = std::max(raw_max_z, static_cast<double>(point.z));

            Eigen::Vector4d point_world;

            if (apply_tcw_) {
                const Eigen::Vector4d point_cam(point.x, point.y, point.z, 1.0);
                point_world = twc_ * point_cam;
            } else {
                point_world = Eigen::Vector4d(point.x, point.y, point.z, 1.0);
            }

            if (!std::isfinite(point_world(0)) ||
                !std::isfinite(point_world(1)) ||
                !std::isfinite(point_world(2))) {
                ++invalid_points;
                continue;
            }

            world_min_x = std::min(world_min_x, point_world(0));
            world_min_y = std::min(world_min_y, point_world(1));
            world_min_z = std::min(world_min_z, point_world(2));
            world_max_x = std::max(world_max_x, point_world(0));
            world_max_y = std::max(world_max_y, point_world(1));
            world_max_z = std::max(world_max_z, point_world(2));

            octree_->updateNode(
                octomap::point3d(
                    static_cast<float>(point_world(0)),
                    static_cast<float>(point_world(1)),
                    static_cast<float>(point_world(2))),
                true);

            ++valid_points;
        }

        if (valid_points == 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "No valid points were inserted into the OctoMap. Invalid points: %zu",
                invalid_points);
            return false;
        }

        octree_->updateInnerOccupancy();

        double oct_min_x = 0.0;
        double oct_min_y = 0.0;
        double oct_min_z = 0.0;
        double oct_max_x = 0.0;
        double oct_max_y = 0.0;
        double oct_max_z = 0.0;

        octree_->getMetricMin(oct_min_x, oct_min_y, oct_min_z);
        octree_->getMetricMax(oct_max_x, oct_max_y, oct_max_z);

        RCLCPP_INFO(
            this->get_logger(),
            "Raw PCD range: x[%.3f, %.3f], y[%.3f, %.3f], z[%.3f, %.3f]",
            raw_min_x, raw_max_x,
            raw_min_y, raw_max_y,
            raw_min_z, raw_max_z);

        RCLCPP_INFO(
            this->get_logger(),
            "World range used for OctoMap: x[%.3f, %.3f], y[%.3f, %.3f], z[%.3f, %.3f]",
            world_min_x, world_max_x,
            world_min_y, world_max_y,
            world_min_z, world_max_z);

        RCLCPP_INFO(
            this->get_logger(),
            "OctoMap metric range: x[%.3f, %.3f], y[%.3f, %.3f], z[%.3f, %.3f]",
            oct_min_x, oct_max_x,
            oct_min_y, oct_max_y,
            oct_min_z, oct_max_z);

        RCLCPP_INFO(
            this->get_logger(),
            "Built OctoMap: valid points = %zu, invalid points = %zu, nodes = %zu",
            valid_points,
            invalid_points,
            octree_->size());

        if (!octree_->writeBinary(output_file_)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to write binary OctoMap to %s",
                output_file_.c_str());
        } else {
            RCLCPP_INFO(
                this->get_logger(),
                "Saved binary OctoMap to %s",
                output_file_.c_str());
        }

        return true;
    }

    void publishOctomap()
    {
        if (!octree_) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Octree is null");
            return;
        }

        octomap_msgs::msg::Octomap msg;

        if (!octomap_msgs::fullMapToMsg(*octree_, msg)) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Failed to convert OctoMap to message");
            return;
        }

        // fullMapToMsg 之后重新设置 header，避免 header 被转换函数覆盖。
        msg.header.frame_id = frame_id_;
        msg.header.stamp = this->now();

        octomap_publisher_->publish(msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "Publishing OctoMap on %s, frame_id = %s, nodes = %zu",
            topic_name_.c_str(),
            frame_id_.c_str(),
            octree_->size());
    }

private:
    std::string pcd_file_;
    std::string output_file_;
    std::string frame_id_;
    std::string topic_name_;

    double resolution_ = 0.05;
    double publish_period_ = 1.0;
    bool apply_tcw_ = false;

    Eigen::Matrix4d tcw_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d twc_ = Eigen::Matrix4d::Identity();

    std::shared_ptr<octomap::OcTree> octree_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<PCDToOctomapNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(
            rclcpp::get_logger("pcd_to_octomap_node"),
            "Exception: %s",
            e.what());
    }

    rclcpp::shutdown();
    return 0;
}

