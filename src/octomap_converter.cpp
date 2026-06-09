#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap_msgs/conversions.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

class OctomapConverter : public rclcpp::Node
{
public:
    OctomapConverter() : Node("octomap_converter")
    {
        // 配置参数
        this->declare_parameter<std::string>("input_pointcloud_topic", "/orb_slam3/dense_points");
        this->declare_parameter<double>("resolution", 0.05);
        this->declare_parameter<double>("max_range", 5.0);
        this->declare_parameter<bool>("color", true);

        this->get_parameter("input_pointcloud_topic", input_pointcloud_topic_);
        this->get_parameter("resolution", resolution_);
        this->get_parameter("max_range", max_range_);
        this->get_parameter("color", use_color_);

        // 订阅稠密点云
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_pointcloud_topic_, 10,
            std::bind(&OctomapConverter::pointCloudCallback, this, std::placeholders::_1));

        // 发布OctoMap
        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(
            "/orb_slam3/octomap", 10);

        // 初始化OctoMap
        if (use_color_) {
            color_tree_ = new octomap::ColorOcTree(resolution_);
        } else {
            tree_ = new octomap::OcTree(resolution_);
        }

        RCLCPP_INFO(this->get_logger(), "OctoMap converter node started");
        RCLCPP_INFO(this->get_logger(), "Input point cloud topic: %s",
                    input_pointcloud_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Resolution: %f, Max range: %f, Use color: %s",
                    resolution_, max_range_, use_color_ ? "true" : "false");
    }

    ~OctomapConverter()
    {
        if (tree_) delete tree_;
        if (color_tree_) delete color_tree_;
    }

private:
    static int fieldOffset(const sensor_msgs::msg::PointCloud2& msg, const std::string& name)
    {
        for (const auto& field : msg.fields) {
            if (field.name == name) {
                return static_cast<int>(field.offset);
            }
        }
        return -1;
    }

    static float readFloat32(const std::uint8_t* point_data, int offset)
    {
        float value = 0.0f;
        std::memcpy(&value, point_data + offset, sizeof(value));
        return value;
    }

    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud2ToPCL(
        const sensor_msgs::msg::PointCloud2& msg)
    {
        const int x_offset = fieldOffset(msg, "x");
        const int y_offset = fieldOffset(msg, "y");
        const int z_offset = fieldOffset(msg, "z");
        const int rgb_offset = fieldOffset(msg, "rgb");
        const int r_offset = fieldOffset(msg, "r");
        const int g_offset = fieldOffset(msg, "g");
        const int b_offset = fieldOffset(msg, "b");

        if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
            throw std::runtime_error("PointCloud2 is missing x/y/z fields");
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->is_dense = msg.is_dense;
        cloud->height = 1;
        cloud->points.reserve(static_cast<std::size_t>(msg.width) * msg.height);

        for (std::uint32_t row = 0; row < msg.height; ++row) {
            const std::uint8_t* row_data = msg.data.data() + row * msg.row_step;
            for (std::uint32_t col = 0; col < msg.width; ++col) {
                const std::uint8_t* point_data = row_data + col * msg.point_step;

                pcl::PointXYZRGB point;
                point.x = readFloat32(point_data, x_offset);
                point.y = readFloat32(point_data, y_offset);
                point.z = readFloat32(point_data, z_offset);

                if (rgb_offset >= 0) {
                    std::uint32_t rgb = 0;
                    std::memcpy(&rgb, point_data + rgb_offset, sizeof(rgb));
                    point.r = static_cast<std::uint8_t>((rgb >> 16) & 0xff);
                    point.g = static_cast<std::uint8_t>((rgb >> 8) & 0xff);
                    point.b = static_cast<std::uint8_t>(rgb & 0xff);
                } else if (r_offset >= 0 && g_offset >= 0 && b_offset >= 0) {
                    point.r = *(point_data + r_offset);
                    point.g = *(point_data + g_offset);
                    point.b = *(point_data + b_offset);
                } else {
                    point.r = 255;
                    point.g = 255;
                    point.b = 255;
                }

                cloud->points.push_back(point);
            }
        }

        cloud->width = static_cast<std::uint32_t>(cloud->points.size());
        return cloud;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pointCloud2ToPCL(*msg);

        // 清除旧地图
        if (use_color_) {
            color_tree_->clear();
        } else {
            tree_->clear();
        }

        // 插入点云到OctoMap
        for (const auto& point : cloud->points) {
            // 检查点是否有效
            if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
                // 检查点是否在最大范围内
                double distance = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
                if (distance <= max_range_) {
                    if (use_color_) {
                        color_tree_->updateNode(point.x, point.y, point.z, true);
                        // 设置颜色
                        color_tree_->setNodeColor(point.x, point.y, point.z, 
                                            point.r, point.g, point.b);
                    } else {
                        tree_->updateNode(point.x, point.y, point.z, true);
                    }
                }
            }
        }

        // 构建OctoMap
        if (use_color_) {
            color_tree_->updateInnerOccupancy();
        } else {
            tree_->updateInnerOccupancy();
        }

        // 发布OctoMap
        octomap_msgs::msg::Octomap octomap_msg;
        octomap_msg.header.frame_id = msg->header.frame_id;
        octomap_msg.header.stamp = this->now();
        
        if (use_color_) {
            octomap_msgs::fullMapToMsg(*color_tree_, octomap_msg);
        } else {
            octomap_msgs::fullMapToMsg(*tree_, octomap_msg);
        }

        octomap_pub_->publish(octomap_msg);
        RCLCPP_INFO(this->get_logger(), "Published OctoMap with %zu nodes", 
                    use_color_ ? color_tree_->size() : tree_->size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;

    double resolution_;
    double max_range_;
    bool use_color_;
    std::string input_pointcloud_topic_;

    octomap::OcTree* tree_ = nullptr;
    octomap::ColorOcTree* color_tree_ = nullptr;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctomapConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
