#include <rclcpp/rclcpp.hpp>
#include "ros2_slam_publisher.h"

#include <string>
#include <vector>
#include <signal.h>

namespace
{
    bool parseBool(const std::string &value, bool default_value)
    {
        if (value == "true" || value == "True" || value == "TRUE" || value == "1")
        {
            return true;
        }
        if (value == "false" || value == "False" || value == "FALSE" || value == "0")
        {
            return false;
        }
        return default_value;
    }
}  // namespace

// Global pointer for signal handler
std::shared_ptr<ORB_SLAM3::Ros2SlamPublisher> g_node = nullptr;

void signal_handler(int signal) {
    std::cout << "Interrupt signal (" << signal << ") received. Shutting down..." << std::endl;
    
    if (g_node) {
        g_node->RequestFinish();
    }
    
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    // Set up signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);

    std::vector<std::string> positional_args;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--ros-args")
        {
            break;
        }
        positional_args.push_back(arg);
    }

    // Default parameters
    std::string voc_file = "/home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt";
    std::string settings_file = "/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/RealSense_D435i.yaml";
    bool use_dense = true;
    bool use_viewer = true;
    std::string rgb_topic = "/camera/camera/color/image_raw";
    std::string depth_topic = "/camera/camera/aligned_depth_to_color/image_raw";

    // Parse command line arguments
    if (positional_args.size() >= 1) voc_file = positional_args[0];
    if (positional_args.size() >= 2) settings_file = positional_args[1];
    if (positional_args.size() >= 3) use_dense = parseBool(positional_args[2], use_dense);
    if (positional_args.size() >= 4) rgb_topic = positional_args[3];
    if (positional_args.size() >= 5) depth_topic = positional_args[4];
    if (positional_args.size() >= 6) use_viewer = parseBool(positional_args[5], use_viewer);

    // Validate required parameters
    if (voc_file.empty() || settings_file.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("orb_slam3_main"),
            "Usage: ros2 run orbslam3_dense_ros2 orb_slam3_main "
            "<vocabulary> <settings> [use_dense(true/false)] [rgb_topic] [depth_topic] [use_viewer(true/false)]");
        rclcpp::shutdown();
        return 1;
    }

    try {
        g_node = std::make_shared<ORB_SLAM3::Ros2SlamPublisher>(
            voc_file, settings_file, use_dense, use_viewer, rgb_topic, depth_topic);

        RCLCPP_INFO(g_node->get_logger(), "ORB-SLAM3 ROS2 system started. Waiting for synchronized RGB+Depth images...");

        // Run the node
        g_node->Run();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("orb_slam3_main"), 
                    "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    std::cout << "ORB-SLAM3 ROS2 system shut down cleanly." << std::endl;
    rclcpp::shutdown();
    return 0;
}