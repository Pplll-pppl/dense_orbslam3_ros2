#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <opencv2/imgcodecs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "System.h"
#include "Tracking.h"
#include "KeyFrame.h"
#include "PointCloudMappingRGBD.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace fs = std::filesystem;

namespace
{
struct DatasetFrame
{
    double timestamp = 0.0;
    std::string rgb_path;
    std::string depth_path;
};

struct CliOptions
{
    std::string vocabulary;
    std::string settings;
    std::string sequence_path;
    std::string association_file;
    std::string root_dir;
    std::string tum_spec;
    std::string dense_mode;
    bool use_viewer = true;
};

bool ParseBool(const std::string& value, bool default_value)
{
    if (value == "true" || value == "True" || value == "TRUE" || value == "1")
        return true;
    if (value == "false" || value == "False" || value == "FALSE" || value == "0")
        return false;
    return default_value;
}

void PrintUsage()
{
    std::cout
        << "Usage:\n"
        << "  ros2 run orbslam3_dense_ros2 rgbd_dataset_dense_node "
        << "<vocabulary> <settings> <sequence_path> <association_file> [use_viewer] [dense_mode]\n\n"
        << "TUM style:\n"
        << "  ros2 run orbslam3_dense_ros2 rgbd_dataset_dense_node "
        << "--voc VOC --param YAML --tum SEQUENCE:ASSOCIATION\n\n"
        << "Folder style:\n"
        << "  ros2 run orbslam3_dense_ros2 rgbd_dataset_dense_node "
        << "--voc VOC --param YAML --root_dir DATASET_ROOT\n\n"
        << "dense_mode: realtime | deferred | both\n"
        << "ROS parameters are also supported after --ros-args.\n";
}

CliOptions ParseCliOptions(int argc, char** argv)
{
    CliOptions options;
    std::vector<std::string> positional;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--ros-args")
            break;

        auto require_value = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc)
                throw std::runtime_error("missing value for " + name);
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h")
        {
            PrintUsage();
            std::exit(0);
        }
        else if (arg == "--voc" || arg == "-v")
        {
            options.vocabulary = require_value(arg);
        }
        else if (arg == "--param" || arg == "--settings" || arg == "-p")
        {
            options.settings = require_value(arg);
        }
        else if (arg == "--tum" || arg == "-t")
        {
            options.tum_spec = require_value(arg);
        }
        else if (arg == "--sequence")
        {
            options.sequence_path = require_value(arg);
        }
        else if (arg == "--assoc" || arg == "--association")
        {
            options.association_file = require_value(arg);
        }
        else if (arg == "--root_dir" || arg == "--root-dir" || arg == "-r")
        {
            options.root_dir = require_value(arg);
        }
        else if (arg == "--viewer")
        {
            options.use_viewer = ParseBool(require_value(arg), options.use_viewer);
        }
        else if (arg == "--dense-mode")
        {
            options.dense_mode = require_value(arg);
        }
        else
        {
            positional.push_back(arg);
        }
    }

    if (positional.size() >= 1 && options.vocabulary.empty())
        options.vocabulary = positional[0];
    if (positional.size() >= 2 && options.settings.empty())
        options.settings = positional[1];
    if (positional.size() >= 3 && options.sequence_path.empty())
        options.sequence_path = positional[2];
    if (positional.size() >= 4 && options.association_file.empty())
        options.association_file = positional[3];
    if (positional.size() >= 5)
        options.use_viewer = ParseBool(positional[4], options.use_viewer);
    if (positional.size() >= 6 && options.dense_mode.empty())
        options.dense_mode = positional[5];

    if (!options.tum_spec.empty())
    {
        const std::size_t sep = options.tum_spec.find(':');
        if (sep == std::string::npos)
            throw std::runtime_error("--tum must be formatted as SEQUENCE_PATH:ASSOCIATION_FILE");
        options.sequence_path = options.tum_spec.substr(0, sep);
        options.association_file = options.tum_spec.substr(sep + 1);
    }

    return options;
}

bool TryParseDouble(const std::string& text, double& value)
{
    try
    {
        std::size_t consumed = 0;
        value = std::stod(text, &consumed);
        return consumed == text.size();
    }
    catch (...)
    {
        return false;
    }
}

std::vector<std::string> ListRegularFilesSorted(const fs::path& directory)
{
    std::vector<std::string> files;
    if (!fs::exists(directory) || !fs::is_directory(directory))
        return files;

    for (const auto& entry : fs::directory_iterator(directory))
    {
        if (entry.is_regular_file())
            files.push_back(entry.path().filename().string());
    }

    std::sort(files.begin(), files.end(), [](const std::string& a, const std::string& b) {
        double av = 0.0;
        double bv = 0.0;
        const bool a_numeric = TryParseDouble(fs::path(a).stem().string(), av);
        const bool b_numeric = TryParseDouble(fs::path(b).stem().string(), bv);
        if (a_numeric && b_numeric)
            return av < bv;
        return a < b;
    });

    return files;
}

std::vector<DatasetFrame> LoadTumAssociation(const std::string& sequence_path,
                                             const std::string& association_file)
{
    std::ifstream file(association_file);
    if (!file.is_open())
        throw std::runtime_error("failed to open association file: " + association_file);

    std::vector<DatasetFrame> frames;
    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
            continue;

        std::stringstream ss(line);
        double rgb_time = 0.0;
        double depth_time = 0.0;
        std::string rgb_rel;
        std::string depth_rel;
        if (!(ss >> rgb_time >> rgb_rel >> depth_time >> depth_rel))
            continue;

        DatasetFrame frame;
        frame.timestamp = rgb_time;
        frame.rgb_path = (fs::path(sequence_path) / rgb_rel).string();
        frame.depth_path = (fs::path(sequence_path) / depth_rel).string();
        frames.push_back(frame);
    }

    return frames;
}

std::vector<DatasetFrame> LoadFolderDataset(const std::string& root_dir, double fps)
{
    const fs::path root(root_dir);
    const fs::path rgb_dir = root / "rgb";
    const fs::path depth_dir = root / "depth";

    std::vector<std::string> rgb_files = ListRegularFilesSorted(rgb_dir);
    std::vector<std::string> depth_files = ListRegularFilesSorted(depth_dir);

    if (rgb_files.empty() || depth_files.empty())
        throw std::runtime_error("root_dir must contain non-empty rgb and depth folders");
    if (rgb_files.size() != depth_files.size())
        throw std::runtime_error("rgb/depth image count mismatch");

    if (fps <= 0.0)
        fps = 30.0;

    std::vector<DatasetFrame> frames;
    frames.reserve(rgb_files.size());
    for (std::size_t i = 0; i < rgb_files.size(); ++i)
    {
        DatasetFrame frame;
        frame.timestamp = static_cast<double>(i) / fps;
        frame.rgb_path = (rgb_dir / rgb_files[i]).string();
        frame.depth_path = (depth_dir / depth_files[i]).string();
        frames.push_back(frame);
    }

    return frames;
}

bool FindFrameForTimestamp(const std::vector<DatasetFrame>& frames,
                           double timestamp,
                           double tolerance,
                           DatasetFrame& out_frame)
{
    if (frames.empty())
        return false;

    auto it = std::lower_bound(frames.begin(), frames.end(), timestamp,
                               [](const DatasetFrame& frame, double t) {
                                   return frame.timestamp < t;
                               });

    auto best = frames.end();
    double best_diff = std::numeric_limits<double>::max();

    if (it != frames.end())
    {
        best = it;
        best_diff = std::fabs(it->timestamp - timestamp);
    }
    if (it != frames.begin())
    {
        auto prev = it - 1;
        const double diff = std::fabs(prev->timestamp - timestamp);
        if (diff < best_diff)
        {
            best = prev;
            best_diff = diff;
        }
    }

    if (best == frames.end() || best_diff > tolerance)
        return false;

    out_frame = *best;
    return true;
}
} // namespace

class RgbdDatasetDenseNode : public rclcpp::Node
{
public:
    explicit RgbdDatasetDenseNode(const CliOptions& cli_options)
        : Node("orb_slam3_rgbd_dataset_dense_node")
    {
        const std::string default_vocabulary =
            "/home/ricky/WCR_ws/dense_orbslam3/Vocabulary/ORBvoc.txt";
        const std::string default_settings =
            "/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/TUM1.yaml";

        vocabulary_ = declare_parameter<std::string>(
            "vocabulary", cli_options.vocabulary.empty() ? default_vocabulary : cli_options.vocabulary);
        settings_ = declare_parameter<std::string>(
            "settings", cli_options.settings.empty() ? default_settings : cli_options.settings);
        sequence_path_ = declare_parameter<std::string>("sequence_path", cli_options.sequence_path);
        association_file_ = declare_parameter<std::string>("association_file", cli_options.association_file);
        root_dir_ = declare_parameter<std::string>("root_dir", cli_options.root_dir);
        use_viewer_ = declare_parameter<bool>("use_viewer", cli_options.use_viewer);
        dense_build_mode_ = declare_parameter<std::string>(
            "dense_build_mode", cli_options.dense_mode.empty() ? "realtime" : cli_options.dense_mode);
        dense_resolution_ = declare_parameter<double>("dense_resolution", 0.01);
        dense_mean_k_ = declare_parameter<double>("dense_mean_k", 50.0);
        dense_std_thresh_ = declare_parameter<double>("dense_std_thresh", 2.0);
        dense_unit_ = declare_parameter<double>("dense_unit", 5000.0);
        dataset_fps_ = declare_parameter<double>("dataset_fps", 30.0);
        playback_speed_ = declare_parameter<double>("playback_speed", 1.0);
        max_frames_ = declare_parameter<int>("max_frames", -1);
        keep_alive_after_finish_ = declare_parameter<bool>("keep_alive_after_finish", true);
        dense_topic_ = declare_parameter<std::string>("dense_pointcloud_topic", "/orb_slam3/dense_points");
        publish_period_ms_ = declare_parameter<int>("dense_publish_period_ms", 500);
        deferred_timestamp_tolerance_ =
            declare_parameter<double>("deferred_timestamp_tolerance", 1e-4);

        if (dense_build_mode_ != "realtime" && dense_build_mode_ != "deferred" &&
            dense_build_mode_ != "both")
        {
            throw std::runtime_error("dense_build_mode must be realtime, deferred, or both");
        }

        dense_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            dense_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

        const auto timer_period =
            std::chrono::milliseconds(std::max(50, publish_period_ms_));
        dense_publish_timer_ = create_wall_timer(
            timer_period, std::bind(&RgbdDatasetDenseNode::PublishDenseCloud, this));

        RCLCPP_INFO(get_logger(), "Vocabulary: %s", vocabulary_.c_str());
        RCLCPP_INFO(get_logger(), "Settings: %s", settings_.c_str());
        RCLCPP_INFO(get_logger(), "Sequence path: %s", sequence_path_.c_str());
        RCLCPP_INFO(get_logger(), "Association file: %s", association_file_.c_str());
        RCLCPP_INFO(get_logger(), "Root dir: %s", root_dir_.c_str());
        RCLCPP_INFO(get_logger(), "Use viewer: %s", use_viewer_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "Dense mode: %s", dense_build_mode_.c_str());
        RCLCPP_INFO(get_logger(), "Dense topic: %s", dense_topic_.c_str());

        playback_thread_ = std::thread(&RgbdDatasetDenseNode::PlaybackDataset, this);
    }

    ~RgbdDatasetDenseNode() override
    {
        stop_requested_.store(true);
        if (playback_thread_.joinable())
            playback_thread_.join();

        ShutdownSlam();
        ShutdownDenseMapper();
    }

private:
    bool RealtimeDenseEnabled() const
    {
        return dense_build_mode_ == "realtime" || dense_build_mode_ == "both";
    }

    bool DeferredDenseEnabled() const
    {
        return dense_build_mode_ == "deferred" || dense_build_mode_ == "both";
    }

    void PlaybackDataset()
    {
        try
        {
            LoadDataset();
            if (frames_.empty())
                throw std::runtime_error("dataset contains no RGB-D frames");

            slam_ = std::make_unique<ORB_SLAM3::System>(
                vocabulary_, settings_, ORB_SLAM3::System::RGBD, use_viewer_);

            if (RealtimeDenseEnabled())
                ResetDenseMapper();

            std::unordered_set<unsigned long> inserted_keyframe_ids;

            RCLCPP_INFO(get_logger(), "Loaded %zu RGB-D frames. Starting playback.", frames_.size());
            const auto playback_start = std::chrono::steady_clock::now();

            for (std::size_t i = 0; rclcpp::ok() && !stop_requested_.load() && i < frames_.size(); ++i)
            {
                if (max_frames_ > 0 && static_cast<int>(i) >= max_frames_)
                    break;

                const DatasetFrame& frame = frames_[i];
                cv::Mat rgb = cv::imread(frame.rgb_path, cv::IMREAD_UNCHANGED);
                cv::Mat depth = cv::imread(frame.depth_path, cv::IMREAD_UNCHANGED);
                if (rgb.empty() || depth.empty())
                {
                    RCLCPP_WARN(get_logger(), "Skip frame %zu. Failed to read %s / %s",
                                i, frame.rgb_path.c_str(), frame.depth_path.c_str());
                    continue;
                }

                const auto track_start = std::chrono::steady_clock::now();
                slam_->TrackRGBD(rgb, depth, frame.timestamp);
                const auto track_end = std::chrono::steady_clock::now();

                if (RealtimeDenseEnabled())
                    InsertLatestKeyFrameForDense(rgb, depth, inserted_keyframe_ids);

                if (i == 0 || i % 50 == 0)
                {
                    const double elapsed =
                        std::chrono::duration<double>(track_end - playback_start).count();
                    RCLCPP_INFO(get_logger(), "Dataset progress: %zu/%zu, elapsed %.2f s",
                                i, frames_.size(), elapsed);
                }

                SleepForDatasetTiming(i, track_start, track_end);
            }

            RCLCPP_INFO(get_logger(), "Dataset playback finished.");

            if (DeferredDenseEnabled())
                BuildDeferredDenseMapFromAtlas();

            if (!keep_alive_after_finish_)
            {
                ShutdownSlam();
                ShutdownDenseMapper();
                rclcpp::shutdown();
            }
            else
            {
                RCLCPP_INFO(get_logger(),
                            "Keeping node alive for Pangolin/RViz inspection. Press Ctrl-C to exit.");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Dataset dense node failed: %s", e.what());
            rclcpp::shutdown();
        }
    }

    void LoadDataset()
    {
        if (!sequence_path_.empty() && !association_file_.empty())
        {
            frames_ = LoadTumAssociation(sequence_path_, association_file_);
        }
        else if (!root_dir_.empty())
        {
            frames_ = LoadFolderDataset(root_dir_, dataset_fps_);
        }
        else
        {
            throw std::runtime_error(
                "set sequence_path + association_file, or set root_dir containing rgb/depth folders");
        }

        std::sort(frames_.begin(), frames_.end(),
                  [](const DatasetFrame& a, const DatasetFrame& b) {
                      return a.timestamp < b.timestamp;
                  });
    }

    void InsertLatestKeyFrameForDense(cv::Mat& rgb,
                                      cv::Mat& depth,
                                      std::unordered_set<unsigned long>& inserted_keyframe_ids)
    {
        if (!slam_)
            return;

        ORB_SLAM3::Tracking* tracker = slam_->GetTracker();
        if (!tracker)
            return;

        ORB_SLAM3::KeyFrame* keyframe = tracker->GetLastKeyFrame();
        if (!keyframe || keyframe->isBad())
            return;

        if (!inserted_keyframe_ids.insert(keyframe->mnId).second)
            return;

        std::lock_guard<std::mutex> lock(dense_mapper_mutex_);
        if (dense_mapper_)
            dense_mapper_->insertKeyFrame(keyframe, rgb, depth);
    }

    void BuildDeferredDenseMapFromAtlas()
    {
        if (!slam_)
            return;

        RCLCPP_INFO(get_logger(), "Shutting down ORB-SLAM3 before deferred dense Atlas pass.");
        ShutdownSlam();

        std::vector<ORB_SLAM3::KeyFrame*> keyframes = slam_->GetAtlas()->GetAllKeyFrames();
        std::sort(keyframes.begin(), keyframes.end(), ORB_SLAM3::KeyFrame::lId);

        ResetDenseMapper();

        std::size_t enqueued = 0;
        std::size_t skipped_bad = 0;
        std::size_t skipped_missing = 0;

        RCLCPP_INFO(get_logger(), "Deferred dense pass over %zu keyframes.", keyframes.size());

        for (ORB_SLAM3::KeyFrame* keyframe : keyframes)
        {
            if (!keyframe || keyframe->isBad())
            {
                ++skipped_bad;
                continue;
            }

            DatasetFrame frame;
            if (!FindFrameForTimestamp(frames_, keyframe->mTimeStamp,
                                       deferred_timestamp_tolerance_, frame))
            {
                ++skipped_missing;
                continue;
            }

            cv::Mat rgb = cv::imread(frame.rgb_path, cv::IMREAD_UNCHANGED);
            cv::Mat depth = cv::imread(frame.depth_path, cv::IMREAD_UNCHANGED);
            if (rgb.empty() || depth.empty())
            {
                ++skipped_missing;
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(dense_mapper_mutex_);
                if (dense_mapper_)
                    dense_mapper_->insertKeyFrame(keyframe, rgb, depth);
            }

            ++enqueued;
            if (enqueued % 10 == 0)
                RCLCPP_INFO(get_logger(), "Deferred dense enqueued %zu/%zu keyframes",
                            enqueued, keyframes.size());
        }

        RCLCPP_INFO(get_logger(),
                    "Deferred dense enqueue done: enqueued=%zu, skipped_bad=%zu, skipped_missing=%zu",
                    enqueued, skipped_bad, skipped_missing);

        std::lock_guard<std::mutex> lock(dense_mapper_mutex_);
        if (dense_mapper_)
            dense_mapper_->shutdown();
    }

    void SleepForDatasetTiming(std::size_t index,
                               std::chrono::steady_clock::time_point track_start,
                               std::chrono::steady_clock::time_point track_end)
    {
        if (playback_speed_ <= 0.0)
            return;

        double frame_dt = 1.0 / std::max(dataset_fps_, 1.0);
        if (index + 1 < frames_.size())
        {
            const double dataset_dt = frames_[index + 1].timestamp - frames_[index].timestamp;
            if (dataset_dt > 0.0 && dataset_dt < 10.0)
                frame_dt = dataset_dt;
        }

        const double target_dt = frame_dt / playback_speed_;
        const double track_dt = std::chrono::duration<double>(track_end - track_start).count();
        if (track_dt < target_dt)
        {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(target_dt - track_dt));
        }
    }

    void PublishDenseCloud()
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_cloud;
        {
            std::lock_guard<std::mutex> lock(dense_mapper_mutex_);
            if (!dense_mapper_)
                return;
            dense_cloud = dense_mapper_->getGlobalPointCloud();
        }

        if (!dense_cloud || dense_cloud->empty() || dense_cloud->width == 0)
            return;

        sensor_msgs::msg::PointCloud2 cloud_msg = PCLToROS(dense_cloud, "map");
        dense_pub_->publish(cloud_msg);
    }

    sensor_msgs::msg::PointCloud2 PCLToROS(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_cloud,
        const std::string& frame_id)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = now();
        cloud_msg.header.frame_id = frame_id;

        cloud_msg.height = 1;
        cloud_msg.width = static_cast<std::uint32_t>(pcl_cloud->points.size());
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
        for (std::size_t i = 0; i < pcl_cloud->points.size(); ++i)
        {
            const auto& point = pcl_cloud->points[i];
            std::uint8_t* dst = cloud_msg.data.data() + i * cloud_msg.point_step;

            const std::uint32_t rgb_packed =
                (static_cast<std::uint32_t>(point.r) << 16) |
                (static_cast<std::uint32_t>(point.g) << 8) |
                static_cast<std::uint32_t>(point.b);
            float rgb_float = 0.0f;
            std::memcpy(&rgb_float, &rgb_packed, sizeof(rgb_float));

            std::memcpy(dst + 0, &point.x, sizeof(float));
            std::memcpy(dst + 4, &point.y, sizeof(float));
            std::memcpy(dst + 8, &point.z, sizeof(float));
            std::memcpy(dst + 12, &rgb_float, sizeof(float));
        }

        return cloud_msg;
    }

    void ResetDenseMapper()
    {
        std::lock_guard<std::mutex> lock(dense_mapper_mutex_);
        if (dense_mapper_)
            dense_mapper_->shutdown();

        dense_mapper_ = std::make_unique<ORB_SLAM3::PointCloudMappingRGBD>(
            dense_resolution_, dense_mean_k_, dense_std_thresh_, dense_unit_);
    }

    void ShutdownDenseMapper()
    {
        std::lock_guard<std::mutex> lock(dense_mapper_mutex_);
        if (dense_mapper_)
        {
            dense_mapper_->shutdown();
            dense_mapper_.reset();
        }
    }

    void ShutdownSlam()
    {
        if (slam_ && !slam_shutdown_)
        {
            slam_->Shutdown();
            slam_shutdown_ = true;
        }
    }

    std::string vocabulary_;
    std::string settings_;
    std::string sequence_path_;
    std::string association_file_;
    std::string root_dir_;
    std::string dense_build_mode_;
    std::string dense_topic_;

    bool use_viewer_ = true;
    bool keep_alive_after_finish_ = true;
    double dense_resolution_ = 0.01;
    double dense_mean_k_ = 50.0;
    double dense_std_thresh_ = 2.0;
    double dense_unit_ = 5000.0;
    double dataset_fps_ = 30.0;
    double playback_speed_ = 1.0;
    double deferred_timestamp_tolerance_ = 1e-4;
    int max_frames_ = -1;
    int publish_period_ms_ = 500;

    std::vector<DatasetFrame> frames_;
    std::unique_ptr<ORB_SLAM3::System> slam_;
    bool slam_shutdown_ = false;

    std::unique_ptr<ORB_SLAM3::PointCloudMappingRGBD> dense_mapper_;
    std::mutex dense_mapper_mutex_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dense_pub_;
    rclcpp::TimerBase::SharedPtr dense_publish_timer_;
    std::thread playback_thread_;
    std::atomic<bool> stop_requested_{false};
};

int main(int argc, char** argv)
{
    CliOptions cli_options;
    try
    {
        cli_options = ParseCliOptions(argc, argv);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Argument error: " << e.what() << std::endl;
        PrintUsage();
        return 1;
    }

    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<RgbdDatasetDenseNode>(cli_options);
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rgbd_dataset_dense_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
