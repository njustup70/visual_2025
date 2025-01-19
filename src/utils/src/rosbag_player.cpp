#include "fmt/core.h"
#include "yaml.h"
#include <csignal>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
void on_exit([[maybe_unused]] int sig)
{
    RCUTILS_LOG_INFO("Exit by Ctrl+C");
    rclcpp::shutdown();
    exit(0);
}

class RosbagPlayer : public rclcpp::Node
{
public:
    RosbagPlayer(const rclcpp::NodeOptions &options)
        : Node("rosbag_player_node", options)
    {
        // this->declare_parameter<std::string>("rosbag_file", "");
        this->declare_parameter("rosbag_root", "./");
        this->get_parameter("rosbag_root", _rosbag_root);
        // this->get_parameter("rosbag_file", rosbag_file);
        // 查找当前目录下带.db3后缀的文件
        std::filesystem::path rosbag_root_abs = std::filesystem::absolute(_rosbag_root);
        fmt::print("rosbag_root_abs: {}\n", rosbag_root_abs.string());
        for (const auto &entry : std::filesystem::directory_iterator(rosbag_root_abs))
        {
            if (entry.path().extension() == ".db3")
            {
                rosbag_file = entry.path().string();
                fmt::print("find rosbag file: {}\n", rosbag_file);
                break;
            }
        }
        if (rosbag_file.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No rosbag file found in %s", _rosbag_root.c_str());
            return;
        }
        // pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10);
        signal(SIGINT, on_exit);

        reader_.open(rosbag_file);
        processing_thread_ = std::make_shared<std::thread>(&RosbagPlayer::play_bag, this);
    }

    ~RosbagPlayer()
    {
        if (processing_thread_ && processing_thread_->joinable())
        {
            processing_thread_->join();
        }
    }

private:
    void getPlayYamlData()
    {
        auto yaml_file = _rosbag_root + "metadata.yaml";
        YAML::Node yaml_node = YAML::LoadFile(yaml_file);
        if (yaml_node["rosbag2_bagfile_information"]["topics_with_message_count"])
        {
            const auto &topics = yaml_node["rosbag2_bagfile_information"]["topics_with_message_count"];
            for (auto topic : topics)
            {
                if (topic["topic_metadata"]["type"].as<std::string>() == "sensor_msgs/ msg/PointCloud2")
                {
                    _pointcloud_topic_name = topic["topic_metadata"]["name"].as<std::string>();
                    fmt::print("find cloudpoint topic: {}\n", _pointcloud_topic_name);
                    break;
                }
                if (topic["topic_metadata"]["type"].as<std::string>() == "sensor_msgs/msg/Imu")
                {
                    _imu_topic_name = topic["topic_metadata"]["name"].as<std::string>();
                    fmt::print("find imu topic: {}\n", _imu_topic_name);
                    break;
                }
                if (topic["topic_metadata"]["type"].as<std::string>() == "tf2_msgs/msg/TFMessage")
                {
                    _tf_topic_name = topic["topic_metadata"]["name"].as<std::string>();
                    fmt::print("find tf topic: {}\n", _tf_topic_name);
                    break;
                }
            }
        }
    }
    void play_bag()
    {
        while (rclcpp::ok())
        {
            if (!reader_.has_next())
            {
                reader_.open(rosbag_file);
            }

            auto start_time = std::chrono::high_resolution_clock::now();
            auto bag_message = reader_.read_next();
            auto ros_time = rclcpp::Clock().now();

            if (bag_message->topic_name == _pointcloud_topic_name)
            {
                auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg, pointcloud_msg.get());
                pointcloud_msg->header.stamp = ros_time;
                pointcloud_publisher_->publish(*pointcloud_msg);
            }
            if (bag_message->topic_name == _imu_topic_name)
            {
                auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
                rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg, imu_msg.get());
                imu_msg->header.stamp = ros_time;
                imu_publisher_->publish(*imu_msg);
            }
            // if()
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            if ((duration < 100) && (duration > 1))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100) - std::chrono::milliseconds(duration));
            }
        }

        RCLCPP_INFO(this->get_logger(), "No more messages in the bag.");
        rclcpp::shutdown();
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rosbag2_cpp::Reader reader_;
    std::shared_ptr<std::thread> processing_thread_;
    std::string rosbag_file;
    std::string _rosbag_root;
    std::string _pointcloud_topic_name;
    std::string _imu_topic_name;
    std::string _tf_topic_name;
};

RCLCPP_COMPONENTS_REGISTER_NODE(RosbagPlayer)
