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
#include <tf2_msgs/msg/tf_message.hpp>
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
        // this->declare_parameter<std::string>("_rosbag_file", "");
        this->declare_parameter("rosbag_root", "./");
        this->get_parameter("rosbag_root", _rosbag_root);
        // this->get_parameter("_rosbag_file", _rosbag_file);
        // 查找当前目录下带.db3后缀的文件
        std::filesystem::path rosbag_root_abs = std::filesystem::absolute(_rosbag_root);
        fmt::print("rosbag_root_abs: {}\n", rosbag_root_abs.string());
        // 绝对路径+文件名
        auto yaml_file_path = rosbag_root_abs.string() + "/metadata.yaml";
        for (const auto &entry : std::filesystem::directory_iterator(rosbag_root_abs))
        {
            if (entry.path().extension() == ".db3")
            {
                _rosbag_file = entry.path().string();
                fmt::print("find rosbag file: {}\n", _rosbag_file);
                break;
            }
        }
        if (_rosbag_file.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No rosbag file found in %s", _rosbag_root.c_str());
            return;
        }
        // _pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10);
        signal(SIGINT, on_exit);

        _reader.open(_rosbag_file);
        getPlayYamlData(yaml_file_path);
        _processing_thread = std::make_shared<std::thread>(&RosbagPlayer::play_bag, this);
    }

    ~RosbagPlayer()
    {
        if (_processing_thread && _processing_thread->joinable())
        {
            _processing_thread->join();
        }
    }

private:
    void getPlayYamlData(std::string yaml_file_path)
    {
        // auto yaml_file = _rosbag_root + "metadata.yaml";
        YAML::Node yaml_node = YAML::LoadFile(yaml_file_path);
        if (yaml_node["rosbag2_bagfile_information"]["topics_with_message_count"])
        {

            const auto &topics = yaml_node["rosbag2_bagfile_information"]["topics_with_message_count"];
            for (auto topic : topics)
            {
                auto topic_type = topic["topic_metadata"]["type"].as<std::string>();
                if (topic_type == "sensor_msgs/msg/PointCloud2")
                {
                    auto pointcloud_topic_name = topic["topic_metadata"]["name"].as<std::string>();
                    fmt::print("find cloudpoint topic: {}\n", _pointcloud_topic_name);
                    _pointcloud_publishers[pointcloud_topic_name] = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_name, 10);
                    // _topic_publish_map[pointcloud_topic_name] = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_name, 10);
                }
                else if (topic_type == "sensor_msgs/msg/Imu")
                {
                    _imu_topic_name = topic["topic_metadata"]["name"].as<std::string>();
                    fmt::print("find imu topic: {}\n", _imu_topic_name);
                    _imu_publishers[_imu_topic_name] = this->create_publisher<sensor_msgs::msg::Imu>(_imu_topic_name, 10);
                    // _topic_publish_map[_imu_topic_name] = this->create_publisher<sensor_msgs::msg::Imu>(_imu_topic_name, 10);
                }
                else if (topic_type == "tf2_msgs/msg/TFMessage")
                {
                    _tf_topic_name = topic["topic_metadata"]["name"].as<std::string>();
                    fmt::print("find tf topic: {}\n", _tf_topic_name);
                    _tf_publishers[_tf_topic_name] = this->create_publisher<tf2_msgs::msg::TFMessage>(_tf_topic_name, 10);
                    // _topic_publish_map[_tf_topic_name] = this->create_publisher<tf2_msgs::msg::TFMessage>(_tf_topic_name, 10);
                }
            }
        }
    }
    void play_bag()
    {
        while (rclcpp::ok())
        {
            if (!_reader.has_next())
            {
                _reader.open(_rosbag_file);
            }

            auto start_time = std::chrono::high_resolution_clock::now();
            auto bag_message = _reader.read_next();
            auto ros_time = rclcpp::Clock().now();
            auto topic_name = bag_message->topic_name;
            if (_pointcloud_publishers.count(topic_name) > 0)
            {
                auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg, pointcloud_msg.get());
                pointcloud_msg->header.stamp = ros_time;
                // _pointcloud_publisher->publish(*pointcloud_msg);
                _pointcloud_publishers[topic_name]->publish(*pointcloud_msg);
            }
            if (_tf_publishers.count(topic_name) > 0)
            {
                auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
                rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg, tf_msg.get());
                for (auto &tf : tf_msg->transforms)
                {
                    tf.header.stamp = ros_time;
                }
                // _tf_publisher->publish(*tf_msg);
                _tf_publishers[_tf_topic_name]->publish(*tf_msg);
            }
            if (_imu_publishers.count(topic_name) > 0)
            {
                auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
                rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg, imu_msg.get());
                imu_msg->header.stamp = ros_time;
                // _imu_publisher->publish(*imu_msg);
                _imu_publishers[_imu_topic_name]->publish(*imu_msg);
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

    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> _pointcloud_publishers;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> _imu_publishers;
    std::unordered_map<std::string, rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr> _tf_publishers;
    rosbag2_cpp::Reader _reader;
    std::shared_ptr<std::thread> _processing_thread;
    std::string _rosbag_file;
    std::string _rosbag_root;
    std::string _pointcloud_topic_name;
    std::string _imu_topic_name;
    std::string _tf_topic_name;
    // 存话题名称与对应publish的映射
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> _topic_publish_map;
};

RCLCPP_COMPONENTS_REGISTER_NODE(RosbagPlayer)
