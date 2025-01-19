#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <thread>
#include <csignal>

void on_exit([[maybe_unused]] int sig) {
    RCUTILS_LOG_INFO("Exit by Ctrl+C");
    rclcpp::shutdown();
    exit(0);
}

class RosbagPlayer : public rclcpp::Node {
public:
    RosbagPlayer(const rclcpp::NodeOptions & options)
        : Node("rosbag_player_node", options) {
        this->declare_parameter<std::string>("rosbag_file", "");
        this->get_parameter("rosbag_file", rosbag_file);

        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10);
        signal(SIGINT, on_exit);

        reader_.open(rosbag_file);
        processing_thread_ = std::make_shared<std::thread>(&RosbagPlayer::play_bag, this);
    }

    ~RosbagPlayer() {
        if (processing_thread_ && processing_thread_->joinable()) {
            processing_thread_->join();
        }
    }

private:
    void play_bag() {
        while (rclcpp::ok()) {
            if (!reader_.has_next()) {
                reader_.open(rosbag_file);
            }

            auto start_time = std::chrono::high_resolution_clock::now();
            auto bag_message = reader_.read_next();
            auto ros_time = rclcpp::Clock().now();

            if (bag_message->topic_name == "/livox/lidar") {
                auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg, pointcloud_msg.get());
                pointcloud_msg->header.stamp = ros_time;
                pointcloud_publisher_->publish(*pointcloud_msg);
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            if ((duration < 100) && (duration > 1)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100) - std::chrono::milliseconds(duration));
            }
        }

        RCLCPP_INFO(this->get_logger(), "No more messages in the bag.");
        rclcpp::shutdown();
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rosbag2_cpp::Reader reader_;
    std::shared_ptr<std::thread> processing_thread_;
    std::string rosbag_file;
};

RCLCPP_COMPONENTS_REGISTER_NODE(RosbagPlayer)
