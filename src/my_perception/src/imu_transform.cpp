#include <Eigen/Dense>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

class ImuTransform : public rclcpp::Node
{
public:
    ImuTransform(const rclcpp::NodeOptions &options)
        : Node("imu_transform_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "IMU 转换节点已启动");
        // 读取 YAML 配置
        std::string yaml_file = "config/imu_transform.yaml";
        this->declare_parameter("imu_topic", "/imu_raw");
        this->declare_parameter("imu_transformed_topic", "/imu_lidar_frame");
        this->declare_parameter("imu_frame", "imu_link");
        this->declare_parameter("lidar_frame", "lidar_link");
        this->declare_parameter("Calibration_file", yaml_file);
        this->declare_parameter("pub_tf", true);
        this->declare_parameter("use_transform", true);
        this->declare_parameter("use_grivaty2m", false);
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        imu_transformed_topic_ = this->get_parameter("imu_transformed_topic").as_string();
        imu_frame_ = this->get_parameter("imu_frame").as_string();
        lidar_frame_ = this->get_parameter("lidar_frame").as_string();
        // 订阅 IMU 话题
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            { imuCallback(msg); });

        // 发布转换后的 IMU 话题
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_transformed_topic_, 10);
        parseYaml(get_parameter("Calibration_file").as_string());
        // 发布 TF 变换
        if (this->get_parameter("pub_tf").as_bool())
        {
            static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            publishStaticTF();
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    const double _gravity = 9.81;
    std::string imu_topic_, imu_transformed_topic_, imu_frame_, lidar_frame_;
    Eigen::Matrix4d imu_to_lidar_;

    void parseYaml(const std::string &file_path)
    {
        std::ifstream file(file_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开 YAML 文件: %s", file_path.c_str());
            rclcpp::shutdown();
        }

        Eigen::Matrix4d lidar_to_imu;
        std::string line;
        bool matrix_started = false;
        int row = 0;

        while (std::getline(file, line))
        {
            // 识别矩阵部分
            if (line.find("Homogeneous Transformation Matrix from LiDAR to IMU") != std::string::npos)
            {
                matrix_started = true;
                row = 0;
                continue;
            }

            if (matrix_started)
            {
                std::istringstream iss(line);
                double val;
                int col = 0;

                while (iss >> val)
                {
                    lidar_to_imu(row, col) = val;
                    col++;
                }

                row++;
                if (row >= 4)
                    break; // 只解析 4 行
            }
        }

        file.close();

        // 计算 IMU 到 LiDAR 变换矩阵（逆矩阵）
        imu_to_lidar_ = lidar_to_imu.inverse();
        RCLCPP_INFO(this->get_logger(), "IMU to LiDAR 变换矩阵计算完成");
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        sensor_msgs::msg::Imu transformed_msg = *msg;
        if (get_parameter("use_grivaty2m").as_bool())
        {
            transformed_msg.linear_acceleration.z *= _gravity;
            transformed_msg.linear_acceleration.x *= _gravity;
            transformed_msg.linear_acceleration.y *= _gravity;
        }
        if (get_parameter("use_transform").as_bool())
        {
            Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            Eigen::Vector3d acc_transformed = imu_to_lidar_.block<3, 3>(0, 0) * acc;
            transformed_msg.linear_acceleration.x = acc_transformed.x();
            transformed_msg.linear_acceleration.y = acc_transformed.y();
            transformed_msg.linear_acceleration.z = acc_transformed.z();

            // 旋转 IMU 角速度数据
            Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            Eigen::Vector3d gyro_transformed = imu_to_lidar_.block<3, 3>(0, 0) * gyro;
            transformed_msg.angular_velocity.x = gyro_transformed.x();
            transformed_msg.angular_velocity.y = gyro_transformed.y();
            transformed_msg.angular_velocity.z = gyro_transformed.z();
        }
        // 旋转 IMU 加速度数据

        // 修改坐标系
        transformed_msg.header.frame_id = lidar_frame_;
        transformed_msg.header.stamp = msg->header.stamp;

        imu_pub_->publish(transformed_msg);
    }

    void publishStaticTF()
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = imu_frame_;
        transformStamped.child_frame_id = lidar_frame_;

        transformStamped.transform.translation.x = imu_to_lidar_(0, 3);
        transformStamped.transform.translation.y = imu_to_lidar_(1, 3);
        transformStamped.transform.translation.z = imu_to_lidar_(2, 3);

        Eigen::Matrix3d rotation_matrix = imu_to_lidar_.block<3, 3>(0, 0);
        Eigen::Quaterniond q(rotation_matrix);

        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        static_broadcaster_->sendTransform(transformStamped);
    }
};

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ImuTransformNode>());
//     rclcpp::shutdown();
//     return 0;
// }
RCLCPP_COMPONENTS_REGISTER_NODE(ImuTransform)