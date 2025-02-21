/**
 * @file pcl_filter.hpp
 * @author Elaina (1463967532@qq.com)
 * @brief ros2处理点云数据节点,一般不修改
 * @version 0.1
 * @date 2024-12-04
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef PCL_HANDLE_HPP
#define PCL_HANDLE_HPP

#include "json.hpp"
#include "pcl_factory.hpp"
#include <any>
#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>

typedef sensor_msgs::msg::PointCloud2 PointCloud2Msg_t;

class PclHandle : public rclcpp::Node
{
public:
    explicit PclHandle(const rclcpp::NodeOptions &options);

protected:
    void fileToJSON(const std::string &file_path, nlohmann::json &json_in);
    void subscriptionCallback(const PointCloud2Msg_t::SharedPtr msg);
    void timerCallback();
    std::shared_ptr<pcl::PointCloud<PCL_Handle::PointXYZITR>> pointToPointXYZITR(const pcl::PointCloud<PCL_Handle::Point_t>::Ptr &point_cloud_ptr);
    nlohmann::json _content;
    PCL_Handle::PclFactory_t _PclPipeLine;
    std::shared_ptr<rclcpp::TimerBase> _spin_timer_ptr;
    std::shared_ptr<rclcpp::TimerBase> _Timer_ptr;
    std::shared_ptr<rclcpp::Subscription<PointCloud2Msg_t>> _Subscription_ptr;
    std::shared_ptr<rclcpp::Publisher<PointCloud2Msg_t>> _Publisher_ptr;
    std::unordered_map<std::string, std::any> _global_ptr;
};

#endif // PCL_HANDLE_HPP
