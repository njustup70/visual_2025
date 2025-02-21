#include "pcl_implementation_ros.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
using namespace PCL_Handle;
void PclPublish_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    (void)PointInput_ptr;
    auto contents = searchParam("settings", content);
    if (_Node_ptr == nullptr)
    {
        _Node_ptr = std::any_cast<rclcpp::Node *>((_global_ptr)->at("RosNode"));
    }
    auto names = content["running"]["pointclouds"];
    for (auto &single_content : contents)
    {
        auto name = single_content["name"].get<std::string>();

        if (std::find(names.begin(), names.end(), name) != names.end())
        {
            auto PointCloud_ptr = std::any_cast<std::shared_ptr<PointCloud_t>>((*_global_ptr)[name]);
            auto target_transform = single_content["target_transform"].get<std::string>();
            auto topic_name = single_content["topic_name"].get<std::string>();
            if (target_transform == "")
            {
                target_transform = content["frame_id"].get<std::string>();
            }
            if (_Publisher_ptr.find(name) == _Publisher_ptr.end())
            {
                // _Publisher_ptr[name] = rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(name, 10);
                _Publisher_ptr[name] = _Node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10);
            }
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*PointCloud_ptr, msg);
            msg.header.stamp = rclcpp::Time(content["stamp"].get<int64_t>());
            msg.header.frame_id = target_transform;
            _Publisher_ptr[name]->publish(msg);
        }
        else
        {
            fmt::print("not found {}\n", name);
        }
    }
}