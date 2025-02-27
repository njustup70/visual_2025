#ifndef __PCL_IMPLMENTATION_ROS_HPP
#define __PCL_IMPLMENTATION_ROS_HPP
#include "pcl_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
using namespace PCL_Handle;
class PclPublish_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PclPublish";
    }

private:
    rclcpp::Node *_Node_ptr = nullptr;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> _Publisher_ptr;
};
#endif