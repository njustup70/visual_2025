#include "pcl_handle.hpp"
#include <filesystem>
#include <fmt/core.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>

PclHandle::PclHandle(const rclcpp::NodeOptions &options)
    : Node("pcl_handle_node", options), _PclPipeLine(_global_ptr)
{
    std::string path = std::filesystem::path(__FILE__).parent_path().string() + "/../config/icp.json";
    path = std::filesystem::canonical(std::filesystem::path(path)).string();
    fmt::print("config path is {}\n", path);
    this->declare_parameter("pub_xyzitr", false);
    this->declare_parameter("config_path", path);
    this->declare_parameter("pipeline_id", 0);
    this->declare_parameter("subscribe_topic", "/livox/lidar/raw");
    this->declare_parameter("publish_topic", "/pointcloud2_filtered");
    this->declare_parameter("rate", 0);
    this->declare_parameter("param_debug", true);
    this->declare_parameter("print_time", true);
    if (get_parameter("param_debug").as_bool())
    {
        const auto milliseconds = std::chrono::milliseconds(1000);
        _Timer_ptr = create_wall_timer(milliseconds, [this]()
                                       { auto file_path = get_parameter("config_path").as_string();
                                         fileToJSON(file_path, _content); });
    }

    _spin_timer_ptr = create_wall_timer(std::chrono::milliseconds(10), [this]()
                                        { _PclPipeLine.spinOnce(_content); });

    _Subscription_ptr = create_subscription<PointCloud2Msg_t>(get_parameter("subscribe_topic").as_string(), 10, [this](PointCloud2Msg_t::SharedPtr msg)
                                                              { subscriptionCallback(msg); });
    _Publisher_ptr = create_publisher<PointCloud2Msg_t>(get_parameter("publish_topic").as_string(), 10);

    fileToJSON(get_parameter("config_path").as_string(), _content);
    int pipeline_id = get_parameter("pipeline_id").as_int();
    auto pcl_pipe_line = _content["pcl_pipe_line"][pipeline_id].get<std::vector<std::string>>();
    for (auto &name : pcl_pipe_line)
    {
        _PclPipeLine.createHandle(name, _content);
    }

    rclcpp::Node *RosNode_ptr = this;
    _PclPipeLine.addGlobalPtr("RosNode", RosNode_ptr);

    _PclPipeLine.enablePrintTime(get_parameter("print_time").as_bool());
}

void PclHandle::fileToJSON(const std::string &file_path, nlohmann::json &json_in)
{
    try
    {
        std::ifstream file(file_path);
        if (!file.is_open())
        {
            throw std::runtime_error("无法打开文件: " + file_path);
        }
        nlohmann::json temp_json;
        file >> temp_json;
        json_in.update(temp_json);
        file.close();
    }
    catch (const std::exception &e)
    {
        fmt::print("fileToJSON error: {}\n", e.what());
    }
}

void PclHandle::subscriptionCallback(const PointCloud2Msg_t::SharedPtr msg)
{
    std::shared_ptr<PCL_Handle::PointCloud_t> PointCloud_ptr(new PCL_Handle::PointCloud_t);
    nlohmann::json temp_content = _content;
    pcl::fromROSMsg(*msg, *PointCloud_ptr);
    if (PointCloud_ptr->empty())
    {
        return;
    }
    temp_content["stamp"] = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    temp_content["frame_id"] = msg->header.frame_id;
    _PclPipeLine.handlePCL(PointCloud_ptr, temp_content);

    PointCloud2Msg_t msg_out;
    if (get_parameter("pub_xyzitr").as_bool())
    {
        auto PointCloud_ptr_xyzitr = pointToPointXYZITR(PointCloud_ptr);
        pcl::toROSMsg(*PointCloud_ptr_xyzitr, msg_out);
    }
    else
    {
        pcl::toROSMsg(*PointCloud_ptr, msg_out);
    }
    msg_out.header = msg->header;
    _Publisher_ptr->publish(msg_out);
}

std::shared_ptr<pcl::PointCloud<PCL_Handle::PointXYZITR>> PclHandle::pointToPointXYZITR(const pcl::PointCloud<PCL_Handle::Point_t>::Ptr &point_cloud_ptr)
{
    std::shared_ptr<pcl::PointCloud<PCL_Handle::PointXYZITR>> point_cloud_ptr_xyzitr(new pcl::PointCloud<PCL_Handle::PointXYZITR>);
    point_cloud_ptr_xyzitr->points.resize(point_cloud_ptr->points.size());
    for (size_t i = 0; i < point_cloud_ptr->points.size(); ++i)
    {
        point_cloud_ptr_xyzitr->points[i].x = point_cloud_ptr->points[i].x;
        point_cloud_ptr_xyzitr->points[i].y = point_cloud_ptr->points[i].y;
        point_cloud_ptr_xyzitr->points[i].z = point_cloud_ptr->points[i].z;
        point_cloud_ptr_xyzitr->points[i].intensity = point_cloud_ptr->points[i].intensity;
        point_cloud_ptr_xyzitr->points[i].ring = 0;
        point_cloud_ptr_xyzitr->points[i].time = 0;
    }
    return point_cloud_ptr_xyzitr;
}

RCLCPP_COMPONENTS_REGISTER_NODE(PclHandle)