#include "pcl_icp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
PclIcp::PclIcp(const rclcpp::NodeOptions &options)
    : PclHandle(options), _MapPclPipeLine(_global_ptr)
{

    std::string map_config_path = std::filesystem::path(__FILE__).parent_path().string() + "/../config/map.json";
    map_config_path = std::filesystem::canonical(std::filesystem::path(map_config_path)).string();
    this->declare_parameter("map_config_path", map_config_path);
    this->get_parameter("map_config_path", map_config_path);
    fmt::print("map_config_path is {}\n", map_config_path);

    fileToJSON(map_config_path, _map_init_json);
    auto map_pipe_line = _map_init_json["map_pipe_line"][0].get<std::vector<std::string>>();
    for (auto &name : map_pipe_line)
    {
        _MapPclPipeLine.createHandle(name, _map_init_json);
    }
    _content["stamp"] = rclcpp::Clock().now().nanoseconds() + rclcpp::Clock().now().seconds() * 1e9;
    // 创建一个初始transform
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    std::vector<float> transform_vec(transform.data(), transform.data() + 16);
    _content["running"]["icp_transform"] = transform_vec;
    std::shared_ptr<PCL_Handle::PointCloud_t> PointCloud_ptr(new PCL_Handle::PointCloud_t);
    _MapPclPipeLine.handlePCL(PointCloud_ptr, _map_init_json);
    _content["running"].update(_map_init_json["running"], true);

    // 创建一个定时器
    _timer = this->create_wall_timer(std::chrono::milliseconds(20), [this]()
                                     { _MapPclPipeLine.spinOnce(_map_init_json); });
}
RCLCPP_COMPONENTS_REGISTER_NODE(PclIcp)