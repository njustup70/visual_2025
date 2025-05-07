#ifndef NAV2_COSTMAP_2D__OBSTACLE_LAYER_ULTRA_HPP_
#define NAV2_COSTMAP_2D__OBSTACLE_LAYER_ULTRA_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_costmap_2d
{

class SimpleLaserLayer : public CostmapLayer
{

public:
    SimpleLaserLayer();
    virtual ~SimpleLaserLayer();

    // Layer API Implementation
    virtual void onInitialize() override;
    virtual void updateBounds(
        double robot_x, double robot_y, double robot_yaw,
        double *min_x, double *min_y, double *max_x, double *max_y) override;
    virtual void updateCosts(
        Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) override;
    virtual void reset() override;
    virtual void onFootprintChanged() override;
    virtual bool isClearable() override
    {
        return true;
    }

protected:
    // ROS2 Components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Configuration Parameters
    bool enabled_;
    bool footprint_clearing_enabled_;
    double min_obstacle_height_;
    double max_obstacle_height_;
    int combination_method_;
    std::string global_frame_;
    std::string sensor_frame_;
    double transform_tolerance_;
    double cost_map_width_;
    // Data Processing
    void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
    bool getSensorPose(tf2::Transform &sensor_tf) const;

    // Dynamic Parameters
    rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

} // namespace nav2_costmap_2d

#endif // NAV2_COSTMAP_2D__OBSTACLE_LAYER_ULTRA_HPP_