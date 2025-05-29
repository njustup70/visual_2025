#include <costmap_ultra/obstacle_layer.hpp>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <algorithm>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::ObstacleLayerUltra, nav2_costmap_2d::Layer)

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_costmap_2d
{

ObstacleLayerUltra::~ObstacleLayerUltra()
{
}

void ObstacleLayerUltra::onInitialize()
{
    bool track_unknown_space;
    double transform_tolerance;

    // The topics that we'll subscribe to from the parameter server
    std::string topics_string;

    declareParameter("combination_method", rclcpp::ParameterValue(1));
    declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));
    declareParameter("map_frame", rclcpp::ParameterValue(std::string("map")));
    auto node = node_.lock();
    if (!node)
    {
        throw std::runtime_error{"Failed to lock node"};
    }

    node->get_parameter(name_ + "." + "combination_method", combination_method_);
    node->get_parameter("transform_tolerance", transform_tolerance);
    node->get_parameter(name_ + "." + "observation_sources", topics_string);
    node->get_parameter(name_ + "." + "map_frame", map_frame_);
    // now we need to split the topics based on whitespace which we can use a stringstream for
    std::stringstream ss(topics_string);

    std::string source;
    while (ss >> source)
    {
        // get the parameters for the specific topic
        double expected_update_rate;
        std::string topic, sensor_frame, data_type;

        declareParameter(source + "." + "topic", rclcpp::ParameterValue(source));
        declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
        declareParameter(source + "." + "obstacle_max_range", rclcpp::ParameterValue(2.5));
        declareParameter(source + "." + "obstacle_min_range", rclcpp::ParameterValue(0.0));

        node->get_parameter(name_ + "." + source + "." + "topic", topic);
        node->get_parameter(
            name_ + "." + source + "." + "expected_update_rate",
            expected_update_rate);
        // get the obstacle range for the sensor
        node->get_parameter(name_ + "." + source + "." + "obstacle_max_range", _obstacle_max_range);
        node->get_parameter(name_ + "." + source + "." + "obstacle_min_range", _obstacle_min_range);
    }
    // 创建tf2_ros::Buffer对象
    // tf_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_ = new tf2_ros::Buffer(node->get_clock());
    // 创建tf2_ros::TransformListener对象
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    // 创建订阅者
}
void ObstacleLayerUltra::laserScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan)
{
    auto laser_frame = scan->header.frame_id;
    cloud_ptr_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    float current_angle = scan->angle_min;
    for (auto &point : scan->ranges)
    {
        if (point < scan->range_min || point > scan->range_max || std::isnan(point) || std::isinf(point))
        {
            continue; // Skip invalid points
        }
        current_angle += scan->angle_increment;
        if (point < _obstacle_min_range || point > _obstacle_max_range)
        {
            continue; // Skip points outside the specified range
        }
        cloud_ptr_->points.emplace_back(pcl::PointXYZ{point * std::cos(current_angle),
                                                      point * std::sin(current_angle), 0.0});
    }
}
void ObstacleLayerUltra::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
{
}

void ObstacleLayerUltra::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                     int min_i, int min_j, int max_i, int max_j)
{
    switch (combination_method_)
    {
    case 0: // Overwrite
        updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
        break;
    case 1: // Maximum
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        break;
    default: // Nothing
        break;
    }
}
void ObstacleLayerUltra::reset()
{
    // Reset the costmap to free space
    std::fill(costmap_, costmap_ + getSizeInCellsX() * getSizeInCellsY(), FREE_SPACE);

} // namespace nav2_costmap_2d
} // namespace nav2_costmap_2d