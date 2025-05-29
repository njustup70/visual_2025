#include <costmap_ultra/obstacle_layer.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

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

    auto node = node_.lock();
    if (!node)
    {
        throw std::runtime_error{"Failed to lock node"};
    }

    node->get_parameter(name_ + "." + "combination_method", combination_method_);
    node->get_parameter("transform_tolerance", transform_tolerance);
    node->get_parameter(name_ + "." + "observation_sources", topics_string);

    // now we need to split the topics based on whitespace which we can use a stringstream for
    std::stringstream ss(topics_string);

    std::string source;
    while (ss >> source)
    {
        // get the parameters for the specific topic
        double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
        std::string topic, sensor_frame, data_type;
        bool inf_is_valid, clearing, marking;

        declareParameter(source + "." + "topic", rclcpp::ParameterValue(source));
        declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
        declareParameter(source + "." + "obstacle_max_range", rclcpp::ParameterValue(2.5));
        declareParameter(source + "." + "obstacle_min_range", rclcpp::ParameterValue(0.0));

        node->get_parameter(name_ + "." + source + "." + "topic", topic);
        node->get_parameter(
            name_ + "." + source + "." + "expected_update_rate",
            expected_update_rate);
        // get the obstacle range for the sensor
        double obstacle_max_range, obstacle_min_range;
        node->get_parameter(name_ + "." + source + "." + "obstacle_max_range", obstacle_max_range);
        node->get_parameter(name_ + "." + source + "." + "obstacle_min_range", obstacle_min_range);
    }
}

} // namespace nav2_costmap_2d