#include "simple_laser_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

using namespace std::chrono_literals;

namespace nav2_costmap_2d
{

SimpleLaserLayer::SimpleLaserLayer()
{
}
// : tf_buffer_(std::make_shared<tf2_ros::Buffer>(rclcpp::Clock::ROSClock())), tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {}

SimpleLaserLayer::~SimpleLaserLayer() = default;

void SimpleLaserLayer::onInitialize()
{
    auto node = node_.lock();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Parameter Declaration
    node->declare_parameter(name_ + ".enabled", true);
    node->declare_parameter(name_ + ".footprint_clearing_enabled", true);
    node->declare_parameter(name_ + ".min_obstacle_height", 0.0);
    node->declare_parameter(name_ + ".max_obstacle_height", 2.0);
    node->declare_parameter(name_ + ".combination_method", 1);
    node->declare_parameter(name_ + ".sensor_frame", "");
    node->declare_parameter(name_ + ".transform_tolerance", 0.1);
    node->declare_parameter(name_ + ".cost_map_width", 5.0);
    // Parameter Initialization
    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".footprint_clearing_enabled", footprint_clearing_enabled_);
    node->get_parameter(name_ + ".min_obstacle_height", min_obstacle_height_);
    node->get_parameter(name_ + ".max_obstacle_height", max_obstacle_height_);
    node->get_parameter(name_ + ".combination_method", combination_method_);
    node->get_parameter(name_ + ".sensor_frame", sensor_frame_);
    node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
    node->get_parameter(name_ + ".cost_map_width", cost_map_width_);
    global_frame_ = layered_costmap_->getGlobalFrameID();
    // rolling_window_ = layered_costmap_->isRolling();

    // Dynamic Parameters
    dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(&SimpleLaserLayer::dynamicParametersCallback, this, std::placeholders::_1));

    // LaserScan Subscription
    laser_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&SimpleLaserLayer::laserCallback, this, std::placeholders::_1));

    matchSize();
    current_ = true;
}

void SimpleLaserLayer::laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
    try
    {
        // Get Transform
        tf2::Transform sensor_to_global;
        if (!getSensorPose(sensor_to_global))
            return;
        // 清空costmap
        resetMaps();
        // Process Scan Data
        const double max_range = scan->range_max;
        const double min_range = scan->range_min;
        const double angle_min = scan->angle_min;
        const double angle_increment = scan->angle_increment;

        std::vector<geometry_msgs::msg::Point> points;
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            const double range = scan->ranges[i];
            if (std::isnan(range) || range < min_range || range > max_range)
                continue;

            const double angle = angle_min + i * angle_increment;
            tf2::Vector3 point(range * cos(angle), range * sin(angle), 0);
            point = sensor_to_global * point;

            geometry_msgs::msg::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = 0.0;
            points.push_back(p);
        }

        // Update Costmap
        std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());
        for (const auto &p : points)
        {
            unsigned int mx, my;
            if (worldToMap(p.x, p.y, mx, my))
            {
                setCost(mx, my, LETHAL_OBSTACLE);
            }
        }
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(logger_, "TF Exception: %s", ex.what());
    }
}

bool SimpleLaserLayer::getSensorPose(tf2::Transform &transform) const
{
    try
    {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
            global_frame_, sensor_frame_, tf2::TimePointZero,
            tf2::durationFromSec(transform_tolerance_));
        tf2::fromMsg(tf.transform, transform);
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(logger_, "Failed to get transform: %s", ex.what());
        return false;
    }
}

void SimpleLaserLayer::updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double *min_x, double *min_y, double *max_x, double *max_y)
{
    if (!enabled_)
        return;

    std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());
    // 从机器人坐标系推出全局坐标系
    *min_x = robot_x - cost_map_width_ / 2;
    *min_y = robot_y - cost_map_width_ / 2;
    *max_x = robot_x + cost_map_width_ / 2;
    *max_y = robot_y + cost_map_width_ / 2;

    // *min_x = std::min(*min_x, min_x_);
    // *min_y = std::min(*min_y, min_y_);
    // *max_x = std::max(*max_x, max_x_);
    // *max_y = std::max(*max_y, max_y_);

    // // Reset update area
    // min_x_ = max_x_ = robot_x;
    // min_y_ = max_y_ = robot_y;
}

void SimpleLaserLayer::updateCosts(
    Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_)
        return;

    std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());
    switch (combination_method_)
    {
    case 0: // Overwrite
        updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
        break;
    case 1: // Maximum
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        break;
    default:
        RCLCPP_WARN(logger_, "Unknown combination method: %d", combination_method_);
        break;
    }
}

rcl_interfaces::msg::SetParametersResult
SimpleLaserLayer::dynamicParametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());
    for (const auto &param : parameters)
    {
        const auto param_name = param.get_name().substr(name_.size() + 1);

        if (param_name == "enabled")
        {
            enabled_ = param.as_bool();
        }
        else if (param_name == "min_obstacle_height")
        {
            min_obstacle_height_ = param.as_double();
        }
        else if (param_name == "max_obstacle_height")
        {
            max_obstacle_height_ = param.as_double();
        }
        else if (param_name == "combination_method")
        {
            combination_method_ = param.as_int();
        }
    }
    return result;
}

void SimpleLaserLayer::reset()
{
    std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());
    resetMaps();
}

void SimpleLaserLayer::onFootprintChanged()
{
    // Implement footprint change handling if needed
}

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SimpleLaserLayer, nav2_costmap_2d::Layer)

} // namespace nav2_costmap_2d