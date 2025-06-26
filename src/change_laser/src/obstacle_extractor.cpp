#include <geometry_msgs/msg/pose_array.hpp>
#include <iomanip>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

static constexpr int8_t OBSTACLE_THRESHOLD = 65; // 障碍物阈值

class ObstacleExtractor : public rclcpp::Node
{
public:
    ObstacleExtractor()
        : Node("obstacle_extractor"), clear_count_(0)
    {
        // 声明动态参数（含区域过滤参数）
        this->declare_parameter("costmap_topic", "/local_costmap/costmap");
        this->declare_parameter("region_origin_x", 0.0);    // 区域原点X
        this->declare_parameter("region_origin_y", 0.0);    // 区域原点Y
        this->declare_parameter("region_width", 1e6);       // 区域宽度（默认覆盖全图）
        this->declare_parameter("region_height", 1e6);      // 区域高度（默认覆盖全图）
        update_parameters();

        // 初始化订阅器和发布器
        recreate_subscription();
        obstacle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/global_obstacles", 10);

        // 注册参数回调
        param_callback_ = this->add_on_set_parameters_callback(
            std::bind(&ObstacleExtractor::param_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "节点已启动，监听话题: %s", costmap_topic_.c_str());
    }

private:
    void update_parameters()
    {
        costmap_topic_ = this->get_parameter("costmap_topic").as_string();
        region_origin_x_ = this->get_parameter("region_origin_x").as_double();
        region_origin_y_ = this->get_parameter("region_origin_y").as_double();
        region_width_ = this->get_parameter("region_width").as_double();
        region_height_ = this->get_parameter("region_height").as_double();
    }

    void recreate_subscription()
    {
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_, 10,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                costmap_callback(msg);
            });
    }

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        geometry_msgs::msg::PoseArray obstacles;
        obstacles.header = msg->header;

        const auto &data = msg->data;
        const double res = msg->info.resolution;
        const double ox = msg->info.origin.position.x;
        const double oy = msg->info.origin.position.y;
        const uint32_t width = msg->info.width;
        const uint32_t height = msg->info.height;

        // 计算矩形边界[6](@ref)
        const double region_x_min = region_origin_x_;
        const double region_x_max = region_origin_x_ + region_width_;
        const double region_y_min = region_origin_y_;
        const double region_y_max = region_origin_y_ + region_height_;

        bool has_obstacles = false;
        for (uint32_t idx = 0; idx < data.size(); ++idx)
        {
            if (data[idx] > OBSTACLE_THRESHOLD && data[idx] != -1)
            {
                const uint32_t i = idx / width; // 行索引
                const uint32_t j = idx % width; // 列索引

                // 计算地图坐标系下的坐标[2](@ref)
                const double x = ox + (j + 0.5) * res;
                const double y = oy + (i + 0.5) * res;

                // 区域过滤检查
                if (x >= region_x_min && x <= region_x_max && 
                    y >= region_y_min && y <= region_y_max)
                {
                    geometry_msgs::msg::Pose p;
                    p.position.x = x;
                    p.position.y = y;
                    obstacles.poses.push_back(p);
                    has_obstacles = true;
                }
            }
        }

        // 障碍物计数与发布逻辑
        if (has_obstacles) {
            clear_count_ = 0;
            obstacle_pub_->publish(obstacles);
            RCLCPP_INFO(this->get_logger(), "发布 %ld 个障碍点 | 区域: (%.1f,%.1f)~%.1fx%.1f", 
                        obstacles.poses.size(), region_x_min, region_y_min, 
                        region_width_, region_height_);
            print_obstacle_info(obstacles, res, width, height);
        } else {
            if (++clear_count_ >= 5) {
                geometry_msgs::msg::PoseArray empty_obstacles;
                empty_obstacles.header.stamp = this->now();
                empty_obstacles.header.frame_id = msg->header.frame_id;
                obstacle_pub_->publish(empty_obstacles);
                RCLCPP_WARN(this->get_logger(), "连续5次未检测到障碍物，已清空障碍点");
                clear_count_ = 0;
            } else {
                RCLCPP_WARN(this->get_logger(), "未检测到障碍物 (%d/5)", clear_count_);
            }
        }
    }

    void print_obstacle_info(const geometry_msgs::msg::PoseArray &obstacles,
                             double resolution, uint32_t width, uint32_t height)
    {
        RCLCPP_DEBUG(this->get_logger(), "==============================");
        RCLCPP_DEBUG(this->get_logger(), "检测到 %ld 个障碍点", obstacles.poses.size());
        RCLCPP_DEBUG(this->get_logger(), "代价地图尺寸: %u x %u (%.2f m x %.2f m)",
                    width, height, width * resolution, height * resolution);
        
        // 仅当障碍点较少时打印详细信息
        if (obstacles.poses.size() <= 20) {
            for (size_t i = 0; i < obstacles.poses.size(); ++i) {
                const auto &pose = obstacles.poses[i];
                RCLCPP_DEBUG(this->get_logger(), "障碍点 %zu: (%.2f, %.2f)",
                            i + 1, pose.position.x, pose.position.y);
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "==============================");
    }

    // 参数回调函数（支持动态更新区域）[6](@ref)
    rcl_interfaces::msg::SetParametersResult param_callback(
        const std::vector<rclcpp::Parameter> &params)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto &param : params) {
            if (param.get_name() == "costmap_topic") {
                costmap_topic_ = param.as_string();
                recreate_subscription();
                RCLCPP_INFO(this->get_logger(), "切换话题至: %s", costmap_topic_.c_str());
            }
            else if (param.get_name() == "region_origin_x") {
                region_origin_x_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "更新区域原点X: %.2f", region_origin_x_);
            }
            else if (param.get_name() == "region_origin_y") {
                region_origin_y_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "更新区域原点Y: %.2f", region_origin_y_);
            }
            else if (param.get_name() == "region_width") {
                region_width_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "更新区域宽度: %.2f", region_width_);
            }
            else if (param.get_name() == "region_height") {
                region_height_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "更新区域高度: %.2f", region_height_);
            }
        }
        return result;
    }

    // 成员变量
    std::string costmap_topic_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
    int clear_count_;
    
    // 区域过滤参数
    double region_origin_x_;
    double region_origin_y_;
    double region_width_;
    double region_height_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleExtractor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}