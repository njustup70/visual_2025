#include <geometry_msgs/msg/pose_array.hpp>
#include <iomanip>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp> // 修改为 OccupancyGrid
#include <rclcpp/rclcpp.hpp>

// 定义障碍物阈值
static constexpr int8_t OBSTACLE_THRESHOLD = 65; // 通常 >65 视为障碍物

class ObstacleExtractor : public rclcpp::Node
{
public:
    ObstacleExtractor()
        : Node("obstacle_extractor")
    {
        // 1. 动态参数声明
        this->declare_parameter("costmap_topic", "/local_costmap/costmap");
        update_parameters();

        // 2. 初始化订阅器
        recreate_subscription();

        // 3. 障碍点发布器
        obstacle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/global_obstacles", 10);

        // 4. 注册参数回调
        param_callback_ = this->add_on_set_parameters_callback(
            std::bind(&ObstacleExtractor::param_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "节点已启动，监听话题: %s", costmap_topic_.c_str());
    }

private:
    void update_parameters()
    {
        costmap_topic_ = this->get_parameter("costmap_topic").as_string();
    }

    void recreate_subscription()
    {
        // 修改为订阅 OccupancyGrid 类型
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_, 10,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
            {
                costmap_callback(msg);
            });
    }

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        geometry_msgs::msg::PoseArray obstacles;
        obstacles.header = msg->header; // 继承全局坐标系

        const auto &data = msg->data;
        const double res = msg->info.resolution; // 修改字段访问
        const double ox = msg->info.origin.position.x;
        const double oy = msg->info.origin.position.y;
        const uint32_t width = msg->info.width;
        const uint32_t height = msg->info.height;

        // 预分配内存优化
        obstacles.poses.reserve(data.size() / 20);

        // 单次遍历优化
        for (uint32_t idx = 0; idx < data.size(); ++idx)
        {
            // 修改障碍物检测逻辑：值 > OBSTACLE_THRESHOLD 视为障碍物
            if (data[idx] > OBSTACLE_THRESHOLD && data[idx] != -1)
            {
                // 计算栅格坐标
                const uint32_t i = idx / width;
                const uint32_t j = idx % width;

                geometry_msgs::msg::Pose p;
                p.position.x = ox + (j + 0.5) * res;
                p.position.y = oy + (i + 0.5) * res;
                obstacles.poses.push_back(p);
            }
        }

        // 打印障碍点信息
        print_obstacle_info(obstacles, res, width, height);

        // 发布前检查避免空消息
        if (!obstacles.poses.empty())
        {
            obstacle_pub_->publish(obstacles);
            RCLCPP_INFO(this->get_logger(), "发布 %ld 个障碍点", obstacles.poses.size());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "未检测到障碍物");
        }
    }

    // 打印障碍点详细信息
    void print_obstacle_info(const geometry_msgs::msg::PoseArray &obstacles,
                             double resolution, uint32_t width, uint32_t height)
    {
        RCLCPP_INFO(this->get_logger(), "==============================");
        RCLCPP_INFO(this->get_logger(), "检测到 %ld 个障碍点", obstacles.poses.size());
        RCLCPP_INFO(this->get_logger(), "代价地图尺寸: %u x %u (%.2f m x %.2f m)",
                    width, height, width * resolution, height * resolution);
        RCLCPP_INFO(this->get_logger(), "分辨率: %.3f m", resolution);

        // 打印前5个障碍点位置
        size_t print_count = std::min(static_cast<size_t>(5), obstacles.poses.size());
        for (size_t i = 0; i < print_count; ++i)
        {
            const auto &pose = obstacles.poses[i];
            RCLCPP_INFO(this->get_logger(), "障碍点 %zu: (%.2f, %.2f)",
                        i + 1, pose.position.x, pose.position.y);
        }

        if (obstacles.poses.size() > print_count)
        {
            RCLCPP_INFO(this->get_logger(), "... 还有 %ld 个障碍点未显示",
                        obstacles.poses.size() - print_count);
        }

        RCLCPP_INFO(this->get_logger(), "==============================");
    }

    // 动态参数回调
    rcl_interfaces::msg::SetParametersResult param_callback(
        const std::vector<rclcpp::Parameter> &params)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto &param : params)
        {
            if (param.get_name() == "costmap_topic")
            {
                costmap_topic_ = param.as_string();
                recreate_subscription();
                RCLCPP_INFO(this->get_logger(), "切换话题至: %s", costmap_topic_.c_str());
            }
        }
        return result;
    }

    // 成员变量
    std::string costmap_topic_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_; // 修改类型
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleExtractor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}