#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header

class RandomGoalGenerator(Node):
    def __init__(self):
        super().__init__('random_goal_generator')
        
        # 订阅全局代价地图
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',  # 全局代价地图话题
            self.costmap_callback,
            10)
        self.subscription  # 防止未使用变量警告
        
        # 发布导航目标
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',  # Navigation2默认目标话题
            10)
        
        self.costmap_data = None
        self.get_logger().info("随机目标生成节点已启动，等待代价地图...")

    def costmap_callback(self, msg):
        """处理代价地图回调"""
        self.costmap_data = msg
        self.get_logger().info("接收到代价地图，尺寸: {}x{}".format(
            msg.info.width, msg.info.height))
        
        # 生成随机目标点
        goal_point = self.generate_random_goal()
        if goal_point:
            self.publish_goal(goal_point)

    def is_traversable(self, x, y):
        """判断栅格是否可通行"""
        if not self.costmap_data:
            return False
        
        width = self.costmap_data.info.width
        index = y * width + x
        
        # 检查索引是否有效
        if index < 0 or index >= len(self.costmap_data.data):
            return False
        
        cost = self.costmap_data.data[index]
        
        # 判定逻辑：代价值≤50视为可通行区域
        # 可根据实际需求调整阈值
        return cost <= 50

    def generate_random_goal(self):
        """生成随机可达目标点"""
        if not self.costmap_data:
            self.get_logger().warn("尚未接收到代价地图数据！")
            return None
        
        width = self.costmap_data.info.width
        height = self.costmap_data.info.height
        traversable_cells = []
        
        # 遍历所有栅格，筛选可通行点
        # 添加安全边界：只选择代价值很低的区域（<10）
        for y in range(height):
            for x in range(width):
                if self.is_traversable(x, y) and self.costmap_data.data[y * width + x] < 10:
                    traversable_cells.append((x, y))
        
        if not traversable_cells:
            self.get_logger().error("未找到可通行区域！")
            return None
        
        # 随机选择可达点
        goal_x, goal_y = random.choice(traversable_cells)
        self.get_logger().info("随机选择栅格坐标: ({}, {})".format(goal_x, goal_y))
        
        # 栅格坐标 → 世界坐标
        resolution = self.costmap_data.info.resolution
        origin_x = self.costmap_data.info.origin.position.x
        origin_y = self.costmap_data.info.origin.position.y
        
        # 取栅格中心点
        world_x = origin_x + (goal_x + 0.5) * resolution
        world_y = origin_y + (goal_y + 0.5) * resolution
        
        self.get_logger().info("转换后的世界坐标: ({:.2f}, {:.2f})".format(world_x, world_y))
        return Point(x=world_x, y=world_y)

    def publish_goal(self, point):
        """发布导航目标"""
        goal_msg = PoseStamped()
        goal_msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id="map"  # 必须与costmap的frame_id一致
        )
        goal_msg.pose.position = point
        goal_msg.pose.orientation.w = 1.0  # 默认朝向
        
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info("已发布新导航目标: x={:.2f}, y={:.2f}".format(
            point.x, point.y))

def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()