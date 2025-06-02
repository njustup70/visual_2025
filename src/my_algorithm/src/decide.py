#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import random
import time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import Header

class RandomPointGenerator:
    """独立的地图处理和随机点生成模块"""
    def __init__(self):
        self.costmap_data = None
        self.map_info_printed = False  # 新增标志位，确保只打印一次
    
    def update_costmap(self, msg):
        """更新代价地图数据"""
        self.costmap_data = msg
        
        # 首次收到地图时打印信息
        if not self.map_info_printed and self.costmap_data:
            self.print_map_info()
            self.map_info_printed = True
    
    def print_map_info(self):
        """打印代价地图大小和物理边界信息"""
        info = self.costmap_data.info
        width = info.width
        height = info.height
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        
        # 计算物理边界
        min_x = origin_x
        max_x = origin_x + width * resolution
        min_y = origin_y
        max_y = origin_y + height * resolution
        
        # 计算四角坐标（栅格中心点）
        corners = {
            "左下角": (min_x + 0.5 * resolution, min_y + 0.5 * resolution),
            "右下角": (max_x - 0.5 * resolution, min_y + 0.5 * resolution),
            "左上角": (min_x + 0.5 * resolution, max_y - 0.5 * resolution),
            "右上角": (max_x - 0.5 * resolution, max_y - 0.5 * resolution)
        }
        
        # 打印地图信息
        print("\n" + "="*60)
        print(f"📐 收到代价地图: {width}×{height} 栅格 (分辨率: {resolution:.3f} m/栅格)")
        print(f"📍 物理边界范围:")
        print(f"   X: [{min_x:.3f}, {max_x:.3f}]")
        print(f"   Y: [{min_y:.3f}, {max_y:.3f}]")
        print("🗺️ 四角坐标 (栅格中心点):")
        for corner, (x, y) in corners.items():
            print(f"   {corner}: ({x:.3f}, {y:.3f})")
        print("="*60 + "\n")
    
    def is_traversable(self, x, y):
        """判断栅格是否可通行"""
        if not self.costmap_data:
            return False
        
        width = self.costmap_data.info.width
        index = y * width + x
        
        if index < 0 or index >= len(self.costmap_data.data):
            return False
        
        cost = self.costmap_data.data[index]
        return cost <= 50

    def generate_random_goal(self):
        """生成随机可达目标点"""
        if not self.costmap_data:
            return None
        
        width = self.costmap_data.info.width
        height = self.costmap_data.info.height
        traversable_cells = []
        
        # 遍历所有栅格，筛选可通行点
        for y in range(height):
            for x in range(width):
                if self.is_traversable(x, y) and self.costmap_data.data[y * width + x] < 10:
                    traversable_cells.append((x, y))
        
        if not traversable_cells:
            return None
            
        goal_x, goal_y = random.choice(traversable_cells)
        resolution = self.costmap_data.info.resolution
        origin_x = self.costmap_data.info.origin.position.x
        origin_y = self.costmap_data.info.origin.position.y
        world_x = origin_x + (goal_x + 0.5) * resolution
        world_y = origin_y + (goal_y + 0.5) * resolution
        
        return Point(x=world_x, y=world_y)

class NavigationHandler:
    """导航处理和状态管理模块"""
    IDLE = 0
    NAVIGATING = 1
    
    def __init__(self, node):
        self.node = node
        self.current_state = self.IDLE
        self.current_goal_handle = None
        self.goal_timeout = 60.0
        self.last_goal_time = 0.0
        self.failure_count = 0
        self.max_failures = 3
        self.current_goal = None
        self.last_publish_time = 0.0
        
        # 创建Action客户端
        self.nav_client = ActionClient(
            self.node, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        # 发布导航目标
        self.goal_publisher = self.node.create_publisher(
            PoseStamped,
            '/goal_pose',
            10)
    
    def publish_goal(self, point):
        """发布导航目标（添加5秒间隔控制）"""
        # 检查是否满足5秒间隔要求
        current_time = time.time()
        if current_time - self.last_publish_time < 5.0:
            wait_time = 5.0 - (current_time - self.last_publish_time)
            self.node.get_logger().info(f"等待 {wait_time:.2f} 秒后发布目标...")
            time.sleep(wait_time)
        
        goal_msg = PoseStamped()
        goal_msg.header = Header(
            stamp=self.node.get_clock().now().to_msg(),
            frame_id="map"
        )
        goal_msg.pose.position = point
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_publisher.publish(goal_msg)
        self.last_publish_time = time.time()
        
        # 仅在新目标生成时打印详细信息
        if self.failure_count >= self.max_failures:
            self.node.get_logger().info(f"⚠️ 连续失败{self.max_failures}次，生成新目标点")
            self.node.get_logger().info(f"📍 新目标坐标: x={point.x:.2f}, y={point.y:.2f}")
        
        # 发送导航目标
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(
            nav_goal, 
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """处理目标响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.handle_failure()
            return
            
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """处理导航反馈（检查超时）"""
        current_time = time.time()
        if current_time - self.last_goal_time > self.goal_timeout:
            self.cancel_navigation()
    
    def nav_result_callback(self, future):
        """处理导航结果"""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('✅ 导航成功')
            self.failure_count = 0
            self.reset_state()
        else:
            self.handle_failure()
    
    def handle_failure(self):
        """统一处理导航失败情况"""
        self.failure_count += 1
        
        if self.failure_count < self.max_failures:
            self.node.get_logger().info(f'导航失败，当前连续失败次数: {self.failure_count}/3')
            # 重新发布同一目标点
            self.publish_goal(self.current_goal)
        else:
            self.node.get_logger().info(f'⚠️ 连续失败{self.max_failures}次，重新生成目标点')
            self.failure_count = 0
            # 关键修复：重置状态为空闲
            self.reset_state()
    
    def cancel_navigation(self):
        """取消当前导航"""
        if self.current_goal_handle:
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)
    
    def cancel_done_callback(self, future):
        """取消操作完成回调"""
        self.handle_failure()
    
    def reset_state(self):
        """重置状态为空闲"""
        self.current_state = self.IDLE
        self.current_goal_handle = None
    
    def set_current_goal(self, goal):
        """设置当前目标点"""
        self.current_goal = goal
        self.last_goal_time = time.time()
        self.current_state = self.NAVIGATING

class RandomGoalGenerator(Node):
    """主节点类，协调随机点生成和导航处理"""
    def __init__(self):
        super().__init__('random_goal_generator')
        
        # 初始化模块
        self.point_generator = RandomPointGenerator()
        self.navigation_handler = NavigationHandler(self)
        
        # 订阅局部代价地图
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10)
        
        self.get_logger().info("随机目标生成节点已启动，等待代价地图...")

    def costmap_callback(self, msg):
        """处理代价地图回调"""
        # 更新点生成器的地图数据
        self.point_generator.update_costmap(msg)
        
        # 仅当空闲状态时生成新目标
        if self.navigation_handler.current_state == NavigationHandler.IDLE:
            goal_point = self.point_generator.generate_random_goal()
            if goal_point:
                self.navigation_handler.set_current_goal(goal_point)
                self.navigation_handler.publish_goal(goal_point)

def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.navigation_handler.cancel_navigation()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()