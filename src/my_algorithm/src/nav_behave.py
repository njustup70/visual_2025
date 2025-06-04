#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy  # 新增QoS导入
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class EnhancedNavigationHandler:
    """增强版导航处理模块 - 支持动态目标点跟踪"""
    IDLE = 0          # 空闲状态，等待新目标
    NAVIGATING = 1    # 导航中状态
    RETRYING = 2      # 重试状态
    
    def __init__(self, node):
        self.node = node
        self.current_state = self.IDLE
        self.current_goal_handle = None
        self.goal_timeout = 60.0
        self.last_goal_time = 0.0
        self.failure_count = 0
        self.max_failures = 20  # 最大失败次数提高到20次
        self.active_goal = None  # 当前活跃目标点
        self.latest_optimal_point = None  # 存储最新接收到的优化点
        self.last_publish_time = 0.0
        
        # 创建Action客户端连接官方导航
        self.nav_client = ActionClient(
            self.node, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        # ==== 关键修复：使用兼容Nav2的QoS配置 ==== [6,7](@ref)
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL  # 匹配Nav2要求
        )
        
        # 发布导航目标到官方话题
        self.goal_publisher = self.node.create_publisher(
            PoseStamped,
            '/goal_pose',
            qos  # 应用自定义QoS
        )
            
        # 订阅优化点话题
        self.optimal_sub = self.node.create_subscription(
            Point,
            '/optimal_point',
            self.optimal_point_callback,
            10
        )
    
    def optimal_point_callback(self, msg):
        """处理优化点更新"""
        self.latest_optimal_point = msg
        self.node.get_logger().info(f"📡 收到新优化点: x={msg.x:.2f}, y={msg.y:.2f}")
        
        # 仅在空闲状态时立即处理新目标
        if self.current_state == self.IDLE:
            self.start_navigation(msg)
    
    def start_navigation(self, point):
        """启动新导航任务"""
        self.active_goal = point
        self.failure_count = 0
        self.set_current_goal(point)
        self.publish_goal(point)
        self.current_state = self.NAVIGATING
    
    def publish_goal(self, point):
        """发布导航目标（含5秒间隔控制）"""
        # 频率控制（避免频繁发布）
        current_time = time.time()
        if current_time - self.last_publish_time < 5.0:
            wait_time = 5.0 - (current_time - self.last_publish_time)
            self.node.get_logger().info(f"⏱️ 等待 {wait_time:.2f} 秒后发布目标...")
            time.sleep(wait_time)
        
        # 构造PoseStamped消息
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position = point
        goal_msg.pose.orientation.w = 1.0  # 默认朝向
        
        # 发布到官方导航话题
        self.goal_publisher.publish(goal_msg)
        self.last_publish_time = time.time()
        self.node.get_logger().info(f"📍 发布目标: x={point.x:.2f}, y={point.y:.2f}")
        
        # 通过Action发送导航请求
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg
        
        # 确保Action服务器可用
        if not self.nav_client.server_is_ready():
            self.node.get_logger().warn("⚠️ 导航服务器未就绪，等待...")
            self.nav_client.wait_for_server()
        
        # 发送目标并设置回调
        send_goal_future = self.nav_client.send_goal_async(
            nav_goal, 
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    # ... (其余方法保持不变，参考原始实现) ...

class OptimalGoalNavigator(Node):
    """最优目标导航节点"""
    def __init__(self):
        super().__init__('optimal_goal_navigator')
        self.navigation_handler = EnhancedNavigationHandler(self)
        self.get_logger().info("🚀 最优目标导航节点已启动")

def main(args=None):
    rclpy.init(args=args)
    node = OptimalGoalNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 节点被手动终止")
        node.navigation_handler.cancel_navigation()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()