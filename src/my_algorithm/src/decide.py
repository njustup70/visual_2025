#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import random
import math
import time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, PoseArray, Pose
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import Header
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, ParameterType

class RandomPointGenerator:
    """独立的地图处理和随机点生成模块"""
    def __init__(self, node): 
        self.node = node
        self.costmap_data = None
        self.map_info_printed = False
        self.candidate_points = []  # 存储候选点
        
        # 声明动态参数
        self.node.declare_parameter('map_x', 7.0)  # 地图X尺寸
        self.node.declare_parameter('map_y', -14.0)  # 地图Y尺寸
        self.node.declare_parameter('origin_x', 0.0)  # 原点X坐标
        self.node.declare_parameter('origin_y', 0.0)  # 原点Y坐标
        self.node.declare_parameter('center_x', 3.5)  # 圆心X坐标
        self.node.declare_parameter('center_y', -14.0)  # 圆心Y坐标
        self.node.declare_parameter('radius_min', 3.0)  # 最小半径
        self.node.declare_parameter('radius_max', 4.0)  # 最大半径
        self.node.declare_parameter('num_points', 36)  # 候选点数量
        
        # 新增：发布模式参数 (fixed/dynamic)
        self.node.declare_parameter(
            'publish_mode', 'fixed',
            ParameterDescriptor(
                description='候选点发布模式: fixed=固定一组点, dynamic=持续生成新点',
                type=ParameterType.PARAMETER_STRING,
                read_only=False,
                additional_constraints="Allowed values: ['fixed', 'dynamic']"
            )
        )
        
        # 新增：持续发布参数
        self.node.declare_parameter(
            'continuous_publish', True,
            ParameterDescriptor(
                description='是否持续发布候选点',
                type=ParameterType.PARAMETER_BOOL
            )
        )
        
        # 新增：发布频率参数 (Hz)
        self.node.declare_parameter(
            'publish_frequency', 1.0,
            ParameterDescriptor(
                description='候选点发布频率 (Hz)',
                type=ParameterType.PARAMETER_DOUBLE
            )
        )
        
        # 创建候选点发布器
        self.points_pub = self.node.create_publisher(
            PoseArray,
            '/points_select',
            10
        )
        
        # 添加参数回调
        self.param_callback = self.node.add_on_set_parameters_callback(
            self.param_callback_handler
        )
        
        # 创建定时器用于持续发布
        self.create_publish_timer()
    
    def create_publish_timer(self):
        """创建或更新发布定时器"""
        # 如果已有定时器，先取消
        if hasattr(self, 'publish_timer'):
            self.publish_timer.cancel()
        
        # 获取发布频率参数
        frequency = self.node.get_parameter('publish_frequency').value
        if frequency <= 0:
            frequency = 1.0  # 默认1Hz
        
        # 创建新定时器
        self.publish_timer = self.node.create_timer(
            1.0 / frequency,  # 秒
            self.publish_candidate_points
        )
    
    def param_callback_handler(self, params):
        """处理参数更新"""
        for param in params:
            param_name = param.name
            # 当圆心、半径或发布模式变化时重新生成点
            if param_name in ['center_x', 'center_y', 'radius_min', 'radius_max', 'num_points', 'publish_mode']:
                self.node.get_logger().info(
                    f"参数更新: {param_name} = {param.value}"
                )
                if self.costmap_data:
                    self.generate_candidate_points()
            
            # 当发布频率变化时更新定时器
            elif param_name == 'publish_frequency':
                self.create_publish_timer()
            
            # 当持续发布设置变化时
            elif param_name == 'continuous_publish':
                if param.value:
                    self.create_publish_timer()
                elif hasattr(self, 'publish_timer'):
                    self.publish_timer.cancel()
        
        return SetParametersResult(successful=True)
    
    def update_costmap(self, msg):
        """更新代价地图数据"""
        self.costmap_data = msg
        
        # 首次收到地图时打印信息
        if not self.map_info_printed and self.costmap_data:
            self.print_map_info()
            self.map_info_printed = True
            # 生成候选点
            self.generate_candidate_points()
        
        # 动态模式下每次地图更新都生成新点
        elif self.node.get_parameter('publish_mode').value == 'dynamic':
            self.generate_candidate_points()
    
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

    def generate_candidate_points(self):
        """在圆环区域内生成均匀分布的候选点"""
        # 获取动态参数值
        center_x = self.node.get_parameter('center_x').value
        center_y = self.node.get_parameter('center_y').value
        radius_min = self.node.get_parameter('radius_min').value
        radius_max = self.node.get_parameter('radius_max').value
        num_points = self.node.get_parameter('num_points').value
        
        # 清空候选点列表
        self.candidate_points = []
        
        # 在圆环区域内均匀生成点
        for i in range(num_points):
            # 计算角度（均匀分布）
            angle = 2 * math.pi * i / num_points
            
            # 在半径范围内随机选择半径
            radius = random.uniform(radius_min, radius_max)
            
            # 计算点的坐标
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            # 添加到候选点列表
            self.candidate_points.append((x, y))
        
        # 发布候选点
        self.publish_candidate_points()
    
    def publish_candidate_points(self):
        """发布候选点到/points_select话题"""
        if not self.candidate_points:
            return
            
        pose_array = PoseArray()
        pose_array.header = Header(
            stamp=self.node.get_clock().now().to_msg(),
            frame_id="map"
        )
        
        for point in self.candidate_points:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = 0.0
            pose.orientation.w = 1.0  # 无旋转
            pose_array.poses.append(pose)
        
        self.points_pub.publish(pose_array)
        
        # 获取当前发布模式
        publish_mode = self.node.get_parameter('publish_mode').value
        mode_info = "固定" if publish_mode == 'fixed' else "动态"
        
        # 仅在调试时记录日志，避免频繁输出
        if self.node.get_clock().now().nanoseconds % 10 == 0:  # 每10次发布记录一次
            self.node.get_logger().info(
                f"发布 {len(self.candidate_points)} 个候选点到 /points_select ({mode_info}模式)"
            )
    
    def generate_random_goal(self):
        """从候选点中随机选择可达目标点"""
        if not self.costmap_data or not self.candidate_points:
            return None
        
        # 打乱候选点顺序
        shuffled_points = self.candidate_points.copy()
        random.shuffle(shuffled_points)
        
        # 查找第一个可通行的点
        for point in shuffled_points:
            # 将世界坐标转换为栅格坐标
            resolution = self.costmap_data.info.resolution
            origin_x = self.costmap_data.info.origin.position.x
            origin_y = self.costmap_data.info.origin.position.y
            
            grid_x = int((point[0] - origin_x) / resolution)
            grid_y = int((point[1] - origin_y) / resolution)
            
            # 检查是否可通行
            if self.is_traversable(grid_x, grid_y):
                return Point(x=point[0], y=point[1])
        
        return None

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
        
        # 声明参数并设置默认值
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')  # 新增代价地图话题参数
        
        # 获取参数值
        costmap_topic = self.get_parameter('costmap_topic').value
        
        self.get_logger().info(f"使用代价地图话题: {costmap_topic}")
        
        # 初始化模块 - 修复：传递当前节点实例
        self.point_generator = RandomPointGenerator(self)  # 关键修复
        self.navigation_handler = NavigationHandler(self)
        
        # 订阅局部代价地图（使用参数化的话题名称）
        self.subscription = self.create_subscription(
            OccupancyGrid,
            costmap_topic,  # 使用参数化的代价地图话题
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