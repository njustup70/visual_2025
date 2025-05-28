import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rclpy.parameter import Parameter
import math

def yaw_to_quaternion(yaw):
    """将 yaw (弧度) 转换为四元数 (绕 Z 轴旋转)"""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )

class NavToPointNode(Node):
    def __init__(self):
        super().__init__('auto_nav_node')

        # 声明参数（默认值可修改）
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 1.5)
        self.declare_parameter('goal_yaw', 0.0)  # 单位：弧度

        # 获取参数值
        goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        goal_yaw = self.get_parameter('goal_yaw').get_parameter_value().double_value

        # 构造目标点
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position = Point(x=goal_x, y=goal_y, z=0.0)
        goal_pose.pose.orientation = yaw_to_quaternion(goal_yaw)

        # 创建导航动作客户端
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.send_goal(goal_pose)

    def send_goal(self, goal_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('❌ 目标被拒绝')
            return
        self.get_logger().info('✅ 导航已启动')

def main(args=None):
    rclpy.init(args=args)
    node = NavToPointNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
