import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
import math
from geometry_msgs.msg import TransformStamped
from filterpy.kalman import KalmanFilter
class KalmanNode(Node):
    def __init__(self):
        super().__init__('kalman_node')
        self.get_logger().info("Kalman滤波器节点已启动")
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('publish_tf_name', 'base_link_imu')
        self.declare_parameter('hz',100)
        self.declare_parameter('kalman_model',0)
        #主预测的更新时差    
        dt=1.0/self.get_parameter('hz').value
        # 观测x y yaw坐标极其速度加速度
        model=self.get_parameter('kalman_model').value
        # self.kf=filter.kalman
        if model==0:
            # 线性模型
            self.kf = KalmanFilter(dim_x=9, dim_z=6)
        # self.kf = KalmanFilter(dim_x=9, dim_z=6)
        self.kf.x=np.zeros((9, 1)) # 状态向量
        # self.kf.F=np.array(
        
        # )
        #外部输入为速度,不是加速度
        self.B=np.array(
            [
                [dt, 0, 0],
                [0, dt, 0],
                [0, 0, dt],
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0]
            ]
        )
        self.kf.B=self.B
        # self.kf.Q
        self.cmd_vel=np.zeros(3) # 控制输入
        self.odom=np.zeros(3)
        #tf只观测x y yaw
        self.H_tf=np.array(
            [
                [1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0],
            ]
        )
        #imu观测加速度与角速度
        self.H_imu=np.array(
            [
                [0,0,0,1,0,0,0,0,0],
                [0,0,0,0,1,0,0,0,0],
                [0,0,0,0,0,1,0,0,0],      
            ]
        )   
        self.create_subscription(Twist, '/cmd_vel',self.cmd_vel_callback,1)
        self.create_subscription(TFMessage,'/tf',self.tf_callback,1)
    def cmd_vel_callback(self, msg:Twist):
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[2] = msg.angular.z
        
    def prdict_handler(self):
        # 预测
        self.kf.predict()
        
    def tf_callback(self, msg:TFMessage):
        #寻找其中odom到base_link的tf
        transform_temp : TransformStamped= None
        for transform in msg.transforms:
            if transform.child_frame_id == 'base_link' and transform.header.frame_id == 'odom':
                transform_temp = transform
                break
        if transform_temp is None:
            return
        translation = transform_temp.transform.translation
        rotation = transform_temp.transform.rotation

        # 保存 x, y
        self.odom[0] = translation.x
        self.odom[1] = translation.y

        yaw= self.get_yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)

        # 保存 yaw
        self.odom[3] = yaw                        
        # 提取位置和方向
        #选择相信雷达数据
        
    def get_yaw_from_quaternion(x, y, z, w):
        """根据四元数返回 yaw偏航角"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    def imu_callback(self, msg:Imu):
        # 提取加速度与角速度
        acc_x= msg.linear_acceleration.x
        acc_y= msg.linear_acceleration.y
        angular_z= msg.angular_velocity.z*180/math.pi
       
                    