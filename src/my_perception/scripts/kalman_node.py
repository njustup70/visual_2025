import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
import math
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from EFK import FlexibleKalmanFilter

class KalmanNode(Node):
    def __init__(self):
        super().__init__('kalman_node')
        self.get_logger().info("Kalman滤波器节点已启动")
        self.declare_parameter('imu_topic', '/imu_transformed')
        self.declare_parameter('publish_tf_name', 'base_link_imu')
        self.declare_parameter('hz',100)
        self.declare_parameter('kalman_model',0)
        
        # 时间参数
        self.dt = 1.0/self.get_parameter('hz').value
        self.last_time = self.get_clock().now()
        
        # 状态向量 [x, y, yaw]
        self.kf = FlexibleKalmanFilter(dim_x=3)
        self.kf.x = np.zeros((3, 1))  # 初始化状态
        
        # 构建状态转移矩阵
        self.kf.F = np.zeros((3,3))
        
        # 控制输入矩阵
        self.B= np.zeros((3, 3))
        self.kf.B = self.B
        
        # 测量矩阵 - 组合版本
        self.H = np.array([
            [1,0,0],
            [0,1,0],
            [0,0,1]
        ])
        self.kf.H = self.H
        
        # 过程噪声协方差矩阵（根据物理特性调整）
        self.kf.Q = np.zeros((3, 3))
        self.kf.Q[0:3, 0:3] = Q_discrete_white_noise(dim=3, dt=self.dt, var=0.01)  # 位置和角度噪声
        
        # 测量噪声协方差矩阵（根据传感器精度调整）
        self.R_tf = np.diag([0.01, 0.01, 0.01])  # TF测量噪声（x,y,yaw）
        self.R_imu = np.diag([0.1, 0.1, 0.1])    # IMU测量噪声（ax,ay,ayaw）
        
        # 初始估计误差协方差
        self.kf.P = np.eye(3) * 100.0
        
        # 控制输入和测量缓存
        self.cmd_vel = np.zeros(3)  # 控制输入[vx, vy, vyaw]
        self.odom = np.zeros(3)     # [x, y, yaw]
        self.odom_last = np.zeros(3)     # [x, y, yaw]
        self.odom_d = np.zeros(3)     # [vx, vy, vyaw]观测值

        # self.imu_data = np.zeros(3) # [ax, ay, ayaw]
        
        # 创建订阅者
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 1)
        # self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.imu_callback, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 创建定时器
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.tf_broadcaster = TransformBroadcaster(self)

    def cmd_vel_callback(self, msg: Twist):
        """处理速度指令"""
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[2] = msg.angular.z
        
    def tf_callback(self, msg: TFMessage):
        """处理TF消息"""
        transform_temp = None
        # for transform in msg.transforms:
        #     if transform.child_frame_id == 'base_link' and transform.header.frame_id == 'odom':
        #         transform_temp = transform
        #         break
                
        # if transform_temp is None:
        #     return
        try:
            transform_temp = self.tf_buffer.lookup_transform('odom', 'base_link',time=Time())
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return
        # 提取位置信息
        translation = transform_temp.transform.translation
        rotation = transform_temp.transform.rotation

        # 保存x, y, yaw
        self.odom[0] = translation.x
        self.odom[1] = translation.y
        self.odom[2] = self.get_yaw_from_quaternion(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        self.odom_d = (self.odom - self.odom_last)/self.dt  # 计算速度观测值

        self.odom_last = self.odom.copy()  # 保存上一次的odom值
        # 执行基于TF的更新
        self.update_tf()
        
    def imu_callback(self, msg: Imu):
        """处理IMU消息"""
        # 提取加速度与角速度（转换为弧度）
        self.imu_data[0] = msg.linear_acceleration.x
        self.imu_data[1] = msg.linear_acceleration.y
        self.imu_data[2] = msg.angular_velocity.z  # 已经是rad/s
        
        # 执行基于IMU的更新
        self.update_imu()
        
    def timer_callback(self):
        """定时器回调 - 执行预测步骤"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        dt = self.dt  # 实际时间差
        yaw = self.kf.x[2, 0]  # 使用卡尔曼滤波的状态估计值，而非观测值
        vx = self.cmd_vel[0]    # 使用控制输入速度
        vy = self.cmd_vel[1]
        omega = self.cmd_vel[2]  # 控制输入角速度

        # 恒速模型下的状态转移矩阵（假设平面运动，yaw为偏航角）
        self.kf.F = np.array([
            [1, 0, -vx*dt*np.sin(yaw) - vy*dt*np.cos(yaw)],  # x对yaw的导数
            [1, 0,  vx*dt*np.cos(yaw) - vy*dt*np.sin(yaw)],  # y对yaw的导数
            [0, 0, 1]
        ])  # 简化示例，需根据实际模型调整

        # 控制矩阵B（将控制输入[vx, vy, omega]映射到状态导数）
        self.kf.B = np.array([
            [dt*np.cos(yaw), -dt*np.sin(yaw), 0],
            [dt*np.sin(yaw),  dt*np.cos(yaw), 0],
            [0, 0, dt]
        ])

        # # 更新状态转移矩阵
        # self.kf.F[0,2] = -self.odom_d[0]*np.sin(self.odom[0]) -self.odom_d[1]*np.cos(self.odom[2])
        # self.kf.F[1,2] = self.odom_d[0]*np.cos(self.odom[0]) -self.odom_d[1]*np.sin(self.odom[2])

        # self.kf.B[0,0] = np.cos(self.odom_d[2])
        # self.kf.B[0,1] = -np.sin(self.odom_d[2])
        # self.kf.B[1,0] = np.sin(self.odom_d[2])
        # self.kf.B[1,1] = np.cos(self.odom_d[2])
        # self.kf.B[2,2] = 1
        
        # self.kf.F = self.kf.F * dt
        # self.kf.B = self.kf.F * dt
        
        # 执行预测步骤（考虑控制输入）
        self.kf.predict(u=self.cmd_vel.reshape(-1, 1))
        
        # 发布融合后的状态
        self.publish_fused_state()
        
    def update_tf(self):
        """基于TF数据更新滤波器"""
        # 构建测量向量 [x, y, yaw]
        z = np.array([
            [self.odom[0]],
            [self.odom[1]],
            [self.odom[2]]
        ])
        
        # 构建临时测量矩阵
        H_temp = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        
        # 执行更新步骤
        self.kf.update(z, H=H_temp, R=self.R_tf)
        
    def update_imu(self):
        """基于IMU数据更新滤波器"""
        # # 构建测量向量 [ax, ay, ayaw]
        # z = np.array([
        #     [self.imu_data[0]],
        #     [self.imu_data[1]],
        #     [self.imu_data[2]]
        # ])
        
        # # 构建临时测量矩阵（只测量加速度和角加速度）
        # H_temp = np.array([
        #     [0, 0, 0, 0, 0, 0, 1, 0, 0],
        #     [0, 0, 0, 0, 0, 0, 0, 1, 0],
        #     [0, 0, 0, 0, 0, 0, 0, 0, 1]
        # ])
        
        # # 执行更新步骤
        # self.kf.update(z, H=H_temp, R=self.R_imu)
        
    def publish_fused_state(self):
        """发布融合后的状态"""
        tf_pub=TransformStamped()
        tf_pub.header.stamp = self.get_clock().now().to_msg()
        tf_pub.header.frame_id = 'odom'
        tf_pub.child_frame_id = self.get_parameter('publish_tf_name').value
        tf_pub.transform.translation.x = self.kf.x[0, 0]
        tf_pub.transform.translation.y = self.kf.x[1, 0]
        tf_pub.transform.translation.z = 0.0
        tf_pub.transform.rotation.x = 0.0
        tf_pub.transform.rotation.y = 0.0
        tf_pub.transform.rotation.z = math.sin(self.kf.x[2, 0] / 2.0)
        tf_pub.transform.rotation.w = math.cos(self.kf.x[2, 0] / 2.0)
        self.tf_broadcaster.sendTransform(tf_pub)
        
        # 这里可以添加发布融合后状态的代码
        #构造新的tf
        # self.get_logger().debug(f"Fused State: {fused_state.flatten()}")
        
    @staticmethod
    def get_yaw_from_quaternion(x, y, z, w):
        """根据四元数返回yaw偏航角"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw    
def main(args=None):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init(args=args)
    node = KalmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()