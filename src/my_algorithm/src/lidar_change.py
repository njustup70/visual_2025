#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener
from laser_geometry import LaserProjection


class LaserToOdomConverter(Node):
    def __init__(self):
        super().__init__('laser_to_odom_converter')
        
        # 参数配置
        self.declare_parameter('input_laser_topic', '/laser/sd/raw')
        self.declare_parameter('output_topic', '/laser/sd/transformed')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('laser_frame', 'laser_link')
        
        input_topic = self.get_parameter('input_laser_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        
        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 订阅与发布
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            10
        )
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )
        
        # 激光数据转点云工具
        self.laser_projector = LaserProjection()

    def scan_callback(self, scan_msg):
        try:
            # 1. 获取坐标系变换（使用最新可用变换替代时间戳匹配）
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,  # 目标坐标系 (如"odom")
                self.laser_frame,   # 源坐标系 (如"laser")
                rclpy.time.Time(),  # 使用最新可用变换[6,7](@ref)
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # 2. LaserScan → PointCloud2（使用ROS2内置方法）
            cloud_msg = self.laser_projector.projectLaser(scan_msg)
            
            # 3. 替代do_transform_cloud的两种方法：
            # 方法A：使用tf2_geometry_msgs（推荐）
            from tf2_geometry_msgs import do_transform_cloud
            cloud_in_odom = do_transform_cloud(cloud_msg, transform)
            
            # 方法B：手动转换（备选方案）
            # cloud_in_odom = self.manual_transform(cloud_msg, transform)
            
            cloud_in_odom.header.frame_id = self.target_frame
            
            # 4. 发布转换后的点云
            self.pointcloud_pub.publish(cloud_in_odom)
            
        except (tf2_ros.LookupException, 
                tf2_ros.ExtrapolationException, 
                tf2_ros.ConnectivityException) as e:
            self.get_logger().error(f"TF异常: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserToOdomConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()