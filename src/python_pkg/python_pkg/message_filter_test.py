#!/usr/bin/env python3
import rclpy
import rclpy.duration
import rclpy.executors
from rclpy.node import Node
import message_filters
import tf2_ros
from sensor_msgs.msg import LaserScan
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__('laser_scan_subscriber_test')

        # 固定参数
        self.topic_name = '/MS200/scan'
        self.global_frame = 'map'
        self.transform_tolerance = 1.5  # 容忍时间（秒）

        # 创建tf2 Buffer和Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self,spin_thread=True)

        # message_filters subscriber
        # self.scan_sub = message_filters.Subscriber(self, LaserScan, self.topic_name,queue_size=1)
        
        # 直接注册回调
        # self.scan_sub.registerCallback(self.scan_callback)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.topic_name,
            self.scan_callback,
            1
        )
        self.get_logger().info('LaserScan Subscriber initialized.')

    def scan_callback(self, msg:LaserScan):
        # 每次收到数据，尝试查找tf变换
        print("flag\n")
        try:
            # 打印缓冲区
            self.get_logger().info(f"Buffer size: {self.tf_buffer.get_latest_common_time(self.global_frame, msg.header.frame_id)}")
            stamp=msg.header.stamp
            stamp.sec+=1
            now = self.get_clock().now()
            trans = self.tf_buffer.lookup_transform(
                self.global_frame,
                msg.header.frame_id,
                stamp,
                timeout=rclpy.duration.Duration(seconds=self.transform_tolerance)  # 设置容忍时间
            )
            self.get_logger().info(
                f"Got transform! {msg.header.frame_id} -> {self.global_frame} at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"Dropped message at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanSubscriber()
    exe=rclpy.executors.MultiThreadedExecutor()
    exe.add_node(node)
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # node.destroy_node()
    # rclpy.shutdown()
    exe.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
