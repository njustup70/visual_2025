import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
from scipy.fft import fft, fftfreq
import os
from datetime import datetime
import csv

class ImuNoiseAnalyzer(Node):
    def __init__(self):
        super().__init__('imu_noise_analyzer')
        self.get_logger().info("IMU 噪声分析节点已启动 - 400Hz 版本")

        # 参数配置
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('sample_count', 120000)  # 采集 120000 条数据 (约 5 分钟 @ 400Hz)
        
        self.imu_topic = self.get_parameter('imu_topic').value
        self.sample_count = self.get_parameter('sample_count').value
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        
        # 数据存储
        self.acc_data = []  # 加速度数据 (m/s²)
        self.gyro_data = []  # 角速度数据 (rad/s)
        self.fs = 400  # 采样频率

        # 结果保存路径
        # self.save_dir = os.path.join(os.path.expanduser('~'), 'imu_noise_analysis')
        #路径为同一个功能包的result文件夹下
        self.save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../src/python_pkg/result')
        self.save_dir = os.path.abspath(self.save_dir)
        print("存放路径为{}".format(self.save_dir))
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.get_logger().info(f"正在采集静态数据，目标条数: {self.sample_count} ...")

    def imu_callback(self, msg):
        # 存储原始数据
        acc = [msg.linear_acceleration.x, 
               msg.linear_acceleration.y,
               msg.linear_acceleration.z]
        
        gyro = [msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z]
        
        self.acc_data.append(acc)
        self.gyro_data.append(gyro)

        # 检查采集数量
        if len(self.acc_data) >= self.sample_count:
            self.get_logger().info("数据采集完成，开始分析...")
            self.analyze_all()
            self.save_raw_data()
            self.destroy_node()

    def analyze_all(self):
        acc_array = np.array(self.acc_data)
        gyro_array = np.array(self.gyro_data)
        
        # 基本统计特征
        self.analyze_basic_stats(acc_array, gyro_array)
        
        # 频域分析
        self.analyze_frequency_domain(acc_array, gyro_array)
        
        # Allan方差分析
        self.analyze_allan_variance(acc_array, gyro_array)

    def analyze_basic_stats(self, acc, gyro):
        # 移除重力影响 (假设Z轴向上)
        acc[:, 2] -= 9.81  # 减去重力加速度
        
        for i, axis in enumerate(['X', 'Y', 'Z']):
            # 加速度分析
            acc_mean = np.mean(acc[:, i])
            acc_std = np.std(acc[:, i])
            
            # 角速度分析
            gyro_mean = np.mean(gyro[:, i])
            gyro_std = np.std(gyro[:, i])
            
            self.get_logger().info(f"\n{axis}轴分析结果:")
            self.get_logger().info(f"加速度零偏: {acc_mean:.6f} m/s², 标准差: {acc_std:.6f} m/s²")
            self.get_logger().info(f"角速度零偏: {gyro_mean:.6f} rad/s, 标准差: {gyro_std:.6f} rad/s")

    def analyze_frequency_domain(self, acc, gyro):
        # 分析加速度计X轴和陀螺仪Z轴
        plt.figure(figsize=(12, 8))
        
        # 加速度计频谱
        self.plot_fft(acc[:, 0], "加速度 X 轴", subplot=211)
        
        # 陀螺仪频谱
        self.plot_fft(gyro[:, 2], "角速度 Z 轴", subplot=212)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.save_dir, 'frequency_analysis.png'))
        plt.close()

    def plot_fft(self, data, title, subplot=None):
        N = len(data)
        freqs = fftfreq(N, 1 / self.fs)[:N//2]
        fft_values = np.abs(fft(data))[:N//2] * 2 / N
        
        if subplot:
            plt.subplot(subplot)
            
        plt.semilogy(freqs, fft_values)
        plt.xlabel('频率 (Hz)')
        plt.ylabel('幅值')
        plt.title(title)
        plt.grid(True)
        plt.xlim(0, self.fs/2)

    def analyze_allan_variance(self, acc, gyro):
        tau, acc_adev = self.allan_deviation(acc[:, 0], self.fs)
        _, gyro_adev = self.allan_deviation(gyro[:, 2], self.fs)
        
        plt.figure(figsize=(10, 6))
        plt.loglog(tau, acc_adev, label='加速度 X轴')
        plt.loglog(tau, gyro_adev, label='角速度 Z轴')
        plt.xlabel('积分时间 τ (s)')
        plt.ylabel('Allan 偏差')
        plt.title('Allan方差分析')
        plt.legend()
        plt.grid(True)
        plt.savefig(os.path.join(self.save_dir, 'allan_variance.png'))
        plt.close()

    def allan_deviation(self, data, fs):
        max_tau = len(data) // (10 * fs)  # 最大积分时间10秒
        tau = np.logspace(-2, np.log10(max_tau), 100)
        
        adev = []
        for t in tau:
            n = int(t * fs)
            if n == 0:
                continue
                
            # 分段计算方差
            m = len(data) // n
            sigma2 = np.var([np.mean(data[i*n:(i+1)*n]) for i in range(m)])
            adev.append(np.sqrt(sigma2))
            
        return tau[:len(adev)], np.array(adev)

    def save_raw_data(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.save_dir, f'raw_data_{timestamp}.csv')
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z'])
            for a, g in zip(self.acc_data, self.gyro_data):
                writer.writerow([*a, *g])
                
        self.get_logger().info(f"原始数据已保存至: {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNoiseAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
