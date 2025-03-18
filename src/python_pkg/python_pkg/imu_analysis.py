import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib
# 设置无GUI的后端
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
from scipy.fft import fft, fftfreq
import os
from datetime import datetime
import csv

# 配置中文字体（需要系统安装中文字体）
plt.rcParams['font.sans-serif'] = ['SimHei']  # Windows系统常用中文字体
plt.rcParams['axes.unicode_minus'] = False

class ImuNoiseAnalyzer(Node):
    def __init__(self):
        super().__init__('imu_noise_analyzer')
        self.get_logger().info("IMU噪声分析节点已启动 - 400Hz版本")

        # 参数配置
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('sample_count', 120000)  # 约5分钟数据
        print("需要采集的数据量: ", self.get_parameter('sample_count').value)
        print("IMU话题: ", self.get_parameter('imu_topic').value)
        self.imu_topic = self.get_parameter('imu_topic').value
        self.sample_count = self.get_parameter('sample_count').value
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        
        # 数据存储
        self.acc_data = []
        self.gyro_data = []
        self.fs = 400

        # 结果保存路径
        self.save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../src/python_pkg/result')
        self.save_dir = os.path.abspath(self.save_dir)
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f"数据保存路径: {self.save_dir}")

    def imu_callback(self, msg: Imu):
        acc = [msg.linear_acceleration.x, 
               msg.linear_acceleration.y,
               msg.linear_acceleration.z]
        
        gyro = [msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z]
        
        self.acc_data.append(acc)
        self.gyro_data.append(gyro)

        if len(self.acc_data) >= self.sample_count:
            self.get_logger().info("数据采集完成，开始分析...")
            self.analyze_all()
            self.save_raw_data()
            self.destroy_node()

    def analyze_all(self):
        acc_array = np.array(self.acc_data)
        gyro_array = np.array(self.gyro_data)
        
        # 基本统计分析
        self.analyze_basic_stats(acc_array, gyro_array)
        
        # 频域分析
        self.analyze_frequency_domain(acc_array, gyro_array)
        
        # Allan方差分析
        self.analyze_allan_variance(acc_array, gyro_array)

    def analyze_basic_stats(self, acc, gyro):
        stats = []
        for i, axis in enumerate(['X', 'Y', 'Z']):
            # 加速度统计
            acc_mean = np.mean(acc[:, i])
            acc_std = np.std(acc[:, i])
            
            # 角速度统计
            gyro_mean = np.mean(gyro[:, i])
            gyro_std = np.std(gyro[:, i])
            
            stats.append(f"{axis}轴加速度 - 均值: {acc_mean:.6f} m/s², 标准差: {acc_std:.6f} m/s²")
            stats.append(f"{axis}轴角速度 - 均值: {gyro_mean:.6f} rad/s, 标准差: {gyro_std:.6f} rad/s")
        
        # 保存统计结果
        with open(os.path.join(self.save_dir, 'statistics.txt'), 'w') as f:
            f.write("\n".join(stats))
        
        self.get_logger().info("\n基本统计结果:\n" + "\n".join(stats))

    def analyze_frequency_domain(self, acc, gyro):
        fig = plt.figure(figsize=(14, 10))
        
        # 加速度计频谱分析
        for i, axis in enumerate(['X', 'Y', 'Z']):
            plt.subplot(3, 2, i*2+1)
            self.plot_fft(acc[:, i], f"Acceleration {axis}-axis Spectrum")
            
        # 陀螺仪频谱分析    
        for i, axis in enumerate(['X', 'Y', 'Z']):
            plt.subplot(3, 2, i*2+2)
            self.plot_fft(gyro[:, i], f"Angular Velocity {axis}-axis Spectrum")
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.save_dir, 'frequency_analysis.png'), dpi=150)
        plt.close(fig)

    def plot_fft(self, data, title):
        N = len(data)
        freqs = fftfreq(N, 1/self.fs)[:N//2]
        fft_val = np.abs(fft(data))[:N//2] * 2/N
        
        plt.semilogy(freqs, fft_val)
        plt.title(title)
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Amplitude')
        plt.grid(True)
        plt.xlim(0, self.fs//2)

    def analyze_allan_variance(self, acc, gyro):
        fig, ax = plt.subplots(2, 1, figsize=(12, 10))
        
        # 加速度计Allan方差
        for i, axis in enumerate(['X', 'Y', 'Z']):
            tau, adev = self.allan_deviation(acc[:, i], self.fs)
            ax[0].loglog(tau, adev, label=f'{axis}-axis')
        
        # 陀螺仪Allan方差
        for i, axis in enumerate(['X', 'Y', 'Z']):
            tau, adev = self.allan_deviation(gyro[:, i], self.fs)
            ax[1].loglog(tau, adev, label=f'{axis}-axis')
        
        ax[0].set_title('Accelerometer Allan Variance Analysis')
        ax[0].set_ylabel('ADEV (m/s²)')
        ax[1].set_title('Gyroscope Allan Variance Analysis')
        ax[1].set_ylabel('ADEV (rad/s)')
        
        for a in ax:
            a.set_xlabel('Integration Time τ (s)')
            a.legend()
            a.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.save_dir, 'allan_variance.png'), dpi=150)
        plt.close(fig)

    def allan_deviation(self, data, fs):
        n = len(data)
        max_tau = n // (10 * fs)  # 最大积分时间10秒
        tau = np.logspace(-2, np.log10(max_tau), 100)
        
        adev = []
        for t in tau:
            m = int(t * fs)
            if m < 1:
                continue
                
            num_groups = n // m
            groups = data[:num_groups*m].reshape(-1, m)
            means = groups.mean(axis=1)
            adev.append(np.sqrt(0.5 * np.mean(np.diff(means)**2)))
        
        valid_tau = tau[:len(adev)]
        return valid_tau, np.array(adev)

    def save_raw_data(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.save_dir, f'raw_data_{timestamp}.csv')
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['acc_x','acc_y','acc_z','gyro_x','gyro_y','gyro_z'])
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