import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import matplotlib as mpl

class PointMatcher:
    def __init__(self):
        # 定义源坐标系中的四个点 (x, y, yaw)
        self.source_points = [
            [0.0, 0.0, 0],    # 点1
            [0.0, 8-0.357-0.3288, 0],    # 点2
            [15.0-0.39-0.13255, 0.0, 180],    # 点3
            [15.0-0.39-0.13255, 8-0.357-0.3288, 180]     # 点4
        ]
        
        # 定义目标坐标系中的四个点 (x, y, yaw)
        self.target_points = [
            [-0.008, 0.005, 1.2],   # 左下
            [0.098, -7.266, 1.9],   # 右下
            [14.029, -6.456, -178.8],   # 右上
            [13.935, 0.837, -179.6]    # 左上
        ]
        
        # 存储变换参数 (tx, ty, theta, scale)
        self.transform_params = None
        self.transformed_points = None
        
    def transform_point(self, point, params):
        """应用仿射变换到单个点"""
        tx, ty, theta, scale = params
        x, y, yaw = point
        
        # 转换为弧度
        theta_rad = np.deg2rad(theta)
        
        # 应用旋转、缩放和平移
        x_new = scale * (x * np.cos(theta_rad) - y * np.sin(theta_rad)) + tx
        y_new = scale * (x * np.sin(theta_rad) + y * np.cos(theta_rad)) + ty
        yaw_new = (yaw + theta) % 360  # 更新朝向
        
        return [x_new, y_new, yaw_new]
    
    def loss_function(self, params):
        """计算当前变换参数下的总误差"""
        total_error = 0
        for src, tgt in zip(self.source_points, self.target_points):
            transformed = self.transform_point(src, params)
            # 位置误差 (欧氏距离)
            pos_error = np.sqrt((transformed[0] - tgt[0])**2 + (transformed[1] - tgt[1])**2)
            # 朝向误差 (考虑角度环绕)
            angle_error = min(
                abs(transformed[2] - tgt[2]),
                360 - abs(transformed[2] - tgt[2])
            ) / 180  # 归一化到[0, 1]
            # 组合误差 (位置误差权重1，角度误差权重0.5)
            total_error += pos_error + 0.5 * angle_error
        return total_error
    
    def match_points(self):
        """执行匹配优化"""
        # 初始猜测 (tx, ty, theta, scale)
        initial_guess = [0, 0, 0, 1]
        
        # 设置边界约束
        bounds = [(-10, 10), (-10, 10), (-180, 180), (0.5, 1.5)]
        
        # 最小化损失函数
        result = minimize(self.loss_function, initial_guess, bounds=bounds, method='L-BFGS-B')
        
        # 保存最优参数
        self.transform_params = result.x
        print(f"匹配参数: tx={result.x[0]:.4f}, ty={result.x[1]:.4f}, "
              f"θ={result.x[2]:.4f}°, scale={result.x[3]:.4f}")
        
        # 应用变换到所有点
        self.transformed_points = [self.transform_point(p, self.transform_params) 
                                  for p in self.source_points]
        return self.transform_params
    
    def visualize(self):
        """可视化源点、目标点和变换后的点"""
        # 设置绘图参数
        plt.figure(figsize=(10, 8))
        plt.title("四点点匹配可视化")
        plt.grid(True)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.axis('equal')
        
        # 创建颜色列表
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
        
        # 绘制源点
        for i, (x, y, yaw) in enumerate(self.source_points):
            plt.scatter(x, y, s=120, c=colors[i], marker='o', edgecolors='k', label=f'源点{i+1}')
            self._draw_arrow(x, y, yaw, colors[i], 1.0)
        
        # 绘制目标点
        for i, (x, y, yaw) in enumerate(self.target_points):
            plt.scatter(x, y, s=120, c=colors[i], marker='s', edgecolors='k', label=f'目标点{i+1}')
            self._draw_arrow(x, y, yaw, colors[i], 0.7)
        
        # 绘制变换后的点
        if self.transformed_points:
            for i, (x, y, yaw) in enumerate(self.transformed_points):
                plt.scatter(x, y, s=120, c=colors[i], marker='^', edgecolors='k', label=f'变换点{i+1}')
                self._draw_arrow(x, y, yaw, colors[i], 1.0)
        
        # 添加图例并避免重复
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys(), loc='best')
        
        plt.tight_layout()
        plt.show()
    
    def _draw_arrow(self, x, y, yaw, color, length):
        """绘制表示方向的箭头"""
        angle_rad = np.deg2rad(yaw)
        dx = length * np.cos(angle_rad)
        dy = length * np.sin(angle_rad)
        plt.arrow(x, y, dx, dy, 
                  head_width=0.2, 
                  head_length=0.3, 
                  fc=color, 
                  ec='k', 
                  alpha=0.7)

# 使用示例
if __name__ == "__main__":
    matcher = PointMatcher()
    print("正在计算匹配参数...")
    params = matcher.match_points()
    
    print("\n源点:")
    for i, p in enumerate(matcher.source_points):
        print(f"点{i+1}: x={p[0]:.2f}, y={p[1]:.2f}, yaw={p[2]:.2f}°")
    
    print("\n目标点:")
    for i, p in enumerate(matcher.target_points):
        print(f"点{i+1}: x={p[0]:.2f}, y={p[1]:.2f}, yaw={p[2]:.2f}°")
    
    print("\n变换后的点:")
    for i, p in enumerate(matcher.transformed_points):
        print(f"点{i+1}: x={p[0]:.2f}, y={p[1]:.2f}, yaw={p[2]:.2f}°")
    
    print("\n正在生成可视化...")
    matcher.visualize()