import socket
import struct
import time
import numpy as np
import math
from collections import deque

class PointCloudSender:
    def __init__(self, ip="127.0.0.1", port=8080):
        self.UDP_IP = ip
        self.UDP_PORT = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65507)
        self.time = 0.0
        self.running = True
        
        # 点云参数
        self.max_points = 800     # 总点数
        self.trail_length = 100   # 轨迹保留长度
        
        # 螺旋参数
        self.base_radius = 2.0    # 基础半径
        self.radius_growth = 0.1  # 半径增长率
        self.height_speed = 0.5   # 上升速度
        self.rotation_speed = 2.0 # 旋转速度
        
        # 存储历史轨迹
        self.history = deque(maxlen=self.trail_length)
        self.current_height = 0
        self.current_angle = 0
        
        # 粒子效果参数
        self.particle_spread = 0.5  # 粒子扩散范围
        self.particle_size = 0.2    # 粒子大小

        # 添加计数器
        self.total_points_sent = 0
        self.frame_counter = 0
        self.last_print_time = time.time()
        
    def update_position(self):
        """更新螺旋位置"""
        # 计算当前半径（随高度增加）
        current_radius = self.base_radius + self.current_height * self.radius_growth
        
        # 计算小车位置
        x = current_radius * math.cos(self.current_angle)
        y = current_radius * math.sin(self.current_angle)
        z = self.current_height
        
        # 更新角度和高度
        self.current_angle += self.rotation_speed * 0.05  # 每帧旋转量
        self.current_height += self.height_speed * 0.05   # 每帧上升高度
        
        # 存储位置
        self.history.append((x, y, z))
        return x, y, z

    def generate_spiral_points(self) -> list:
        """生成螺旋点云"""
        points = []
        
        # 生成主螺旋轨迹点
        for i, pos in enumerate(self.history):
            # 计算该段的点数
            segment_points = max(3, int(self.max_points / self.trail_length))
            
            # 轨迹渐变效果
            alpha = i / len(self.history)  # 0到1的渐变值
            
            # 为每个历史位置生成一圈点云
            for _ in range(segment_points):
                # 在位置周围随机生成点
                angle = np.random.uniform(0, 2 * np.pi)
                radius = np.random.uniform(0, self.particle_spread * (1 + alpha))
                
                # 计算偏移
                dx = radius * math.cos(angle)
                dy = radius * math.sin(angle)
                dz = np.random.uniform(-self.particle_size, self.particle_size)
                
                # 添加螺旋运动效果
                spiral_effect = 0.3 * math.sin(angle * 3 + self.time * 5)
                
                point = (
                    pos[0] + dx + spiral_effect,
                    pos[1] + dy + spiral_effect,
                    pos[2] + dz
                )
                points.append(point)
        
        # 添加特殊效果点
        self.add_special_effects(points)
        
        return points
    
    def add_special_effects(self, points):
        """添加特殊效果"""
        if not self.history:
            return
        
        current_pos = self.history[-1]
        
        # 添加光环效果
        num_halo_points = 100
        halo_radius = self.particle_spread * 2
        for i in range(num_halo_points):
            angle = (i / num_halo_points) * 2 * np.pi
            radius = halo_radius * (1 + 0.2 * math.sin(angle * 3 + self.time * 4))
            
            point = (
                current_pos[0] + radius * math.cos(angle),
                current_pos[1] + radius * math.sin(angle),
                current_pos[2] + 0.2 * math.sin(self.time * 10)
            )
            points.append(point)
        
        # 添加上升粒子
        num_rising_particles = 50
        for _ in range(num_rising_particles):
            angle = np.random.uniform(0, 2 * np.pi)
            radius = np.random.uniform(0, self.particle_spread)
            height = np.random.uniform(0, 1.0)
            
            point = (
                current_pos[0] + radius * math.cos(angle),
                current_pos[1] + radius * math.sin(angle),
                current_pos[2] + height
            )
            points.append(point)

    def pack_data(self, points: list) -> bytes:
        """打包数据"""
        # 获取当前位置作为车辆位置
        current_pos = self.history[-1] if self.history else (0, 0, 0)
        
        # 打包车辆位置
        data = struct.pack('fff', 
                         current_pos[0],
                         current_pos[1],
                         current_pos[2])
        
        # 打包点云数据
        for point in points:
            data += struct.pack('ffff', 
                              point[0], point[1], point[2], 0.0)

        # 更新计数器
        self.total_points_sent += len(points)
        self.frame_counter += 1
        
        # 每秒打印一次统计信息
        current_time = time.time()
        if current_time - self.last_print_time >= 1.0:
            fps = self.frame_counter / (current_time - self.last_print_time)
            print(f"发送统计: 当前帧点数={len(points)}, "
                  f"累计发送点数={self.total_points_sent}, "
                  f"FPS={fps:.1f}, "
                  f"数据包大小={len(data)/1024:.1f}KB")
            
            # 重置计数器
            self.frame_counter = 0
            self.last_print_time = current_time
            
        return data

    def run(self):
        """主循环"""
        try:
            print("开始发送点云数据...")
            while self.running:
                # 更新位置
                self.update_position()
                
                # 生成点云
                points = self.generate_spiral_points()
                data = self.pack_data(points)
                
                try:
                    self.sock.sendto(data, (self.UDP_IP, self.UDP_PORT))
                except Exception as e:
                    print(f"发送错误: {e}")
                    print(f"当前点数: {len(points)}")
                
                time.sleep(0.05)  # 20Hz
                self.time += 0.05
                
        except KeyboardInterrupt:
            print("正在停止发送...")
        finally:
            print(f"总共发送点数: {self.total_points_sent}")
            self.sock.close()

    def stop(self):
        """停止发送"""
        self.running = False

def main():
    sender = PointCloudSender()
    try:
        sender.run()
    except KeyboardInterrupt:
        sender.stop()
        print("\n程序已停止")

if __name__ == "__main__":
    main()