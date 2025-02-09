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
        self.max_points = 1000  # 龙的总点数
        self.scale = 1.0       # 龙的大小
        
        # 龙的运动参数
        self.car_pos = [0.0, 0.0, 0.0]  # 小车位置
        self.car_speed = 2.0            # 小车速度
        self.dragon_segments = 50       # 龙的段数
        self.points_per_segment = self.max_points // self.dragon_segments
        
        # 存储龙的历史位置
        self.history = deque(maxlen=self.dragon_segments)
        self.init_history()
        
    def init_history(self):
        """初始化龙的历史位置"""
        for i in range(self.dragon_segments):
            self.history.append((0, 0, 0))
            
    def update_car_position(self):
        """更新小车位置"""
        t = self.time
        # 让小车沿着一个有趣的路径移动
        self.car_pos[0] = 10 * math.sin(t * 0.5)  # X方向
        self.car_pos[1] = 10 * math.cos(t * 0.3)  # Y方向
        self.car_pos[2] = 2 + math.sin(t * 1.0)   # Z方向小幅上下浮动
        
        # 更新历史位置
        self.history.append(tuple(self.car_pos))

    def generate_segment_points(self, pos1, pos2, segment_index) -> list:
        """生成龙身体某一段的点云"""
        points = []
        t = self.time
        
        # 计算段的中心和方向
        center = np.array(pos1)
        direction = np.array(pos2) - center
        
        # 根据段在龙身上的位置调整大小
        segment_scale = self.scale * (0.8 + 0.4 * (1 - segment_index / self.dragon_segments))
        
        for _ in range(self.points_per_segment):
            # 生成围绕中心的环形点云
            angle = np.random.uniform(0, 2 * np.pi)
            radius = segment_scale * (0.5 + 0.2 * math.sin(angle * 3 + t * 2))
            
            # 计算点的位置
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = np.random.uniform(-0.2, 0.2) * segment_scale
            
            # 添加波浪效果
            wave = 0.3 * segment_scale * math.sin(angle * 2 + t * 3 + segment_index * 0.2)
            
            # 将点添加到中心位置
            point = center + np.array([x, y, z + wave])
            points.append(tuple(point))
            
        return points

    def generate_dragon_points(self) -> list:
        """生成整条龙的点云"""
        all_points = []
        
        # 为每段生成点云
        for i in range(len(self.history) - 1):
            pos1 = self.history[i]
            pos2 = self.history[i + 1]
            segment_points = self.generate_segment_points(pos1, pos2, i)
            all_points.extend(segment_points)
            
        # 生成龙头特效
        head_points = self.generate_dragon_head()
        all_points.extend(head_points)
        
        return all_points
    
    def generate_dragon_head(self) -> list:
        """生成龙头的特殊效果"""
        points = []
        head_pos = self.car_pos
        t = self.time
        
        # 生成更密集的龙头点云
        for _ in range(self.points_per_segment * 2):
            angle = np.random.uniform(0, 2 * np.pi)
            radius = self.scale * (1.0 + 0.3 * math.sin(angle * 2 + t * 3))
            
            x = radius * math.cos(angle) * 1.5  # 龙头稍微扁平
            y = radius * math.sin(angle)
            z = np.random.uniform(-0.2, 0.2) * self.scale
            
            # 添加呼吸效果
            breath = 0.2 * self.scale * math.sin(t * 5)
            
            point = (
                head_pos[0] + x,
                head_pos[1] + y,
                head_pos[2] + z + breath
            )
            points.append(point)
            
        return points

    def pack_data(self, points: list) -> bytes:
        """打包数据"""
        # 车辆位置（龙头位置）
        data = struct.pack('fff', 
                         self.car_pos[0],
                         self.car_pos[1],
                         self.car_pos[2])
        
        # 点云数据
        for point in points:
            data += struct.pack('ffff', 
                              point[0], point[1], point[2], 0.0)
        return data

    def run(self):
        """主循环"""
        try:
            while self.running:
                # 更新位置
                self.update_car_position()
                
                # 生成点云
                points = self.generate_dragon_points()
                data = self.pack_data(points)
                
                try:
                    self.sock.sendto(data, (self.UDP_IP, self.UDP_PORT))
                except Exception as e:
                    print(f"发送错误: {e}")
                
                time.sleep(0.05)  # 20Hz
                self.time += 0.05
                
        except KeyboardInterrupt:
            print("正在停止发送...")
        finally:
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