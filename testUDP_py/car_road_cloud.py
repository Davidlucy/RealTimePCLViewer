import socket
import struct
import time
import numpy as np
import math
from collections import deque

class CarPointCloudSender:
    def __init__(self, ip="127.0.0.1", port=8080):
        self.UDP_IP = ip
        self.UDP_PORT = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65507)
        self.time = 0.0
        self.running = True
        
        # 点云参数
        self.max_points = 4000
        self.history_length = 10
        
        # 车辆运动参数
        self.car_speed = 5.0  # 米/秒
        self.car_pos = np.array([0.0, 0.0, 0.5])  # 起始位置
        self.car_direction = 0.0  # 朝向角度（弧度）
        self.turn_rate = 0.0  # 转向速率
        
        # 道路参数
        self.road_width = 8.0  # 道路宽度
        self.lane_width = 4.0  # 车道宽度
        self.road_points_density = 2.5  # 每平方米的点数
        
        # 历史轨迹
        self.position_history = deque(maxlen=self.history_length)
        
        # 统计信息
        self.total_points_sent = 0
        self.frame_counter = 0
        self.last_print_time = time.time()

    def update_car_position(self):
        """更新车辆位置"""
        # 模拟简单的转向（可以根据时间改变转向）
        self.turn_rate = 0.2 * math.sin(self.time * 0.2)  # 缓慢的左右转向
        self.car_direction += self.turn_rate * 0.05
        
        # 计算新位置
        movement = np.array([
            math.cos(self.car_direction) * self.car_speed * 0.05,
            math.sin(self.car_direction) * self.car_speed * 0.05,
            0
        ])
        new_pos = self.car_pos + movement
        
        # 确保车辆不会偏离道路太远
        if abs(new_pos[1]) > self.road_width/3:  # 使用road_width/3作为边界
            # 如果即将偏离道路，调整方向
            self.car_direction -= np.sign(new_pos[1]) * 0.1
        else:
            self.car_pos = new_pos
        
        # 保持高度恒定
        self.car_pos[2] = 0.5
        
        # 存储位置历史
        self.position_history.append(self.car_pos.copy())

    def generate_road_points(self) -> list:
        """生成道路点云"""
        points = []
        
        # 在车辆周围生成道路点
        view_distance = 40.0  # 前方可视距离
        side_distance = 15.0  # 两侧可视距离
        
        # 获取车辆前方的区域
        forward = np.array([math.cos(self.car_direction), math.sin(self.car_direction), 0])
        right = np.array([-math.sin(self.car_direction), math.cos(self.car_direction), 0])
        
        # 生成道路表面点
        num_points = int(view_distance * self.road_width * self.road_points_density)
        for _ in range(num_points):
            # 在车辆前方区域随机生成点
            length = np.random.uniform(-8, view_distance)  # 允许在车后方也生成一些点
            width = np.random.uniform(-self.road_width/2, self.road_width/2)
            
            point = self.car_pos + forward * length + right * width
            point[2] = 0  # 确保在地面上
            
            # 添加一些随机起伏
            point[2] += np.random.uniform(-0.01, 0.01)
            
            points.append(tuple(point))
        
        # 添加道路标线
        self.add_road_markings(points, forward, right)
        
        # 添加路边环境
        self.add_roadside_environment(points, forward, right)
        
        return points

    def add_road_markings(self, points, forward, right):
        """添加道路标线"""
        # 车道分隔线
        line_points = 400
        for i in range(line_points):
            dist = i * 0.5  # 每0.5米一个点
            
            # 中心线（虚线）
            if (int(dist) % 3) < 2:  # 虚线效果
                for _ in range(3):  # 每个位置生成多个点使线条更粗
                    offset = np.random.uniform(-0.05, 0.05)
                    center_point = self.car_pos + forward * dist + right * offset
                    points.append(tuple(center_point))
            
            # 左右车道线（实线）
            for _ in range(3):  # 加粗车道线
                offset = np.random.uniform(-0.05, 0.05)
                left_lane = self.car_pos + forward * dist + right * (self.lane_width/2 + offset)
                right_lane = self.car_pos + forward * dist + right * (-self.lane_width/2 + offset)
                points.append(tuple(left_lane))
                points.append(tuple(right_lane))
            
            # 道路边缘线
            for _ in range(3):  # 加粗边缘线
                offset = np.random.uniform(-0.05, 0.05)
                left_edge = self.car_pos + forward * dist + right * (self.road_width/2 + offset)
                right_edge = self.car_pos + forward * dist + right * (-self.road_width/2 + offset)
                points.append(tuple(left_edge))
                points.append(tuple(right_edge))

    def add_roadside_environment(self, points, forward, right):
        """添加路边环境（如路障、树木等）"""
        # 在路边随机添加物体
        for side in [-1, 1]:  # 左右两侧
            for i in range(20):  # 增加路边物体数量
                dist = np.random.uniform(5, 35)
                width = side * (self.road_width/2 + np.random.uniform(1, 4))
                
                base_point = self.car_pos + forward * dist + right * width
                
                # 生成垂直的点柱（模拟路灯或树木）
                height = np.random.uniform(3, 6)
                for h in np.linspace(0, height, 20):  # 增加垂直点数
                    # 添加一些随机偏移使物体更自然
                    offset_x = np.random.uniform(-0.1, 0.1)
                    offset_y = np.random.uniform(-0.1, 0.1)
                    point = base_point + np.array([offset_x, offset_y, h])
                    points.append(tuple(point))
                    
                    # 添加一些横向点使物体更丰满
                    for _ in range(3):
                        rad = np.random.uniform(0, 0.3)
                        angle = np.random.uniform(0, 2 * np.pi)
                        extra_point = point + np.array([rad * math.cos(angle), 
                                                      rad * math.sin(angle), 
                                                      np.random.uniform(-0.1, 0.1)])
                        points.append(tuple(extra_point))

    def pack_data(self, points: list) -> bytes:
        """打包数据"""
        # 计算单个点的大小（3个float坐标 + 1个float强度 = 16字节）
        point_size = 16  # 4 * 4 bytes
        # 计算车辆位置的大小（3个float = 12字节）
        pos_size = 12    # 3 * 4 bytes
        
        # 计算可以在一个包中发送的最大点数
        # UDP推荐最大包大小约为65507字节
        max_points_per_packet = (65507 - pos_size) // point_size
        
        # 如果点数过多，只取前面的点
        if len(points) > max_points_per_packet:
            points = points[:max_points_per_packet]
        
        # 打包车辆位置
        data = struct.pack('fff', 
                         self.car_pos[0],
                         self.car_pos[1],
                         self.car_pos[2])
        
        # 打包点云数据
        for point in points:
            data += struct.pack('ffff', 
                              point[0], point[1], point[2], 0.0)
        
        # 更新统计信息
        self.total_points_sent += len(points)
        self.frame_counter += 1
        
        current_time = time.time()
        if current_time - self.last_print_time >= 1.0:
            fps = self.frame_counter / (current_time - self.last_print_time)
            print(f"统计信息: 点数={len(points)}, "
                  f"总计点数={self.total_points_sent}, "
                  f"FPS={fps:.1f}, "
                  f"数据包大小={len(data)/1024:.1f}KB")
            
            self.frame_counter = 0
            self.last_print_time = current_time
            
        return data

    def run(self):
        """主循环"""
        try:
            print("开始发送车辆点云数据...")
            while self.running:
                # 更新车辆位置
                self.update_car_position()
                
                # 生成点云
                points = self.generate_road_points()
                data = self.pack_data(points)
                
                try:
                    self.sock.sendto(data, (self.UDP_IP, self.UDP_PORT))
                except Exception as e:
                    print(f"发送错误: {e}")
                    print(f"点数: {len(points)}")
                
                time.sleep(0.05)  # 20Hz
                self.time += 0.05
                
        except KeyboardInterrupt:
            print("正在停止发送...")
        finally:
            print(f"总发送点数: {self.total_points_sent}")
            self.sock.close()

    def stop(self):
        """停止发送"""
        self.running = False

def main():
    sender = CarPointCloudSender()
    try:
        sender.run()
    except KeyboardInterrupt:
        sender.stop()
        print("\n程序已停止")

if __name__ == "__main__":
    main()