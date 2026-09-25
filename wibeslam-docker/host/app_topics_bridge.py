#!/usr/bin/env python3
"""
Мост для приложения "ROS Robot":
Публикует топики, которые ожидает приложение:
- /robot_pose (PoseStamped) — текущая поза робота
- /scan_points (Path) — траектория робота на карте
- /laserscan_to_pointcloud (PointCloud2) — облако точек лидара (опционально)
"""
import struct
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
from collections import deque

class AppTopicsBridge(Node):
    def __init__(self):
        super().__init__('app_topics_bridge')
        
        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Паблишеры
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.path_pub = self.create_publisher(Path, '/scan_points', 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/laserscan_to_pointcloud', 10)
        
        # Подписка на LaserScan
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # История траектории (последние 500 точек)
        self.trajectory = deque(maxlen=500)
        
        # Таймеры
        self.create_timer(0.1, self.publish_pose_and_path)  # 10 Hz
        
        self.get_logger().info('AppTopicsBridge started')

    def scan_callback(self, msg):
        """Конвертирует LaserScan в PointCloud2"""
        points = []
        angle = msg.angle_min
        
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0
                points.append((x, y, z))
            angle += msg.angle_increment
        
        if not points:
            return
        
        pc = PointCloud2()
        pc.header = msg.header
        pc.height = 1
        pc.width = len(points)
        pc.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        pc.is_bigendian = False
        pc.point_step = 12
        pc.row_step = 12 * len(points)
        pc.is_dense = True
        pc.data = b''.join(struct.pack('<fff', *p) for p in points)
        
        self.pc_pub.publish(pc)

    def publish_pose_and_path(self):
        """Публикует текущую позу и траекторию"""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            # Текущая поза
            pose_stamped = PoseStamped()
            pose_stamped.header = transform.header
            pose_stamped.pose.position = transform.transform.translation
            pose_stamped.pose.orientation = transform.transform.rotation
            
            self.pose_pub.publish(pose_stamped)
            
            # Добавляем в траекторию (только если позиция изменилась)
            current_pos = transform.transform.translation
            if self.trajectory:
                last_pos = self.trajectory[-1].pose.position
                dist = math.sqrt(
                    (current_pos.x - last_pos.x)**2 + 
                    (current_pos.y - last_pos.y)**2
                )
                if dist > 0.05:  # добавляем точку каждые 5 см
                    self.trajectory.append(pose_stamped)
            else:
                self.trajectory.append(pose_stamped)
            
            # Публикуем траекторию
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = self.get_clock().now().to_msg()
            path.poses = list(self.trajectory)
            
            self.path_pub.publish(path)
            
        except Exception as e:
            # TF ещё не готов
            pass

def main():
    rclpy.init()
    node = AppTopicsBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()