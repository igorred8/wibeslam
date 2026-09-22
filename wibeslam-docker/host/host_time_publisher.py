#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import time

class HostTimePublisher(Node):
    def __init__(self):
        super().__init__('host_time_publisher')
        self.publisher_ = self.create_publisher(Int64, '/host_time', 10)
        self.timer = self.create_timer(1.0, self.publish_time)

    def publish_time(self):
        msg = Int64()
        msg.data = int(time.time() * 1000)  # время в миллисекундах
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = HostTimePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()