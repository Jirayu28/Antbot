#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameFix(Node):
    def __init__(self):
        super().__init__('scan_frame_fix')
        # พารามิเตอร์กรอบปลายทาง ปรับได้จาก launch
        self.declare_parameter('target_frame', 'lidar_sensor_link')
        self.target = self.get_parameter('target_frame').get_parameter_value().string_value
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

    def cb(self, msg: LaserScan):
        msg.header.frame_id = self.target
        self.pub.publish(msg)

def main():
    rclpy.init()
    n = ScanFrameFix()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
