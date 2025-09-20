#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserFrameFix(Node):
    def __init__(self):
        super().__init__('laser_frame_fix')
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.callback, 10)
        self.pub = self.create_publisher(
            LaserScan, '/scan_fixed', 10)

    def callback(self, msg):
        # 🔧 เปลี่ยน frame_id
        msg.header.frame_id = "lidar_link"
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserFrameFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
