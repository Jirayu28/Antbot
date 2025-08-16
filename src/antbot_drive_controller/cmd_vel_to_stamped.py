# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelToStamped(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_stamped')
        self.in_topic  = self.declare_parameter('in_topic',  '/cmd_vel').get_parameter_value().string_value
        self.out_topic = self.declare_parameter('out_topic', '/tricycle_controller/cmd_vel').get_parameter_value().string_value
        self.frame_id  = self.declare_parameter('frame_id', 'base_link').get_parameter_value().string_value

        self.pub = self.create_publisher(TwistStamped, self.out_topic, 10)
        self.sub = self.create_subscription(Twist, self.in_topic, self.cb, 10)

        self.get_logger().info(f'Bridging {self.in_topic} (Twist) -> {self.out_topic} (TwistStamped), frame_id={self.frame_id}')

    def cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(CmdVelToStamped())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
