# antbot_drive_controller/cmd_vel_to_stamped.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class Adapter(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_stamped')
        in_topic  = self.declare_parameter('input_topic', '/cmd_vel').get_parameter_value().string_value
        out_topic = self.declare_parameter('output_topic', '/tricycle_controller/cmd_vel').get_parameter_value().string_value
        self.pub = self.create_publisher(TwistStamped, out_topic, 10)
        self.sub = self.create_subscription(Twist, in_topic, self.cb, 10)
        self.get_logger().info(f'Bridge: {in_topic} (Twist) -> {out_topic} (TwistStamped)')

    def cb(self, msg: Twist):
        st = TwistStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'base_link'
        st.twist = msg
        self.pub.publish(st)

def main():
    rclpy.init()
    rclpy.spin(Adapter())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
