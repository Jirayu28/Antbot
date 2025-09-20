#!/usr/bin/env python3
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu

class ImuFrameRelay(Node):
    def __init__(self):
        super().__init__('imu_frame_relay')
        self.declare_parameter('input_topic', '/imu')
        self.declare_parameter('output_topic', '/imu')
        self.declare_parameter('frame_id', 'imu_link')

        in_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.sub = self.create_subscription(Imu, in_topic, self.cb, qos)
        self.pub = self.create_publisher(Imu, out_topic, qos)
        self.get_logger().info(f'IMU relay: {in_topic} -> {out_topic} (frame_id={self.frame_id})')

    def cb(self, msg: Imu):
        out = copy.deepcopy(msg)
        out.header.frame_id = self.frame_id

        def _fix_cov(cov):
            if all(c == 0.0 for c in cov):
                cov[0] = cov[4] = cov[8] = 1e-3  # diag
            return cov

        out.orientation_covariance = _fix_cov(list(out.orientation_covariance))
        out.angular_velocity_covariance = _fix_cov(list(out.angular_velocity_covariance))
        out.linear_acceleration_covariance = _fix_cov(list(out.linear_acceleration_covariance))
        self.pub.publish(out)

def main():
    rclpy.init()
    node = ImuFrameRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
