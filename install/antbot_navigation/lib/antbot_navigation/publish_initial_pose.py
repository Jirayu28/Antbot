#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

class InitialPoseOnce(Node):
    def __init__(self):
        super().__init__(
            'publish_initial_pose',
            automatically_declare_parameters_from_overrides=True
        )

        # -------- parameters from launch --------
        frame_id = self._get_param('frame_id', 'map')
        x        = float(self._get_param('x', 0.0))
        y        = float(self._get_param('y', 0.0))
        yaw_deg  = float(self._get_param('yaw_deg', 0.0))

        wait_for_subs_sec = float(self._get_param('wait_for_subscribers_sec', 3.0))
        extra_delay_sec   = float(self._get_param('extra_delay_sec', 1.0))
        backstamp_sec     = float(self._get_param('backstamp_sec', 0.5))
        self.tf_parent    = str(self._get_param('tf_parent', 'odom'))
        self.tf_child     = str(self._get_param('tf_child',  'base_link'))

        # เผื่อกับ transform_tolerance ของ AMCL (วินาที) + กันชนเล็กน้อย
        self.amcl_transform_tolerance = float(self._get_param('amcl_transform_tolerance', 0.10))
        self.eps_sec = 0.02  # 20 ms

        # ต้องการให้ TF ล่าสุดห่างจาก now ไม่เกิน margin นี้
        self.tf_fresh_margin = Duration(seconds=0.015)

        # QoS: ทนทาน ให้ AMCL ที่มาทีหลังยังรับได้
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)

        # เตรียมข้อความ
        yaw = math.radians(yaw_deg)
        self.msg = PoseWithCovarianceStamped()
        self.msg.header.frame_id = frame_id
        self.msg.pose.pose.position.x = x
        self.msg.pose.pose.position.y = y
        self.msg.pose.pose.position.z = 0.0
        self.msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        self.msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.msg.pose.covariance = [
            0.25, 0,    0, 0, 0, 0,
            0,    0.25, 0, 0, 0, 0,
            0,    0,    1e6,0, 0, 0,
            0,    0,    0, 1e6,0, 0,
            0,    0,    0, 0, 1e6,0,
            0,    0,    0, 0, 0,   0.07
        ]
        self.backstamp = Duration(seconds=backstamp_sec)

        # TF
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        # สถานะรอ
        self.deadline    = self.get_clock().now() + Duration(seconds=wait_for_subs_sec)
        self.extra_delay = Duration(seconds=extra_delay_sec)
        self.start_time  = self.get_clock().now()
        self.wait_timer  = self.create_timer(0.05, self._wait_tick)

        self.get_logger().info(
            f'publish_initial_pose: waiting for subs & TF ({self.tf_parent}->{self.tf_child}); '
            f'backstamp={backstamp_sec:.2f}s, extra_delay={extra_delay_sec:.2f}s, '
            f'amcl_tol={self.amcl_transform_tolerance:.2f}s'
        )

    # -------- utils --------
    def _get_param(self, name, default):
        try:
            if self.has_parameter(name):
                return self.get_parameter(name).value
        except Exception:
            pass
        return default

    def _tf_ready(self) -> bool:
        try:
            return self.tf_buf.can_transform(
                self.tf_parent, self.tf_child, Time(), timeout=Duration(seconds=0.1)
            )
        except Exception:
            return False

    def _latest_tf_time(self) -> Time | None:
        try:
            t = self.tf_buf.lookup_transform(self.tf_parent, self.tf_child, Time())
            return Time.from_msg(t.header.stamp)
        except Exception:
            return None

    # -------- main flow --------
    def _wait_tick(self):
        now = self.get_clock().now()
        have_sub = (self.pub.get_subscription_count() > 0) or (now >= self.deadline)
        tf_ok    = self._tf_ready()
        delayed  = (now - self.start_time) >= self.extra_delay

        # TF ต้องสดพอ (now - tf_latest <= margin)
        fresh = False
        if tf_ok:
            t_tf = self._latest_tf_time()
            if t_tf is not None:
                fresh = (now - t_tf) <= self.tf_fresh_margin

        if not (have_sub and tf_ok and delayed and fresh):
            return

        self.wait_timer.cancel()
        self._send_once()

    def _send_once(self):
        now  = self.get_clock().now()
        t_tf = self._latest_tf_time()

        # ⬇⬇⬇ แก้ตรงนี้: รวม tolerances เป็นวินาที แล้วค่อยสร้าง Duration เดียว
        tol_plus_eps_sec = self.amcl_transform_tolerance + self.eps_sec
        if t_tf is not None:
            cand_a = t_tf - Duration(seconds=tol_plus_eps_sec)  # ไม่ใช้ (tol + eps)
            cand_b = now - self.backstamp
            if cand_a < cand_b:
                stamp_time = cand_a
                src = f"tf_latest({t_tf.nanoseconds/1e9:.3f})-({tol_plus_eps_sec:.3f}s)"
            else:
                stamp_time = cand_b
                src = "now-backstamp"
        else:
            stamp_time = now - self.backstamp
            src = "now-backstamp(fallback)"

        self.msg.header.stamp = stamp_time.to_msg()
        self.pub.publish(self.msg)
        self.get_logger().info(
            f'Initial pose published to /initialpose (stamp from {src}, '
            f'final={stamp_time.nanoseconds/1e9:.3f}s)'
        )
        rclpy.shutdown()

def main():
    rclpy.init()
    node = InitialPoseOnce()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
