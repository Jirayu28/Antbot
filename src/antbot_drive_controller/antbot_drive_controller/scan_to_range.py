# antbot_drive_controller/scan_to_range.py
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range

class ScanToRange(Node):
    def __init__(self):
        super().__init__('scan_to_range')
        # พารามิเตอร์ปรับได้จาก launch
        self.topic_in  = self.declare_parameter('topic_in',  '/ultrasonic').get_parameter_value().string_value
        self.topic_out = self.declare_parameter('topic_out', '/ultrasonic_range').get_parameter_value().string_value
        self.window_deg = float(self.declare_parameter('window_deg', 10.0).value)   # กว้างมุมรอบแกนหน้า
        self.use_min = bool(self.declare_parameter('use_min', True).value)          # ใช้ค่าต่ำสุดในหน้าต่าง

        self.sub = self.create_subscription(LaserScan, self.topic_in, self.cb_scan, 10)
        self.pub = self.create_publisher(Range, self.topic_out, 10)

        self.get_logger().info(f'Listening: {self.topic_in} -> Publishing Range: {self.topic_out}')

    def cb_scan(self, msg: LaserScan):
        # ตั้งหน้าต่างรอบมุม 0 องศา (ทิศ "หน้า" ของเฟรมเซนเซอร์)
        half = math.radians(self.window_deg) / 2.0
        inc  = msg.angle_increment

        # คำนวณอินเด็กซ์ขอบหน้าต่าง
        start_ang = max(msg.angle_min, -half)
        end_ang   = min(msg.angle_max,  half)
        start_i = max(0, int(round((start_ang - msg.angle_min) / inc)))
        end_i   = min(len(msg.ranges)-1, int(round((end_ang - msg.angle_min) / inc)))

        # ดึงค่าระยะในหน้าต่าง
        window = [r for r in msg.ranges[start_i:end_i+1] if math.isfinite(r)]
        if not window:
            rng = msg.range_max  # ไม่เห็นก็ใส่ไกลสุด
        else:
            rng = min(window) if self.use_min else sum(window)/len(window)

        out = Range()
        out.header = msg.header                     # ใช้ frame เดียวกับสแกน
        out.radiation_type = Range.ULTRASOUND       # อัลตราโซนิก
        out.field_of_view  = (end_i - start_i + 1) * inc
        out.min_range      = msg.range_min
        out.max_range      = msg.range_max
        out.range          = float(rng)

        self.pub.publish(out)

def main():
    rclpy.init()
    node = ScanToRange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
