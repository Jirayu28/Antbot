# -*- coding: utf-8 -*-
import sys, termios, tty, select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
Teleop (WSAD) : publish /cmd_vel (Twist)
  w/s : linear +/- 
  a/d : angular +/- 
  x/h : reduce linear/angular
  SPACE: stop
  q    : quit
"""

def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lin = 0.0
        self.ang = 0.0
        self.step_lin = 0.05
        self.step_ang = 0.1
        self.timer = self.create_timer(0.05, self.tick)
        print(HELP)

    def tick(self):
        key = get_key(0.01)
        if key == 'w': self.lin += self.step_lin
        elif key == 's': self.lin -= self.step_lin
        elif key == 'a': self.ang += self.step_ang
        elif key == 'd': self.ang -= self.step_ang
        elif key == 'x': self.lin *= 0.5
        elif key == 'h': self.ang *= 0.5
        elif key == ' ': self.lin = 0.0; self.ang = 0.0
        elif key == 'q':
            print("\nBye.")
            rclpy.shutdown()
            return

        msg = Twist()
        msg.linear.x = self.lin
        msg.angular.z = self.ang
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
