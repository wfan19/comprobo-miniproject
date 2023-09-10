import tty
import select
import sys
import termios

import rclpy
from rclpy.node import Node, Publisher

from geometry_msgs.msg import Twist

class TeleopNode(Node):
    cmd_vel_pub: Publisher = None

    def __init__(self):
        super().__init__("Teleop")

        period = 1/20
        self.timer = self.create_timer(period, self.timer_tick)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def timer_tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        # Check for keyboard input
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print(now)
        print(key)

        if key == '\x03':
            raise KeyboardInterrupt

def main():
    rclpy.init()
    teleop_node = TeleopNode()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()