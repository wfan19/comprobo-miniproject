import tty
import select
import sys
import termios

from pynput import keyboard

import rclpy
from rclpy.node import Node, Publisher

from geometry_msgs.msg import Twist
import numpy as np

def twist_v_to_msg(v_twist: np.ndarray) -> Twist:
    twist_out = Twist()

    v_twist.dtype = float

    twist_out.linear.x = v_twist[0]
    twist_out.linear.y = v_twist[1]
    twist_out.linear.z = v_twist[2]
    twist_out.angular.x = v_twist[3]
    twist_out.angular.y = v_twist[4]
    twist_out.angular.z = v_twist[5]

    return twist_out
class TeleopNode(Node):
    cmd_vel_pub: Publisher = None

    v = 0.3
    omega = 0.5
    twist_base = np.array([v, 0, 0, 0, 0, omega], dtype=float)
    dirs_out = np.diag([0, 0, 0, 0, 0, 0])
    

    def __init__(self):
        super().__init__("Teleop")

        period = 1/20
        self.timer = self.create_timer(period, self.timer_tick)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.settings = termios.tcgetattr(sys.stdin)

        def on_press(key):
            u_key = getattr(key, 'char', None)
            if u_key == u'w':
                self.dirs_out[0, 0] = 1
            elif u_key == u's':
                self.dirs_out[0, 0] = -1
            elif u_key == u'a' or u_key == u'q':
                self.dirs_out[5, 5] = 1
            elif u_key == u'd' or u_key == u'e':
                self.dirs_out[5, 5] = -1

        def on_release(key):
            u_key = getattr(key, 'char', None)
            if u_key == u'w' or u_key == u's':
                self.dirs_out[0, 0] = 0
            elif u_key == u'a' or u_key == u'q' or u_key == u'e' or u_key == u'd':
                self.dirs_out[5, 5] = 0
        
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

    def timer_tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        twist_out = self.dirs_out @ self.twist_base
        self.cmd_vel_pub.publish(twist_v_to_msg(twist_out))

def main():
    rclpy.init()
    teleop_node = TeleopNode()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()