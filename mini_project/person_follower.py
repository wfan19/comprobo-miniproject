
import rclpy
from rclpy.node import Node, Publisher, Subscription, Service

from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32

import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt

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

class WallFollowerNode(Node):

    scan_sub: Subscription = None
    hough_serv: Service = None
    marker_pub: Publisher = None
    twist_pub: Publisher = None
    current_scan_points: np.ndarray = None

    last_person_rtheta = np.array([0, 0])
    person = np.array([0, 0])
    
    def __init__(self):
        super().__init__("WallFollowerNode")
        
        self.person_sub =self.create_subscription(Point, "/person", self.on_person, 3)

        control_hz = 5;
        self.timer = self.create_timer(1/control_hz, self.on_timer)
        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 2)

    def on_person(self, msg: Point):
        self.person = np.array([msg.x, msg.y, msg.z])

    def on_timer(self):
        # Convert the person coords into polar:
        r = (self.person[0]**2 + self.person[1]**2) ** 0.5
        theta = np.arctan2(self.person[1], self.person[0])
        person_rtheta = np.array([r, theta])
        d_rtheta = person_rtheta - self.last_person_rtheta
        
        state = np.concatenate([person_rtheta, d_rtheta])
        target_state = np.array([0, 0, 0, 0])
        error = target_state - state

        K_v = np.array([-0.4, 0, 0.05, 0])      # K_p_r, K_p_theta, K_d_r, K_d_theta
        K_omega = np.array([0, 0.05, 0, 0.1])   # K_p_r, K_p_theta, K_d_r, K_d_theta 

        v_x =  K_v @ error
        omega =  K_omega @ error
        omega *= v_x        # Scale rotation gains by the current linear speed: the closer we are, the slower we want to rotate to minimize overshoot

        twist_cmd = np.array([v_x, 0, 0, 0, 0, omega])
        
        twist_out = twist_v_to_msg(twist_cmd)
        self.twist_pub.publish(twist_out)
        self.last_person_r_theta = person_rtheta

def main():
    rclpy.init()

    wall_follower_node = WallFollowerNode()
    rclpy.spin(wall_follower_node)

    wall_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()