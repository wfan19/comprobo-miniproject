import rclpy
from rclpy.node import Node, Publisher, Subscription, Service

from geometry_msgs.msg import Twist
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

    rho = 0
    theta = 0
    
    last_rho_error = 0
    
    def __init__(self):
        super().__init__("WallFollowerNode")
        
        self.rho_sub =self.create_subscription(Float32, "/wall/rho", self.on_rho, 10)
        self.theta_sub = self.create_subscription(Float32, "/wall/theta", self.on_theta, 10)

        control_hz = 50;
        self.timer = self.create_timer(1/control_hz, self.on_timer)
        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 2)

    def on_rho(self, msg):
        self.rho = msg.data
    
    def on_theta(self, msg):
        self.theta = msg.data

    def on_timer(self):
        ## Compute current control output
        # Check if the wall is on our left or right side based on the theta parameter
        wall_side = -1 if self.theta > np.pi else 1

        rho_goal = wall_side * 0.4
        rho_error = rho_goal - self.rho

        # theta = self.wall[1]
        # if dist_to_wall < 0:
        #     theta = theta - np.pi
        # r_goal = np.exp((self.wall[1] % np.pi) *1j)
        # r_self = np.exp(0*1j)
        # r_diff = r_goal / r_self
        # theta_error = np.imag(np.log(r_diff)) % 3.14

        theta_goal = np.pi/2
        theta_error = theta_goal - self.theta

        # Compute the heading setpoint (outer loop)
        # kP_theta = -0.5
        kP_theta = -0
        kP_rho = -1
        kD_rho = 0

        print(f"Theta: {self.theta}, Rho: {self.rho} Rho raw: {self.rho}")
        print(f"Theta error: {theta_error}, Rho error: {rho_error}")

        # Compute the angular velocity setpoint (inner loop)
        omega = kP_theta * theta_error + kP_rho * rho_error + kD_rho * (rho_error - self.last_rho_error) * (1/50)
        self.last_rho_error = rho_error

        # Publish the message
        v = 0.15
        twist_out = twist_v_to_msg(np.array([v, 0, 0, 0, 0, omega]))
        self.twist_pub.publish(twist_out)

def main():
    rclpy.init()

    wall_follower_node = WallFollowerNode()
    rclpy.spin(wall_follower_node)

    wall_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()