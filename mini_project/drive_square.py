import rclpy
from rclpy.node import Node, Publisher

from geometry_msgs.msg import Twist
import numpy as np

# from abc import ABC

# class State(ABC):
#     @staticmethod
#     def execute():
#         pass

#     def exit():
#         pass

# class Forward(ABC)

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

class DriveSquareNode(Node):
    cmd_vel_pub: Publisher = None

    state = "turning"

    last_state_change_time = 0
    max_state_times = {
        "forward": 10,  # s
        "turning": 10,  # s
    }
    
    def __init__(self):
        super().__init__("drive_square_pub")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        timer_period = 1/20;
        self.timer = self.create_timer(timer_period, self.timer_tick)

        self.state_funcs = {
            "forward": self.exec_forward,
            "turning": self.exec_turning,
        }

        self.exit_funcs = {
            "forward": self.exit_forward,
            "turning": self.exit_turning,
        }

    def timer_tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        state_time = now - self.last_state_change_time
        
        max_state_time = self.max_state_times[self.state]
        if state_time > max_state_time: 
            # Call the corresponding state transition function
            self.exit_funcs[self.state]()
            self.last_state_change_time = now
        else:
            # Call the corresponding state-execution function
            self.state_funcs[self.state]()

    def exec_forward(self):
        # Publish a move-forward twist command
        v = 0.5 / (self.max_state_times["forward"])
        v_twist = np.array([v, 0, 0, 0, 0, 0])
        twist_out = twist_v_to_msg(v_twist)
        self.cmd_vel_pub.publish(twist_out)

    def exec_turning(self):
        # Publish a rotate-in-place twist command
        omega = (np.pi/2) / (self.max_state_times["turning"])
        v_twist = np.array([0, 0, 0, 0, 0, omega])
        twist_out = twist_v_to_msg(v_twist)
        self.cmd_vel_pub.publish(twist_out)

    def exit_forward(self):
        # Publish a zero twist command
        v_twist = np.array([0, 0, 0, 0, 0, 0])
        twist_out = twist_v_to_msg(v_twist)
        self.cmd_vel_pub.publish(twist_out)

        # Switch the state to turning
        self.state = "turning"

    def exit_turning(self):
        # Publish a zero twist command
        v_twist = np.array([0, 0, 0, 0, 0, 0])
        twist_out = twist_v_to_msg(v_twist)
        self.cmd_vel_pub.publish(twist_out)

        # Switch the state to forward
        self.state = "forward"

def main(args=None):
    rclpy.init(args=args)
    drive_square_node = DriveSquareNode()
    
    rclpy.spin(drive_square_node)
    
    drive_square_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()