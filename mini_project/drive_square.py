import rclpy
from rclpy.node import Node
from datetime import datetime

from geometry_msgs.msg import Twist

# from abc import ABC

# class State(ABC):
#     @staticmethod
#     def execute():
#         pass

#     def exit():
#         pass

# class Forward(ABC)


class DriveSquareNode(Node):
    cmd_vel_pub = None

    state = "forward"

    last_state_change_time = 0
    max_state_times = {
        "forward": 5000,
        "turning": 2000,
    }
    
    def __init__(self):
        super().__init__("drive_square_pub")
        self.cmd_vel_pub = self.create_publisher(Twist)
        
        timer_period = 1/50;
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
        now = self.get_clock().now()
        state_time = now - self.last_state_change_time
        
        max_state_time = self.max_state_times[self.state]
        if state_time > max_state_time: 
            # Call the corresponding state transition function
            self.state_transition_funcs[self.state]()
        else:
            self.state_funcs[self.state]()

    def exec_forward(self):
        # Publish a move-forward twist command
        pass

    def exec_turning(self):
        # Publish a rotate-in-place twist command
        pass

    def exit_forward(self):
        # Publish a zero twist command
        # Switch the state to turning
        pass

    def exit_turning(self):
        # Publish a zero twist command
        # Switch the state to forward
        pass

def main(args=None):
    rclpy.init(args=args)

if __name__ == "__main__":
    main()