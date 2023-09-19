import rclpy
from rclpy.node import Node, Publisher, Subscription, Service

from geometry_msgs.msg import Pose, Vector3, Point, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np

class PersonDetector(Node):

    current_scan_points: np.ndarray
    scan_sub: Subscription = None
    marker_pub: Publisher = None
    person_pub: Publisher = None

    def __init__(self):
        super().__init__("PersonDetectorNode")
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.marker_pub = self.create_publisher(Marker, "/marker", 10)
        self.person_pub = self.create_publisher(Point, "/person", 10)
    
    def on_scan(self, scan_msg):
        rs = np.array(scan_msg.ranges)
        thetas = np.linspace(0, 2*np.pi, len(rs))

        i_far_enough = rs > 0.2
        rs = rs[i_far_enough]
        thetas = thetas[i_far_enough]

        i_not_inf = ~np.isinf(rs)
        rs = rs[i_not_inf]
        thetas = thetas[i_not_inf]

        xs = np.cos(thetas) * rs
        ys = np.sin(thetas) * rs
        xy = np.array([xs, ys])
        self.current_scan_points = xy

        search_x = [0.5, 2]
        search_y = [-3, 3]

        in_x = (xy[0, :] > search_x[0]) & (xy[0, :] < search_x[1])
        in_y = (xy[1, :] > search_y[0]) & (xy[0, :] < search_y[1])
        in_bounds = in_x & in_y

        xy = np.diag([-1, -1]) @ xy
        xy_in_bounds = xy[:, in_bounds]
        means = np.average(xy_in_bounds, axis=1)

        # Flip it to align the frame with the robot frame
        person_x = means[0]
        person_y = means[1]

        self.target = np.array([person_x, person_y])

        marker_out = Marker(
            id = 0,
            ns = "/wall_markers",
            type=Marker.CUBE,
            action=Marker.ADD,
            header = scan_msg.header,
            pose=Pose(position=Point(x=person_x, y=person_y, z=0.)),
            color = ColorRGBA(a=1., r=0., g=1., b=0.),
            scale = Vector3(x=.2, y=.2, z=.2)
        )

        self.marker_pub.publish(marker_out)
        self.person_pub.publish(Point(x=person_x, y=person_y, z=0.))

def main():
    rclpy.init()

    node = PersonDetector()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()