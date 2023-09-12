import rclpy
from rclpy.node import Node, Publisher, Subscription, Service

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt

class WallFollowerNode(Node):

    scan_sub: Subscription = None
    hough_serv: Service = None
    current_scan_points: np.ndarray = None
    
    def __init__(self):
        super().__init__("WallFollowerNode")
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.hough_serv = self.create_service(Empty, "/plot_hough", self.plot_hough)
    
    def on_scan(self, scan_msg):
        rs = np.array(scan_msg.ranges)
        thetas = np.linspace(0, 2*np.pi, len(rs))

        i_far_enough = rs > 0.1
        rs = rs[i_far_enough]
        thetas = thetas[i_far_enough]
        
        xs = np.cos(thetas) * rs
        ys = np.sin(thetas) * rs
        self.current_scan_points = np.array([xs, ys])

    def plot_hough(self, request, response):
        xy = self.current_scan_points

        ## Perform hough transform of the scan points
        # For each point, sweep through theta-space to find the rho-values
        n_scans = xy.shape[1]
        n_sample_pts = 100
        thetas = np.linspace(0, np.pi, n_sample_pts)

        rhos = np.zeros([n_scans, n_sample_pts])
        for i, pt in enumerate(xy.T):
            rhos[i, :] = pt[0] * np.sin(thetas) + pt[1] * np.cos(thetas)

        thetas_all = np.matlib.repmat(thetas, 1, n_scans).flatten();
        rhos_all = rhos.flatten()

        heatmap, theta_edges, rho_edges = np.histogram2d(thetas_all, rhos_all, bins=(100, 200))
        extent = [theta_edges[0], theta_edges[-1], rho_edges[0], rho_edges[-1]]        

        i_max = np.argmax(heatmap.T)
        rho_wall = rhos_all[i_max]
        theta_wall = thetas_all[i_max]

        fig, axs = plt.subplots(1, 2)
        axs[0].scatter(xy[0, :], xy[1, :])
        axs[0].axis('equal')
        axs[0].set_xlabel("X (m)")
        axs[0].set_ylabel("Y (m)")
        axs[0].set_title("World space")

        axs[1].imshow(heatmap.T, extent=extent, origin='lower')
        axs[1].set_title("Parameter space")
        axs[1].set_xlabel("theta (rad)")
        axs[1].set_ylabel("rho (m)")
        
        plt.show()

        return response


def main():
    rclpy.init()

    wall_follower_node = WallFollowerNode()
    rclpy.spin(wall_follower_node)

    wall_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()