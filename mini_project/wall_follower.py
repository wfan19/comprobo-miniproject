import rclpy
from rclpy.node import Node, Publisher, Subscription, Service

from geometry_msgs.msg import Vector3, Point, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

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

    wall: np.ndarray = np.array([0, 0])
    
    last_rho_error = 0
    
    def __init__(self):
        super().__init__("WallFollowerNode")
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.hough_serv = self.create_service(Empty, "/plot_hough", self.plot_hough)
        self.marker_pub = self.create_publisher(Marker, "/wall_marker", 10)
        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 2)

        control_hz = 50;
        self.timer = self.create_timer(1/control_hz, self.on_timer)

    def on_timer(self):
        ## Compute current control output
        # Fetch the robot horizontal distance to wall, which is the Rho parameter of the wall line
        dist_to_wall = self.wall[0]

        # Check if the wall is on our left or right side based on the theta parameter
        wall_side = -1 if (self.wall[1] < np.pi/2 or self.wall[1] > 3*np.pi/2) else 1
        signed_dist_to_wall = dist_to_wall * wall_side

        rho_goal = 0.4
        rho_error = rho_goal - signed_dist_to_wall

        theta = self.wall[1]
        if dist_to_wall < 0:
            theta = theta - np.pi
        r_goal = np.exp((self.wall[1] % np.pi) *1j)
        r_self = np.exp(0*1j)
        r_diff = r_goal / r_self
        theta_error = np.imag(np.log(r_diff)) % 3.14

        # Compute the heading setpoint (outer loop)
        # kP_theta = -0.5
        kP_theta = -1
        kP_rho = -2
        kD_rho = -3

        print(f"Theta: {theta}, Rho: {signed_dist_to_wall} Rho raw: {dist_to_wall}")
        print(f"Theta error: {theta_error}, Rho error: {rho_error}")

        # Compute the angular velocity setpoint (inner loop)
        omega = kP_theta * theta_error + kP_rho * rho_error + kD_rho * (rho_error - self.last_rho_error) * (1/50)
        self.last_rho_error = rho_error

        # Publish the message
        v = 0.15
        twist_out = twist_v_to_msg(np.array([v, 0, 0, 0, 0, omega]))
        self.twist_pub.publish(twist_out)
    
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

        # # Publish markers for the wall that we've detected
        rho_wall, theta_wall, _1, _2 = self.hough_tform()

        theta_wall += np.pi     # Transform the wall from scanner-frame to world frame

        x_range = [np.min(xy[0, :]), np.max(xy[0, :])]
        wall_x = np.linspace(x_range[0], x_range[1], 100)
        wall_y = 1/np.cos(theta_wall) * (-np.sin(theta_wall) * wall_x + rho_wall)

        self.wall = np.array([rho_wall, theta_wall])

        # Flip it to align the frame with the robot frame
        points = [Point(x=x, y=y, z=0.0) for (x, y) in zip(wall_x, wall_y)]

        marker_out = Marker(
            id = 0,
            ns = "/wall_markers",
            type=Marker.CUBE_LIST,
            action=Marker.ADD,
            header = scan_msg.header,
            points = points,
            colors = [ColorRGBA(a=1., r=0., g=1., b=0.) for i in range(len(points))],
            scale = Vector3(x=.2, y=.2, z=.2)
        )

        self.marker_pub.publish(marker_out)

    def plot_hough(self, request, response):
        xy = self.current_scan_points
        rho_wall, theta_wall, heatmap, extent = self.hough_tform()

        print(f"Rho wall: {rho_wall}, Theta wall: {theta_wall}")

        # Plot the line we have singled out in the world space
        x_range = [np.min(xy[0, :]), np.max(xy[0, :])]
        wall_x = np.linspace(x_range[0], x_range[1], 100)
        wall_y = 1/np.cos(theta_wall) * (-np.sin(theta_wall) * wall_x + rho_wall)

        fig, axs = plt.subplots(1, 2)
        axs[0].scatter(xy[0, :], xy[1, :])
        axs[0].scatter(wall_x, wall_y)
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

    def hough_tform(self):
        xy = self.current_scan_points
        ## Perform hough transform of the scan points
        # For each point, sweep through theta-space to find the rho-values
        n_scans = xy.shape[1]
        n_sample_pts = 100
        thetas = np.linspace(0, np.pi, n_sample_pts)

        mat_rhos = np.zeros([n_scans, n_sample_pts])
        for i, pt in enumerate(xy.T):
            mat_rhos[i, :] = pt[0] * np.sin(thetas) + pt[1] * np.cos(thetas)

        mat_thetas = np.matlib.repmat(thetas, 1, n_scans)

        thetas_all = mat_thetas.flatten();
        rhos_all = mat_rhos.flatten()

        heatmap, theta_edges, rho_edges = np.histogram2d(thetas_all, rhos_all, bins=(100, 200))
        extent = [theta_edges[0], theta_edges[-1], rho_edges[0], rho_edges[-1]]        

        [R, T] = np.meshgrid(rho_edges[0:-1], theta_edges[0:-1])

        i_max = np.argmax(heatmap)
        rho_wall = R.flatten()[i_max]
        theta_wall = T.flatten()[i_max]

        return rho_wall, theta_wall, heatmap, extent

def main():
    rclpy.init()

    wall_follower_node = WallFollowerNode()
    rclpy.spin(wall_follower_node)

    wall_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()