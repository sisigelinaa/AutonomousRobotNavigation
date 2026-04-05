import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import numpy as np
import math

class RobotPlotter(Node):
    def __init__(self):
        super().__init__('robot_plotter')
        
        # --- Subscribers ---
        # 1. Ground Truth — Gazebo odometry (exact in simulation: no physical drift)
        self.create_subscription(Odometry, '/odom', self.gt_callback, 10)
        
        # 2. Kalman Filter Estimate
        self.create_subscription(PoseStamped, '/kf_estimate', self.kf_callback, 10)
        
        # 3. Noisy Measurements
        self.create_subscription(PoseStamped, '/noisy_measurement', self.meas_callback, 10)
        # self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.meas_callback, 10)
        # 4. Map
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))

        # --- Data Containers ---
        self.gt_x, self.gt_y = [], []
        self.kf_x, self.kf_y = [], []
        self.meas_x, self.meas_y = [], []
        
        # Error History (Distance from Ground Truth)
        self.time_steps = []
        self.error_kf = []
        self.error_meas = []
        self.step_count = 0

        self.map_data = None
        self.current_gt = None # Store latest GT for error calc

        # --- Plot Setup ---
        plt.ion()
        # Create 2 subplots: Map on top (big), Error on bottom (small)
        self.fig, (self.ax_map, self.ax_err) = plt.subplots(2, 1, figsize=(10, 10), 
                                                            gridspec_kw={'height_ratios': [3, 1]})
        self.fig.tight_layout(pad=3.0)

        # 1. MAP PLOT
        self.ax_map.set_title("Robot Path: Truth vs Filter vs Noise")
        self.ax_map.set_xlabel("X (m)")
        self.ax_map.set_ylabel("Y (m)")
        
        # Styles: GT is thick/transparent, KF is thin/solid on top
        self.gt_line, = self.ax_map.plot([], [], color='red', linewidth=4, alpha=0.5, label='Ground Truth')
        self.meas_dots, = self.ax_map.plot([], [], 'b.', markersize=3, alpha=0.3, label='Noisy GPS/AMCL')
        self.kf_line, = self.ax_map.plot([], [], color='lime', linestyle='--', linewidth=2, label='Kalman Filter', zorder=10)
        self.ax_map.legend(loc="upper right")

        # 2. ERROR PLOT
        self.ax_err.set_title("Real-Time Error Analysis (Lower is Better)")
        self.ax_err.set_xlabel("Time Steps")
        self.ax_err.set_ylabel("Position Error (meters)")
        self.ax_err.grid(True)
        
        self.err_meas_line, = self.ax_err.plot([], [], 'b-', linewidth=1, alpha=0.5, label='Raw Error')
        self.err_kf_line, = self.ax_err.plot([], [], 'g-', linewidth=2, label='Filter Error')
        self.ax_err.legend()

        print("Dual-Plot Visualizer Online...")

    def gt_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.gt_x.append(x)
        self.gt_y.append(y)
        self.current_gt = (x, y)
        
    def kf_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.kf_x.append(x)
        self.kf_y.append(y)
        
        # Calculate Error immediately if we have GT
        if self.current_gt:
            gt_x, gt_y = self.current_gt
            dist = math.hypot(gt_x - x, gt_y - y)
            self.error_kf.append(dist)
            
            # Sync timestamp
            if len(self.time_steps) < len(self.error_kf):
                self.time_steps.append(self.step_count)
                self.step_count += 1

    def meas_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.meas_x.append(x)
        self.meas_y.append(y)
        
        # Calculate Error immediately
        if self.current_gt:
            gt_x, gt_y = self.current_gt
            dist = math.hypot(gt_x - x, gt_y - y)
            self.error_meas.append(dist)

    def map_callback(self, msg):
        if self.map_data is None:
            w, h = msg.info.width, msg.info.height
            data = np.array(msg.data).reshape((h, w))
            extent = [
                msg.info.origin.position.x,
                msg.info.origin.position.x + w * msg.info.resolution,
                msg.info.origin.position.y,
                msg.info.origin.position.y + h * msg.info.resolution
            ]
            self.ax_map.imshow(data, cmap='gray_r', origin='lower', extent=extent, alpha=0.5)
            self.map_data = True

    def update_plot(self):
        # Map plot: show whatever data is available
        self.gt_line.set_data(self.gt_x, self.gt_y)
        self.kf_line.set_data(self.kf_x, self.kf_y)
        self.meas_dots.set_data(self.meas_x, self.meas_y)
        self.ax_map.relim()
        self.ax_map.autoscale_view()

        # Error plot: only meaningful once GT is available
        min_len = min(len(self.time_steps), len(self.error_kf), len(self.error_meas))
        if min_len > 0:
            self.err_meas_line.set_data(self.time_steps[:min_len], self.error_meas[:min_len])
            self.err_kf_line.set_data(self.time_steps[:min_len], self.error_kf[:min_len])
            self.ax_err.relim()
            self.ax_err.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main():
    import threading
    rclpy.init()
    node = RobotPlotter()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok():
            node.update_plot()
            plt.pause(0.2)  # Drives the GUI event loop; keeps window alive
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()