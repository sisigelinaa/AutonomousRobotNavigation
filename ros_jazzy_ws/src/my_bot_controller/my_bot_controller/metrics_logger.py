import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty


class MetricsLogger(Node):
    """
    Records navigation metrics during a scenario run and saves two CSV files
    on completion:

      {sensor_type}_{timestamp}_timestep.csv  — 10 Hz per-sample data
      {sensor_type}_{timestamp}_goals.csv     — one row per goal
    """

    def __init__(self):
        super().__init__('metrics_logger')

        self.declare_parameter('sensor_type', 'unknown')
        self.declare_parameter('output_dir', os.path.expanduser('~/ros_metrics'))

        self.sensor_type = self.get_parameter('sensor_type').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        os.makedirs(self.output_dir, exist_ok=True)

        # ---- per-run state ----
        self.run_start = self.get_clock().now()

        # ---- per-goal state ----
        self.current_goal_id = -1
        self.current_goal_xy = (0.0, 0.0)
        self.goal_start_time = None
        self.path_length = 0.0
        self.replan_count = 0
        self.last_odom_x = None
        self.last_odom_y = None

        # ---- latest sensor values ----
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.kf_x = 0.0
        self.kf_y = 0.0
        self.meas_x = 0.0
        self.meas_y = 0.0
        self.amcl_cov_trace = 0.0

        # ---- accumulated rows ----
        self.timestep_rows = []
        self.goal_rows = []

        # ---- subscriptions ----
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(PoseStamped, '/kf_estimate', self._kf_cb, 10)
        self.create_subscription(PoseStamped, '/noisy_measurement', self._meas_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10
        )
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_pose_cb, 10)
        self.create_subscription(Bool, '/goal_reached', self._goal_reached_cb, 10)
        self.create_subscription(Empty, '/replan_event', self._replan_cb, 10)
        self.create_subscription(Bool, '/scenario_complete', self._scenario_done_cb, 10)

        # record at 10 Hz
        self.create_timer(0.1, self._record_timestep)

        self.get_logger().info(
            f'MetricsLogger ready. sensor_type={self.sensor_type}  '
            f'output={self.output_dir}'
        )

    # ------------------------------------------------------------------ #
    # Subscription callbacks                                               #
    # ------------------------------------------------------------------ #

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_odom_x is not None and self.current_goal_id >= 0:
            self.path_length += math.hypot(x - self.last_odom_x, y - self.last_odom_y)
        self.last_odom_x = x
        self.last_odom_y = y
        self.odom_x = x
        self.odom_y = y

    def _kf_cb(self, msg: PoseStamped):
        self.kf_x = msg.pose.position.x
        self.kf_y = msg.pose.position.y

    def _meas_cb(self, msg: PoseStamped):
        self.meas_x = msg.pose.position.x
        self.meas_y = msg.pose.position.y

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        # cov is a 36-element row-major 6×6 matrix; [0]=x_var, [7]=y_var
        self.amcl_cov_trace = cov[0] + cov[7]

    def _goal_pose_cb(self, msg: PoseStamped):
        # Finalise the previous goal if we never received goal_reached (timeout/skip)
        if self.current_goal_id >= 0 and self.goal_start_time is not None:
            self._save_goal_row(success=False)

        self.current_goal_id += 1
        self.current_goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.goal_start_time = self.get_clock().now()
        self.path_length = 0.0
        self.replan_count = 0
        self.last_odom_x = None
        self.last_odom_y = None

    def _goal_reached_cb(self, msg: Bool):
        if msg.data and self.current_goal_id >= 0 and self.goal_start_time is not None:
            self._save_goal_row(success=True)
            self.goal_start_time = None

    def _replan_cb(self, _msg: Empty):
        self.replan_count += 1

    def _scenario_done_cb(self, msg: Bool):
        if msg.data:
            self.save_csv()

    # ------------------------------------------------------------------ #
    # Recording                                                            #
    # ------------------------------------------------------------------ #

    def _record_timestep(self):
        if self.current_goal_id < 0:
            return
        elapsed = (self.get_clock().now() - self.run_start).nanoseconds / 1e9
        kf_err = math.hypot(self.kf_x - self.odom_x, self.kf_y - self.odom_y)
        meas_err = math.hypot(self.meas_x - self.odom_x, self.meas_y - self.odom_y)
        self.timestep_rows.append({
            'timestamp':      round(elapsed, 2),
            'sensor_type':    self.sensor_type,
            'goal_id':        self.current_goal_id,
            'odom_x':         round(self.odom_x, 4),
            'odom_y':         round(self.odom_y, 4),
            'kf_x':           round(self.kf_x, 4),
            'kf_y':           round(self.kf_y, 4),
            'meas_x':         round(self.meas_x, 4),
            'meas_y':         round(self.meas_y, 4),
            'amcl_cov_trace': round(self.amcl_cov_trace, 6),
            'kf_error':       round(kf_err, 4),
            'meas_error':     round(meas_err, 4),
        })

    def _save_goal_row(self, success: bool):
        elapsed = (
            (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
            if self.goal_start_time else 0.0
        )
        self.goal_rows.append({
            'sensor_type':  self.sensor_type,
            'goal_id':      self.current_goal_id,
            'goal_x':       self.current_goal_xy[0],
            'goal_y':       self.current_goal_xy[1],
            'time_to_goal': round(elapsed, 3),
            'path_length':  round(self.path_length, 3),
            'replan_count': self.replan_count,
            'success':      success,
        })

    # ------------------------------------------------------------------ #
    # CSV export                                                           #
    # ------------------------------------------------------------------ #

    def save_csv(self):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        if self.timestep_rows:
            path = os.path.join(
                self.output_dir, f'{self.sensor_type}_{ts}_timestep.csv'
            )
            with open(path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.timestep_rows[0].keys())
                writer.writeheader()
                writer.writerows(self.timestep_rows)
            self.get_logger().info(f'Saved timestep data → {path}')

        if self.goal_rows:
            path = os.path.join(
                self.output_dir, f'{self.sensor_type}_{ts}_goals.csv'
            )
            with open(path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.goal_rows[0].keys())
                writer.writeheader()
                writer.writerows(self.goal_rows)
            self.get_logger().info(f'Saved goal data    → {path}')

    def destroy_node(self):
        self.save_csv()
        super().destroy_node()


def main():
    rclpy.init()
    node = MetricsLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
