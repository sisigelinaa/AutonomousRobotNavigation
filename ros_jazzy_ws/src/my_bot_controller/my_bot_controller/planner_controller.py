import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped, Point, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, Imu
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformException
from std_msgs.msg import Bool, Empty
import numpy as np
import math
import heapq
from .kalman_filter import RobotKalmanFilter

class ContinuousNavigator(Node):
    def __init__(self):
        super().__init__('continuous_navigator')
        
        # --- 1. Kalman Filter & State ---
        self.kf = RobotKalmanFilter(dt=0.1)
        self.initialized = False
        self.robot_pos = None 
        
        # --- Odom State ---
        self.odom_v = 0.0      # Measured linear velocity
        self.odom_omega = 0.0  # Measured angular velocity

        # --- 2. Navigation State ---
        self.goal_m = None
        self.grid = None       
        self.static_map = None 
        self.map_info = None
        self.current_path = []
        
        # --- 3. ROS Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.declare_parameter('scan_topic', '/scan')
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.imu_yaw = 0.0
  
        self.victim_detected = False

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Marker, '/planned_path', 1)
        self.kf_pub = self.create_publisher(PoseStamped, '/kf_estimate', 10)
        self.meas_pub = self.create_publisher(PoseStamped, '/noisy_measurement', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)
        self.replan_pub = self.create_publisher(Empty, '/replan_event', 10)
        self.victim_marker_pub = self.create_publisher(Marker, '/victim_marker', 10)

        self.create_subscription(Bool, '/victim_detected', self._victim_cb, 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Navigator Ready.')


    def _victim_cb(self, msg: Bool):
        if msg.data and not self.victim_detected:
            self.victim_detected = True
            self.get_logger().info('Victim detected — stopping robot and marking position.')
            self.stop()
            if self.robot_pos is not None:
                self._publish_victim_marker(self.robot_pos[0], self.robot_pos[1])

    def _publish_victim_marker(self, x: float, y: float):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns     = 'victim'
        m.id     = 1
        m.type   = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.5
        m.pose.orientation.w = 1.0
        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.5
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 0.9
        self.victim_marker_pub.publish(m)

    def lidar_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.latest_z = np.array([[x], [y], [yaw]])
        
    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_yaw = math.atan2(siny_cosp, cosy_cosp)
                
    def map_callback(self, msg):
        """Receives static map which presents 1D list of numbers and turns it into 2D matrix."""
        self.map_info = msg.info
        self.static_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.grid = np.copy(self.static_map) 
        
    def odom_callback(self, msg):
        """Extract real-time velocity from wheel encoders."""
        self.odom_v = msg.twist.twist.linear.x
        self.odom_omega = msg.twist.twist.angular.z
        
    def scan_callback(self, msg):
        """"Dynamic Obstacle Layer"""
        if self.robot_pos is None or self.static_map is None: return
        
        # 1. Reset grid to static map
        self.grid = np.copy(self.static_map)
        
        rx, ry, yaw = self.robot_pos
        angle = msg.angle_min
        inflation_radius = 2

        # 2. Add Laser Obstacles with Inflation
        for r in msg.ranges:
            if 0.25 < r < msg.range_max - 0.1:
                ox = rx + r * math.cos(yaw + angle)
                oy = ry + r * math.sin(yaw + angle)
                gx, gy = self.world_to_grid(ox, oy)
                if 0 <= gx < self.map_info.width and 0 <= gy < self.map_info.height:
                    for dx in range(-inflation_radius, inflation_radius + 1):
                        for dy in range(-inflation_radius, inflation_radius + 1):
                            ix, iy = gx + dx, gy + dy
                            if 0 <= ix < self.map_info.width and 0 <= iy < self.map_info.height:
                                self.grid[iy, ix] = 100
            angle += msg.angle_increment

        # 3. FORCE CLEAR ROBOT FOOTPRINT
        gx, gy = self.world_to_grid(rx, ry)
        clear_radius = 4
        for dx in range(-clear_radius, clear_radius + 1):
            for dy in range(-clear_radius, clear_radius + 1):
                cx, cy = gx + dx, gy + dy
                if 0 <= cx < self.map_info.width and 0 <= cy < self.map_info.height:
                    self.grid[cy, cx] = 0

    def get_measured_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            return (t.transform.translation.x, t.transform.translation.y, yaw)
        except TransformException:
            return None

    def control_loop(self):
        meas = self.get_measured_pose()

        # --- A. Initialization ---
        if not self.initialized:
            if meas is not None:
                self.kf.x = np.array([[meas[0]], [meas[1]], [meas[2]]])
                self.initialized = True
                self.get_logger().info("Kalman Filter Initialized.")
            return

        # --- B. Kalman Filter ---
        self.kf.predict(self.odom_v, self.odom_omega)
        if meas:
            z = np.array([[meas[0]],
                          [meas[1]],
                          [self.imu_yaw]])

            self.kf.update(z)

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = meas[0]; msg.pose.position.y = meas[1]
            self.meas_pub.publish(msg)

        kf_x, kf_y, kf_yaw = self.kf.x[0,0], self.kf.x[1,0], self.kf.x[2,0]
        self.robot_pos = (kf_x, kf_y, kf_yaw)

        # Publish Filtered Estimate
        kf_msg = PoseStamped()
        kf_msg.header.stamp = self.get_clock().now().to_msg()
        kf_msg.header.frame_id = "map"
        kf_msg.pose.position.x = kf_x; kf_msg.pose.position.y = kf_y
        self.kf_pub.publish(kf_msg)

        # --- C. Navigation ---
        if self.victim_detected:
            self.stop()
            return

        if self.goal_m and self.current_path:
            if self.is_path_blocked():
                self.get_logger().warn("Blocked! Replanning...")
                self.replan_pub.publish(Empty())
                self.plan_path()
                return
            self.navigate()
        else:
            self.stop()

    def navigate(self):
        rx, ry, yaw = self.robot_pos
        lookahead = min(len(self.current_path) - 1, 5)
        tx, ty = self.grid_to_world(*self.current_path[lookahead])

        dist = math.hypot(tx - rx, ty - ry)
        target_angle = math.atan2(ty - ry, tx - rx)
        error = math.atan2(math.sin(target_angle - yaw), math.cos(target_angle - yaw))

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Check Distance to Final Goal
        final_tx, final_ty = self.grid_to_world(*self.current_path[-1])
        if math.hypot(final_tx - rx, final_ty - ry) < 0.25:
            self.get_logger().info("Goal Reached!")
            self.goal_reached_pub.publish(Bool(data=True))
            self.goal_m = None; self.current_path = []
            self.stop(); return

        # Pop Waypoint
        if dist < 0.3: 
            self.current_path.pop(0)
            return

        # P-Controller
        if abs(error) > 0.6:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 1.0 * error
        else:
            msg.twist.linear.x = 0.25
            msg.twist.angular.z = 1.5 * error
        
        self.cmd_pub.publish(msg)

    def is_path_blocked(self):
        if self.grid is None or len(self.current_path) < 3: return False
        check_len = min(len(self.current_path), 15)
        for i in range(2, check_len):
            gx, gy = self.current_path[i]
            if 0 <= gx < self.map_info.width and 0 <= gy < self.map_info.height:
                if self.grid[gy, gx] > 80: return True
        return False

    def plan_path(self):
        if self.grid is None:
            self.get_logger().warn("Cannot Plan: Map is None")
            return
        if self.goal_m is None or self.robot_pos is None: return

        rx, ry, _ = self.robot_pos
        start = self.world_to_grid(rx, ry)
        gx, gy = self.world_to_grid(self.goal_m[0], self.goal_m[1])
        
        # Check if GOAL is valid
        if 0 <= gx < self.map_info.width and 0 <= gy < self.map_info.height:
            if self.grid[gy, gx] > 80:
                self.get_logger().warn(f"Goal REJECTED: Inside an obstacle (Val: {self.grid[gy, gx]})")
                self.goal_m = None
                return
        
        # Clear start area
        for i in range(-4, 5):
            for j in range(-4, 5):
                sy, sx = start[1]+i, start[0]+j
                if 0 <= sx < self.map_info.width and 0 <= sy < self.map_info.height:
                    self.grid[sy, sx] = 0

        self.get_logger().info("Running A*...")
        path = self.run_astar(start, (gx, gy))
        
        if path:
            self.get_logger().info(f"Path Found: {len(path)} steps.")
            self.current_path = path
            self.publish_path_marker(path)
        else:
            self.get_logger().error("A* FAILED: No path found!")
            self.stop()

    def stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(msg)

    def run_astar(self, start, goal):
        def h(p1, p2): return math.hypot(p1[0]-p2[0], p1[1]-p2[1])
        open_list = [(0, start)]
        came_from = {}; g_score = {start: 0}
        count = 0
        while open_list and count < 8000:
            count += 1
            curr = heapq.heappop(open_list)[1]
            if h(curr, goal) < 2.0:
                res = []
                while curr in came_from: res.append(curr); curr = came_from[curr]
                return res[::-1]
            for dx, dy in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                nx, ny = curr[0]+dx, curr[1]+dy
                if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height:
                    if self.grid[ny, nx] > 80: continue
                    step_cost = math.sqrt(2) if dx != 0 and dy != 0 else 1.0
                    new_g = g_score[curr] + step_cost
                    if new_g < g_score.get((nx, ny), float('inf')):
                        came_from[(nx, ny)] = curr
                        g_score[(nx, ny)] = new_g
                        heapq.heappush(open_list, (new_g + h((nx, ny), goal), (nx, ny)))
        return None

    def goal_callback(self, msg):
        self.goal_m = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"New Goal Received: {self.goal_m}")
        self.plan_path()

    def world_to_grid(self, x, y):
        gx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.map_info.resolution + self.map_info.origin.position.x
        y = gy * self.map_info.resolution + self.map_info.origin.position.y
        return x, y

    def publish_path_marker(self, path):
        m = Marker()
        m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.LINE_STRIP; m.id = 0
        m.scale.x = 0.05; m.color.g = 1.0; m.color.a = 1.0
        for p in path:
            wx, wy = self.grid_to_world(*p)
            m.points.append(Point(x=wx, y=wy, z=0.0))
        self.path_pub.publish(m)

def main():
    rclpy.init(); node = ContinuousNavigator(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()