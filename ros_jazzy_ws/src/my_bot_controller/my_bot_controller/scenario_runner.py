import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class ScenarioRunner(Node):
    """
    Publishes a fixed sequence of goals to /goal_pose one at a time.
    Waits for /goal_reached=True before advancing, with a per-goal timeout.
    Publishes /scenario_complete=True when all goals are done.
    """

    def __init__(self):
        super().__init__('scenario_runner')

        # tb3_world goals (square around origin)
        # self.declare_parameter('goal_positions', [1.0, 0.0, 1.0, -1.0, 0.0, -1.0, 0.0, 0.0])
        # t3_house goals
        self.declare_parameter('goal_positions', [6.367633819580078, 1.5474462509155273, 1.158562421798706, 2.318802833557129, 5.187938690185547, -1.5932047367095947,-3.483253002166748, 2.4418678283691406])
        self.declare_parameter('goal_timeout', 90.0)
        self.declare_parameter('startup_delay', 8.0)

        flat = list(self.get_parameter('goal_positions').get_parameter_value().double_array_value)
        self.goals = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
        self.goal_timeout = self.get_parameter('goal_timeout').get_parameter_value().double_value
        startup_delay = self.get_parameter('startup_delay').get_parameter_value().double_value

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.done_pub = self.create_publisher(Bool, '/scenario_complete', 10)
        self.create_subscription(Bool, '/goal_reached', self._goal_reached_cb, 10)

        self.current_idx = 0
        self.goal_active = False
        self._timeout_timer = None
        self._delay_timer = None

        self._startup_timer = self.create_timer(startup_delay, self._startup_cb)
        self.get_logger().info(
            f'ScenarioRunner: {len(self.goals)} goals queued. '
            f'Starting in {startup_delay:.0f}s.'
        )


    def _startup_cb(self):
        self._startup_timer.cancel()
        self._send_current_goal()

    def _timeout_cb(self):
        self._timeout_timer.cancel()
        if self.goal_active:
            self.get_logger().warn(
                f'Goal {self.current_idx + 1}/{len(self.goals)} timed out. Advancing.'
            )
            self._advance()

    def _delay_cb(self):
        self._delay_timer.cancel()
        self._send_current_goal()

    # ------------------------------------------------------------------ #
    # Core logic                                                           #
    # ------------------------------------------------------------------ #

    def _send_current_goal(self):
        if self.current_idx >= len(self.goals):
            self.get_logger().info('All goals completed. Scenario done.')
            self.done_pub.publish(Bool(data=True))
            return

        gx, gy = self.goals[self.current_idx]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(gx)
        msg.pose.position.y = float(gy)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

        self.goal_active = True
        self._timeout_timer = self.create_timer(self.goal_timeout, self._timeout_cb)
        self.get_logger().info(
            f'Goal {self.current_idx + 1}/{len(self.goals)}: ({gx:.2f}, {gy:.2f})'
        )

    def _goal_reached_cb(self, msg: Bool):
        if msg.data and self.goal_active:
            self.get_logger().info(f'Goal {self.current_idx + 1} reached.')
            self._advance()

    def _advance(self):
        self.goal_active = False
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
        self.current_idx += 1
        self._delay_timer = self.create_timer(2.0, self._delay_cb)


def main():
    rclpy.init()
    node = ScenarioRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
