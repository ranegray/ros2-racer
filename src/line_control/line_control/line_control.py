#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped


class LineControlNode(Node):
    def __init__(self):
        super().__init__('line_control_node')
        self.latest_near = None
        self.latest_far = None

        self.near_sub = self.create_subscription(
            PointStamped, "line_goal_point", self._near_callback, 10
        )
        self.far_sub = self.create_subscription(
            PointStamped, "line_goal_point_far", self._far_callback, 10
        )
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        # Control parameters
        self.image_width = 640
        self.image_center_x = self.image_width / 2.0
        self.target_x = 400.0            # pixel column where a centered line appears (camera is mounted off-center)
        self.steering_kp = 1.25          # proportional gain on normalized near-band offset
        self.lookahead_steering_kp = 1.5 # gain when only the far band sees the line (corner entry)
        self.cruise_speed = 0.30         # speed during "on" pulses; must clear the ESC dead zone
        self.base_duty = 0.5             # fraction of duty_window the throttle is on, on a straight
        self.min_duty = 0.25             # duty in tightest corner / lookahead-only mode
        self.duty_window_s = 1.5         # length of one on/off cycle (s); longer = more time per turn
        self.goal_timeout = 1.0          # stop if no goal received for this long (s)

        self.get_logger().info("Line Control Node has started!")

    def _near_callback(self, msg):
        self.latest_near = msg

    def _far_callback(self, msg):
        self.latest_far = msg

    def _extract(self, msg):
        """Return (x, y) in pixels if msg is fresh and non-empty, else None."""
        if msg is None:
            return None
        now = self.get_clock().now()
        age = (now - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        if age > self.goal_timeout:
            return None
        if msg.point.x == 0.0 and msg.point.y == 0.0:
            return None
        return (msg.point.x, msg.point.y)

    def control_loop(self):
        cmd = Twist()

        near = self._extract(self.latest_near)
        far = self._extract(self.latest_far)

        if near is None and far is None:
            self.publisher_.publish(cmd)
            return

        if near is not None:
            offset = (near[0] - self.target_x) / self.image_center_x
            steer = self.steering_kp * offset

            if far is not None:
                curvature = abs(far[0] - near[0]) / self.image_center_x
                curvature = min(1.0, curvature)
                duty = self.base_duty - (self.base_duty - self.min_duty) * curvature
            else:
                duty = self.min_duty
        else:
            offset = (far[0] - self.target_x) / self.image_center_x
            steer = self.lookahead_steering_kp * offset
            duty = self.min_duty

        # Throttle pulse: ESC dead zone forces all-or-nothing on linear.x, so we
        # gate cruise_speed by a duty cycle to get a slower effective pace while
        # keeping every "on" pulse strong enough to actually move the rover.
        now_s = self.get_clock().now().nanoseconds / 1e9
        phase = now_s % self.duty_window_s
        throttle_on = phase < self.duty_window_s * duty

        cmd.angular.z = max(-1.0, min(1.0, steer))
        cmd.linear.x = self.cruise_speed if throttle_on else 0.0
        self.publisher_.publish(cmd)
        self.get_logger().debug(
            f"on={throttle_on} duty={duty:.2f} steer={cmd.angular.z:.2f} "
            f"near={near} far={far}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LineControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Line Control Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
