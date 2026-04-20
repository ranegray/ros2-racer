#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped


class LineControlNode(Node):
    def __init__(self):
        super().__init__("line_control_node")
        self.latest_follow = None
        self.latest_turn = None

        self.follow_sub = self.create_subscription(
            PointStamped, "line_follow_point", self._follow_callback, 10
        )
        self.turn_sub = self.create_subscription(
            PointStamped, "line_turn_point", self._turn_callback, 10
        )
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.image_width = 640
        self.image_center_x = self.image_width / 2.0
        self.target_x = 400.0  # pixel column where a centered line appears (camera is mounted off-center)
        self.steering_kp = 1.25  # proportional gain on follow-point offset
        self.turn_steering_kp = 2.2  # proportional gain on turn-point offset (override)
        self.base_speed = 0.35  # forward speed when following (m/s)
        self.turn_speed = 0.31  # forward speed when turning (m/s)
        self.goal_timeout = 1.0  # stop if no goal received for this long (s)

        self.get_logger().info("Line Control Node has started!")

    def _follow_callback(self, msg):
        self.latest_follow = msg

    def _turn_callback(self, msg):
        self.latest_turn = msg

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

        follow = self._extract(self.latest_follow)
        turn = self._extract(self.latest_turn)

        if turn is not None:
            # Right-turn override: detector saw a chunk of tape far to the right.
            # Commit to the turn and steer toward it.
            offset = (turn[0] - self.target_x) / self.image_center_x
            steer = self.turn_steering_kp * offset
            speed = self.turn_speed
        elif follow is not None:
            offset = (follow[0] - self.target_x) / self.image_center_x
            steer = self.steering_kp * offset
            speed = self.base_speed
        else:
            self.publisher_.publish(cmd)
            return

        cmd.angular.z = max(-1.0, min(1.0, steer))
        cmd.linear.x = speed
        self.publisher_.publish(cmd)
        self.get_logger().debug(
            f"speed={cmd.linear.x:.2f} steer={cmd.angular.z:.2f} "
            f"follow={follow} turn={turn}"
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


if __name__ == "__main__":
    main()
