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
        self.steering_kp = 3.0   # proportional gain on follow-point offset
        self.steering_kd = 5.0   # derivative gain on follow-point offset
        self.turn_steering_kp = (
            3.5   # proportional gain on turn-point offset (override)
        )
        self.base_speed = 0.55  # forward speed when following (m/s)
        self.turn_speed = 0.55  # forward speed when turning (m/s)
        self.speed_scale = 0.5  # how much steering reduces speed (0=no reduction, 1=full stop at max steer)
        self.min_speed = 0.45   # floor — motors must always get at least this much throttle
        self.steering_trim = 0.1  # positive = right bias; tune to counteract physical left-pull of wheels
        self.goal_timeout = 1.0  # stop if no goal received for this long (s)

        self.prev_offset = 0.0       # previous normalized offset for D term
        self.dt = 0.1                # control loop period (s)
        self.last_known_offset = 0.0 # sign of last seen offset; drives recovery steer when line is lost

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

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

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
            self.prev_offset = offset  # keep prev_offset current to avoid D spike on return to follow
            self.last_known_offset = offset
        elif follow is not None:
            offset = (follow[0] - self.target_x) / self.image_center_x
            d_offset = (offset - self.prev_offset) / self.dt
            self.prev_offset = offset
            self.last_known_offset = offset
            steer = self.steering_kp * offset + self.steering_kd * d_offset
            speed = self.base_speed
        else:
            # Line lost — steer full toward the side we last saw it on
            if self.last_known_offset > 0:
                cmd.angular.z = 1.0    # full right
            elif self.last_known_offset < 0:
                cmd.angular.z = -1.0   # full left
            cmd.linear.x = self.min_speed
            self.publisher_.publish(cmd)
            return

        cmd.angular.z = max(-1.0, min(1.0, steer + self.steering_trim))
        if turn is not None:
            cmd.linear.x = speed  # committed turn — no speed reduction
        else:
            cmd.linear.x = max(self.min_speed, speed * (1.0 - abs(cmd.angular.z) * self.speed_scale))
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
