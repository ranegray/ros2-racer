#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped


class LineControlNode(Node):
    def __init__(self):
        super().__init__("line_control_node")
        self.latest_follow = None

        self.follow_sub = self.create_subscription(
            PointStamped, "line_follow_point", self._follow_callback, 10
        )
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.image_width = 640
        self.image_center_x = self.image_width / 2.0
        self.target_x = 400.0  # pixel column where a centered line appears (camera is mounted off-center)
        self.steering_kp = 1.2   # proportional gain on follow-point offset
        self.steering_kd = 4.0   # derivative gain on follow-point offset
        self.base_speed = 0.4   # forward speed when following (m/s)
        self.speed_scale = 0.5       # how much steering reduces speed during following
        self.min_speed = 0.32        # speed floor during following
        self.steering_trim = 0.1  # positive = right bias; tune to counteract physical left-pull of wheels
        self.goal_timeout = 1.0  # stop if no goal received for this long (s)

        self.prev_offset = 0.0       # previous normalized offset for D term
        self.dt = 0.1                # control loop period (s)
        self.last_known_offset = 0.0 # sign of last seen offset; drives recovery steer when line is lost

        self.get_logger().info("Line Control Node has started!")

    def _follow_callback(self, msg):
        self.latest_follow = msg

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

        if follow is not None:
            offset = (follow[0] - self.target_x) / self.image_center_x
            d_offset = (offset - self.prev_offset) / self.dt
            self.prev_offset = offset
            self.last_known_offset = offset
            steer = self.steering_kp * offset + self.steering_kd * d_offset
            speed = self.base_speed
        else:
            # Line lost — pivot toward the side we last saw it on until found.
            if self.last_known_offset > 0:
                cmd.angular.z = 1.0
            elif self.last_known_offset < 0:
                cmd.angular.z = -1.0
            else:
                cmd.angular.z = 0.0
            cmd.linear.x = self.min_speed
            self.publisher_.publish(cmd)
            return

        cmd.angular.z = max(-1.0, min(1.0, steer + self.steering_trim))
        cmd.linear.x = max(self.min_speed, speed * (1.0 - abs(cmd.angular.z) * self.speed_scale))
        self.publisher_.publish(cmd)
        self.get_logger().debug(
            f"speed={cmd.linear.x:.2f} steer={cmd.angular.z:.2f} follow={follow}"
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
