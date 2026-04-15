#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped


class LineControlNode(Node):
    def __init__(self):
        super().__init__('line_control_node')
        self.latest_goal = None

        self.subscriber = self.create_subscription(
            PointStamped, "line_goal_point", self.goal_callback, 10
        )
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        # Control parameters
        self.image_width = 640
        self.image_center_x = self.image_width / 2.0
        self.steering_kp = 1.2        # proportional gain on normalized pixel offset
        self.drive_speed = 0.4        # fixed forward speed (m/s)
        self.goal_timeout = 1.0       # stop if no goal received for this long (s)

        self.get_logger().info("Line Control Node has started!")

    def goal_callback(self, msg):
        self.latest_goal = msg

    def control_loop(self):
        cmd = Twist()

        if self.latest_goal is None:
            self.publisher_.publish(cmd)
            return

        now = self.get_clock().now()
        goal_time = rclpy.time.Time.from_msg(self.latest_goal.header.stamp)
        age = (now - goal_time).nanoseconds / 1e9
        if age > self.goal_timeout:
            self.get_logger().info("Line goal stale, stopping.")
            self.publisher_.publish(cmd)
            return

        px = self.latest_goal.point.x  # pixel x of line centroid
        py = self.latest_goal.point.y  # pixel y of line centroid

        # line_detector publishes an empty PointStamped (0,0,0) when no line is found
        if px == 0.0 and py == 0.0:
            self.get_logger().info("No line detected, stopping.")
            self.publisher_.publish(cmd)
            return

        # Normalized lateral offset from image center: [-1, 1]
        # Positive = line is right of center -> steer right (negative angular.z in REP-103)
        offset = (px - self.image_center_x) / self.image_center_x
        steer = -self.steering_kp * offset
        cmd.angular.z = max(-1.0, min(1.0, steer))
        cmd.linear.x = self.drive_speed

        self.publisher_.publish(cmd)
        self.get_logger().debug(
            f"Driving: speed={cmd.linear.x:.2f}, steer={cmd.angular.z:.2f}, "
            f"px={px:.0f}, offset={offset:.2f}"
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
