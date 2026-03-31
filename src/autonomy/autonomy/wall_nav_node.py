"""
wall_nav_node.py

Controller for autonomous wall following behavior.
"""

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class WallNavNode(Node):
    def __init__(self):
        super().__init__("wall_nav_node")

        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()

    def _setup_parameters(self):
        pass

    def _setup_publishers(self):
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def _setup_subscriptions(self):
        self.obstacle_dist_sub = self.create_subscription(
            Float32, "/perception/front_distance", self.obstacle_callback, 10
        )

    def obstacle_callback(self, msg):
        dist_to_front_obstacle = msg.data
        drive_cmd = Twist()

        if dist_to_front_obstacle > 1.4:
            self.get_logger().info("Path is clear. Moving forward at 0.4 m/s.")
            drive_cmd.linear.x = 0.4
        else:
            self.get_logger().warn(
                f"Obstacle detected at {dist_to_front_obstacle:.2f} m. Stopping."
            )
            drive_cmd.linear.x = 0.0

        self.cmd_pub.publish(drive_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WallNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
