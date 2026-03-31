"""
RPLIDAR depth perception for autonomous detection of wall

sub to LIDAR sensor /scan topic -> publish distance from wall
"""

import math
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class DepthNode(Node):
    def __init__(self):
        super().__init__("depth_node")

        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()

    def _setup_parameters(self):
        pass

    def _setup_publishers(self):
        self.obstacle_dist_pub = self.create_publisher(
            Float32, "/perception/front_distance", 10
        )

    def _setup_subscriptions(self):
        """lidar depth sub"""
        self.depth_sub = self.create_subscription(
            LaserScan, "/scan", self.depth_callback, 10
        )

    def depth_callback(self, msg):
        # Average over a small window around 0° (index 90) for robustness
        center = 360
        half_window = 5
        readings = [
            r for r in msg.ranges[center - half_window : center + half_window + 1]
            if math.isfinite(r)
        ]
        dist_to_front_obstacle = sum(readings) / len(readings) if readings else float("inf")

        dist_msg = Float32()
        dist_msg.data = dist_to_front_obstacle
        self.get_logger().info(
            f"[depth_node] Distance to front obstacle: {dist_to_front_obstacle:.2f} m"
        )
        self.obstacle_dist_pub.publish(dist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
