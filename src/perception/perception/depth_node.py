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
        # Average over a small angular window around 0° (forward) for robustness.
        # The index of 0° depends on the scan's angle_min/angle_increment, which
        # varies by driver config — compute it instead of hard-coding.
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return
        center = int(round((0.0 - msg.angle_min) / msg.angle_increment)) % n
        half_window = 5
        # Collect window with wrap-around so we handle scans where 0° lies near
        # the seam between angle_max and angle_min.
        readings = [
            r for i in range(-half_window, half_window + 1)
            for r in (msg.ranges[(center + i) % n],)
            if math.isfinite(r) and r > 0.0
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
