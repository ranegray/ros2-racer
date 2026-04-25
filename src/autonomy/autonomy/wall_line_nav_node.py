"""
wall_line_nav_node.py
Inputs:
  /scan                       sensor_msgs/LaserScan
  /perception/front_distance  std_msgs/Float32  (lidar-derived front clearance)
  /camera/color/image_raw     sensor_msgs/Image (raw RealSense color frame)
  /line_follow_point          geometry_msgs/PointStamped (from line_detector)
  /line_lookahead_point       geometry_msgs/PointStamped (from line_detector)

Output:
  /cmd_vel                    geometry_msgs/Twist
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PointStamped, Twist


class WallLineNavNode(Node):
    def __init__(self):
        super().__init__("wall_line_nav_node")

        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()

        self.latest_scan: LaserScan | None = None
        self.latest_front_distance: float | None = None
        self.latest_image: Image | None = None
        self.latest_follow_point: PointStamped | None = None
        self.latest_lookahead_point: PointStamped | None = None

        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def _setup_parameters(self):
        self.declare_parameter("max_linear_speed", 0.4)
        self.declare_parameter("max_angular_speed", 1.0)

    def _setup_subscriptions(self):
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.front_distance_sub = self.create_subscription(
            Float32, "/perception/front_distance", self.front_distance_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 10
        )
        self.follow_point_sub = self.create_subscription(
            PointStamped, "/line_follow_point", self.follow_point_callback, 10
        )
        self.lookahead_point_sub = self.create_subscription(
            PointStamped, "/line_lookahead_point", self.lookahead_point_callback, 10
        )

    def _setup_publishers(self):
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def front_distance_callback(self, msg: Float32):
        self.latest_front_distance = msg.data

    def image_callback(self, msg: Image):
        self.latest_image = msg

    def follow_point_callback(self, msg: PointStamped):
        self.latest_follow_point = msg

    def lookahead_point_callback(self, msg: PointStamped):
        self.latest_lookahead_point = msg

    def control_loop(self):
        # TODO: fuse lidar + line inputs to produce a Twist command.
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WallLineNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
