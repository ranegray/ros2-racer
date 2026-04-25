"""
wall_line_nav_node.py
Inputs:
  /scan                       sensor_msgs/LaserScan
  /perception/front_distance  std_msgs/Float32  (lidar-derived front clearance)
  /camera/color/image_raw     sensor_msgs/Image (raw RealSense color frame)
  /line_follow_point          geometry_msgs/PointStamped (from line_detector)
  /line_lookahead_point       geometry_msgs/PointStamped (from line_detector)
  /line_detector/debug_image  sensor_msgs/Image (line detector debug overlay)

Output:
  /cmd_vel                    geometry_msgs/Twist
"""

import math

import rclpy
from rclpy.duration import Duration
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
        self.latest_line_debug_image: Image | None = None

        self.mode = "forward"
        self.turn_until = None
        self.cooldown_until = None
        self.right_open_scan_run = 0
        self.last_scan_stamp = None

        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def _setup_parameters(self):
        self.declare_parameter("forward_speed", 0.30)
        self.declare_parameter("turn_linear_speed", 0.10)
        self.declare_parameter("right_turn_angular_speed", 0.70)
        self.declare_parameter("right_turn_duration", 2.25)
        self.declare_parameter("turn_cooldown", 2.0)
        self.declare_parameter("right_open_distance", 2.0)
        self.declare_parameter("front_clear_distance", 0.8)
        self.declare_parameter("right_open_fraction", 0.70)
        self.declare_parameter("right_open_required_scans", 3)
        self.declare_parameter("right_open_min_samples", 10)
        self.declare_parameter("front_clear_min_samples", 3)

        self.forward_speed = self.get_parameter("forward_speed").value
        self.turn_linear_speed = self.get_parameter("turn_linear_speed").value
        self.right_turn_angular_speed = self.get_parameter(
            "right_turn_angular_speed"
        ).value
        self.right_turn_duration = self.get_parameter("right_turn_duration").value
        self.turn_cooldown = self.get_parameter("turn_cooldown").value
        self.right_open_distance = self.get_parameter("right_open_distance").value
        self.front_clear_distance = self.get_parameter("front_clear_distance").value
        self.right_open_fraction = self.get_parameter("right_open_fraction").value
        self.right_open_required_scans = self.get_parameter(
            "right_open_required_scans"
        ).value
        self.right_open_min_samples = self.get_parameter("right_open_min_samples").value
        self.front_clear_min_samples = self.get_parameter(
            "front_clear_min_samples"
        ).value

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
        self.line_debug_image_sub = self.create_subscription(
            Image, "/line_detector/debug_image", self.line_debug_image_callback, 1
        )

    def _setup_publishers(self):
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_scan_stamp:
            return
        self.last_scan_stamp = stamp

        if self._detect_clear_right_hallway(msg):
            self.right_open_scan_run += 1
        else:
            self.right_open_scan_run = 0

    def front_distance_callback(self, msg: Float32):
        self.latest_front_distance = msg.data

    def image_callback(self, msg: Image):
        self.latest_image = msg

    def follow_point_callback(self, msg: PointStamped):
        self.latest_follow_point = msg

    def lookahead_point_callback(self, msg: PointStamped):
        self.latest_lookahead_point = msg

    def line_debug_image_callback(self, msg: Image):
        self.latest_line_debug_image = msg

    def control_loop(self):
        now = self.get_clock().now()
        cmd = Twist()

        if self.mode == "turn_right":
            if self.turn_until is not None and now < self.turn_until:
                cmd.linear.x = self.turn_linear_speed
                cmd.angular.z = self.right_turn_angular_speed
                self.cmd_pub.publish(cmd)
                return

            self.mode = "forward"
            self.turn_until = None
            self.cooldown_until = now + Duration(seconds=self.turn_cooldown)
            self.right_open_scan_run = 0

        if (
            self.cooldown_until is None or now >= self.cooldown_until
        ) and self.right_open_scan_run >= self.right_open_required_scans:
            self.mode = "turn_right"
            self.turn_until = now + Duration(seconds=self.right_turn_duration)
            self.get_logger().info("Clear right hallway detected; turning right")
            cmd.linear.x = self.turn_linear_speed
            cmd.angular.z = self.right_turn_angular_speed
            self.cmd_pub.publish(cmd)
            return

        cmd.linear.x = self.forward_speed
        self.cmd_pub.publish(cmd)

    def _detect_clear_right_hallway(self, scan: LaserScan) -> bool:
        right_ranges = self._sector_ranges(scan, -100.0, -55.0)
        front_ranges = self._sector_ranges(scan, -15.0, 15.0)
        if (
            len(right_ranges) < self.right_open_min_samples
            or len(front_ranges) < self.front_clear_min_samples
        ):
            return False

        right_clear = self._clear_fraction(
            right_ranges, self.right_open_distance, scan.range_max
        )
        front_min = min(front_ranges)

        return (
            right_clear >= self.right_open_fraction
            and front_min >= self.front_clear_distance
        )

    def _sector_ranges(self, scan: LaserScan, start_deg: float, end_deg: float):
        start = math.radians(start_deg)
        end = math.radians(end_deg)
        ranges = []
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if angle < start or angle > end:
                continue
            if math.isinf(r):
                ranges.append(scan.range_max)
            elif math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                ranges.append(r)
        return ranges

    @staticmethod
    def _clear_fraction(ranges, threshold: float, range_max: float) -> float:
        if not ranges:
            return 0.0
        capped_threshold = min(threshold, range_max)
        clear_count = sum(1 for r in ranges if r >= capped_threshold)
        return clear_count / len(ranges)


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
