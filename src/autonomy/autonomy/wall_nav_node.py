"""
Minimal wall navigation node skeleton.

This version intentionally keeps only the ROS 2 structure:
- subscribes to `/scan`
- publishes to `cmd_vel`
- declares a few starter parameters

The behavior is intentionally narrow:
- sample the right-side lidar distance
- compare it to a target distance
- apply a simple proportional steering correction

It does not try to detect corners or make explicit turning decisions.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class WallNavNode(Node):
    def __init__(self):
        super().__init__("wall_nav_node")
        self._setup_parameters()
        self._setup_publishers()
        self._setup_subscriptions()

        self.latest_scan = None

    def _setup_parameters(self):
        # Starter parameters to tune as you add behavior.
        self.declare_parameter("target_distance", 0.8)
        self.declare_parameter("forward_speed", 0.3)
        self.declare_parameter("steering_gain", 0.0)
        self.declare_parameter("wall_lost_distance", 5.0)

    def _setup_publishers(self):
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def _setup_subscriptions(self):
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _get_range_at_angle(self, scan: LaserScan, angle_deg: float) -> float:
        """Return the range nearest a requested angle in degrees."""
        if not scan.ranges:
            return float("inf")

        target = math.radians(angle_deg)
        best_range = float("inf")
        best_error = float("inf")

        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            error = abs(self._wrap_angle(angle - target))
            if error < best_error:
                best_error = error
                best_range = distance

        return best_range

    def compute_command(self, scan: LaserScan) -> Twist:
        """
        Minimal wall-distance controller.
        """
        cmd = Twist()

        target_distance = self.get_parameter("target_distance").value
        forward_speed = self.get_parameter("forward_speed").value
        steering_gain = self.get_parameter("steering_gain").value
        wall_lost_distance = self.get_parameter("wall_lost_distance").value
        right_distance = self._get_range_at_angle(scan, -90.0)

        if not math.isfinite(right_distance) or right_distance > wall_lost_distance:
            self.get_logger().info("TURN DETECTED")
            return cmd

        error = target_distance - right_distance
        cmd.angular.z = float(steering_gain * error)

        cmd.linear.x = float(forward_speed)
        return cmd

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        cmd = self.compute_command(msg)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WallNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
