"""
wall_nav_node.py

PD wall-following controller. Subscribes to /scan, averages the LIDAR rays
pointing at the hallway walls, and drives a Twist command on /cmd_vel that
holds the car near a right-wall target while biasing its position away from
the left wall.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallNavNode(Node):
    def __init__(self):
        super().__init__("wall_nav_node")

        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()

        self._prev_error = 0.0
        self._prev_time = None

    def _setup_parameters(self):
        # Tunable live via `ros2 param set /wall_nav_node <name> <value>`.
        self.declare_parameter("kp", 1.1)
        self.declare_parameter("kd", 0.3)
        self.declare_parameter("target_distance", 0.6)
        self.declare_parameter("forward_speed", 0.3)
        self.declare_parameter("balance_kp", 0.45)
        self.declare_parameter("balance_ratio", 2.0)
        # Right-wall window in degrees. 0°=front, window is centered on the
        # right-hand side of the car as reported by this LIDAR mount.
        self.declare_parameter("right_angle_min_deg", 70.0)
        self.declare_parameter("right_angle_max_deg", 110.0)
        # Left-wall window mirrors the right-side sensing window.
        self.declare_parameter("left_angle_min_deg", -110.0)
        self.declare_parameter("left_angle_max_deg", -70.0)

    def _setup_publishers(self):
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def _setup_subscriptions(self):
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

    @staticmethod
    def _wrap(angle):
        """Wrap an angle to (-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _window_distance(
        self,
        msg: LaserScan,
        min_angle_deg: float,
        max_angle_deg: float,
    ) -> float:
        """Mean distance of valid rays in the configured angular window."""
        lo = self._wrap(math.radians(min_angle_deg))
        hi = self._wrap(math.radians(max_angle_deg))

        readings = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = self._wrap(msg.angle_min + i * msg.angle_increment)
            # The window wraps across ±pi when lo > hi after normalization.
            in_window = (lo <= angle <= hi) if lo <= hi else (angle >= lo or angle <= hi)
            if in_window:
                readings.append(r)

        if not readings:
            return float("inf")
        return sum(readings) / len(readings)

    def _right_wall_distance(self, msg: LaserScan) -> float:
        """Mean distance of valid rays in the configured right-side window."""
        return self._window_distance(
            msg,
            self.get_parameter("right_angle_min_deg").value,
            self.get_parameter("right_angle_max_deg").value,
        )

    def _left_wall_distance(self, msg: LaserScan) -> float:
        """Mean distance of valid rays in the configured left-side window."""
        return self._window_distance(
            msg,
            self.get_parameter("left_angle_min_deg").value,
            self.get_parameter("left_angle_max_deg").value,
        )

    def scan_callback(self, msg: LaserScan):
        right_dist = self._right_wall_distance(msg)
        left_dist = self._left_wall_distance(msg)

        if not math.isfinite(right_dist):
            self.get_logger().warn("No valid right-wall readings; stopping.")
            self.cmd_pub.publish(Twist())
            return

        target = self.get_parameter("target_distance").value
        kp = self.get_parameter("kp").value
        kd = self.get_parameter("kd").value
        balance_kp = self.get_parameter("balance_kp").value
        balance_ratio = self.get_parameter("balance_ratio").value
        forward_speed = self.get_parameter("forward_speed").value

        # Positive error => too close to the right wall => steer away.
        # This rover's angular.z is inverted relative to REP-103, so the
        # PD output is negated to make steering point away from the wall.
        right_error = target - right_dist

        balance_error = 0.0
        if math.isfinite(left_dist):
            # For a 66/33 hallway split, target left distance ~= 2x right distance.
            # Negative error means the rover is too close to the left wall.
            balance_error = left_dist - (balance_ratio * right_dist)

        error = right_error + (balance_kp * balance_error)

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._prev_time is None:
            d_error = 0.0
        else:
            dt = now - self._prev_time
            d_error = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error
        self._prev_time = now

        steering = -(kp * error + kd * d_error)

        drive_cmd = Twist()
        drive_cmd.linear.x = float(forward_speed)
        drive_cmd.angular.z = float(steering)
        self.cmd_pub.publish(drive_cmd)

        self.get_logger().info(
            f"left={left_dist:.2f}m right={right_dist:.2f}m target={target:.2f} "
            f"right_err={right_error:+.2f} balance_err={balance_error:+.2f} "
            f"err={error:+.2f} dErr={d_error:+.2f} steer={steering:+.2f}"
        )


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
