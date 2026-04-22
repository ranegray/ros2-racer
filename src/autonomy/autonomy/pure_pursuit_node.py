#!/usr/bin/env python3
"""
pure_pursuit_node.py

Lap 2 controller: pure pursuit path tracking on the path recorded in Lap 1.

Algorithm (Ackermann-native):
  1. Find the closest point on the recorded path.
  2. Walk forward along the path until the lookahead point is L_d metres away.
  3. Compute the heading error α = angle from robot heading to lookahead point.
  4. Steering angle: δ = atan2(2 * L * sin(α), L_d)
  5. cmd_vel: angular.z = v * tan(δ) / L

Parameters (ROS):
  path_file     — path to recorded_path.yaml  (default ~/.ros/recorded_path.yaml)
  lookahead     — L_d in metres               (default 0.5)
  speed         — forward speed m/s           (default 0.30)
  wheelbase     — L in metres                 (default 0.25)
  goal_tolerance— stop when within this of final pose (default 0.3 m)
"""

import math
import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        self.declare_parameter("path_file", os.path.expanduser("~/.ros/recorded_path.yaml"))
        self.declare_parameter("lookahead", 0.5)
        self.declare_parameter("speed", 0.30)
        self.declare_parameter("wheelbase", 0.25)
        self.declare_parameter("goal_tolerance", 0.3)

        self._path_file    = self.get_parameter("path_file").value
        self._L_d          = self.get_parameter("lookahead").value
        self._speed        = self.get_parameter("speed").value
        self._L            = self.get_parameter("wheelbase").value
        self._goal_tol     = self.get_parameter("goal_tolerance").value

        self._path: list[tuple[float, float]] = []
        self._closest_idx = 0
        self._done = False

        self._load_path()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.create_timer(0.05, self._control_loop)  # 20 Hz
        self.get_logger().info(
            f"Pure pursuit started — {len(self._path)} waypoints, "
            f"L_d={self._L_d} m, speed={self._speed} m/s"
        )

    def _load_path(self):
        try:
            with open(self._path_file) as f:
                data = yaml.safe_load(f)
            self._path = [(p["x"], p["y"]) for p in data["poses"]]
            self.get_logger().info(f"Loaded {len(self._path)} path points from {self._path_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to load path: {e}")

    def _control_loop(self):
        if self._done or not self._path:
            return

        try:
            tf = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        robot_yaw = 2.0 * math.atan2(qz, qw)

        # Check if we've reached the end of the path
        gx, gy = self._path[-1]
        if math.hypot(rx - gx, ry - gy) < self._goal_tol:
            self.get_logger().info("Goal reached — stopping")
            self._done = True
            self._cmd_pub.publish(Twist())
            return

        # Advance closest index
        while self._closest_idx < len(self._path) - 1:
            cx, cy = self._path[self._closest_idx]
            nx, ny = self._path[self._closest_idx + 1]
            if math.hypot(rx - nx, ry - ny) < math.hypot(rx - cx, ry - cy):
                self._closest_idx += 1
            else:
                break

        # Find lookahead point: walk forward from closest until arc-length >= L_d
        lookahead_pt = None
        accum = 0.0
        for i in range(self._closest_idx, len(self._path) - 1):
            ax, ay = self._path[i]
            bx, by = self._path[i + 1]
            seg_len = math.hypot(bx - ax, by - ay)
            if accum + seg_len >= self._L_d:
                # Interpolate along this segment
                t = (self._L_d - accum) / seg_len
                lookahead_pt = (ax + t * (bx - ax), ay + t * (by - ay))
                break
            accum += seg_len

        if lookahead_pt is None:
            # Past end of path — aim for final waypoint
            lookahead_pt = self._path[-1]

        lx, ly = lookahead_pt
        dx = lx - rx
        dy = ly - ry
        dist = math.hypot(dx, dy)

        if dist < 0.01:
            self._cmd_pub.publish(Twist())
            return

        # Heading error in robot frame
        target_angle = math.atan2(dy, dx)
        alpha = target_angle - robot_yaw
        # Normalise to [-π, π]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # Pure pursuit steering
        L_d_eff = max(dist, 0.1)
        delta = math.atan2(2.0 * self._L * math.sin(alpha), L_d_eff)
        angular_z = self._speed * math.tan(delta) / self._L
        angular_z = max(-2.0, min(2.0, angular_z))

        cmd = Twist()
        cmd.linear.x = self._speed
        cmd.angular.z = angular_z
        self._cmd_pub.publish(cmd)

        self.get_logger().debug(
            f"alpha={math.degrees(alpha):.1f}° delta={math.degrees(delta):.1f}° "
            f"angular_z={angular_z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
