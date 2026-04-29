#!/usr/bin/env python3
"""
pure_pursuit_node.py

Lap 2 controller: pure pursuit path tracking on the A*-planned path from
path_planner_node.  Falls back to the recorded yaml path if no planned path
has been received by the time racing mode starts.

Algorithm (Ackermann-native):
  1. Find the closest point on the path.
  2. Walk forward along the path until the lookahead point is L_d metres away.
  3. Compute the heading error α = angle from robot heading to lookahead point.
  4. Steering angle: δ = atan2(2 * L * sin(α), L_d)
  5. cmd_vel: angular.z = v * tan(δ) / L   (sign convention: positive = left in ROS)

Note on sign convention: this rover maps cmd_vel angular.z identically to
wall_follower — positive angular.z drives left.  Pure pursuit computes α
positive-left by convention, so the output sign is correct without inversion.

Parameters:
  path_file      — fallback yaml path  (default ~/.ros/recorded_path.yaml)
  lookahead      — L_d in metres       (default 0.5)
  speed          — forward speed m/s   (default 0.30)
  wheelbase      — L in metres         (default 0.165)
  goal_tolerance — stop when within this of final pose (default 0.3 m)
"""

import math
import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
import tf2_ros


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        self.declare_parameter("path_file", os.path.expanduser("~/.ros/recorded_path.yaml"))
        self.declare_parameter("lookahead", 0.5)
        self.declare_parameter("speed", 0.30)
        self.declare_parameter("wheelbase", 0.165)
        self.declare_parameter("goal_tolerance", 0.3)

        self._path_file  = self.get_parameter("path_file").value
        self._L_d        = self.get_parameter("lookahead").value
        self._speed      = self.get_parameter("speed").value
        self._L          = self.get_parameter("wheelbase").value
        self._goal_tol   = self.get_parameter("goal_tolerance").value

        self._path: list[tuple[float, float]] = []
        self._closest_idx = 0
        self._done   = False
        self._active = False  # wait for racing mode

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._cmd_pub     = self.create_publisher(Twist, "cmd_vel", 10)

        self.create_subscription(String, "/slam_coordinator/mode", self._mode_cb, 10)
        self.create_subscription(Path, "/planned_path", self._path_cb, 1)

        self.create_timer(0.05, self._control_loop)  # 20 Hz
        self.get_logger().info(
            f"Pure pursuit ready — L_d={self._L_d} m, speed={self._speed} m/s. "
            f"Waiting for racing mode."
        )

    # ------------------------------------------------------------------

    def _path_cb(self, msg: Path):
        if not msg.poses:
            return
        was_active = self._active and bool(self._path)
        self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._closest_idx = 0
        self._done = False
        if was_active:
            self.get_logger().info(
                f"Planned path received mid-run — switching to {len(self._path)} waypoints"
            )
        else:
            self.get_logger().info(f"Received planned path with {len(self._path)} waypoints")

    def _mode_cb(self, msg: String):
        if msg.data == "racing" and not self._active:
            self._active = True
            if not self._path:
                self._load_fallback_path()
            self.get_logger().info(
                f"Mode → RACING: pure pursuit activated ({len(self._path)} waypoints)"
            )

    def _load_fallback_path(self):
        try:
            with open(self._path_file) as f:
                data = yaml.safe_load(f)
            self._path = [(p["x"], p["y"]) for p in data["poses"]]
            self.get_logger().warn(
                f"No planned path received — falling back to recorded yaml "
                f"({len(self._path)} waypoints)"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load fallback path: {e}")

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        if not self._active or not self._path:
            return

        try:
            tf = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        robot_yaw = 2.0 * math.atan2(qz, qw)

        # Advance closest index (wraps for closed loop — _closest_idx is unbounded)
        n = len(self._path)
        for _ in range(n):
            curr = self._closest_idx % n
            nxt  = (self._closest_idx + 1) % n
            cx, cy = self._path[curr]
            nx, ny = self._path[nxt]
            if math.hypot(rx - nx, ry - ny) < math.hypot(rx - cx, ry - cy):
                self._closest_idx += 1
            else:
                break

        # Find lookahead point at arc-length L_d ahead (wraps for closed loop)
        lookahead_pt = None
        accum = 0.0
        for k in range(n):
            i = (self._closest_idx + k) % n
            j = (self._closest_idx + k + 1) % n
            ax, ay = self._path[i]
            bx, by = self._path[j]
            seg_len = math.hypot(bx - ax, by - ay)
            if accum + seg_len >= self._L_d:
                t = (self._L_d - accum) / seg_len
                lookahead_pt = (ax + t * (bx - ax), ay + t * (by - ay))
                break
            accum += seg_len

        if lookahead_pt is None:
            lookahead_pt = self._path[self._closest_idx % n]

        lx, ly = lookahead_pt
        dx = lx - rx
        dy = ly - ry
        dist = math.hypot(dx, dy)

        if dist < 0.01:
            self._cmd_pub.publish(Twist())
            return

        target_angle = math.atan2(dy, dx)
        alpha = math.atan2(math.sin(target_angle - robot_yaw),
                           math.cos(target_angle - robot_yaw))

        L_d_eff = max(dist, 0.1)
        delta = math.atan2(2.0 * self._L * math.sin(alpha), L_d_eff)
        angular_z = self._speed * math.tan(delta) / self._L
        angular_z = max(-2.0, min(2.0, angular_z))

        cmd = Twist()
        cmd.linear.x  = self._speed
        cmd.angular.z = angular_z
        self._cmd_pub.publish(cmd)

        self.get_logger().debug(
            f"α={math.degrees(alpha):.1f}° δ={math.degrees(delta):.1f}° "
            f"ω={angular_z:.2f}"
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
