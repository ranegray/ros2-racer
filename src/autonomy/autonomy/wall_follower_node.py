#!/usr/bin/env python3
"""
wall_follower_node.py

Lap 1 controller: follow the right wall at a fixed distance using a PD
controller on the LaserScan.

Scan geometry (RPLIDAR A1, angle_min=-π, angle_increment positive):
  0°   = forward
  90°  = left
  -90° = right  (index ≈ N/2 - N/4)

Control logic:
  right_dist  = mean of rays in [-90° ± RIGHT_CONE_DEG]
  front_dist  = min  of rays in [  0° ± FRONT_CONE_DEG]

  error  = right_dist - TARGET_DIST
  steer  = kp * error + kd * d_error   (positive steer = left = positive angular.z)

Corner logic (overrides PD):
  - right wall gone (right_dist > WALL_GONE_THRESH)  → turn right hard
  - front obstacle  (front_dist  < FRONT_STOP_THRESH) → turn left hard
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# --- Tunable constants -------------------------------------------
TARGET_DIST      = 0.90   # m — desired distance from right wall
WALL_GONE_THRESH = 3.0    # m — right wall considered gone above this (high to avoid doorway false triggers)
FRONT_SLOW_THRESH = 0.8   # m — start slowing when front closer than this
FRONT_STOP_THRESH = 0.45  # m — turn left hard when front closer than this
RIGHT_CONE_DEG   = 20     # ± degrees around -90° for right-wall rays
FRONT_CONE_DEG   = 40     # ± degrees around 0° for front rays

KP = 1.2           # proportional gain
KD = 0.4           # derivative gain

BASE_SPEED  = 0.40   # m/s forward during mapping
TURN_SPEED  = 0.28   # m/s during hard turns
HARD_STEER  = 1.2    # rad/s for hard-turn overrides (softer to avoid wall crashes on false trigger)
MAX_STEER   = 2.0    # rad/s cap
# -----------------------------------------------------------------


def _cone_indices(angle_min, angle_increment, n_rays, center_deg, half_deg):
    """Return indices of rays within center_deg ± half_deg."""
    center_rad = math.radians(center_deg)
    lo = center_rad - math.radians(half_deg)
    hi = center_rad + math.radians(half_deg)
    indices = []
    for i in range(n_rays):
        a = angle_min + i * angle_increment
        if lo <= a <= hi:
            indices.append(i)
    return indices


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower_node")

        self._prev_error = 0.0
        self._last_scan_t = None
        self._wall_gone_count = 0  # consecutive scans with right wall absent

        self._right_idx: list[int] = []
        self._front_idx: list[int] = []
        self._idx_computed = False

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)

        self.get_logger().info(
            f"Wall follower started — target {TARGET_DIST} m from right wall"
        )

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)

        # Compute ray index cones once from the first message
        if not self._idx_computed:
            self._right_idx = _cone_indices(
                msg.angle_min, msg.angle_increment, n, -90, RIGHT_CONE_DEG
            )
            self._front_idx = _cone_indices(
                msg.angle_min, msg.angle_increment, n, 0, FRONT_CONE_DEG
            )
            self._idx_computed = True
            self.get_logger().info(
                f"Ray cones: right={len(self._right_idx)} rays, "
                f"front={len(self._front_idx)} rays"
            )

        ranges = np.array(msg.ranges, dtype=np.float32)

        def valid_mean(idx):
            vals = ranges[idx]
            good = vals[(vals > msg.range_min) & (vals < msg.range_max) & np.isfinite(vals)]
            return float(np.mean(good)) if len(good) > 0 else msg.range_max

        def valid_min(idx):
            vals = ranges[idx]
            good = vals[(vals > msg.range_min) & (vals < msg.range_max) & np.isfinite(vals)]
            return float(np.min(good)) if len(good) > 0 else msg.range_max

        right_dist = valid_mean(self._right_idx)
        front_dist = valid_min(self._front_idx)

        now = self.get_clock().now()
        dt = 0.1
        if self._last_scan_t is not None:
            dt = max(0.01, (now - self._last_scan_t).nanoseconds / 1e9)
        self._last_scan_t = now

        cmd = Twist()

        # Front obstacle — turn left (negative = left on this rover)
        if front_dist < FRONT_STOP_THRESH:
            cmd.linear.x = TURN_SPEED * 0.3
            cmd.angular.z = -HARD_STEER
            self._cmd_pub.publish(cmd)
            self.get_logger().debug(f"FRONT OBSTACLE {front_dist:.2f} m — turning left")
            return

        # Right wall gone — only commit to hard right after 5 consecutive scans
        # (filters out windows and doorways which cause a brief spike)
        if right_dist > WALL_GONE_THRESH:
            self._wall_gone_count += 1
        else:
            self._wall_gone_count = 0

        if self._wall_gone_count >= 5:
            cmd.linear.x = TURN_SPEED
            cmd.angular.z = HARD_STEER
            self._cmd_pub.publish(cmd)
            self.get_logger().debug(f"RIGHT WALL GONE {right_dist:.2f} m ({self._wall_gone_count} scans) — turning right")
            return

        # PD wall follow
        # error > 0 = too far from wall = steer right (positive angular.z on this rover)
        error = right_dist - TARGET_DIST
        d_error = (error - self._prev_error) / dt
        self._prev_error = error

        steer = KP * error + KD * d_error
        steer = max(-MAX_STEER, min(MAX_STEER, steer))

        # Slow down when approaching front obstacle
        speed = BASE_SPEED
        if front_dist < FRONT_SLOW_THRESH:
            speed *= front_dist / FRONT_SLOW_THRESH

        cmd.linear.x = speed
        cmd.angular.z = steer
        self._cmd_pub.publish(cmd)

        self.get_logger().debug(
            f"right={right_dist:.2f} front={front_dist:.2f} "
            f"err={error:.2f} steer={steer:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
