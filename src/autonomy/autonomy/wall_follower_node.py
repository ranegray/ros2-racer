#!/usr/bin/env python3
"""
wall_follower_node.py

Dual-wall centering controller.

  Both walls present  → PD to stay equidistant from left and right wall
  Only right wall gone → go straight, no reaction
  Only left wall gone  → go straight, no reaction
  Both walls gone      → turn right (genuine open corner)

This naturally ignores entranceways/windows on the right: the left wall
stays present so no corner turn is triggered.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# --- Tunable constants -------------------------------------------
WALL_GONE_THRESH  = 1.8   # m — wall considered absent above this
BOTH_GONE_SCANS   = 5     # consecutive scans with BOTH walls gone before turning right
FRONT_SLOW_THRESH = 0.8   # m — start slowing
FRONT_STOP_THRESH = 0.45  # m — hard left turn
RIGHT_CONE_DEG    = 20    # ± degrees around -90° for right-wall rays
LEFT_CONE_DEG     = 20    # ± degrees around +90° for left-wall rays
FRONT_CONE_DEG    = 40    # ± degrees around 0° for front rays

KP = 1.2
KD = 0.4

BASE_SPEED  = 0.40
TURN_SPEED  = 0.28
HARD_STEER  = 1.6
MAX_STEER   = 2.0
# -----------------------------------------------------------------


def _cone_indices(angle_min, angle_increment, n_rays, center_deg, half_deg):
    center_rad = math.radians(center_deg)
    lo = center_rad - math.radians(half_deg)
    hi = center_rad + math.radians(half_deg)
    return [
        i for i in range(n_rays)
        if lo <= angle_min + i * angle_increment <= hi
    ]


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower_node")

        self._prev_error = 0.0
        self._last_scan_t = None
        self._both_gone_count = 0

        self._right_idx = []
        self._left_idx = []
        self._front_idx = []
        self._idx_computed = False

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)

        self.get_logger().info("Dual-wall follower started")

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)

        if not self._idx_computed:
            self._right_idx = _cone_indices(msg.angle_min, msg.angle_increment, n, -90, RIGHT_CONE_DEG)
            self._left_idx  = _cone_indices(msg.angle_min, msg.angle_increment, n,  90, LEFT_CONE_DEG)
            self._front_idx = _cone_indices(msg.angle_min, msg.angle_increment, n,   0, FRONT_CONE_DEG)
            self._idx_computed = True
            self.get_logger().info(
                f"Ray cones: right={len(self._right_idx)}, "
                f"left={len(self._left_idx)}, front={len(self._front_idx)}"
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
        left_dist  = valid_mean(self._left_idx)
        front_dist = valid_min(self._front_idx)

        now = self.get_clock().now()
        dt = 0.1
        if self._last_scan_t is not None:
            dt = max(0.01, (now - self._last_scan_t).nanoseconds / 1e9)
        self._last_scan_t = now

        cmd = Twist()

        # Front obstacle — turn left
        if front_dist < FRONT_STOP_THRESH:
            cmd.linear.x = TURN_SPEED * 0.5
            cmd.angular.z = HARD_STEER
            self._cmd_pub.publish(cmd)
            self.get_logger().info(f"FRONT {front_dist:.2f}m — hard left")
            return

        right_gone = right_dist > WALL_GONE_THRESH
        left_gone  = left_dist  > WALL_GONE_THRESH

        # Both walls gone — genuine open corner, turn right
        if right_gone and left_gone:
            self._both_gone_count += 1
            if self._both_gone_count >= BOTH_GONE_SCANS:
                cmd.linear.x = TURN_SPEED
                cmd.angular.z = -HARD_STEER
                self._cmd_pub.publish(cmd)
                self.get_logger().info(
                    f"BOTH WALLS GONE ({self._both_gone_count} scans) — turning right"
                )
                return
            # Haven't confirmed yet — go straight
            cmd.linear.x = BASE_SPEED
            self._cmd_pub.publish(cmd)
            return
        else:
            self._both_gone_count = 0

        # Only one wall gone — go straight, no correction
        if right_gone or left_gone:
            cmd.linear.x = BASE_SPEED
            self._cmd_pub.publish(cmd)
            self.get_logger().info(
                f"ONE WALL GONE  right={right_dist:.2f}  left={left_dist:.2f} — straight"
            )
            return

        # Both walls present — center between them
        # error > 0: right farther than left → too close to left → steer right (positive)
        error = right_dist - left_dist
        d_error = (error - self._prev_error) / dt
        self._prev_error = error

        steer = KP * error + KD * d_error
        steer = max(-MAX_STEER, min(MAX_STEER, steer))

        speed = BASE_SPEED
        if front_dist < FRONT_SLOW_THRESH:
            speed *= front_dist / FRONT_SLOW_THRESH

        cmd.linear.x = speed
        cmd.angular.z = steer
        self._cmd_pub.publish(cmd)

        self.get_logger().info(
            f"right={right_dist:.2f}  left={left_dist:.2f}  "
            f"err={error:.2f}  steer={steer:.2f}"
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
