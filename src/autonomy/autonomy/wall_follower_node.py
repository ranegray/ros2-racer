#!/usr/bin/env python3
"""
wall_follower_node.py

Right-wall follower with:
  - Cosine-weighted right-wall distance over a wide scan band
  - IMU yaw-rate gate: suppress wall-gone trigger if robot is already turning
  - Hysteresis: require 15 consecutive scans before committing to a right turn
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3


# --- Tunable constants -------------------------------------------
TARGET_DIST       = 0.55   # m — minimum desired distance (react if closer than this)
DRIFT_THRESH      = 1.0    # m — only steer back toward wall if farther than this
WALL_GONE_THRESH  = 3.0    # m — right wall gone above this
WALL_GONE_SCANS   = 15     # consecutive scans before committing to right turn (1.5 s @ 10 Hz)
TURNING_YAW_GATE  = 0.25   # rad/s — suppress wall-gone if |gyro_z| > this
FRONT_SLOW_THRESH = 0.8    # m — start slowing
FRONT_STOP_THRESH = 0.45   # m — hard left turn
RIGHT_CONE_DEG    = 60     # ± degrees around -90° for right-wall rays (wider = smoother)
FRONT_CONE_DEG    = 40     # ± degrees around 0° for front rays

KP = 1.2
KD = 0.4

BASE_SPEED  = 0.40
TURN_SPEED  = 0.28
HARD_STEER  = 1.2
MAX_STEER   = 2.0
# -----------------------------------------------------------------

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


def _cone_indices_with_weights(angle_min, angle_increment, n_rays, center_deg, half_deg):
    """Return (indices, cosine_weights) for rays within center_deg ± half_deg.
    Rays perpendicular to the wall (closest to center_deg) are weighted highest.
    """
    center_rad = math.radians(center_deg)
    half_rad = math.radians(half_deg)
    lo = center_rad - half_rad
    hi = center_rad + half_rad
    indices = []
    weights = []
    for i in range(n_rays):
        a = angle_min + i * angle_increment
        if lo <= a <= hi:
            indices.append(i)
            # cos weight: 1.0 at center, 0.0 at edges
            weights.append(math.cos((a - center_rad) / half_rad * math.pi / 2))
    return indices, np.array(weights, dtype=np.float32)


def _cone_indices(angle_min, angle_increment, n_rays, center_deg, half_deg):
    idx, _ = _cone_indices_with_weights(angle_min, angle_increment, n_rays, center_deg, half_deg)
    return idx


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower_node")

        self._prev_error = 0.0
        self._last_scan_t = None
        self._wall_gone_count = 0
        self._gyro_z = 0.0         # latest yaw rate from IMU (rad/s)
        self._in_right_turn = False
        self._turn_accumulated = 0.0  # radians turned so far during hard right
        self._turn_start_t = None

        self._right_idx = []
        self._right_weights = np.array([])
        self._front_idx = []
        self._idx_computed = False

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        # IMU yaw rate — match rover_node's BEST_EFFORT QoS
        self.create_subscription(Vector3, "imu/gyro", self._gyro_cb, _SENSOR_QOS)

        self.get_logger().info(
            f"Wall follower started — target {TARGET_DIST} m from right wall"
        )

    def _gyro_cb(self, msg: Vector3):
        self._gyro_z = msg.z  # rad/s, positive = left yaw

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)

        if not self._idx_computed:
            self._right_idx, self._right_weights = _cone_indices_with_weights(
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

        def weighted_wall_dist(idx, weights):
            """Cosine-weighted mean of valid right-wall rays."""
            vals = ranges[idx]
            valid = (vals > msg.range_min) & (vals < msg.range_max) & np.isfinite(vals)
            if not valid.any():
                return float(msg.range_max)
            w = weights[valid]
            return float(np.average(vals[valid], weights=w))

        def valid_min(idx):
            vals = ranges[idx]
            good = vals[(vals > msg.range_min) & (vals < msg.range_max) & np.isfinite(vals)]
            return float(np.min(good)) if len(good) > 0 else msg.range_max

        right_dist = weighted_wall_dist(self._right_idx, self._right_weights)
        front_dist = valid_min(self._front_idx)

        now = self.get_clock().now()
        dt = 0.1
        if self._last_scan_t is not None:
            dt = max(0.01, (now - self._last_scan_t).nanoseconds / 1e9)
        self._last_scan_t = now

        cmd = Twist()

        # Front obstacle — hard left
        if front_dist < FRONT_STOP_THRESH:
            cmd.linear.x = TURN_SPEED * 0.3
            cmd.angular.z = -HARD_STEER
            self._cmd_pub.publish(cmd)
            self.get_logger().debug(f"FRONT OBSTACLE {front_dist:.2f} m — turning left")
            return

        # If currently executing a right turn, integrate gyro to track rotation.
        # Exit once 90° accumulated or wall reappears.
        if self._in_right_turn:
            self._turn_accumulated += abs(self._gyro_z) * dt
            if self._turn_accumulated >= math.pi / 2 or right_dist < WALL_GONE_THRESH:
                self._in_right_turn = False
                self._turn_accumulated = 0.0
                self._wall_gone_count = 0
                self.get_logger().info(
                    f"Right turn complete — {math.degrees(self._turn_accumulated):.0f}° "
                    f"turned, right_dist={right_dist:.2f}"
                )
                # Fall through to PD
            else:
                cmd.linear.x = TURN_SPEED
                cmd.angular.z = HARD_STEER
                self._cmd_pub.publish(cmd)
                return

        # Wall-gone hysteresis + IMU gate: only count when going straight
        if right_dist > WALL_GONE_THRESH and abs(self._gyro_z) < TURNING_YAW_GATE:
            self._wall_gone_count += 1
        else:
            self._wall_gone_count = 0

        if self._wall_gone_count >= WALL_GONE_SCANS:
            self._in_right_turn = True
            self._turn_accumulated = 0.0
            self._wall_gone_count = 0
            self.get_logger().info("Starting right turn")
            cmd.linear.x = TURN_SPEED
            cmd.angular.z = HARD_STEER
            self._cmd_pub.publish(cmd)
            return

        # PD wall follow — asymmetric dead zone:
        #   right_dist < TARGET_DIST  → too close, steer away (negative error)
        #   TARGET_DIST..DRIFT_THRESH → acceptable band, no correction
        #   right_dist > DRIFT_THRESH → drifting too far, steer back (positive error)
        if right_dist < TARGET_DIST:
            error = right_dist - TARGET_DIST   # negative → steer left (away)
        elif right_dist > DRIFT_THRESH:
            error = right_dist - DRIFT_THRESH  # positive → steer right (toward wall)
        else:
            error = 0.0

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

        self.get_logger().debug(
            f"right={right_dist:.2f} front={front_dist:.2f} "
            f"err={error:.2f} steer={steer:.2f} gyro_z={self._gyro_z:.2f}"
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
