#!/usr/bin/env python3
"""
wall_follower_node.py

F1TENTH look-ahead right-wall follower with junction-turn detection.

Right wall: F1TENTH two-ray estimator (rays at -45° and -90°) gives projected
            distance D_ahead and heading angle alpha. Right-wall presence is
            debounced symmetrically (N consecutive scans to flip).

Left wall:  ±20° cone average. Used as a balance term while the right PD is
            active, and as a guard nudge when right is gone in narrow corridors.

Three branches:
  Front blocked    → AVOID: full-lock right (subtle left if / wall up close).
  Right wall gone  → RAY_A open + (left also gone OR front close) → turn right.
                     Otherwise go straight, nudging away from a close left wall.
  Right wall ok    → F1TENTH PD on D_ahead with α-feedback + left balance.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

# --- Tunable constants -------------------------------------------
# Right-wall F1TENTH estimator
RAY_A_DEG = -45.0
RAY_B_DEG = -90.0
RAY_HALF_WIN_DEG = 3.0
LOOK_AHEAD = 0.5  # m — D_ahead projection
TARGET_DIST = 0.8  # m — desired distance from right wall
MAX_PLAUSIBLE = 3.5  # m — beyond this = right wall gone

# Symmetric debounce on right-wall presence (replaces spike + sticky-recovery)
PRESENCE_DEBOUNCE_SCANS = 3

# Left wall
LEFT_CONE_DEG = 20
WALL_GONE_THRESH = 1.8  # m — left wall absent above this
WALL_SAFE_DIST = 1.0  # m — nudge away if left wall closer than this
BALANCE_KP = 0.5

# Right wall crash avoidance
RIGHT_CRASH_THRESH = 1.0
RIGHT_CRASH_KP = 1.8

# Front safety
FRONT_CONE_DEG = 40  # wide cone for slowing
CENTER_CONE_DEG = 15  # narrow cone for AVOID trigger
FRONT_SLOW_THRESH = 2.0

# AVOID — front blocked → turn (right by default; subtle left if / wall close)
FRONT_AVOID_THRESH = 2.8
AVOID_CONFIRM_SCANS = 2
FRONT_AVOID_DEG = 25.0
FRONT_AVOID_MIN_ASYM = 0.15
# Gap detection: diagonal looking through window/doorway → asymmetry is garbage
FRONT_AVOID_MAX_DIAG_MULT = 3.0
FRONT_AVOID_ABS_GAP_THRESH = 3.5
# Close-approach: ignore gap filter, allow subtle left escape from / walls
AVOID_CRASH_CLOSE_DIST = 1.5
AVOID_CRASH_LEFT_MAX = 1.0

# Right-gone hallway detection (RAY_A reads far → junction; close → alcove)
RIGHT_OPEN_THRESH = 1.5
RIGHT_HALLWAY_MAX_RAY = 6.0  # windows/glass read NaN or 8m+
RIGHT_OPEN_CONFIRM_SCANS = 3
RIGHT_OPEN_FRONT_GATE = 3.5  # alcoves have clear fronts; junctions have a wall

# PD gains
KP = 0.8
KD = 0.15
K_ALPHA = 2.0  # wall-angle feedback

# Watchdog
SCAN_TIMEOUT_S = 1.0

# Output
# SIGN = -1: positive pre-sign → negative angular.z → LEFT on this rover.
SIGN = -1.0
STEERING_TRIM = 0.0
MAX_STEER = 2.0
BASE_SPEED = 1.20
TURN_SPEED = 0.55
SPEED_ALPHA_SCALE_DEG = 90.0
# -----------------------------------------------------------------


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower_node")

        # Timestamp: elapsed seconds since node start
        self._start_time = self.get_clock().now().nanoseconds * 1e-9

        self.declare_parameter("speed_override", -1.0)
        _override = self.get_parameter("speed_override").value
        if _override > 0.0:
            global BASE_SPEED, TURN_SPEED
            BASE_SPEED = _override
            TURN_SPEED = min(TURN_SPEED, _override)
            self._log_info(
                f"speed_override active: BASE_SPEED={BASE_SPEED} m/s  TURN_SPEED={TURN_SPEED} m/s"
            )

        # PD state
        self._prev_error = 0.0
        self._prev_time = None

        # Debounce / confirm counters
        self._right_present = True  # debounced presence
        self._opposite_run = 0  # consecutive scans disagreeing with current state
        self._avoid_confirm = 0
        self._right_open_count = 0

        # Cone caches (built on first scan)
        self._left_idx = []
        self._front_idx = []
        self._center_idx = []
        self._idx_cached = False

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # depth=1: always process the freshest scan; drop queued stale scans.
        _scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(LaserScan, "/scan_nav", self._scan_cb, _scan_qos)
        self._last_scan_time: float = 0.0
        self._watchdog = self.create_timer(0.5, self._watchdog_cb)

        self._log_info("F1TENTH wall follower started")

    # --- helpers ------------------------------------------------------

    def _elapsed(self) -> float:
        """Seconds since node start."""
        return self.get_clock().now().nanoseconds * 1e-9 - self._start_time

    def _log_info(self, msg: str):
        self.get_logger().info(f"[t={self._elapsed():.1f}s] {msg}")

    def _log_warn(self, msg: str):
        self.get_logger().warn(f"[t={self._elapsed():.1f}s] {msg}")

    def _publish(self, cmd: Twist):
        cmd.angular.z += STEERING_TRIM
        self._cmd_pub.publish(cmd)

    def _watchdog_cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_scan_time > 0 and (now - self._last_scan_time) > SCAN_TIMEOUT_S:
            self._cmd_pub.publish(Twist())
            self._log_warn(
                f"No scan for {now - self._last_scan_time:.1f}s — stopping rover"
            )

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _ray_at_angle(
        self, msg: LaserScan, target_deg: float, half_win_deg: float
    ) -> float:
        """Mean of valid rays within half_win_deg of target_deg. NaN if none."""
        target = self._wrap(math.radians(target_deg))
        half = math.radians(half_win_deg)
        readings = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            if (
                abs(self._wrap(msg.angle_min + i * msg.angle_increment - target))
                <= half
            ):
                readings.append(r)
        return sum(readings) / len(readings) if readings else float("nan")

    def _right_wall_state(self, msg: LaserScan):
        """F1TENTH two-ray look-ahead estimator. Returns (D_ahead, alpha) or (nan, nan)."""
        a = self._ray_at_angle(msg, RAY_A_DEG, RAY_HALF_WIN_DEG)
        b = self._ray_at_angle(msg, RAY_B_DEG, RAY_HALF_WIN_DEG)
        if not math.isfinite(b):
            return float("nan"), float("nan")
        if not math.isfinite(a):
            return b, 0.0
        theta = abs(math.radians(RAY_B_DEG - RAY_A_DEG))
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        D_now = b * math.cos(alpha)
        D_ahead = D_now + LOOK_AHEAD * math.sin(alpha)
        return D_ahead, alpha

    def _cone_indices(self, msg: LaserScan, center_deg: float, half_deg: float):
        n = len(msg.ranges)
        lo = math.radians(center_deg - half_deg)
        hi = math.radians(center_deg + half_deg)
        return [
            i for i in range(n) if lo <= msg.angle_min + i * msg.angle_increment <= hi
        ]

    @staticmethod
    def _cone_stat(ranges, idx, msg, reducer):
        """Apply reducer (np.mean / np.min / lambda v: np.percentile(v, 30)) to valid rays."""
        v = ranges[idx]
        good = v[(v > msg.range_min) & (v < msg.range_max) & np.isfinite(v)]
        return float(reducer(good)) if len(good) else float(msg.range_max)

    # --- main loop ----------------------------------------------------

    def _scan_cb(self, msg: LaserScan):
        self._last_scan_time = self.get_clock().now().nanoseconds * 1e-9

        if not self._idx_cached:
            self._left_idx = self._cone_indices(msg, 90.0, LEFT_CONE_DEG)
            self._front_idx = self._cone_indices(msg, 0.0, FRONT_CONE_DEG)
            self._center_idx = self._cone_indices(msg, 0.0, CENTER_CONE_DEG)
            self._idx_cached = True
            self._log_info(
                f"Cone caches: left={len(self._left_idx)} "
                f"front={len(self._front_idx)} center={len(self._center_idx)}"
            )

        ranges = np.array(msg.ranges, dtype=np.float32)
        now_s = self.get_clock().now().nanoseconds * 1e-9

        left_dist = self._cone_stat(ranges, self._left_idx, msg, np.mean)
        front_dist = self._cone_stat(ranges, self._front_idx, msg, np.min)
        center_dist = self._cone_stat(
            ranges, self._center_idx, msg, lambda v: np.percentile(v, 30)
        )
        D_ahead, alpha = self._right_wall_state(msg)

        # Front-blocked AVOID override (debounced)
        if center_dist < FRONT_AVOID_THRESH:
            self._avoid_confirm += 1
        else:
            self._avoid_confirm = 0
        if self._avoid_confirm >= AVOID_CONFIRM_SCANS:
            if self._handle_avoid(msg, center_dist):
                return

        # Symmetric debounce on right-wall presence
        right_present_raw = math.isfinite(D_ahead) and D_ahead < MAX_PLAUSIBLE
        if right_present_raw == self._right_present:
            self._opposite_run = 0
        else:
            self._opposite_run += 1
            if self._opposite_run >= PRESENCE_DEBOUNCE_SCANS:
                self._right_present = right_present_raw
                self._opposite_run = 0
        right_gone = not self._right_present
        left_gone = left_dist > WALL_GONE_THRESH

        if right_gone:
            self._handle_right_gone(msg, left_dist, left_gone, center_dist)
            return

        self._right_open_count = 0

        # Right wall present — F1TENTH PD
        if D_ahead < RIGHT_CRASH_THRESH:
            steer = max(-MAX_STEER, -RIGHT_CRASH_KP * (RIGHT_CRASH_THRESH - D_ahead))
            cmd = Twist()
            cmd.linear.x = TURN_SPEED
            cmd.angular.z = steer
            self._publish(cmd)
            self._log_info(f"RIGHT CRASH  D={D_ahead:.2f}m  steer={steer:+.2f}")
            return

        error = TARGET_DIST - D_ahead
        if self._prev_time is None:
            d_error = 0.0
        else:
            dt = now_s - self._prev_time
            d_error = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error
        self._prev_time = now_s

        pre_sign = KP * error + KD * d_error - K_ALPHA * alpha
        if not left_gone:
            pre_sign -= BALANCE_KP * (D_ahead - left_dist)

        steering = max(-MAX_STEER, min(MAX_STEER, SIGN * pre_sign))

        alpha_scale = math.radians(SPEED_ALPHA_SCALE_DEG)
        speed = max(TURN_SPEED, BASE_SPEED * max(0.0, 1.0 - abs(alpha) / alpha_scale))
        near_dist = min(center_dist, front_dist)
        if near_dist < FRONT_SLOW_THRESH:
            speed = max(TURN_SPEED, speed * near_dist / FRONT_SLOW_THRESH)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steering
        self._publish(cmd)

        tag = "F1TENTH+bal" if not left_gone else "F1TENTH"
        self._log_info(
            f"{tag}  D={D_ahead:.2f}m α={math.degrees(alpha):+.1f}°  "
            f"left={left_dist:.2f}  err={error:+.2f}  steer={steering:+.2f}  v={speed:.2f}"
        )

    # --- branch handlers ---------------------------------------------

    def _handle_avoid(self, msg: LaserScan, center_dist: float) -> bool:
        """Front-blocked override. Returns True if a command was published."""
        front_L = self._ray_at_angle(msg, +FRONT_AVOID_DEG, RAY_HALF_WIN_DEG)
        front_R = self._ray_at_angle(msg, -FRONT_AVOID_DEG, RAY_HALF_WIN_DEG)
        if not (math.isfinite(front_L) and math.isfinite(front_R)):
            return False
        gap = (
            front_L > FRONT_AVOID_ABS_GAP_THRESH
            or front_R > FRONT_AVOID_ABS_GAP_THRESH
            or front_L > center_dist * FRONT_AVOID_MAX_DIAG_MULT
            or front_R > center_dist * FRONT_AVOID_MAX_DIAG_MULT
        )
        if gap and center_dist >= AVOID_CRASH_CLOSE_DIST:
            self._log_info(
                f"AVOID SKIP (gap) ctr={center_dist:.2f}m  L={front_L:.2f} R={front_R:.2f}"
            )
            return False
        asymmetry = front_R - front_L
        speed = max(TURN_SPEED, BASE_SPEED * (center_dist / FRONT_AVOID_THRESH))
        # / wall up close → subtle left escape; everything else → full right.
        if asymmetry < -FRONT_AVOID_MIN_ASYM and center_dist < AVOID_CRASH_CLOSE_DIST:
            steer = -AVOID_CRASH_LEFT_MAX
        else:
            steer = MAX_STEER
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steer
        self._publish(cmd)
        self._log_info(
            f"AVOID ctr={center_dist:.2f}m  L={front_L:.2f} R={front_R:.2f}  "
            f"asym={asymmetry:+.2f}  steer={steer:+.2f}"
        )
        return True

    def _handle_right_gone(
        self, msg: LaserScan, left_dist: float, left_gone: bool, center_dist: float
    ):
        ray_a = self._ray_at_angle(msg, RAY_A_DEG, RAY_HALF_WIN_DEG)
        # NaN at -45° = no return = open (treated like a far read)
        open_hallway = (not math.isfinite(ray_a)) or (
            RIGHT_OPEN_THRESH < ray_a < RIGHT_HALLWAY_MAX_RAY
        )
        if open_hallway:
            self._right_open_count += 1
        else:
            self._right_open_count = 0

        # Turn right when hallway is confirmed AND either:
        #   - left wall is also gone (no alcove possible — must be a junction), OR
        #   - the front is close enough to be a junction (alcoves have clear fronts)
        confirmed = self._right_open_count >= RIGHT_OPEN_CONFIRM_SCANS
        turn = confirmed and (left_gone or center_dist < RIGHT_OPEN_FRONT_GATE)

        if turn:
            cmd = Twist()
            cmd.linear.x = TURN_SPEED
            cmd.angular.z = MAX_STEER
            self._publish(cmd)
            self._log_info(
                f"RIGHT GONE+TURN  ray_a={ray_a:.2f}m  ctr={center_dist:.2f}m  "
                f"left_gone={left_gone}  n={self._right_open_count}"
            )
            return

        steer = 0.0
        if left_dist < WALL_SAFE_DIST:
            steer = min(KP * (WALL_SAFE_DIST - left_dist), MAX_STEER)
        cmd = Twist()
        cmd.linear.x = BASE_SPEED
        cmd.angular.z = steer
        self._publish(cmd)
        self._log_info(
            f"RIGHT GONE+WAIT  ray_a={ray_a:.2f}m  ctr={center_dist:.2f}m  "
            f"left={left_dist:.2f}  n={self._right_open_count}  steer={steer:.2f}"
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
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
