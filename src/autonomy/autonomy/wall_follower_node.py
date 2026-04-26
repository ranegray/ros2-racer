#!/usr/bin/env python3
# THIS IS REALLY CLOSE — DO NOT BREAK
"""
wall_follower_node.py

Hybrid F1TENTH look-ahead + dual-wall controller for Lap 1.

Right wall: F1TENTH two-ray estimator (rays at -45° and -90°) gives
            projected distance D_ahead and heading angle alpha.
            A spike detector filters doorway glitches from real corners.
            Sticky recovery requires N consecutive valid scans to exit
            lost mode so a brief valid scan mid-corner doesn't abort the turn.

Left wall:  Simple ±20° cone average. Acts as a balance correction when
            present, and as a guard: right gone + left present = entranceway
            (go straight), right gone + left gone = real corner (turn right).

Four modes:
  Both walls present   → F1TENTH PD + α-feedback + left-wall balance
  Only left wall gone  → Pure F1TENTH right-wall following
  Only right wall gone → Go straight; nudge away from left if too close
  Both walls gone      → Coast straight (COAST_S), then timed right turn
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3


# --- Tunable constants -------------------------------------------
# Right-wall F1TENTH estimator
RAY_A_DEG           = -45.0  # forward-right diagonal ray (degrees)
RAY_B_DEG           = -90.0  # perpendicular-right ray (degrees)
RAY_HALF_WIN_DEG    =   3.0  # cone half-width per ray (degrees)
LOOK_AHEAD          =   0.5  # m — lookahead for D_ahead projection
TARGET_DIST         =   0.8  # m — desired distance from right wall
MAX_PLAUSIBLE       =   3.5  # m — beyond this = right wall gone

# Spike detector (doorways cause brief spikes; corners cause sustained loss)
MAX_D_JUMP          =   0.8  # m — max D_ahead change per scan
MAX_ALPHA_JUMP_DEG  =  60.0  # degrees — max alpha change per scan
SPIKE_STALE_S       =   0.7  # s — invalidate spike history after this gap

# Sticky recovery: require N consecutive valid scans to exit lost mode
LOST_RECOVERY_SCANS =     3

# Corner handling (both walls gone)
COAST_S             =   0.3  # s — coast straight before committing
COMMIT_TURN_S       =   2.0  # s — duration of full-lock right turn

# Left wall
LEFT_CONE_DEG       =    20  # ± degrees around +90° for left-wall rays
WALL_GONE_THRESH    =   1.8  # m — left wall absent above this
WALL_SAFE_DIST      =   1.0  # m — nudge away if remaining wall closer than this
BALANCE_KP          =   0.3  # left-wall balance correction gain

# Front safety
FRONT_CONE_DEG      =    40  # ± degrees around 0° — wide cone for slowing only
CENTER_CONE_DEG     =    15  # ± degrees around 0° — narrow cone for stop/avoid (ignores side walls)
FRONT_SLOW_THRESH   =   0.8  # m — start slowing (wide cone)
FRONT_STOP_THRESH   =  0.45  # m — hard emergency turn (narrow cone only)


# Crash avoidance — steer away from angled approaching wall
# Uses two diagonal rays (+/-FRONT_AVOID_DEG) to detect wall angle:
#   front_R - front_L > 0  →  wall like \  →  steer right (positive)
#   front_R - front_L < 0  →  wall like /  →  steer left  (negative)
FRONT_AVOID_THRESH  =   2.0  # m — start applying angle correction
FRONT_AVOID_DEG     =  25.0  # degrees for the diagonal front rays
FRONT_AVOID_MIN_ASYM=  0.15  # m — ignore asymmetry smaller than this
FRONT_AVOID_KP      =   1.5  # gain on asymmetry → steer correction
# Gap detection: if a diagonal reads much further than centre, it passed through
# a gap (doorway, window) — asymmetry is garbage, skip AVOID entirely.
FRONT_AVOID_MAX_DIAG_MULT  = 3.0   # diagonal > N × center_dist → relative gap
FRONT_AVOID_ABS_GAP_THRESH = 2.0   # m — diagonal > this absolute → window/glass door

# Right-turn junction detection
# When right wall is gone, RAY_A (-45°) reads far → open right hallway → turn right.
# When RAY_A reads close → doorway recess → go straight as normal.
RIGHT_OPEN_THRESH   = 1.5   # m — RAY_A beyond this = right hallway confirmed
# Alpha-based proactive right turn: when wall starts swinging away (alpha > threshold),
# add extra rightward steering kick to start turning before the wall fully disappears.
ALPHA_TURN_THRESH_DEG = 15.0  # degrees — alpha above this triggers the boost
ALPHA_TURN_KP         =  1.5  # gain on (alpha - threshold) → extra right steer

# PD + feedback gains
KP            = 0.8
KD            = 0.15
K_ALPHA       = 3.5   # wall-angle (alpha) feedback gain
K_YAW         = 0.15  # IMU yaw-rate damping gain
D_ERR_ALPHA   = 0.5   # exponential smoothing on derivative (1=raw, 0=frozen)
MAX_ERROR     = 0.4   # clip distance error before PD

# Output
# SIGN = -1: positive pre-sign value → negative angular.z → LEFT on this rover.
# (positive angular.z = RIGHT on this rover, opposite REP-103)
SIGN          = -1.0
STEERING_TRIM =  0.0  # positive = trim right, negative = trim left (mechanical bias correction)
MAX_STEER     =  2.0  # ±2.0 = full servo lock
BASE_SPEED    =  0.40  # m/s nominal
TURN_SPEED    =  0.34  # m/s minimum (wall-lost / corners)
SPEED_ALPHA_SCALE_DEG = 90.0  # alpha (deg) at which speed hits TURN_SPEED
# -----------------------------------------------------------------


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower_node")

        # PD state
        self._prev_error   = 0.0
        self._prev_d_error = 0.0
        self._prev_time    = None

        # Spike detector state
        self._last_valid_D     = None
        self._last_valid_alpha = None
        self._last_valid_time  = None

        # Right-wall sticky-recovery state
        self._right_lost_since = None  # seconds when right wall first went absent
        self._valid_run        = 0     # consecutive valid scans since last loss

        # Both-walls-gone corner state machine
        self._both_lost_since  = None  # seconds when both walls first gone

        # IMU yaw rate (rad/s)
        self._yaw_rate = 0.0

        # Cone index caches (built on first scan)
        self._left_idx   = []
        self._front_idx  = []
        self._center_idx = []
        self._idx_cached = False

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.create_subscription(
            LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data
        )
        _sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(Vector3, "imu/gyro", self._gyro_cb, _sensor_qos)

        self.get_logger().info("Hybrid F1TENTH+dual-wall follower started")

    # ------------------------------------------------------------------
    # IMU callback
    # ------------------------------------------------------------------

    def _publish(self, cmd: Twist):
        """Apply mechanical steering trim then publish."""
        cmd.angular.z += STEERING_TRIM
        self._cmd_pub.publish(cmd)

    def _gyro_cb(self, msg: Vector3):
        self._yaw_rate = msg.z

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _ray_at_angle(self, msg: LaserScan, target_deg: float, half_win_deg: float) -> float:
        """Mean of valid rays within half_win_deg of target_deg. NaN if none."""
        target = self._wrap(math.radians(target_deg))
        half   = math.radians(half_win_deg)
        readings = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            if abs(self._wrap(msg.angle_min + i * msg.angle_increment - target)) <= half:
                readings.append(r)
        return sum(readings) / len(readings) if readings else float("nan")

    def _right_wall_state(self, msg: LaserScan):
        """
        F1TENTH two-ray look-ahead estimator.
        Returns (D_ahead, alpha):
          alpha   — heading angle relative to right wall (0 = parallel, <0 = yawed toward wall)
          D_ahead — projected perpendicular distance LOOK_AHEAD metres ahead
        Returns (nan, nan) when the perpendicular beam is missing.
        """
        a = self._ray_at_angle(msg, RAY_A_DEG, RAY_HALF_WIN_DEG)  # forward-right diagonal
        b = self._ray_at_angle(msg, RAY_B_DEG,  RAY_HALF_WIN_DEG)  # perpendicular-right

        if not math.isfinite(b):
            return float("nan"), float("nan")
        if not math.isfinite(a):
            return b, 0.0  # perpendicular only: no angle, distance is b

        theta   = abs(math.radians(RAY_B_DEG - RAY_A_DEG))
        alpha   = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        D_now   = b * math.cos(alpha)
        D_ahead = D_now + LOOK_AHEAD * math.sin(alpha)
        return D_ahead, alpha

    def _cone_indices(self, msg: LaserScan, center_deg: float, half_deg: float):
        n = len(msg.ranges)
        lo = math.radians(center_deg - half_deg)
        hi = math.radians(center_deg + half_deg)
        return [i for i in range(n) if lo <= msg.angle_min + i * msg.angle_increment <= hi]

    # ------------------------------------------------------------------
    # Main scan callback
    # ------------------------------------------------------------------

    def _scan_cb(self, msg: LaserScan):

        # Build cone caches once
        if not self._idx_cached:
            self._left_idx   = self._cone_indices(msg,  90.0, LEFT_CONE_DEG)
            self._front_idx  = self._cone_indices(msg,   0.0, FRONT_CONE_DEG)
            self._center_idx = self._cone_indices(msg,   0.0, CENTER_CONE_DEG)
            self._idx_cached = True
            self.get_logger().info(
                f"Cone caches: left={len(self._left_idx)} "
                f"front={len(self._front_idx)} center={len(self._center_idx)}"
            )

        ranges = np.array(msg.ranges, dtype=np.float32)

        def cone_mean(idx):
            v = ranges[idx]
            good = v[(v > msg.range_min) & (v < msg.range_max) & np.isfinite(v)]
            return float(np.mean(good)) if len(good) else float(msg.range_max)

        def cone_min(idx):
            v = ranges[idx]
            good = v[(v > msg.range_min) & (v < msg.range_max) & np.isfinite(v)]
            return float(np.min(good)) if len(good) else float(msg.range_max)

        now_s = self.get_clock().now().nanoseconds * 1e-9

        left_dist   = cone_mean(self._left_idx)
        front_dist  = cone_min(self._front_idx)   # wide — used for slowing only
        center_dist = cone_min(self._center_idx)  # narrow — used for stop/avoid
        D_ahead, alpha = self._right_wall_state(msg)

        # --- Emergency stop: only if straight-ahead (narrow) cone is blocked ---
        if center_dist < FRONT_STOP_THRESH:
            cmd = Twist()
            cmd.linear.x  = TURN_SPEED * 0.5
            cmd.angular.z = -MAX_STEER   # hard LEFT (negative = left on this rover)
            self._publish(cmd)
            self.get_logger().info(f"EMERGENCY ctr={center_dist:.2f}m — hard left")
            return

        # --- Crash avoidance: steer away from angled approaching wall ---
        # Uses narrow center_dist so a gap straight ahead won't trigger it.
        if center_dist < FRONT_AVOID_THRESH:
            front_L = self._ray_at_angle(msg, +FRONT_AVOID_DEG, RAY_HALF_WIN_DEG)
            front_R = self._ray_at_angle(msg, -FRONT_AVOID_DEG, RAY_HALF_WIN_DEG)
            if math.isfinite(front_L) and math.isfinite(front_R):
                max_diag = center_dist * FRONT_AVOID_MAX_DIAG_MULT
                rel_gap = front_L > max_diag or front_R > max_diag
                abs_gap = front_R > FRONT_AVOID_ABS_GAP_THRESH or front_L > FRONT_AVOID_ABS_GAP_THRESH
                if rel_gap or abs_gap:
                    reason = "abs" if abs_gap else "rel"
                    self.get_logger().info(
                        f"AVOID SKIP ({reason}) ctr={center_dist:.2f}m  "
                        f"L={front_L:.2f} R={front_R:.2f}"
                    )
                    # fall through to normal wall following
                else:
                    asymmetry = front_R - front_L
                    speed = max(TURN_SPEED, BASE_SPEED * (center_dist / FRONT_AVOID_THRESH))
                    if abs(asymmetry) > FRONT_AVOID_MIN_ASYM:
                        # Angled wall (\ or /) — steer away from it
                        proximity = 1.0 - center_dist / FRONT_AVOID_THRESH
                        avoid_steer = FRONT_AVOID_KP * asymmetry * (1.0 + proximity)
                        avoid_steer = max(-MAX_STEER, min(MAX_STEER, avoid_steer))
                        cmd = Twist()
                        cmd.linear.x  = speed
                        cmd.angular.z = avoid_steer
                        self._publish(cmd)
                        self.get_logger().info(
                            f"AVOID ctr={center_dist:.2f}m  "
                            f"L={front_L:.2f} R={front_R:.2f}  "
                            f"asym={asymmetry:+.2f}  steer={avoid_steer:+.2f}"
                        )
                    else:
                        # Symmetric solid wall (--------) — always turn right
                        cmd = Twist()
                        cmd.linear.x  = speed
                        cmd.angular.z = MAX_STEER  # full-lock RIGHT
                        self._publish(cmd)
                        self.get_logger().info(
                            f"SOLID WALL ctr={center_dist:.2f}m  "
                            f"L={front_L:.2f} R={front_R:.2f}  turning right"
                        )
                    return

        # --- Spike detector ---
        is_spike = False
        if (
            math.isfinite(D_ahead)
            and self._last_valid_time is not None
            and (now_s - self._last_valid_time) < SPIKE_STALE_S
        ):
            dD = abs(D_ahead - self._last_valid_D)
            da = abs(math.degrees(alpha - self._last_valid_alpha))
            if dD > MAX_D_JUMP or da > MAX_ALPHA_JUMP_DEG:
                is_spike = True
                self.get_logger().info(f"spike: ΔD={dD:.2f}m Δα={da:.1f}°")

        # Expire stale spike history
        if (
            self._last_valid_time is not None
            and (now_s - self._last_valid_time) >= SPIKE_STALE_S
        ):
            self._last_valid_D = self._last_valid_alpha = self._last_valid_time = None

        # --- Right-wall loss + sticky recovery ---
        right_gone_now = (
            not math.isfinite(D_ahead) or D_ahead > MAX_PLAUSIBLE or is_spike
        )

        if right_gone_now:
            self._valid_run = 0
            if self._right_lost_since is None:
                self._right_lost_since = now_s
        else:
            self._valid_run += 1
            self._last_valid_D     = D_ahead
            self._last_valid_alpha = alpha
            self._last_valid_time  = now_s

        # Still "lost" until N consecutive valid scans confirm recovery
        right_gone = right_gone_now or (
            self._right_lost_since is not None
            and self._valid_run < LOST_RECOVERY_SCANS
        )
        if not right_gone:
            self._right_lost_since = None

        left_gone = left_dist > WALL_GONE_THRESH

        # ==============================================================
        # CASE 1: Both walls gone → coast then timed right turn
        # ==============================================================
        if right_gone and left_gone:
            if self._both_lost_since is None:
                self._both_lost_since = now_s
            lost_for = now_s - self._both_lost_since

            self._prev_error = self._prev_d_error = 0.0
            self._prev_time  = now_s

            cmd = Twist()
            cmd.linear.x = TURN_SPEED
            if lost_for < COAST_S:
                cmd.angular.z = 0.0
                mode = "coast"
            elif lost_for < COAST_S + COMMIT_TURN_S:
                cmd.angular.z = MAX_STEER   # full-lock RIGHT (positive = right)
                mode = "turn"
            else:
                cmd.angular.z = 0.0         # release; restarts next scan if still lost
                self._both_lost_since = None
                mode = "release"

            self._publish(cmd)
            self.get_logger().info(
                f"BOTH GONE ({lost_for:.2f}s) {mode}  left={left_dist:.2f}"
            )
            return

        self._both_lost_since = None  # at least one wall present

        # ==============================================================
        # CASE 2: Only right wall gone
        # Use RAY_A (-45°) to distinguish:
        #   far read → open right hallway → turn right
        #   close read → doorway recess → go straight
        # ==============================================================
        if right_gone:
            ray_a = self._ray_at_angle(msg, RAY_A_DEG, RAY_HALF_WIN_DEG)
            if not math.isfinite(ray_a) or ray_a > RIGHT_OPEN_THRESH:
                # Right hallway confirmed — turn right
                cmd = Twist()
                cmd.linear.x  = TURN_SPEED
                cmd.angular.z = MAX_STEER
                self._publish(cmd)
                self.get_logger().info(
                    f"RIGHT GONE+HALLWAY  ray_a={ray_a:.2f}m  turning right"
                )
            else:
                # Doorway recess — go straight, nudge away from left if close
                steer = 0.0
                if left_dist < WALL_SAFE_DIST:
                    steer = KP * (WALL_SAFE_DIST - left_dist)
                    steer = min(steer, MAX_STEER)
                cmd = Twist()
                cmd.linear.x  = BASE_SPEED
                cmd.angular.z = steer
                self._publish(cmd)
                self.get_logger().info(
                    f"RIGHT GONE+DOORWAY  ray_a={ray_a:.2f}m  left={left_dist:.2f}  steer={steer:.2f}"
                )
            return

        # ==============================================================
        # CASE 3/4: Right wall present → F1TENTH PD (+ balance if left present)
        # ==============================================================
        error = TARGET_DIST - D_ahead
        error = max(-MAX_ERROR, min(MAX_ERROR, error))

        if self._prev_time is None:
            d_error = 0.0
        else:
            dt      = now_s - self._prev_time
            raw_d   = (error - self._prev_error) / dt if dt > 0 else 0.0
            d_error = D_ERR_ALPHA * raw_d + (1.0 - D_ERR_ALPHA) * self._prev_d_error
        self._prev_error   = error
        self._prev_d_error = d_error
        self._prev_time    = now_s

        # Pre-sign: PD + alpha-feedback + IMU yaw damping
        pre_sign = KP * error + KD * d_error - K_ALPHA * alpha - K_YAW * self._yaw_rate

        # Left-wall balance: when right is farther than left (drifted left), push right
        if not left_gone:
            pre_sign -= BALANCE_KP * (D_ahead - left_dist)

        # Alpha turn boost: when wall is actively swinging right (approaching junction),
        # add extra rightward kick proportional to how far alpha exceeds the threshold.
        # Applied after balance so it's stronger — subtracting pre_sign = more RIGHT.
        alpha_deg = math.degrees(alpha)
        if alpha_deg > ALPHA_TURN_THRESH_DEG:
            pre_sign -= ALPHA_TURN_KP * (alpha_deg - ALPHA_TURN_THRESH_DEG)

        steering = SIGN * pre_sign
        steering = max(-MAX_STEER, min(MAX_STEER, steering))

        # Speed: ease off as wall angle grows; only slow for things directly ahead (center cone)
        alpha_scale = math.radians(SPEED_ALPHA_SCALE_DEG)
        speed = max(TURN_SPEED, BASE_SPEED * max(0.0, 1.0 - abs(alpha) / alpha_scale))
        if center_dist < FRONT_SLOW_THRESH:
            speed = max(TURN_SPEED, speed * center_dist / FRONT_SLOW_THRESH)

        cmd = Twist()
        cmd.linear.x  = speed
        cmd.angular.z = steering
        self._publish(cmd)

        tag = "F1TENTH+bal" if not left_gone else "F1TENTH"
        self.get_logger().info(
            f"{tag}  D={D_ahead:.2f}m α={math.degrees(alpha):+.1f}°  "
            f"left={left_dist:.2f}  err={error:+.2f}  steer={steering:+.2f}  v={speed:.2f}"
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
