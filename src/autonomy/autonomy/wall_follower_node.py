#!/usr/bin/env python3
"""
wall_follower_node.py

Minimum-viable F1TENTH-style right-wall follower for a 4-right-turn course.

Modes (in priority order):
  1. Both walls gone   → coast briefly, then timed full-lock right turn
  2. Right wall gone   → check the 45° ray:
                            far / no return → open hallway, turn right
                            close           → doorway recess, go straight
  3. Front wall close  → emergency full-lock right turn
  4. Right wall present → F1TENTH PD on (distance error, heading angle)
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# --- Right-wall F1TENTH estimator -----------------------------------
RAY_A_DEG = -45.0          # forward-right diagonal
RAY_B_DEG = -90.0          # perpendicular right
RAY_HALF_WIN_DEG = 3.0     # half-width per ray cone
LOOK_AHEAD = 0.5           # m, projects perpendicular distance ahead
TARGET_DIST = 1.0          # m, desired distance from right wall
MAX_PLAUSIBLE = 3.5        # m, beyond this → right wall is "gone"

# --- Spike rejection (doorways, windows) ----------------------------
MAX_D_JUMP = 0.8           # m per scan
MAX_ALPHA_JUMP_DEG = 30.0  # degrees per scan
SPIKE_STALE_S = 0.7        # forget anchor reading after this gap

# --- Sticky loss recovery -------------------------------------------
LOST_RECOVERY_SCANS = 3    # consecutive valid scans to exit "lost"

# --- Both-walls-gone corner -----------------------------------------
COAST_S = 0.4              # coast straight before committing
COMMIT_TURN_S = 2.0        # full-lock right-turn duration

# --- Left wall ------------------------------------------------------
LEFT_CONE_DEG = 20         # ± degrees around +90°
WALL_GONE_THRESH = 1.8     # m, left wall absent above this

# --- Right-hallway junction (uses RAY_A) ----------------------------
RIGHT_OPEN_THRESH = 1.5    # m, RAY_A beyond this → open hallway
RIGHT_HALLWAY_MAX = 6.0    # m, upper bound (windows read 8m+)
RIGHT_OPEN_CONFIRM = 3     # scans to confirm before turning

# --- Front safety ---------------------------------------------------
CENTER_CONE_DEG = 15       # narrow front cone
FRONT_STOP_THRESH = 1.2    # m, force right turn if wall this close ahead

# --- PD gains -------------------------------------------------------
KP = 0.8
KD = 0.15
K_ALPHA = 2.0              # heading-angle feedback
MAX_ERROR = 0.4

# --- Output ---------------------------------------------------------
# Rover convention: positive angular.z = RIGHT (opposite of REP-103),
# so SIGN flips the PD output once at the end.
SIGN = -1.0
MAX_STEER = 2.0            # ±2.0 = full lock
BASE_SPEED = 1.20          # m/s nominal
TURN_SPEED = 0.55          # m/s during corners / wall loss

# --- Watchdog -------------------------------------------------------
SCAN_TIMEOUT_S = 1.0


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower_node")

        # PD state
        self._prev_error = 0.0
        self._prev_time = None

        # Spike / loss state
        self._last_valid_D = None
        self._last_valid_alpha = None
        self._last_valid_time = None
        self._valid_run = 0

        # Both-walls-gone state machine
        self._both_lost_since = None

        # Right-hallway debouncer
        self._right_open_count = 0

        # Watchdog
        self._last_scan_time = 0.0
        self._watchdog_stopped = False

        # SLAM/race coordinator can deactivate us
        self._active = True

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(LaserScan, "/scan_nav", self._scan_cb, scan_qos)
        self.create_subscription(String, "/slam_coordinator/mode", self._mode_cb, 10)
        self.create_timer(0.5, self._watchdog_cb)

        self.get_logger().info("Wall follower (MVP) started")

    # ------------------------------------------------------------------
    # Coordinator + watchdog
    # ------------------------------------------------------------------

    def _mode_cb(self, msg: String):
        if msg.data in ("saving", "ready", "racing") and self._active:
            self._active = False
            self._cmd_pub.publish(Twist())
            self.get_logger().info(f"Mode → {msg.data.upper()}: stopping")

    def _watchdog_cb(self):
        if not self._active:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_scan_time > 0 and (now - self._last_scan_time) > SCAN_TIMEOUT_S:
            if not self._watchdog_stopped:
                self.get_logger().warn(
                    f"No scan for {now - self._last_scan_time:.1f}s — stopping"
                )
                self._watchdog_stopped = True
            self._cmd_pub.publish(Twist())

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _valid_rays_in_cone(self, msg: LaserScan, center_deg: float, half_deg: float):
        """Yield valid ranges within half_deg of center_deg."""
        target = self._wrap(math.radians(center_deg))
        half = math.radians(half_deg)
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            if abs(self._wrap(msg.angle_min + i * msg.angle_increment - target)) <= half:
                yield r

    def _cone_mean(self, msg: LaserScan, center_deg: float, half_deg: float) -> float:
        vals = list(self._valid_rays_in_cone(msg, center_deg, half_deg))
        return sum(vals) / len(vals) if vals else float("nan")

    def _cone_min(self, msg: LaserScan, center_deg: float, half_deg: float) -> float:
        vals = list(self._valid_rays_in_cone(msg, center_deg, half_deg))
        return min(vals) if vals else float(msg.range_max)

    def _right_wall_state(self, msg: LaserScan):
        """
        F1TENTH two-ray look-ahead estimator.

        Returns (D_ahead, alpha):
          alpha   — heading angle to wall (0 = parallel, <0 = yawed toward it)
          D_ahead — projected perpendicular distance LOOK_AHEAD m ahead
        Returns (nan, nan) if the perpendicular ray is missing.
        """
        a = self._cone_mean(msg, RAY_A_DEG, RAY_HALF_WIN_DEG)
        b = self._cone_mean(msg, RAY_B_DEG, RAY_HALF_WIN_DEG)
        if not math.isfinite(b):
            return float("nan"), float("nan")
        if not math.isfinite(a):
            return b, 0.0  # perpendicular only: distance b, no angle info

        theta = abs(math.radians(RAY_B_DEG - RAY_A_DEG))
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        D_now = b * math.cos(alpha)
        D_ahead = D_now + LOOK_AHEAD * math.sin(alpha)
        return D_ahead, alpha

    # ------------------------------------------------------------------
    # Main scan callback
    # ------------------------------------------------------------------

    def _scan_cb(self, msg: LaserScan):
        if not self._active:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        self._last_scan_time = now
        if self._watchdog_stopped:
            self._watchdog_stopped = False
            self.get_logger().info("Scan recovered — resuming")

        D_ahead, alpha = self._right_wall_state(msg)
        left_dist = self._cone_mean(msg, 90.0, LEFT_CONE_DEG)
        center_dist = self._cone_min(msg, 0.0, CENTER_CONE_DEG)

        # --- Spike check on (D_ahead, alpha) ---
        is_spike = False
        if (
            math.isfinite(D_ahead)
            and self._last_valid_time is not None
            and (now - self._last_valid_time) < SPIKE_STALE_S
        ):
            dD = abs(D_ahead - self._last_valid_D)
            da = abs(math.degrees(alpha - self._last_valid_alpha))
            is_spike = dD > MAX_D_JUMP or da > MAX_ALPHA_JUMP_DEG

        if (
            self._last_valid_time is not None
            and (now - self._last_valid_time) >= SPIKE_STALE_S
        ):
            self._last_valid_D = self._last_valid_alpha = self._last_valid_time = None

        # --- Right-wall presence with sticky recovery ---
        right_gone_now = (
            not math.isfinite(D_ahead) or D_ahead > MAX_PLAUSIBLE or is_spike
        )
        if right_gone_now:
            self._valid_run = 0
        else:
            self._valid_run += 1
            self._last_valid_D = D_ahead
            self._last_valid_alpha = alpha
            self._last_valid_time = now

        right_gone = right_gone_now or self._valid_run < LOST_RECOVERY_SCANS
        left_gone = (not math.isfinite(left_dist)) or left_dist > WALL_GONE_THRESH

        # ==============================================================
        # 1. Both walls gone — coast then timed full-lock right turn
        # ==============================================================
        if right_gone and left_gone:
            if self._both_lost_since is None:
                self._both_lost_since = now
                self._right_open_count = 0
            elapsed = now - self._both_lost_since

            cmd = Twist()
            cmd.linear.x = TURN_SPEED
            if elapsed < COAST_S:
                cmd.angular.z = 0.0
                phase = "coast"
            elif elapsed < COAST_S + COMMIT_TURN_S:
                cmd.angular.z = MAX_STEER
                phase = "turn"
            else:
                cmd.angular.z = 0.0
                self._both_lost_since = None
                phase = "release"

            self._cmd_pub.publish(cmd)
            self._prev_time = None  # PD restart on exit
            self.get_logger().info(
                f"BOTH-GONE {phase} ({elapsed:.2f}s) "
                f"left={left_dist:.2f} ctr={center_dist:.2f}"
            )
            return

        self._both_lost_since = None

        # ==============================================================
        # 2. Right wall gone — open hallway → turn right, else go straight
        # ==============================================================
        if right_gone:
            ray_a = self._cone_mean(msg, RAY_A_DEG, RAY_HALF_WIN_DEG)
            # No return at 45° = open space = treat like a far reading
            open_hallway = (not math.isfinite(ray_a)) or (
                RIGHT_OPEN_THRESH < ray_a < RIGHT_HALLWAY_MAX
            )
            if open_hallway:
                self._right_open_count += 1
            else:
                self._right_open_count = 0

            confirmed = (
                open_hallway
                and self._right_open_count >= RIGHT_OPEN_CONFIRM
                and center_dist < 3.5
            )
            cmd = Twist()
            if confirmed:
                cmd.linear.x = TURN_SPEED
                cmd.angular.z = MAX_STEER
                self.get_logger().info(
                    f"RIGHT-GONE+HALLWAY ray_a={ray_a:.2f} ctr={center_dist:.2f}"
                )
            else:
                cmd.linear.x = BASE_SPEED
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f"RIGHT-GONE+WAIT ray_a={ray_a:.2f} n={self._right_open_count}"
                )
            self._cmd_pub.publish(cmd)
            self._prev_time = None
            return

        self._right_open_count = 0

        # ==============================================================
        # 3. Front wall close — emergency full-lock right turn
        # ==============================================================
        if center_dist < FRONT_STOP_THRESH:
            cmd = Twist()
            cmd.linear.x = TURN_SPEED
            cmd.angular.z = MAX_STEER
            self._cmd_pub.publish(cmd)
            self._prev_time = None
            self.get_logger().info(f"FRONT-STOP ctr={center_dist:.2f} — full right")
            return

        # ==============================================================
        # 4. Right wall present — F1TENTH PD on (error, alpha)
        # ==============================================================
        error = max(-MAX_ERROR, min(MAX_ERROR, TARGET_DIST - D_ahead))
        if self._prev_time is None:
            d_error = 0.0
        else:
            dt = now - self._prev_time
            d_error = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error
        self._prev_time = now

        pre_sign = KP * error + KD * d_error - K_ALPHA * alpha
        steering = max(-MAX_STEER, min(MAX_STEER, SIGN * pre_sign))

        cmd = Twist()
        cmd.linear.x = BASE_SPEED
        cmd.angular.z = steering
        self._cmd_pub.publish(cmd)

        self.get_logger().info(
            f"FOLLOW D={D_ahead:.2f} α={math.degrees(alpha):+.1f}° "
            f"err={error:+.2f} steer={steering:+.2f}"
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
