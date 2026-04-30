"""
wall_nav_node.py

PD wall-following controller. Subscribes to /scan, applies a
morphological min-dilation to the ranges (closes narrow gaps, pulls
through-glass spuriously-far reads back toward nearby wall returns),
then uses a two-ray look-ahead estimator (F1TENTH-style) to compute
the car's angle relative to the right-hand wall and a projected
distance a short look-ahead in front, then drives a Twist on /cmd_vel
that holds the car a fixed distance from that wall. Also subscribes
to imu/gyro and uses the measured yaw rate as an additional damping
term on the steering output.

Right-turn handling: the right wall vanishing (D_ahead > max_plausible
or estimator NaN) IS the corner signal. Doorways/windows lose the wall
briefly; real corners lose it permanently. So we coast straight for
`lost_coast_s` (clears short gaps without committing), then execute a
fixed-duration ~90° right turn (`commit_turn_s` at full lock), then
release back to PD. The phase clock resets on any non-spike good scan,
so a flickery wall (windows, mesh, partial occluders) keeps us in coast
instead of crossing into the turn phase on cumulative-lost time. If
the wall is still missing the next scan, a fresh coast->turn cycle
starts automatically. After `max_lost_time_s` of continuous loss the
car idles (zero velocity) until the wall returns -- prevents endless
spinning in open spaces.

Mode integration: subscribes to /slam_coordinator/mode. Deactivates
(stops publishing cmd_vel) when mode transitions to "saving", "ready",
or "racing" so pure_pursuit_node can take over.
"""

import math
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
from std_msgs.msg import String


class WallNavNode(Node):
    def __init__(self):
        super().__init__("wall_nav_node")
        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()

        self._active = True  # inactive once slam_coordinator says "saving"/"ready"/"racing"
        self._prev_error = 0.0
        self._prev_d_error = 0.0
        self._prev_time = None
        self._lost_since = None
        # Sticky-recovery counter: consecutive valid scans since we last
        # saw wall_lost. We only exit lost mode when this reaches
        # `lost_recovery_scans` -- a single good scan in the middle of a
        # spike-rejected burst does NOT reset the lost timer.
        self._valid_run = 0
        # Time of the last non-spike good scan. Used as the phase-clock
        # anchor inside the lost cycle: when the wall briefly returns
        # (window, mesh fence, etc.), the phase clock resets so a flickery
        # wall doesn't accumulate cumulative-lost time and fire a spurious
        # 90-degree turn. Also drives the open-space idle timeout: it does NOT
        # reset between coast->turn->release cycles, so if the wall has
        # been continuously absent for `max_lost_time_s`, we stop trying.
        # Initialized to the first scan's timestamp so the idle timer has
        # a sensible anchor when we never see the wall.
        self._last_good_time = None
        # For spike detection: last VALID (D_ahead, alpha) and its timestamp.
        self._last_valid_D = None
        self._last_valid_alpha = None
        self._last_valid_time = None
        # Latest IMU yaw rate (rad/s) for yaw-rate damping on the steering.
        self._yaw_rate = 0.0
        # Front crash avoidance state.
        self._prev_asymmetry = 0.0
        self._prev_d_asymmetry = 0.0
        self._avoid_confirm = 0

    def _setup_parameters(self):
        # Tunable live via `ros2 param set /wall_nav_node <name> <value>`.
        # Note: on this robot `cmd_vel.angular.z` is a normalised STEERING
        # command (rover_node maps it as angular.z * 500 clipped to +/-1000,
        # so +/-2 = full lock). Gains are tuned for that, not for rad/s.
        self.declare_parameter("kp", 0.3)
        self.declare_parameter("kd", 0.15)
        # alpha-feedback: counteracts the car's yaw toward/away from the wall
        # on straights AND helps drive turn-in as the wall starts bending
        # away through a corner. Final command:
        # sign * (kp*err + kd*dE - k_alpha*alpha).
        self.declare_parameter("k_alpha", 2.0)
        # Clip |alpha| used in the steering equation. With k_alpha=2.0, an
        # uncapped alpha=35 degrees (typical for a diagonal beam reading through a
        # window) contributes -1.22 rad/s to steering -- 60% of full lock
        # before any other term. Capping at 25 degrees keeps alpha-feedback's max
        # contribution to ~0.87, so PD/yaw still have headroom and a
        # bogus alpha can't single-handedly slam the steering into the wall.
        # Real sharp corners are handled by the wall-lost commit, not alpha.
        self.declare_parameter("max_alpha_deg", 25.0)
        # Clamp the control effort BEFORE bias. +/-2.0 maps to full servo
        # lock (rover uses angular.z*500 clipped to +/-1000).
        self.declare_parameter("max_steering", 2.0)
        # Distance error is clipped to +/-max_error before PD. Stops the
        # controller from panic-saturating when the estimator briefly
        # reports an absurd distance (window, long recess, beam glitch).
        # 1.5 m chosen so the controller has authority to drive ~1+ m
        # corrections (the old 0.4 cap masked target_distance changes
        # entirely -- every error read as +/-0.4 regardless of true gap).
        self.declare_parameter("max_error", 1.5)
        # Anything beyond this is treated as "no wall visible". This is
        # also the right-turn signal: the wall vanishes and stays gone.
        self.declare_parameter("max_plausible_distance", 4.0)
        # Spike detector: a corner grows D and alpha smoothly; a window makes
        # them jump within a single scan. If either delta exceeds its
        # limit, treat the scan as unreliable so it can't masquerade as
        # a corner entry.
        self.declare_parameter("max_d_jump", 0.8)
        # 60 degrees tolerates legitimate fast alpha swings during corner approach
        # (the spike check otherwise rejects them as glitches and starves
        # PD/alpha-feedback of the data it needs to drive the turn-in).
        self.declare_parameter("max_alpha_jump_deg", 60.0)
        # Drop stored last-valid state if no valid scan arrives within
        # this many seconds, so the next scan doesn't compare against
        # stale history.
        self.declare_parameter("spike_stale_s", 0.7)
        # Forward sweep used ONLY for the emergency stop. No corner
        # detection here -- right-wall absence is the corner signal.
        self.declare_parameter("forward_half_window_deg", 20.0)
        # Hard stop if the forward sweep reads closer than this in any
        # mode. Catches "we're driving into a wall" failures regardless
        # of why we got there.
        self.declare_parameter("emergency_stop_fwd_m", 0.0)
        self.declare_parameter("target_distance", 0.3)
        self.declare_parameter("forward_speed", 1.80)
        # Two-ray look-ahead estimator. Angles measured from the car's
        # forward axis (0 degrees), REP-103 convention: +CCW, so the right wall
        # sits at negative angles.
        self.declare_parameter("ray_a_deg", -45.0)  # forward-right beam
        self.declare_parameter("ray_b_deg", -90.0)  # perpendicular-right beam
        self.declare_parameter("ray_half_window_deg", 20.0)
        self.declare_parameter("look_ahead", 0.5)
        # Slow the car down as the wall-angle |alpha| grows (corners, juts).
        # With closed-loop velocity (rover_node use_velocity_feedback=true)
        # the open-loop stall floor is gone -- the PI loop saturates throttle
        # to achieve the commanded m/s regardless of magnitude. Set this to
        # the slowest sustainable corner speed; 0.12 lets corners be taken
        # very slow without stalling. If you ever revert to open-loop in
        # rover_node, raise this back to 0.3-0.4.
        self.declare_parameter("min_forward_speed", 0.25)
        self.declare_parameter("speed_alpha_scale_deg", 45.0)
        # Exponential smoothing on the derivative term (0<a<=1, higher=less smoothing).
        self.declare_parameter("d_error_alpha", 0.5)
        # Steering-output sign. This rover is wired with inverted steering
        # (positive angular.z physically turns RIGHT, not left as REP-103
        # would suggest). If the rover is ever rewired, flip this to +1.
        self.declare_parameter("steering_sign", -1.0)
        # Constant steering offset (in the *post-sign* frame) to trim a
        # mechanically off-centre servo.
        self.declare_parameter("steering_bias", 0.2)
        # Wall-loss handling. Doorways/windows lose the right wall
        # briefly; right-turn corners lose it for good. Coast straight
        # for `lost_coast_s` to clear short gaps, then commit a fixed-
        # duration 90 degree right turn (`commit_turn_s` at full-lock
        # `lost_turn_steering` -- raw post-sign -- +2.0 = full-lock RIGHT
        # on the inverted rover), then hand back to PD. If the wall is
        # still missing on the next scan, a fresh coast-then-turn cycle
        # starts automatically. After `max_lost_time_s` of continuous
        # loss without ever recovering, the car idles (zero velocity)
        # -- the safety net for driving into open space.
        # `commit_turn_s` default of 2.0s is sized for ~45 deg/s rotation
        # at full lock at ~0.4 m/s (`commit_speed`), giving roughly a
        # 90 degree heading change. `lost_coast_s` of 0.3s is long enough to
        # ride out single-scan glitches, short enough that the commit
        # fires near the actual corner (otherwise the car drives
        # several body-lengths past the entrance before turning and
        # ends up too deep in the new corridor to make the turn
        # cleanly). The phase clock resets on any non-spike good scan,
        # so flickery walls (windows, mesh) stay in coast indefinitely
        # rather than accumulating cumulative-lost time into the turn
        # phase.
        self.declare_parameter("lost_coast_s", 0.3)
        self.declare_parameter("lost_turn_steering", 2.0)
        self.declare_parameter("commit_turn_s", 2.0)
        # Speed during the lost cycle (coast + turn + release). Decoupled
        # from v_min so cruise tuning doesn't change corner geometry.
        # 2s x ~45 deg/s rotation at full lock at ~0.4 m/s gives a clean ~90 degrees.
        # At lower speeds a 2s commit only completes a partial turn.
        self.declare_parameter("commit_speed", 0.4)
        # Sticky recovery: require this many consecutive valid scans
        # before declaring wall recovered. Without stickiness, a brief
        # valid scan in the middle of a spike-rejected burst (common
        # during corner approach) resets `_lost_since` and the commit
        # never fires. At ~8-10 Hz scan rate, 3 scans ~= 0.3-0.4s.
        # Decoupled from `lost_coast_s` -- phase clock now uses
        # `_last_good_time`, so the recovery window can outlast coast
        # without firing the turn on a visible wall.
        self.declare_parameter("lost_recovery_scans", 3)
        # Open-space idle timeout. If the wall has been continuously
        # absent for this long (no good scan AT ALL -- flickery walls
        # don't count), stop driving and idle until something changes.
        # Without this, the coast->turn->release cycle repeats forever,
        # spinning the car in place. ~3 cycles at default params.
        self.declare_parameter("max_lost_time_s", 8.0)
        # Damping gain on measured yaw rate (rad/s) from imu/gyro.z. Applied
        # in the REP-103 control frame (before the steering_sign flip), so
        # a positive yaw rate (CCW / left) subtracts from `steering` to
        # oppose current rotation -- works alongside kd (error derivative)
        # and k_alpha (wall-angle feedback).
        self.declare_parameter("k_yaw", 0.15)
        # Morphological min-dilation half-window applied to the scan
        # before any of the ray sampling. For each ray, the output range
        # is the MIN valid range within +/-N degrees of that ray. Closes narrow
        # NaN gaps and pulls bogus through-glass reads back toward the
        # closer wall reading from a few degrees away. Set to 0.0 to
        # disable. 15 degrees was sized for ~2.5ft (0.76m) windows at the
        # 0.8m setpoint -- large enough to catch wall edges adjacent to
        # the window when the diagonal beam punches through.
        self.declare_parameter("scan_dilation_deg", 15.0)
        # Front crash avoidance: two diagonal rays (+/-front_avoid_deg) detect
        # whether the approaching wall is angled or flat.
        #   asymmetry = front_R - front_L > 0  ->  wall like \  ->  steer right
        #   asymmetry < 0                       ->  wall like /  ->  steer left (close only)
        #   |asymmetry| < front_avoid_min_asym  ->  flat wall    ->  full-lock right
        # A gap filter skips avoid when a diagonal reads much farther than the
        # forward distance (doorway/window beside us), unless we are very close.
        self.declare_parameter("front_avoid_thresh", 1.3)        # m -- start checking
        self.declare_parameter("avoid_confirm_scans", 2)          # scans to confirm
        self.declare_parameter("front_avoid_deg", 25.0)           # diagonal angle (deg)
        self.declare_parameter("front_avoid_min_asym", 0.15)      # m -- ignore below this
        self.declare_parameter("front_avoid_kp", 0.15)
        self.declare_parameter("front_avoid_kd", 2.8)
        self.declare_parameter("front_avoid_d_alpha", 0.8)        # D-term low-pass
        self.declare_parameter("front_avoid_max_diag_mult", 3.0)  # diagonal > N*fwd = gap
        self.declare_parameter("front_avoid_abs_gap_thresh", 3.5) # m -- absolute gap limit
        self.declare_parameter("avoid_crash_close_dist", 1.5)     # m -- allow left escape
        self.declare_parameter("avoid_crash_left_max", 1.0)       # max left steer
        self.declare_parameter("avoid_max_speed", 0.6)            # m/s cap during any avoid

    def _setup_publishers(self):
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def _setup_subscriptions(self):
        # RPLIDAR publishes /scan with SENSOR_DATA QoS (best-effort). Using
        # the default (reliable) QoS here causes a silent QoS mismatch and
        # the callback never fires.
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )
        # rover_node publishes imu/gyro with BEST_EFFORT; match it.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.gyro_sub = self.create_subscription(
            Vector3, "imu/gyro", self.gyro_callback, sensor_qos
        )
        self.create_subscription(
            String, "/slam_coordinator/mode", self._mode_cb, 10
        )

    def _mode_cb(self, msg: String):
        if msg.data in ("saving", "ready", "racing") and self._active:
            self._active = False
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f"Mode -> {msg.data.upper()}: wall nav stopping")

    def gyro_callback(self, msg: Vector3):
        self._yaw_rate = msg.z

    @staticmethod
    def _wrap(angle):
        """Wrap an angle to (-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _dilate_scan_ranges(self, msg: LaserScan):
        """Morphological min-dilation of `msg.ranges` over a +/-N degree window.

        For each ray, the output is the MIN valid range within
        +/-`scan_dilation_deg` of that ray's angle. Two effects relevant
        to wall-following:
          - Narrow NaN gaps (windows, doorways narrower than ~2*N degrees)
            get filled by the closest wall reading on either side.
          - A ray that punches through glass and returns a far value
            gets pulled back toward the wall reading a few degrees
            away -- the diagonal beam stops "falling for" windows.
        Returns a Python list of the same length as msg.ranges.
        """
        dilation_deg = self.get_parameter("scan_dilation_deg").value
        if dilation_deg <= 0.0:
            return list(msg.ranges)

        n = len(msg.ranges)
        if n == 0 or msg.angle_increment <= 0.0:
            return list(msg.ranges)

        half = max(1, int(round(math.radians(dilation_deg) / msg.angle_increment)))

        # Mark invalid rays as +inf so they don't pollute the min and
        # don't accidentally win it. NaN/out-of-range/inf all map to inf.
        valid = [
            r if (math.isfinite(r) and msg.range_min <= r <= msg.range_max)
            else float("inf")
            for r in msg.ranges
        ]

        out = [0.0] * n
        for i in range(n):
            lo = max(0, i - half)
            hi = min(n, i + half + 1)
            m = float("inf")
            for j in range(lo, hi):
                if valid[j] < m:
                    m = valid[j]
            # If no valid neighbour in window, restore NaN so downstream
            # validity checks (range_min/max bounds) still reject it.
            out[i] = m if math.isfinite(m) else float("nan")
        return out

    def _ray_at_angle(
        self, msg: LaserScan, target_angle: float, half_window: float
    ) -> float:
        """Mean of valid rays within `half_window` of `target_angle`. NaN if none."""
        target = self._wrap(target_angle)
        readings = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = self._wrap(msg.angle_min + i * msg.angle_increment)
            if abs(self._wrap(angle - target)) <= half_window:
                readings.append(r)
        if not readings:
            return float("nan")
        return sum(readings) / len(readings)

    def _forward_distance(self, msg: LaserScan) -> float:
        """Nearest valid range in a forward-facing window (emergency stop only)."""
        half = math.radians(self.get_parameter("forward_half_window_deg").value)
        target = 0.0
        nearest = float("inf")
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = self._wrap(msg.angle_min + i * msg.angle_increment)
            if abs(self._wrap(angle - target)) <= half:
                if r < nearest:
                    nearest = r
        return nearest

    def _right_wall_state(self, msg: LaserScan):
        """
        Return (D_ahead, alpha):
          alpha   -- car's heading angle relative to the wall (0 = parallel).
          D_ahead -- perpendicular distance to wall a `look_ahead` in front.
        Falls back to single-beam estimates if the other beam is lost to
        a doorway, window, jut, or recess. Both NaN only when both beams
        are missing.
        """
        ray_a_deg = self.get_parameter("ray_a_deg").value
        ray_b_deg = self.get_parameter("ray_b_deg").value
        half = math.radians(self.get_parameter("ray_half_window_deg").value)
        L = self.get_parameter("look_ahead").value

        a = self._ray_at_angle(msg, math.radians(ray_a_deg), half)
        b = self._ray_at_angle(msg, math.radians(ray_b_deg), half)
        a_ok = math.isfinite(a)
        b_ok = math.isfinite(b)

        # Perpendicular (-90 degree) beam missing: report wall lost. Don't fall
        # back to the forward-diagonal alone -- that beam can be hitting
        # whatever's in front of the car (wall ahead, far wall of an
        # intersection) and report it as "right wall", which the PD then
        # follows into impact. A doorway recessed a few feet still
        # returns a valid perpendicular reading off the back of the
        # recess; only true wall-loss (windows, corners, hallway exits)
        # produces a missing perp beam.
        if not b_ok:
            return float("nan"), float("nan")

        # Note: deliberately no geometric sanity check on a/b here.
        # Real right-corner approach IS "a grows large while b stays
        # normal" -- that's the signal we use for early turn-in via
        # alpha-feedback. Rejecting large a/b would suppress the very thing
        # we need to detect corners. Through-glass false positives are
        # mitigated upstream (dilation) and downstream (alpha clip) without
        # killing the corner signal.

        # Forward-diagonal missing: perpendicular only, no look-ahead.
        # `b` is by definition the right wall, so this is safe.
        if not a_ok:
            return b, 0.0

        # Use the magnitude of the angular gap so the formula works
        # regardless of which side of forward the rays sit on.
        theta = abs(math.radians(ray_b_deg - ray_a_deg))
        # F1TENTH estimator: wall angle, then project current distance forward.
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        D_now = b * math.cos(alpha)
        D_ahead = D_now + L * math.sin(alpha)
        return D_ahead, alpha

    def scan_callback(self, msg: LaserScan):
        if not self._active:
            return

        # Pre-process: morphological min-dilation. Closes narrow gaps
        # and suppresses through-glass reads before any ray sampling.
        msg.ranges = self._dilate_scan_ranges(msg)

        D_ahead, alpha = self._right_wall_state(msg)
        fwd = self._forward_distance(msg)

        v_min = self.get_parameter("min_forward_speed").value
        bias = self.get_parameter("steering_bias").value
        sign = self.get_parameter("steering_sign").value
        max_plausible = self.get_parameter("max_plausible_distance").value

        now = self.get_clock().now().nanoseconds * 1e-9

        # Anchor the idle timer to the first scan when the wall has never
        # been visible -- otherwise idle_for would compare against epoch
        # and trip on the very first lost scan.
        if self._last_good_time is None:
            self._last_good_time = now

        # Emergency stop: forward sweep says we're physically close to a
        # wall. Independent safety net -- fires regardless of mode.
        e_stop_fwd = self.get_parameter("emergency_stop_fwd_m").value
        if math.isfinite(fwd) and fwd < e_stop_fwd:
            self.get_logger().warn(
                f"EMERGENCY STOP: fwd={fwd:.2f}m < {e_stop_fwd:.2f}m"
            )
            self.cmd_pub.publish(Twist())
            return

        # --- Front crash avoidance -----------------------------------------------
        # Two diagonal rays at +/-front_avoid_deg measure wall angle:
        #   asymmetry = front_R - front_L > 0  ->  wall like \  ->  steer right
        #   asymmetry < 0                       ->  wall like /  ->  steer left (close)
        #   |asymmetry| small                   ->  flat wall    ->  full-lock right
        # Confirm counter debounces single-scan reflections. Gap filter skips
        # avoid when a diagonal reads much farther than center (doorway/window).
        front_avoid_thresh = self.get_parameter("front_avoid_thresh").value
        avoid_confirm_scans = self.get_parameter("avoid_confirm_scans").value
        avoid_crash_close = self.get_parameter("avoid_crash_close_dist").value

        if fwd < front_avoid_thresh:
            self._avoid_confirm += 1
        else:
            self._avoid_confirm = 0

        if self._avoid_confirm >= avoid_confirm_scans and fwd < front_avoid_thresh:
            diag_half = math.radians(5.0)
            front_avoid_deg_r = math.radians(self.get_parameter("front_avoid_deg").value)
            front_L = self._ray_at_angle(msg, +front_avoid_deg_r, diag_half)
            front_R = self._ray_at_angle(msg, -front_avoid_deg_r, diag_half)

            # NaN diagonal = open space: substitute range_max so asymmetry
            # is large and we steer away from whichever side IS finite.
            # (e.g. approaching a / wall — right diagonal hits, left is open)
            range_max = float(msg.range_max)
            front_L = front_L if math.isfinite(front_L) else range_max
            front_R = front_R if math.isfinite(front_R) else range_max

            max_diag_mult = self.get_parameter("front_avoid_max_diag_mult").value
            abs_gap_thresh = self.get_parameter("front_avoid_abs_gap_thresh").value
            rel_gap = front_L > fwd * max_diag_mult or front_R > fwd * max_diag_mult
            abs_gap = front_L > abs_gap_thresh or front_R > abs_gap_thresh

            if (rel_gap or abs_gap) and fwd > avoid_crash_close:
                self.get_logger().info(
                    f"AVOID SKIP {'abs' if abs_gap else 'rel'}  "
                    f"fwd={fwd:.2f}m L={front_L:.2f} R={front_R:.2f}"
                )
            else:
                avoid_kp = self.get_parameter("front_avoid_kp").value
                avoid_kd = self.get_parameter("front_avoid_kd").value
                avoid_d_alpha = self.get_parameter("front_avoid_d_alpha").value
                min_asym = self.get_parameter("front_avoid_min_asym").value
                crash_left_max = self.get_parameter("avoid_crash_left_max").value
                max_steer_v = self.get_parameter("max_steering").value
                v_max_v = self.get_parameter("forward_speed").value
                proximity = 1.0 - fwd / front_avoid_thresh
                avoid_max_speed = self.get_parameter("avoid_max_speed").value
                avoid_speed = min(avoid_max_speed, max(v_min, v_max_v * (fwd / front_avoid_thresh)))
                asymmetry = front_R - front_L

                # Clamp asymmetry before gains — when one diagonal is open
                # space (substituted with range_max), raw asymmetry can be
                # 7+ m and blow up the steer command. Cap at ±2m so the
                # gains stay in a sensible range regardless of corridor width.
                asymmetry = max(-2.0, min(2.0, asymmetry))
                raw_d_asym = asymmetry - self._prev_asymmetry
                d_asym = (
                    avoid_d_alpha * raw_d_asym
                    + (1.0 - avoid_d_alpha) * self._prev_d_asymmetry
                )
                self._prev_d_asymmetry = d_asym
                self._prev_asymmetry = asymmetry
                if abs(asymmetry) > min_asym:
                    avoid_steer = (
                        avoid_kp * asymmetry * (1.0 + proximity)
                        - avoid_kd * d_asym
                    )
                    avoid_steer = max(-max_steer_v, min(max_steer_v, avoid_steer))
                    avoid_steer += bias
                    self._avoid_confirm = 0  # must reconfirm before firing again
                    cmd = Twist()
                    cmd.linear.x = float(avoid_speed)
                    cmd.angular.z = float(avoid_steer)
                    self.cmd_pub.publish(cmd)
                    self.get_logger().info(
                        f"AVOID fwd={fwd:.2f}m L={front_L:.2f} R={front_R:.2f} "
                        f"asym={asymmetry:+.2f} d={d_asym:+.2f} steer={avoid_steer:+.2f}"
                    )
                    return

        # Spike detector: reject a scan whose (D_ahead, alpha) jumped farther
        # than physically possible from the last valid reading. Prevents
        # a window from masquerading as a corner entry.
        stale_s = self.get_parameter("spike_stale_s").value
        max_d_jump = self.get_parameter("max_d_jump").value
        max_alpha_jump = math.radians(self.get_parameter("max_alpha_jump_deg").value)
        is_spike = False
        if (
            math.isfinite(D_ahead)
            and self._last_valid_time is not None
            and (now - self._last_valid_time) < stale_s
        ):
            dD = abs(D_ahead - self._last_valid_D)
            da = abs(alpha - self._last_valid_alpha)
            if dD > max_d_jump or da > max_alpha_jump:
                is_spike = True
                self.get_logger().info(
                    f"spike rejected: dD={dD:.2f}m da={math.degrees(da):+.1f} deg"
                )
        # Invalidate stale history so we don't compare the next valid
        # scan against an outdated (D, alpha).
        if (
            self._last_valid_time is not None
            and (now - self._last_valid_time) >= stale_s
        ):
            self._last_valid_D = None
            self._last_valid_alpha = None
            self._last_valid_time = None

        # Wall-lost is a GEOMETRIC determination only: perp beam gone
        # (NaN) or D absurd. A spike means "this measurement is
        # untrustworthy," NOT "the wall has disappeared" -- so spikes
        # must NOT advance the lost timer. Coupling them used to let
        # a window-glint burst fire a spurious corner commit mid-hallway.
        wall_lost_now = (not math.isfinite(D_ahead)) or D_ahead > max_plausible

        # Sticky recovery: require N consecutive wall-present scans to
        # clear lost state. Spikes are neutral -- they neither reset
        # nor advance the run, so a spike burst during true wall-loss
        # can't masquerade as recovery, and a spike on a visible wall
        # doesn't kick us into lost mode.
        if wall_lost_now:
            self._valid_run = 0
            if self._lost_since is None:
                self._lost_since = now
        elif is_spike:
            pass
        else:
            self._valid_run += 1
            # Phase/idle clock anchor -- must update HERE (not later in
            # the PD branch) so flickery good scans during sticky
            # recovery still reset the phase clock. Otherwise the
            # cumulative-lost timer drives a spurious turn even though
            # the wall briefly returned.
            self._last_good_time = now

        recovery_scans = self.get_parameter("lost_recovery_scans").value
        treat_as_lost = wall_lost_now or (
            self._lost_since is not None and self._valid_run < recovery_scans
        )

        # Wall lost: doorways/windows lose it briefly, right-turn corners
        # lose it permanently. Four phases:
        #   coast   (0  <= t < lost_coast_s)        -- handles short gaps
        #   turn    (   lost_coast_s <= t < +turn_s) -- fixed-duration ~90 deg
        #   release (t >= lost_coast_s + turn_s)    -- hand back to PD
        #   idle    (idle_for > max_lost_time_s)    -- open-space stop
        # The phase clock `lost_for` uses `_last_good_time` so flickery
        # walls don't accumulate cumulative-lost time; `idle_for` uses
        # _last_good_time WITHOUT the cycle reset, so it tracks true
        # continuous loss across coast->turn->release cycles.
        if treat_as_lost:
            coast_s = self.get_parameter("lost_coast_s").value
            turn_s = self.get_parameter("commit_turn_s").value
            commit_speed = self.get_parameter("commit_speed").value
            max_lost_time_s = self.get_parameter("max_lost_time_s").value

            # Phase clock: time since the wall was last visible OR since
            # this lost cycle began, whichever is later. The `max` means
            # a brief good scan inside the cycle resets the phase, so a
            # flickery wall doesn't drift into the turn phase on
            # cumulative-lost time.
            phase_anchor = max(self._last_good_time, self._lost_since)
            lost_for = now - phase_anchor
            # Idle clock: time since the wall was last actually trusted.
            # Doesn't reset on release -- tracks true continuous loss.
            idle_for = now - self._last_good_time

            # Reset PD state so it doesn't spike when the wall returns.
            self._prev_error = 0.0
            self._prev_d_error = 0.0
            self._prev_time = now

            cmd = Twist()
            cmd.linear.x = float(commit_speed)

            if idle_for > max_lost_time_s:
                # Open space -- give up cycling. Idle until the wall
                # actually comes back; a non-spike good scan will reset
                # _last_good_time and we'll re-enter coast on the next tick.
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                mode = "idle"
            elif lost_for < coast_s:
                # Brief gap (doorway, window) -- coast straight.
                cmd.angular.z = float(bias)
                mode = "coast"
            elif lost_for < coast_s + turn_s:
                # Execute the 90 degree right turn at full lock. Raw post-sign
                # value (no flip, no clamp, no bias) -- +2.0 = full-lock
                # RIGHT on the inverted rover.
                cmd.angular.z = float(self.get_parameter("lost_turn_steering").value)
                mode = "turn"
            else:
                # Turn complete. Release the lost cycle. If the wall is
                # still missing this scan we publish a brief straight so
                # PD doesn't run on NaN; the next scan will start a fresh
                # coast->turn cycle if still lost (or trip idle once
                # idle_for crosses max_lost_time_s).
                cmd.angular.z = float(bias)
                mode = "release"
                self._lost_since = None
                self._valid_run = 0

            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f"wall lost (phase={lost_for:.2f}s idle={idle_for:.2f}s) {mode}: "
                f"steer={cmd.angular.z:+.2f} v={cmd.linear.x:.2f} "
                f"run={self._valid_run}/{recovery_scans}"
            )
            return

        self._lost_since = None

        target = self.get_parameter("target_distance").value
        kp = self.get_parameter("kp").value
        kd = self.get_parameter("kd").value
        k_alpha = self.get_parameter("k_alpha").value
        k_yaw = self.get_parameter("k_yaw").value
        max_steer = self.get_parameter("max_steering").value
        v_max = self.get_parameter("forward_speed").value
        alpha_scale = math.radians(self.get_parameter("speed_alpha_scale_deg").value)
        d_alpha = self.get_parameter("d_error_alpha").value
        max_error = self.get_parameter("max_error").value

        # Spike with wall still visible: fall back to the last trusted
        # (D, alpha) for this scan's PD rather than treating the bad sample
        # as truth. If we don't have recent valid history, skip the PD
        # this scan entirely (hold previous command via no publish).
        if is_spike:
            if self._last_valid_D is not None:
                D_ahead = self._last_valid_D
                alpha = self._last_valid_alpha
            else:
                self.get_logger().info(
                    "spike with no valid history -- skipping PD this scan"
                )
                return
        else:
            # Non-spike, non-lost scan accepted -- record as new reference.
            # (`_last_good_time` is set in the wall-state classification
            # block above so it also updates during sticky recovery.)
            self._last_valid_D = D_ahead
            self._last_valid_alpha = alpha
            self._last_valid_time = now

        # Positive error => too close to the wall => steer left (+angular.z).
        # Clip before the PD so a single outlier reading can't send the
        # gains to full lock.
        error = target - D_ahead
        error = max(-max_error, min(max_error, error))

        if self._prev_time is None:
            d_error = 0.0
        else:
            dt = now - self._prev_time
            raw_d = (error - self._prev_error) / dt if dt > 0 else 0.0
            d_error = d_alpha * raw_d + (1.0 - d_alpha) * self._prev_d_error
        self._prev_error = error
        self._prev_d_error = d_error
        self._prev_time = now

        # PD on distance + alpha-feedback + yaw-rate damping. alpha<0 (car yawed
        # toward right wall) pushes `steering` more positive -> more "turn
        # left" in REP-103 -> actively un-yaws the car while it closes on
        # the target. Yaw-rate term opposes the current rotation directly
        # using the IMU (less noisy than differentiating distance error).
        # Clip alpha used in the alpha-feedback term so a bogus diagonal-beam
        # reading (e.g. through a window) can't single-handedly saturate
        # the steering. Speed scaling below uses unclipped alpha so the
        # car still slows down when alpha is large.
        max_alpha_rad = math.radians(self.get_parameter("max_alpha_deg").value)
        alpha_clipped = max(-max_alpha_rad, min(max_alpha_rad, alpha))
        yaw_rate = self._yaw_rate
        steering = kp * error + kd * d_error - k_alpha * alpha_clipped - k_yaw * yaw_rate
        # Sign-flip (if the rover is wired inverted) then clamp, then bias.
        # Bias shifts the neutral point -- not part of the control effort, so
        # it's applied after the clamp.
        steering = sign * steering
        steering = max(-max_steer, min(max_steer, steering)) + bias

        # Ease off the throttle when the wall is swinging away (corner/jut).
        speed_scale = max(0.0, 1.0 - abs(alpha) / alpha_scale)
        forward_speed = max(v_min, v_max * speed_scale)

        drive_cmd = Twist()
        drive_cmd.linear.x = float(forward_speed)
        drive_cmd.angular.z = float(steering)
        self.cmd_pub.publish(drive_cmd)

        fwd_str = f"{fwd:.2f}" if math.isfinite(fwd) else "  inf"
        self.get_logger().info(
            f"D={D_ahead:.2f}m a={math.degrees(alpha):+.1f}deg fwd={fwd_str}m "
            f"err={error:+.2f} dErr={d_error:+.2f} yaw={yaw_rate:+.2f} "
            f"steer={steering:+.2f} v={forward_speed:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = WallNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish a zero Twist on shutdown so the rover doesn't keep
        # rolling on the last command if rover_node has no watchdog.
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
