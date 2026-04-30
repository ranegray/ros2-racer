"""
wall_nav_node.py

PD wall-following controller. Subscribes to /scan, applies a
morphological min-dilation to the ranges (closes narrow gaps, pulls
through-glass spuriously-far reads back toward nearby wall returns),
samples a single perpendicular ray on the right side, and drives a
Twist on /cmd_vel that holds the car a fixed distance from the wall.
Also subscribes to imu/gyro and uses the measured yaw rate as a
damping term on the steering output. No diagonal look-ahead beam:
corner detection happens entirely through the wall-lost cycle when
the perpendicular beam clears the wall.

Right-turn handling: the right wall vanishing (D > max_plausible
or perp beam NaN) IS the corner signal. Doorways/windows lose the wall
briefly; real corners lose it permanently. So we coast straight for
`lost_coast_s` (clears short gaps without committing), then execute a
fixed-duration ~90° right turn (`commit_turn_s` at full lock), then
release back to PD. The phase clock resets on any good scan, so a
flickery wall (windows, mesh, partial occluders) keeps us in coast
instead of crossing into the turn phase on cumulative-lost time. If
the wall is still missing the next scan, a fresh coast→turn cycle
starts automatically. After `max_lost_time_s` of continuous loss the
car idles (zero velocity) until the wall returns — prevents endless
spinning in open spaces.
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


class WallNavNode(Node):
    def __init__(self):
        super().__init__("wall_nav_node")
        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()

        self._prev_error = 0.0
        self._prev_d_error = 0.0
        self._prev_time = None
        self._lost_since = None
        # Sticky-recovery counter: consecutive valid scans since we last
        # saw wall_lost. We only exit lost mode when this reaches
        # `lost_recovery_scans` — a single good scan in the middle of a
        # spike-rejected burst does NOT reset the lost timer.
        self._valid_run = 0
        # Time of the last non-spike good scan. Used as the phase-clock
        # anchor inside the lost cycle: when the wall briefly returns
        # (window, mesh fence, etc.), the phase clock resets so a flickery
        # wall doesn't accumulate cumulative-lost time and fire a spurious
        # 90° turn. Also drives the open-space idle timeout: it does NOT
        # reset between coast→turn→release cycles, so if the wall has
        # been continuously absent for `max_lost_time_s`, we stop trying.
        # Initialized to the first scan's timestamp so the idle timer has
        # a sensible anchor when we never see the wall.
        self._last_good_time = None
        # For spike detection: last VALID D and its timestamp.
        self._last_valid_D = None
        self._last_valid_time = None
        # Latest IMU yaw rate (rad/s) for yaw-rate damping on the steering.
        self._yaw_rate = 0.0

    def _setup_parameters(self):
        # Tunable live via `ros2 param set /wall_nav_node <name> <value>`.
        # Note: on this robot `cmd_vel.angular.z` is a normalised STEERING
        # command (rover_node maps it as angular.z * 500 clipped to ±1000,
        # so ±2 = full lock). Gains are tuned for that, not for rad/s.
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("kd", 0.15)
        # Clamp the control effort BEFORE bias. ±2.0 maps to full servo
        # lock (rover uses angular.z*500 clipped to ±1000).
        self.declare_parameter("max_steering", 2.0)
        # Distance error is clipped to ±max_error before PD. Stops the
        # controller from panic-saturating when the estimator briefly
        # reports an absurd distance (window, long recess, beam glitch).
        self.declare_parameter("max_error", 1.5)
        # Anything beyond this is treated as "no wall visible". This is
        # also the right-turn signal: the wall vanishes and stays gone.
        self.declare_parameter("max_plausible_distance", 4.0)
        # Spike detector: a corner grows D smoothly; a window makes
        # it jump within a single scan. If the delta exceeds the
        # limit, treat the scan as unreliable.
        self.declare_parameter("max_d_jump", 0.8)
        # Drop stored last-valid state if no valid scan arrives within
        # this many seconds, so the next scan doesn't compare against
        # stale history.
        self.declare_parameter("spike_stale_s", 0.7)
        # Forward sweep used ONLY for the emergency stop. No corner
        # detection here — right-wall absence is the corner signal.
        self.declare_parameter("forward_half_window_deg", 20.0)
        # Hard stop if the forward sweep reads closer than this in any
        # mode. Catches "we're driving into a wall" failures regardless
        # of why we got there.
        self.declare_parameter("emergency_stop_fwd_m", 0.0)
        self.declare_parameter("target_distance", 0.8)
        self.declare_parameter("forward_speed", 1.20)
        # Single perpendicular ray on the right side. Angles measured
        # from the car's forward axis (0°), REP-103 convention: +CCW,
        # so the right wall sits at negative angles. The half-window
        # averages multiple rays in a small arc for noise robustness.
        self.declare_parameter("ray_b_deg", -90.0)
        self.declare_parameter("ray_half_window_deg", 2.0)
        # Exponential smoothing on the derivative term (0<a<=1, higher=less smoothing).
        self.declare_parameter("d_error_alpha", 0.5)
        # Steering-output sign. This rover is wired with inverted steering
        # (positive angular.z physically turns RIGHT, not left as REP-103
        # would suggest). If the rover is ever rewired, flip this to +1.
        self.declare_parameter("steering_sign", -1.0)
        # Constant steering offset (in the *post-sign* frame) to trim a
        # mechanically off-centre servo.
        self.declare_parameter("steering_bias", 0.0)
        # Wall-loss handling. Doorways/windows lose the right wall
        # briefly; right-turn corners lose it for good. Coast straight
        # for `lost_coast_s` to clear short gaps, then commit a fixed-
        # duration 90° right turn (`commit_turn_s` at full-lock
        # `lost_turn_steering` — raw post-sign — +2.0 = full-lock RIGHT
        # on the inverted rover), then hand back to PD. If the wall is
        # still missing on the next scan, a fresh coast-then-turn cycle
        # starts automatically. After `max_lost_time_s` of continuous
        # loss without ever recovering, the car idles (zero velocity)
        # — the safety net for driving into open space.
        # `commit_turn_s` default of 2.0s is sized for ~45°/s rotation
        # at full lock at ~0.4 m/s (`commit_speed`), giving roughly a
        # 90° heading change. `lost_coast_s` of 0.3s is long enough to
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
        # 2s × ~45°/s rotation at full lock at ~0.4 m/s gives a clean ~90°.
        # At lower speeds a 2s commit only completes a partial turn.
        self.declare_parameter("commit_speed", 0.4)
        # Sticky recovery: require this many consecutive valid scans
        # before declaring wall recovered. Without stickiness, a brief
        # valid scan in the middle of a spike-rejected burst (common
        # during corner approach) resets `_lost_since` and the commit
        # never fires. At ~8-10 Hz scan rate, 3 scans ≈ 0.3–0.4s.
        # Decoupled from `lost_coast_s` — phase clock now uses
        # `_last_good_time`, so the recovery window can outlast coast
        # without firing the turn on a visible wall.
        self.declare_parameter("lost_recovery_scans", 3)
        # Open-space idle timeout. If the wall has been continuously
        # absent for this long (no good scan AT ALL — flickery walls
        # don't count), stop driving and idle until something changes.
        # Without this, the coast→turn→release cycle repeats forever,
        # spinning the car in place. ~3 cycles at default params.
        self.declare_parameter("max_lost_time_s", 8.0)
        # Damping gain on measured yaw rate (rad/s) from imu/gyro.z. Applied
        # in the REP-103 control frame (before the steering_sign flip), so
        # a positive yaw rate (CCW / left) subtracts from `steering` to
        # oppose current rotation. With α-feedback removed, this is the
        # primary heading-correction signal alongside the kd derivative.
        self.declare_parameter("k_yaw", 0.15)
        # Morphological min-dilation half-window applied to the scan
        # before any of the ray sampling. For each ray, the output range
        # is the MIN valid range within ±N° of that ray. Closes narrow
        # NaN gaps and pulls bogus through-glass reads back toward the
        # closer wall reading from a few degrees away. Set to 0.0 to
        # disable. 15° was sized for ~2.5ft (0.76m) windows at the
        # 0.8m setpoint — large enough to catch wall edges adjacent to
        # the window when the diagonal beam punches through.
        self.declare_parameter("scan_dilation_deg", 15.0)

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

    def gyro_callback(self, msg: Vector3):
        self._yaw_rate = msg.z

    @staticmethod
    def _wrap(angle):
        """Wrap an angle to (-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _dilate_scan_ranges(self, msg: LaserScan):
        """Morphological min-dilation of `msg.ranges` over a ±N° window.

        For each ray, the output is the MIN valid range within
        ±`scan_dilation_deg` of that ray's angle. Two effects relevant
        to wall-following:
          - Narrow NaN gaps (windows, doorways narrower than ~2*N°)
            get filled by the closest wall reading on either side.
          - A ray that punches through glass and returns a far value
            gets pulled back toward the wall reading a few degrees
            away — the diagonal beam stops "falling for" windows.
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

    def _right_wall_distance(self, msg: LaserScan) -> float:
        """Perpendicular distance to the right wall, NaN if no return."""
        ray_b_deg = self.get_parameter("ray_b_deg").value
        half = math.radians(self.get_parameter("ray_half_window_deg").value)
        return self._ray_at_angle(msg, math.radians(ray_b_deg), half)

    def scan_callback(self, msg: LaserScan):
        # Pre-process: morphological min-dilation. Closes narrow gaps
        # and suppresses through-glass reads before any ray sampling.
        msg.ranges = self._dilate_scan_ranges(msg)

        D = self._right_wall_distance(msg)
        fwd = self._forward_distance(msg)

        bias = self.get_parameter("steering_bias").value
        sign = self.get_parameter("steering_sign").value
        max_plausible = self.get_parameter("max_plausible_distance").value

        now = self.get_clock().now().nanoseconds * 1e-9

        # Anchor the idle timer to the first scan when the wall has never
        # been visible — otherwise idle_for would compare against epoch
        # and trip on the very first lost scan.
        if self._last_good_time is None:
            self._last_good_time = now

        # Emergency stop: forward sweep says we're physically close to a
        # wall. Independent safety net — fires regardless of mode.
        e_stop_fwd = self.get_parameter("emergency_stop_fwd_m").value
        if math.isfinite(fwd) and fwd < e_stop_fwd:
            self.get_logger().warn(
                f"EMERGENCY STOP: fwd={fwd:.2f}m < {e_stop_fwd:.2f}m"
            )
            self.cmd_pub.publish(Twist())
            return

        # Spike detector: reject a scan whose D jumped farther than
        # physically possible from the last valid reading. Prevents a
        # window glint from masquerading as a corner entry.
        stale_s = self.get_parameter("spike_stale_s").value
        max_d_jump = self.get_parameter("max_d_jump").value
        is_spike = False
        if (
            math.isfinite(D)
            and self._last_valid_time is not None
            and (now - self._last_valid_time) < stale_s
        ):
            dD = abs(D - self._last_valid_D)
            if dD > max_d_jump:
                is_spike = True
                self.get_logger().info(f"spike rejected: ΔD={dD:.2f}m")
        # Invalidate stale history so we don't compare the next valid
        # scan against an outdated D.
        if (
            self._last_valid_time is not None
            and (now - self._last_valid_time) >= stale_s
        ):
            self._last_valid_D = None
            self._last_valid_time = None

        # Wall-lost is a GEOMETRIC determination only: perp beam gone
        # (NaN) or D absurd. A spike means "this measurement is
        # untrustworthy," NOT "the wall has disappeared" — so spikes
        # must NOT advance the lost timer. Coupling them used to let
        # a window-glint burst fire a spurious corner commit mid-hallway.
        wall_lost_now = (not math.isfinite(D)) or D > max_plausible

        # Sticky recovery: require N consecutive wall-present scans to
        # clear lost state. Spikes are neutral — they neither reset
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
            # Phase/idle clock anchor — must update HERE (not later in
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
        #   coast   (0  ≤ t < lost_coast_s)        — handles short gaps
        #   turn    (   lost_coast_s ≤ t < +turn_s) — fixed-duration ~90°
        #   release (t ≥ lost_coast_s + turn_s)    — hand back to PD
        #   idle    (idle_for > max_lost_time_s)    — open-space stop
        # The phase clock `lost_for` uses `_last_good_time` so flickery
        # walls don't accumulate cumulative-lost time; `idle_for` uses
        # _last_good_time WITHOUT the cycle reset, so it tracks true
        # continuous loss across coast→turn→release cycles.
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
            # Doesn't reset on release — tracks true continuous loss.
            idle_for = now - self._last_good_time

            # Reset PD state so it doesn't spike when the wall returns.
            self._prev_error = 0.0
            self._prev_d_error = 0.0
            self._prev_time = now

            cmd = Twist()
            cmd.linear.x = float(commit_speed)

            if idle_for > max_lost_time_s:
                # Open space — give up cycling. Idle until the wall
                # actually comes back; a non-spike good scan will reset
                # _last_good_time and we'll re-enter coast on the next tick.
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                mode = "idle"
            elif lost_for < coast_s:
                # Brief gap (doorway, window) — coast straight.
                cmd.angular.z = float(bias)
                mode = "coast"
            elif lost_for < coast_s + turn_s:
                # Execute the 90° right turn at full lock. Raw post-sign
                # value (no flip, no clamp, no bias) — +2.0 = full-lock
                # RIGHT on the inverted rover.
                cmd.angular.z = float(self.get_parameter("lost_turn_steering").value)
                mode = "turn"
            else:
                # Turn complete. Release the lost cycle. If the wall is
                # still missing this scan we publish a brief straight so
                # PD doesn't run on NaN; the next scan will start a fresh
                # coast→turn cycle if still lost (or trip idle once
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
        k_yaw = self.get_parameter("k_yaw").value
        max_steer = self.get_parameter("max_steering").value
        v_max = self.get_parameter("forward_speed").value
        d_alpha = self.get_parameter("d_error_alpha").value
        max_error = self.get_parameter("max_error").value

        # Spike with wall still visible: fall back to the last trusted D
        # for this scan's PD rather than treating the bad sample as
        # truth. If we don't have recent valid history, skip the PD this
        # scan entirely (hold previous command via no publish).
        if is_spike:
            if self._last_valid_D is not None:
                D = self._last_valid_D
            else:
                self.get_logger().info(
                    "spike with no valid history — skipping PD this scan"
                )
                return
        else:
            # Non-spike, non-lost scan accepted — record as new reference.
            # (`_last_good_time` is set in the wall-state classification
            # block above so it also updates during sticky recovery.)
            self._last_valid_D = D
            self._last_valid_time = now

        # Positive error => too close to the wall => steer left (+angular.z).
        # Clip before the PD so a single outlier reading can't send the
        # gains to full lock.
        error = target - D
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

        # PD on distance + yaw-rate damping. Yaw-rate term opposes the
        # current rotation directly using the IMU — corrects heading
        # drift (the job α-feedback used to do, now from a sensor
        # signal that isn't fooled by glass).
        yaw_rate = self._yaw_rate
        steering = kp * error + kd * d_error - k_yaw * yaw_rate
        # Sign-flip (if the rover is wired inverted) then clamp, then bias.
        # Bias shifts the neutral point — not part of the control effort, so
        # it's applied after the clamp.
        steering = sign * steering
        steering = max(-max_steer, min(max_steer, steering)) + bias

        drive_cmd = Twist()
        drive_cmd.linear.x = float(v_max)
        drive_cmd.angular.z = float(steering)
        self.cmd_pub.publish(drive_cmd)

        fwd_str = f"{fwd:.2f}" if math.isfinite(fwd) else "  inf"
        self.get_logger().info(
            f"D={D:.2f}m fwd={fwd_str}m err={error:+.2f} dErr={d_error:+.2f} "
            f"yaw={yaw_rate:+.2f} steer={steering:+.2f} v={v_max:.2f}"
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
