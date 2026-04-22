"""
wall_nav_node.py

PD wall-following controller. Subscribes to /scan, uses a two-ray
look-ahead estimator (F1TENTH-style) to compute the car's angle relative
to the right-hand wall and a projected distance a short look-ahead in
front, then drives a Twist on /cmd_vel that holds the car a fixed
distance from that wall. Also subscribes to imu/gyro and uses the
measured yaw rate as an additional damping term on the steering output.

Right-turn handling: the right wall vanishing (D_ahead > max_plausible
or estimator NaN) IS the corner signal. Doorways/windows lose the wall
briefly; real corners lose it permanently. So we coast straight for
`lost_coast_s` (clears short gaps without committing), then execute a
fixed-duration ~90° right turn (`commit_turn_s` at full lock), then
release back to PD. If the wall is still missing the next scan, a
fresh coast→turn cycle starts automatically.
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
        # For spike detection: last VALID (D_ahead, alpha) and its timestamp.
        self._last_valid_D = None
        self._last_valid_alpha = None
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
        # α-feedback: counteracts the car's yaw toward/away from the wall
        # on straights AND helps drive turn-in as the wall starts bending
        # away through a corner. Final command:
        # sign * (kp·err + kd·dE − k_alpha·α).
        self.declare_parameter("k_alpha", 3.5)
        # Clamp the control effort BEFORE bias. ±2.0 maps to full servo
        # lock (rover uses angular.z*500 clipped to ±1000).
        self.declare_parameter("max_steering", 2.0)
        # Distance error is clipped to ±max_error before PD. Stops the
        # controller from panic-saturating when the estimator briefly
        # reports an absurd distance (window, long recess, beam glitch).
        self.declare_parameter("max_error", 0.4)
        # Anything beyond this is treated as "no wall visible". This is
        # also the right-turn signal: the wall vanishes and stays gone.
        self.declare_parameter("max_plausible_distance", 4.0)
        # Spike detector: a corner grows D and α smoothly; a window makes
        # them jump within a single scan. If either delta exceeds its
        # limit, treat the scan as unreliable so it can't masquerade as
        # a corner entry.
        self.declare_parameter("max_d_jump", 0.8)
        # 60° tolerates legitimate fast α swings during corner approach
        # (the spike check otherwise rejects them as glitches and starves
        # PD/α-feedback of the data it needs to drive the turn-in).
        self.declare_parameter("max_alpha_jump_deg", 60.0)
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
        self.declare_parameter("emergency_stop_fwd_m", 0.45)
        self.declare_parameter("target_distance", 0.8)
        self.declare_parameter("forward_speed", 0.5)
        # Two-ray look-ahead estimator. Angles measured from the car's
        # forward axis (0°), REP-103 convention: +CCW, so the right wall
        # sits at negative angles.
        self.declare_parameter("ray_a_deg", -45.0)  # forward-right beam
        self.declare_parameter("ray_b_deg", -90.0)  # perpendicular-right beam
        self.declare_parameter("ray_half_window_deg", 2.0)
        self.declare_parameter("look_ahead", 0.5)
        # Slow the car down as the wall-angle |α| grows (corners, juts).
        # 0.3 is the rolling stall floor, but 0.4 is needed to break
        # static friction at full steering lock during a corner commit
        # (the lost-cycle "turn" phase) — at 0.3 the car can stick mid-
        # rotation and never complete the 90°.
        self.declare_parameter("min_forward_speed", 0.4)
        self.declare_parameter("speed_alpha_scale_deg", 45.0)
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
        # starts automatically. The emergency-stop on close forward
        # distance is the actual safety net for "we got into open space".
        # `commit_turn_s` default of 2.0s is sized for ~45°/s rotation
        # at full lock at 0.3 m/s, giving roughly a 90° heading change.
        # 0.3s ≈ 2-3 scans at ~8-10 Hz: long enough to ride out single-
        # scan glitches, short enough that the commit fires near the
        # actual corner (otherwise the car drives several body-lengths
        # past the entrance before turning and ends up too deep in the
        # new corridor to make the turn cleanly).
        self.declare_parameter("lost_coast_s", 0.3)
        self.declare_parameter("lost_turn_steering", 2.0)
        self.declare_parameter("commit_turn_s", 2.0)
        # Sticky recovery: require this many consecutive valid scans
        # before declaring wall recovered. Without stickiness, a brief
        # valid scan in the middle of a spike-rejected burst (common
        # during corner approach) resets `_lost_since` and the commit
        # never fires. At ~8-10 Hz scan rate, 3 scans ≈ 0.3–0.4s.
        self.declare_parameter("lost_recovery_scans", 3)
        # Damping gain on measured yaw rate (rad/s) from imu/gyro.z. Applied
        # in the REP-103 control frame (before the steering_sign flip), so
        # a positive yaw rate (CCW / left) subtracts from `steering` to
        # oppose current rotation — works alongside kd (error derivative)
        # and k_alpha (wall-angle feedback).
        self.declare_parameter("k_yaw", 0.15)

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
          alpha   — car's heading angle relative to the wall (0 = parallel).
          D_ahead — perpendicular distance to wall a `look_ahead` in front.
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

        # Perpendicular (-90°) beam missing: report wall lost. Don't fall
        # back to the forward-diagonal alone — that beam can be hitting
        # whatever's in front of the car (wall ahead, far wall of an
        # intersection) and report it as "right wall", which the PD then
        # follows into impact. A doorway recessed a few feet still
        # returns a valid perpendicular reading off the back of the
        # recess; only true wall-loss (windows, corners, hallway exits)
        # produces a missing perp beam.
        if not b_ok:
            return float("nan"), float("nan")

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
        D_ahead, alpha = self._right_wall_state(msg)
        fwd = self._forward_distance(msg)

        v_min = self.get_parameter("min_forward_speed").value
        bias = self.get_parameter("steering_bias").value
        sign = self.get_parameter("steering_sign").value
        max_plausible = self.get_parameter("max_plausible_distance").value

        now = self.get_clock().now().nanoseconds * 1e-9

        # Emergency stop: forward sweep says we're physically close to a
        # wall. Independent safety net — fires regardless of mode.
        e_stop_fwd = self.get_parameter("emergency_stop_fwd_m").value
        if math.isfinite(fwd) and fwd < e_stop_fwd:
            self.get_logger().warn(
                f"EMERGENCY STOP: fwd={fwd:.2f}m < {e_stop_fwd:.2f}m"
            )
            self.cmd_pub.publish(Twist())
            return

        # Spike detector: reject a scan whose (D_ahead, α) jumped farther
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
                    f"spike rejected: ΔD={dD:.2f}m Δα={math.degrees(da):+.1f}°"
                )
        # Invalidate stale history so we don't compare the next valid
        # scan against an outdated (D, α).
        if (
            self._last_valid_time is not None
            and (now - self._last_valid_time) >= stale_s
        ):
            self._last_valid_D = None
            self._last_valid_alpha = None
            self._last_valid_time = None

        # Current-frame wall-lost determination.
        wall_lost_now = (
            (not math.isfinite(D_ahead)) or D_ahead > max_plausible or is_spike
        )

        # Sticky recovery: a single valid scan does NOT end lost mode.
        # During corner approach the spike detector rejects legitimate
        # fast α swings as "spikes" — and the brief valid scans between
        # those rejections used to reset `_lost_since`, which meant the
        # commit never had time to fire. Now we require N consecutive
        # valid scans before clearing lost state.
        if wall_lost_now:
            self._valid_run = 0
            if self._lost_since is None:
                self._lost_since = now
        else:
            self._valid_run += 1

        recovery_scans = self.get_parameter("lost_recovery_scans").value
        treat_as_lost = wall_lost_now or (
            self._lost_since is not None and self._valid_run < recovery_scans
        )

        # Wall lost: doorways/windows lose it briefly, right-turn corners
        # lose it permanently. Three phases:
        #   coast   (0  ≤ t < lost_coast_s)        — handles short gaps
        #   turn    (   lost_coast_s ≤ t < +turn_s) — fixed-duration ~90°
        #   release (t ≥ lost_coast_s + turn_s)    — hand back to PD
        # If the wall is still missing the next scan, the cycle restarts.
        if treat_as_lost:
            lost_for = now - self._lost_since
            coast_s = self.get_parameter("lost_coast_s").value
            turn_s = self.get_parameter("commit_turn_s").value

            # Reset PD state so it doesn't spike when the wall returns.
            self._prev_error = 0.0
            self._prev_d_error = 0.0
            self._prev_time = now

            cmd = Twist()
            cmd.linear.x = float(v_min)

            if lost_for < coast_s:
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
                # coast→turn cycle if still lost.
                cmd.angular.z = float(bias)
                mode = "release"
                self._lost_since = None
                self._valid_run = 0

            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f"wall lost ({lost_for:.2f}s) {mode}: "
                f"steer={cmd.angular.z:+.2f} v={v_min:.2f} "
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

        # Scan accepted — record it as the new reference for spike detection.
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

        # PD on distance + α-feedback + yaw-rate damping. α<0 (car yawed
        # toward right wall) pushes `steering` more positive → more "turn
        # left" in REP-103 → actively un-yaws the car while it closes on
        # the target. Yaw-rate term opposes the current rotation directly
        # using the IMU (less noisy than differentiating distance error).
        yaw_rate = self._yaw_rate
        steering = kp * error + kd * d_error - k_alpha * alpha - k_yaw * yaw_rate
        # Sign-flip (if the rover is wired inverted) then clamp, then bias.
        # Bias shifts the neutral point — not part of the control effort, so
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
            f"D={D_ahead:.2f}m α={math.degrees(alpha):+.1f}° fwd={fwd_str}m "
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
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
