"""
wall_nav_node.py

Minimal PD wall-following controller. Subscribes to /scan, reads the
perpendicular right-side beam, and drives a Twist on /cmd_vel that holds
the car a fixed distance from the right wall.

Right-turn handling: the right wall vanishing (reading missing or beyond
max_plausible_distance) IS the corner signal. Coast straight for
`lost_coast_s` to clear short gaps (doorways, windows), then execute a
fixed-duration full-lock right turn for `commit_turn_s`, then release
back to PD. If the wall is still missing on the next scan, a fresh
coast→turn cycle starts automatically.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallNavNode(Node):
    def __init__(self):
        super().__init__("wall_nav_node")
        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()

        self._prev_error = 0.0
        self._prev_time = None
        self._lost_since = None

    def _setup_parameters(self):
        # `cmd_vel.angular.z` on this robot is a normalised STEERING command
        # (rover_node maps it as angular.z * 500 clipped to ±1000, so ±2 =
        # full lock). Gains are tuned for that, not for rad/s.
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("kd", 0.15)
        self.declare_parameter("max_steering", 2.0)
        # Error is clipped to ±max_error before PD so one bad reading
        # can't send the gains to full lock.
        self.declare_parameter("max_error", 0.4)
        # Anything beyond this is "no wall visible" → lost-wall cycle.
        self.declare_parameter("max_plausible_distance", 4.0)
        # Forward emergency-stop sweep.
        self.declare_parameter("forward_half_window_deg", 20.0)
        self.declare_parameter("emergency_stop_fwd_m", 0.45)
        self.declare_parameter("target_distance", 0.8)
        self.declare_parameter("forward_speed", 0.5)
        # Perpendicular-right beam (REP-103: +CCW, right wall is negative).
        self.declare_parameter("ray_deg", -90.0)
        self.declare_parameter("ray_half_window_deg", 2.0)
        # 0.4 is needed to break static friction at full lock during the
        # commit turn — below that the car can stall mid-rotation.
        self.declare_parameter("min_forward_speed", 0.4)
        # Positive angular.z physically turns RIGHT on this rover (wired
        # inverted vs REP-103). Flip to +1 if rewired.
        self.declare_parameter("steering_sign", -1.0)
        self.declare_parameter("steering_bias", 0.0)
        # Lost-wall cycle: coast for `lost_coast_s` (clears doorways/
        # windows), then full-lock right for `commit_turn_s` (~90° at
        # ~45°/s), then release. `lost_turn_steering` is raw post-sign
        # — +2.0 = full-lock RIGHT on the inverted rover.
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
        self.declare_parameter("k_yaw", 0.00)

    def _setup_publishers(self):
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def _setup_subscriptions(self):
        # RPLIDAR publishes /scan with SENSOR_DATA QoS (best-effort). The
        # default (reliable) QoS causes a silent mismatch and the callback
        # never fires.
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _ray_at_angle(
        self, msg: LaserScan, target_angle: float, half_window: float
    ) -> float:
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
        half = math.radians(self.get_parameter("forward_half_window_deg").value)
        nearest = float("inf")
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = self._wrap(msg.angle_min + i * msg.angle_increment)
            if abs(self._wrap(angle)) <= half:
                if r < nearest:
                    nearest = r
        return nearest

    def scan_callback(self, msg: LaserScan):
        ray_deg = self.get_parameter("ray_deg").value
        ray_half = math.radians(self.get_parameter("ray_half_window_deg").value)
        D = self._ray_at_angle(msg, math.radians(ray_deg), ray_half)
        fwd = self._forward_distance(msg)

        v_min = self.get_parameter("min_forward_speed").value
        bias = self.get_parameter("steering_bias").value
        sign = self.get_parameter("steering_sign").value
        max_plausible = self.get_parameter("max_plausible_distance").value

        now = self.get_clock().now().nanoseconds * 1e-9

        # Emergency stop — independent safety net.
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
        #   coast   (0  ≤ t < lost_coast_s)             — short-gap guard
        #   turn    (t ≥ lost_coast_s, |Δyaw| < target) — full-lock right
        #   release (|Δyaw| ≥ target OR t ≥ turn_s cap) — hand back to PD
        # IMU yaw-delta is the primary turn-exit condition; commit_turn_s
        # is a safety cap if the gyro fails or the car stalls. If the
        # wall is still missing the next scan, the cycle restarts.
        if treat_as_lost:
            lost_for = now - self._lost_since
            coast_s = self.get_parameter("lost_coast_s").value
            turn_s = self.get_parameter("commit_turn_s").value

            # Reset PD state so it doesn't spike when the wall returns.
            self._prev_error = 0.0
            self._prev_time = now

            cmd = Twist()
            cmd.linear.x = float(v_min)

            if lost_for < coast_s:
                cmd.angular.z = float(bias)
                mode = "coast"
            elif lost_for < coast_s + turn_s:
                cmd.angular.z = float(self.get_parameter("lost_turn_steering").value)
                mode = "turn"
            else:
                cmd.angular.z = float(bias)
                mode = "release"
                self._lost_since = None

            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f"wall lost ({lost_for:.2f}s) {mode}: "
                f"steer={cmd.angular.z:+.2f} v={v_min:.2f}"
            )
            return

        self._lost_since = None

        target = self.get_parameter("target_distance").value
        kp = self.get_parameter("kp").value
        kd = self.get_parameter("kd").value
        max_steer = self.get_parameter("max_steering").value
        v = self.get_parameter("forward_speed").value
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
            d_error = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error
        self._prev_time = now

        steering = kp * error + kd * d_error
        # Sign flip (if rover is wired inverted), then clamp, then bias.
        # Bias is a neutral-point trim, not control effort, so it goes
        # after the clamp.
        steering = sign * steering
        steering = max(-max_steer, min(max_steer, steering)) + bias

        forward_speed = max(v_min, v)

        drive_cmd = Twist()
        drive_cmd.linear.x = float(forward_speed)
        drive_cmd.angular.z = float(steering)
        self.cmd_pub.publish(drive_cmd)

        fwd_str = f"{fwd:.2f}" if math.isfinite(fwd) else "  inf"
        self.get_logger().info(
            f"D={D:.2f}m fwd={fwd_str}m err={error:+.2f} dErr={d_error:+.2f} "
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
