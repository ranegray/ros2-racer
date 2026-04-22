"""
wall_nav_node.py

Heading-locked wall follower for a square course with 4 right turns.

Architecture: integrate yaw from imu/gyro.z (with a ~0.8s startup bias
calibration while the car is stationary), snapshot the heading at
cal-end as `reference[0]`. Down each side, HOLD the reference heading
with a small lateral nudge proportional to wall-distance error — the
car drives in a straight line with a tiny crab angle instead of
chasing the wall in a PD loop.

When the right wall vanishes (corner), commit full-lock right until
|Δyaw| reaches `turn_exit_deg` from the pre-commit heading, then
advance the side index (`reference[i] = reference[0] − i·90°`) and
resume heading-hold on the next side. `commit_turn_s` is a safety
cap; the IMU delta is the primary exit condition. After `num_turns`
the car has returned to the starting orientation and stops.

Emergency stop on close forward distance is the independent safety
net across all modes.
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

        # --- Yaw integration (from imu/gyro.z) ---
        self._yaw = 0.0
        self._yaw_rate = 0.0
        self._last_gyro_time = None
        # Gyro bias: averaged during the stationary startup window.
        self._gyro_bias = 0.0
        self._bias_samples = []
        self._bias_start_time = None
        self._bias_done = False

        # --- Square-course state ---
        self._reference_heading = None  # locked at end of bias cal
        self._side_index = 0  # 0..num_turns-1 down each straight
        self._commit_active = False
        self._commit_start_yaw = None
        self._commit_start_time = None
        # Don't trigger a new commit immediately after the last one;
        # the wall may not be back in view yet as the car straightens.
        self._post_turn_until = 0.0

        # --- Straight-mode helpers ---
        self._lost_since = None
        # Last accepted D_ahead (used to reject single-scan spikes
        # without triggering corner logic).
        self._last_valid_D = None
        self._last_valid_time = None

    def _setup_parameters(self):
        # Note: cmd_vel.angular.z is a STEERING command on this rover
        # (rover_node maps angular.z * 500 clipped to ±1000, so ±2.0
        # = full servo lock). Gains are tuned for that, not rad/s.

        # --- Heading-hold control ---
        # Steering per radian of heading error. At max_steering=2.0 and
        # 30° error, k_heading=2.0 puts steering at ~1.05 — strong but
        # not saturated, so the yaw-rate damping still has authority.
        self.declare_parameter("k_heading", 2.0)
        # Yaw-rate damping (rad/s). Opposes rotation directly — less
        # noisy than derivative of heading error.
        self.declare_parameter("k_yaw", 0.3)
        # Distance error → heading bias (rad per metre). Positive when
        # too close to the wall (D<target) so the bias aims the car
        # AWAY from the wall. Clipped by max_lateral_bias_deg.
        self.declare_parameter("k_lat", 0.35)
        self.declare_parameter("max_lateral_bias_deg", 12.0)

        # --- Output shaping ---
        self.declare_parameter("max_steering", 2.0)
        # Rover is wired inverted relative to REP-103 (positive angular.z
        # physically turns RIGHT). Flip to +1 if rewired.
        self.declare_parameter("steering_sign", -1.0)
        self.declare_parameter("steering_bias", 0.0)

        # --- Distance estimator (F1TENTH two-ray) ---
        self.declare_parameter("target_distance", 0.8)
        self.declare_parameter("max_plausible_distance", 4.0)
        self.declare_parameter("ray_a_deg", -45.0)
        self.declare_parameter("ray_b_deg", -90.0)
        self.declare_parameter("ray_half_window_deg", 2.0)
        self.declare_parameter("look_ahead", 0.5)

        # --- Forward sweep (emergency stop only) ---
        self.declare_parameter("forward_half_window_deg", 20.0)
        self.declare_parameter("emergency_stop_fwd_m", 0.45)

        # --- Speed ---
        self.declare_parameter("forward_speed", 0.5)
        self.declare_parameter("min_forward_speed", 0.4)

        # --- IMU bias calibration ---
        # Car must be stationary for this long at startup. Average of
        # gyro.z over the window is subtracted thereafter.
        self.declare_parameter("bias_cal_s", 0.8)

        # --- Corner / commit ---
        # Must be wall-lost this long before committing — absorbs
        # single-scan glitches and lets a window gap pass without a
        # spurious turn.
        self.declare_parameter("lost_coast_s", 0.2)
        # IMU-based exit: turn completes when |Δyaw| ≥ this. A little
        # under 90° so the car is already lined up on the next side
        # as it exits (the steering releases and the car drifts in
        # slightly on its momentum).
        self.declare_parameter("turn_exit_deg", 85.0)
        # Safety cap on commit duration.
        self.declare_parameter("commit_turn_s", 3.5)
        # Grace period after commit exit during which wall-lost does
        # NOT trigger another commit. Gives the controller time to
        # settle on the new reference and the wall to come into view.
        self.declare_parameter("post_turn_s", 1.5)

        # --- Square-course plan ---
        # 4 right turns returns the car to its start orientation. Set
        # to 0 to loop the course indefinitely.
        self.declare_parameter("num_turns", 4)

        # --- Spike rejection (decoupled from wall-lost) ---
        # A scan whose D_ahead jumps more than this from the last
        # accepted value within spike_stale_s is treated as an
        # outlier — D is ignored for this scan's lateral bias, but
        # the commit logic does NOT fire.
        self.declare_parameter("max_d_jump", 0.8)
        self.declare_parameter("spike_stale_s", 0.7)

    def _setup_publishers(self):
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def _setup_subscriptions(self):
        # RPLIDAR publishes /scan with SENSOR_DATA QoS (best-effort).
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.gyro_sub = self.create_subscription(
            Vector3, "imu/gyro", self.gyro_callback, sensor_qos
        )

    # -----------------------------------------------------------------
    # Gyro: bias calibration + yaw integration
    # -----------------------------------------------------------------

    def gyro_callback(self, msg: Vector3):
        now = self.get_clock().now().nanoseconds * 1e-9
        z = msg.z

        if not self._bias_done:
            if self._bias_start_time is None:
                self._bias_start_time = now
            self._bias_samples.append(z)
            self._last_gyro_time = now
            cal_s = self.get_parameter("bias_cal_s").value
            if (now - self._bias_start_time) >= cal_s and len(self._bias_samples) > 0:
                self._gyro_bias = sum(self._bias_samples) / len(self._bias_samples)
                self._bias_done = True
                self._yaw = 0.0
                self._reference_heading = 0.0
                self.get_logger().info(
                    f"gyro bias cal done: bias={self._gyro_bias:+.4f} rad/s "
                    f"from {len(self._bias_samples)} samples — "
                    f"reference heading locked at yaw=0"
                )
            return

        corrected = z - self._gyro_bias
        self._yaw_rate = corrected
        if self._last_gyro_time is None:
            self._last_gyro_time = now
            return
        dt = now - self._last_gyro_time
        self._last_gyro_time = now
        # Guard against large dt (paused clock, etc.).
        if 0.0 < dt < 0.5:
            self._yaw = self._wrap(self._yaw + corrected * dt)

    # -----------------------------------------------------------------
    # LIDAR helpers
    # -----------------------------------------------------------------

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

    def _right_wall_distance(self, msg: LaserScan) -> float:
        """
        Two-ray look-ahead distance to the right wall. NaN if the
        perpendicular beam is missing (true wall loss). Falls back to
        the perpendicular beam alone if only the forward-diagonal is
        missing (partial occlusion).
        """
        ray_a_deg = self.get_parameter("ray_a_deg").value
        ray_b_deg = self.get_parameter("ray_b_deg").value
        half = math.radians(self.get_parameter("ray_half_window_deg").value)
        L = self.get_parameter("look_ahead").value

        a = self._ray_at_angle(msg, math.radians(ray_a_deg), half)
        b = self._ray_at_angle(msg, math.radians(ray_b_deg), half)
        a_ok = math.isfinite(a)
        b_ok = math.isfinite(b)

        if not b_ok:
            # Perp beam missing = wall gone.
            return float("nan")
        if not a_ok:
            # Degraded: perp only, no look-ahead projection.
            return b

        theta = abs(math.radians(ray_b_deg - ray_a_deg))
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        D_now = b * math.cos(alpha)
        return D_now + L * math.sin(alpha)

    # -----------------------------------------------------------------
    # Control
    # -----------------------------------------------------------------

    def _reference_for_side(self, side_index: int) -> float:
        # Every hard turn is a right turn → CW → yaw decreases by π/2.
        return self._wrap(self._reference_heading - (math.pi / 2) * side_index)

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_full_right(self, linear: float):
        """Full-lock right turn routed through steering_sign."""
        sign = self.get_parameter("steering_sign").value
        max_steer = self.get_parameter("max_steering").value
        bias = self.get_parameter("steering_bias").value
        cmd = Twist()
        cmd.linear.x = float(linear)
        # Right in the pre-sign (REP-103) frame is negative angular.z.
        cmd.angular.z = float(sign * (-max_steer) + bias)
        self.cmd_pub.publish(cmd)

    def scan_callback(self, msg: LaserScan):
        now = self.get_clock().now().nanoseconds * 1e-9
        fwd = self._forward_distance(msg)
        D = self._right_wall_distance(msg)

        # Emergency stop — independent of mode.
        e_stop = self.get_parameter("emergency_stop_fwd_m").value
        if math.isfinite(fwd) and fwd < e_stop:
            self.get_logger().warn(
                f"EMERGENCY STOP: fwd={fwd:.2f}m < {e_stop:.2f}m"
            )
            self._publish_stop()
            return

        # Wait for bias calibration to complete before driving.
        if not self._bias_done or self._reference_heading is None:
            self._publish_stop()
            return

        # Course complete.
        num_turns = self.get_parameter("num_turns").value
        if num_turns > 0 and self._side_index >= num_turns:
            self._publish_stop()
            return

        v_min = self.get_parameter("min_forward_speed").value

        # ---- Commit (turn in progress) ----
        if self._commit_active:
            delta = self._wrap(self._yaw - self._commit_start_yaw)
            elapsed = now - self._commit_start_time
            turn_exit = math.radians(self.get_parameter("turn_exit_deg").value)
            timeout = self.get_parameter("commit_turn_s").value
            # Right turn: yaw decreases (CW), so delta is negative.
            done = (-delta) >= turn_exit or elapsed >= timeout
            if not done:
                self._publish_full_right(v_min)
                self.get_logger().info(
                    f"commit: side={self._side_index} "
                    f"Δyaw={math.degrees(-delta):+.1f}°/"
                    f"{math.degrees(turn_exit):.0f}° "
                    f"t={elapsed:.2f}s"
                )
                return
            # Turn complete — advance and fall through to straight mode.
            reason = "timeout" if elapsed >= timeout else "yaw-target"
            self._side_index += 1
            self._commit_active = False
            self._commit_start_yaw = None
            self._commit_start_time = None
            self._lost_since = None
            self._post_turn_until = now + self.get_parameter("post_turn_s").value
            self.get_logger().info(
                f"commit exit ({reason}): advanced to side={self._side_index}, "
                f"yaw={math.degrees(self._yaw):+.1f}° "
                f"new_ref={math.degrees(self._reference_for_side(self._side_index)):+.1f}°"
            )
            # Course complete after this turn?
            if num_turns > 0 and self._side_index >= num_turns:
                self._publish_stop()
                return

        # ---- Spike rejection (D only, does NOT trigger commit) ----
        stale_s = self.get_parameter("spike_stale_s").value
        max_d_jump = self.get_parameter("max_d_jump").value
        D_usable = D
        if (
            math.isfinite(D)
            and self._last_valid_D is not None
            and self._last_valid_time is not None
            and (now - self._last_valid_time) < stale_s
            and abs(D - self._last_valid_D) > max_d_jump
        ):
            self.get_logger().info(
                f"D spike rejected: {D:.2f}m vs last {self._last_valid_D:.2f}m"
            )
            D_usable = float("nan")
        else:
            if math.isfinite(D):
                self._last_valid_D = D
                self._last_valid_time = now
        # Drop stale history.
        if (
            self._last_valid_time is not None
            and (now - self._last_valid_time) >= stale_s
        ):
            self._last_valid_D = None
            self._last_valid_time = None

        # ---- Wall-lost → commit decision ----
        max_plausible = self.get_parameter("max_plausible_distance").value
        wall_lost = (not math.isfinite(D)) or D > max_plausible

        if wall_lost:
            if self._lost_since is None:
                self._lost_since = now
            lost_for = now - self._lost_since
            coast_s = self.get_parameter("lost_coast_s").value
            # Only start a commit after the coast window AND outside
            # the post-turn grace period.
            if lost_for >= coast_s and now >= self._post_turn_until:
                self._commit_active = True
                self._commit_start_yaw = self._yaw
                self._commit_start_time = now
                self._publish_full_right(v_min)
                self.get_logger().info(
                    f"commit start: side={self._side_index} "
                    f"yaw={math.degrees(self._yaw):+.1f}° "
                    f"lost_for={lost_for:.2f}s"
                )
                return
            # Still in coast / post-turn grace — fall through to
            # straight mode with no lateral bias (D unavailable).
        else:
            self._lost_since = None

        # ---- Straight mode: heading-hold + lateral bias ----
        ref = self._reference_for_side(self._side_index)
        k_h = self.get_parameter("k_heading").value
        k_y = self.get_parameter("k_yaw").value
        k_lat = self.get_parameter("k_lat").value
        max_bias = math.radians(self.get_parameter("max_lateral_bias_deg").value)
        target = self.get_parameter("target_distance").value
        max_steer = self.get_parameter("max_steering").value
        sign = self.get_parameter("steering_sign").value
        bias_out = self.get_parameter("steering_bias").value

        # Lateral bias: positive (CCW / left in REP-103) when too close
        # to the right wall. Zero when D unavailable so we just hold
        # the reference heading.
        if math.isfinite(D_usable) and D_usable <= max_plausible:
            lat_bias = k_lat * (target - D_usable)
            lat_bias = max(-max_bias, min(max_bias, lat_bias))
        else:
            lat_bias = 0.0

        setpoint = self._wrap(ref + lat_bias)
        heading_err = self._wrap(setpoint - self._yaw)

        # Pre-sign (REP-103) steering: positive error → positive
        # angular.z → left. Sign-flip applied on output.
        steering_pre = k_h * heading_err - k_y * self._yaw_rate
        steering_pre = max(-max_steer, min(max_steer, steering_pre))
        steering_cmd = sign * steering_pre + bias_out

        # Slow down when heading error is large (entry of a new side,
        # recovery from perturbation).
        v_max = self.get_parameter("forward_speed").value
        err_frac = min(1.0, abs(heading_err) / math.radians(30.0))
        forward_speed = max(v_min, v_max * (1.0 - 0.5 * err_frac))

        cmd = Twist()
        cmd.linear.x = float(forward_speed)
        cmd.angular.z = float(steering_cmd)
        self.cmd_pub.publish(cmd)

        D_str = f"{D_usable:.2f}" if math.isfinite(D_usable) else "  nan"
        fwd_str = f"{fwd:.2f}" if math.isfinite(fwd) else "  inf"
        self.get_logger().info(
            f"side={self._side_index} D={D_str}m fwd={fwd_str}m "
            f"ref={math.degrees(ref):+.1f}° yaw={math.degrees(self._yaw):+.1f}° "
            f"err={math.degrees(heading_err):+.1f}° bias={math.degrees(lat_bias):+.1f}° "
            f"steer={steering_cmd:+.2f} v={forward_speed:.2f}"
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
