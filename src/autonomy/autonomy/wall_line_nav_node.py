"""
wall_line_nav_node.py
Inputs:
  /scan                       sensor_msgs/LaserScan
  /line_follow_point          geometry_msgs/PointStamped (from line_detector)
  /imu/gyro                   geometry_msgs/Vector3 (yaw-rate damping for wall-follow)

Output:
  /cmd_vel                    geometry_msgs/Twist
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Twist, Vector3

RIGHT_TURN_COOLDOWN_S = 10.0


class WallLineNavNode(Node):
    def __init__(self):
        super().__init__("wall_line_nav_node")

        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()

        self.latest_follow_point: PointStamped | None = None
        self.line_visible = False
        self.line_smoothed_offset = 0.0
        self.line_prev_offset = 0.0
        self.line_last_steer = 0.0

        self.mode = "forward"
        self.turn_until = None
        self.cooldown_until = None
        self.right_open_scan_run = 0
        self.last_scan_stamp = None

        # Spike-detector state: last accepted (D_ahead, alpha) and its time.
        self._last_valid_D = None
        self._last_valid_alpha = None
        self._last_valid_time = None
        self._last_debug_log = None

        # Mode-output tracking: name of the active control mode and the
        # last time we logged it. _log_mode emits an immediate transition
        # line on change and a throttled status line otherwise.
        self._mode_str = None
        self._last_mode_log = None

        # Wall-follow fallback PD state. Reset whenever the line is
        # reacquired so the derivative term doesn't spike on next loss.
        self._wall_prev_error = 0.0
        self._wall_prev_d_error = 0.0
        self._wall_prev_time = None
        self._yaw_rate = 0.0

        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def _setup_parameters(self):
        self.declare_parameter("forward_speed", 0.30)
        self.declare_parameter("turn_linear_speed", 0.36)
        self.declare_parameter("right_turn_steering", 2.0)
        self.declare_parameter("right_turn_duration", 3.0)
        # Two-ray F1TENTH-style estimator borrowed from claude/wall-following.
        # The perpendicular beam vanishing or the projected wall distance
        # exceeding `right_open_distance` IS the right-corner signal.
        self.declare_parameter("right_open_distance", 3.0)
        self.declare_parameter("right_open_required_scans", 6)
        self.declare_parameter("ray_a_deg", -45.0)
        self.declare_parameter("ray_b_deg", -90.0)
        self.declare_parameter("ray_half_window_deg", 2.0)
        self.declare_parameter("look_ahead", 0.5)
        # Spike rejector: a real corner grows D and alpha smoothly across
        # scans; a window glints them within one scan. Spikes are NEUTRAL —
        # they neither advance nor reset the lost-run counter.
        self.declare_parameter("max_d_jump", 0.8)
        self.declare_parameter("max_alpha_jump_deg", 60.0)
        self.declare_parameter("spike_stale_s", 0.7)
        self.declare_parameter("debug_log_period_s", 0.5)
        # Line-follow P control on /line_follow_point. Image is 640 wide; the
        # camera is mounted off-centre so a perfectly-centred line lands
        # at column ~400, which is the steering target.
        self.declare_parameter("line_image_width", 640)
        self.declare_parameter("line_target_x", 400.0)
        self.declare_parameter("line_kp", 1.4)
        # D term on smoothed offset reacts to drift before P alone would.
        self.declare_parameter("line_kd", 1.5)
        # Right bias counters mechanical left-pull on the wheels.
        self.declare_parameter("line_steering_trim", 0.0)
        # Keep line steering gentle; scripted right turn handles hard corners.
        self.declare_parameter("line_max_angular", 0.75)
        self.declare_parameter("line_goal_timeout_s", 1.0)
        self.declare_parameter("line_missing_speed", 0.27)
        self.declare_parameter("line_offset_alpha", 0.25)
        self.declare_parameter("line_offset_deadband", 0.0)
        self.declare_parameter("line_max_steer_step", 0.2)
        # Wall-follow fallback (used when the line is missing). PD on the
        # same two-ray right-wall estimator that drives the right-turn
        # detector, ported from claude/wall-following-pd-controller-eeppz.
        # `wall_steering_sign=-1` matches this rover's inverted servo so
        # error>0 (too close) commands a left turn.
        self.declare_parameter("wall_kp", 0.8)
        self.declare_parameter("wall_kd", 0.15)
        self.declare_parameter("wall_k_alpha", 3.5)
        self.declare_parameter("wall_k_yaw", 0.15)
        self.declare_parameter("wall_target_distance", 0.6)
        self.declare_parameter("wall_max_error", 1.5)
        self.declare_parameter("wall_max_steering", 2.0)
        self.declare_parameter("wall_d_error_alpha", 0.5)
        self.declare_parameter("wall_steering_sign", -1.0)
        # Don't fall back to wall-follow on a stale (D, α) — coast straight
        # instead until either rf2o resumes or the line returns.
        self.declare_parameter("wall_data_max_age_s", 0.7)

        self.forward_speed = self.get_parameter("forward_speed").value
        self.turn_linear_speed = self.get_parameter("turn_linear_speed").value
        self.right_turn_steering = self.get_parameter("right_turn_steering").value
        self.right_turn_duration = self.get_parameter("right_turn_duration").value
        self.right_open_distance = self.get_parameter("right_open_distance").value
        self.right_open_required_scans = self.get_parameter(
            "right_open_required_scans"
        ).value
        self.ray_a_deg = self.get_parameter("ray_a_deg").value
        self.ray_b_deg = self.get_parameter("ray_b_deg").value
        self.ray_half_window = math.radians(
            self.get_parameter("ray_half_window_deg").value
        )
        self.look_ahead = self.get_parameter("look_ahead").value
        self.max_d_jump = self.get_parameter("max_d_jump").value
        self.max_alpha_jump = math.radians(
            self.get_parameter("max_alpha_jump_deg").value
        )
        self.spike_stale_s = self.get_parameter("spike_stale_s").value
        self.debug_log_period_s = self.get_parameter("debug_log_period_s").value
        line_image_width = float(self.get_parameter("line_image_width").value)
        self.line_image_center_x = line_image_width / 2.0
        self.line_target_x = self.get_parameter("line_target_x").value
        self.line_kp = self.get_parameter("line_kp").value
        self.line_kd = self.get_parameter("line_kd").value
        self.line_steering_trim = self.get_parameter("line_steering_trim").value
        self.line_max_angular = self.get_parameter("line_max_angular").value
        self.line_goal_timeout_s = self.get_parameter("line_goal_timeout_s").value
        self.line_missing_speed = self.get_parameter("line_missing_speed").value
        self.line_offset_alpha = self.get_parameter("line_offset_alpha").value
        self.line_offset_deadband = self.get_parameter("line_offset_deadband").value
        self.line_max_steer_step = self.get_parameter("line_max_steer_step").value
        self.wall_kp = self.get_parameter("wall_kp").value
        self.wall_kd = self.get_parameter("wall_kd").value
        self.wall_k_alpha = self.get_parameter("wall_k_alpha").value
        self.wall_k_yaw = self.get_parameter("wall_k_yaw").value
        self.wall_target_distance = self.get_parameter("wall_target_distance").value
        self.wall_max_error = self.get_parameter("wall_max_error").value
        self.wall_max_steering = self.get_parameter("wall_max_steering").value
        self.wall_d_error_alpha = self.get_parameter("wall_d_error_alpha").value
        self.wall_steering_sign = self.get_parameter("wall_steering_sign").value
        self.wall_data_max_age_s = self.get_parameter("wall_data_max_age_s").value

    def _setup_subscriptions(self):
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )
        self.follow_point_sub = self.create_subscription(
            PointStamped, "/line_follow_point", self.follow_point_callback, 10
        )
        # rover_node publishes /imu/gyro BEST_EFFORT/VOLATILE.
        gyro_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.gyro_sub = self.create_subscription(
            Vector3, "/imu/gyro", self.gyro_callback, gyro_qos
        )

    def gyro_callback(self, msg: Vector3):
        self._yaw_rate = msg.z

    def _setup_publishers(self):
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def scan_callback(self, msg: LaserScan):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_scan_stamp:
            return
        self.last_scan_stamp = stamp

        D_ahead, alpha = self._right_wall_state(msg)

        now = self.get_clock().now().nanoseconds * 1e-9

        # Spike check: a finite reading that jumped too far from the last
        # accepted scan within the staleness window. NaN bypasses (no
        # finite delta to compute) — true wall-loss isn't a "spike."
        is_spike = False
        if (
            math.isfinite(D_ahead)
            and self._last_valid_time is not None
            and (now - self._last_valid_time) < self.spike_stale_s
        ):
            dD = abs(D_ahead - self._last_valid_D)
            da = abs(alpha - self._last_valid_alpha)
            if dD > self.max_d_jump or da > self.max_alpha_jump:
                is_spike = True
        if (
            self._last_valid_time is not None
            and (now - self._last_valid_time) >= self.spike_stale_s
        ):
            self._last_valid_D = None
            self._last_valid_alpha = None
            self._last_valid_time = None

        wall_lost_now = (
            not math.isfinite(D_ahead)
        ) or D_ahead > self.right_open_distance

        if is_spike:
            pass
        elif wall_lost_now:
            self.right_open_scan_run += 1
        else:
            self.right_open_scan_run = 0
            self._last_valid_D = D_ahead
            self._last_valid_alpha = alpha
            self._last_valid_time = now

        if (
            self._last_debug_log is None
            or (now - self._last_debug_log) >= self.debug_log_period_s
        ):
            self._last_debug_log = now
            D_str = f"{D_ahead:5.2f}" if math.isfinite(D_ahead) else "  NaN"
            a_str = f"{math.degrees(alpha):+6.1f}" if math.isfinite(alpha) else "   NaN"
            self.get_logger().info(
                f"right: D={D_str}m a={a_str}deg lost={int(wall_lost_now)} "
                f"spike={int(is_spike)} run={self.right_open_scan_run}/"
                f"{self.right_open_required_scans}"
            )

    def follow_point_callback(self, msg: PointStamped):
        if msg.point.x == 0.0 and msg.point.y == 0.0:
            self.line_visible = False
            self.latest_follow_point = None
            self.line_smoothed_offset = 0.0
            return
        self.line_visible = True
        self.latest_follow_point = msg

    def control_loop(self):
        now = self.get_clock().now()
        cmd = Twist()

        now_s = now.nanoseconds * 1e-9

        if self.mode == "turn_right":
            if self.turn_until is not None and now < self.turn_until:
                remaining = (self.turn_until - now).nanoseconds * 1e-9
                self._log_mode(
                    now_s,
                    "turn",
                    f"right turn: {remaining:.2f}s remaining "
                    f"v={self.turn_linear_speed:.2f} "
                    f"steer={self.right_turn_steering:.2f}",
                )
                cmd.linear.x = self.turn_linear_speed
                cmd.angular.z = self.right_turn_steering
                self.cmd_pub.publish(cmd)
                return

            self.mode = "forward"
            self.turn_until = None
            self.cooldown_until = now + Duration(seconds=RIGHT_TURN_COOLDOWN_S)
            self.right_open_scan_run = 0
            self.get_logger().info(
                f"[turn] complete: handing back to line PD; "
                f"cooldown {RIGHT_TURN_COOLDOWN_S:.1f}s"
            )

        if (
            self.cooldown_until is None or now >= self.cooldown_until
        ) and self.right_open_scan_run >= self.right_open_required_scans:
            self.mode = "turn_right"
            self.turn_until = now + Duration(seconds=self.right_turn_duration)
            self.get_logger().info(
                "[turn] commit: right wall absent — scripted right turn "
                f"v={self.turn_linear_speed:.2f} "
                f"steer={self.right_turn_steering:.2f} "
                f"duration={self.right_turn_duration:.2f}s"
            )
            self._log_mode(
                now_s,
                "turn",
                f"right turn: starting "
                f"v={self.turn_linear_speed:.2f} "
                f"steer={self.right_turn_steering:.2f}",
            )
            cmd.linear.x = self.turn_linear_speed
            cmd.angular.z = self.right_turn_steering
            self.cmd_pub.publish(cmd)
            return

        self._publish_line_follow(now)

    def _publish_line_follow(self, now):
        cmd = Twist()
        cmd.linear.x = float(self.forward_speed)
        follow = (
            self._extract_line_point(self.latest_follow_point, now)
            if self.line_visible
            else None
        )
        now_s = now.nanoseconds * 1e-9

        if follow is None:
            # Line missing — try the wall-follow fallback first. Only fall
            # through to the coast-straight path if the lidar estimator
            # also has no fresh data.
            if self._publish_wall_follow(now):
                # Reset line PD state so re-acquire doesn't D-spike.
                self.line_prev_offset = self.line_smoothed_offset
                self.line_last_steer = 0.0
                return
            cmd.linear.x = float(self.line_missing_speed)
            self.line_prev_offset = self.line_smoothed_offset
            self.line_last_steer = self._slew_line_steer(0.0)
            cmd.angular.z = float(self.line_last_steer)
            self.cmd_pub.publish(cmd)
            self._log_mode(
                now_s,
                "coast",
                f"line lost & no fresh wall data — "
                f"v={self.line_missing_speed:.2f} steer={self.line_last_steer:+.2f}",
            )
            return

        # Line reacquired — reset wall PD so its derivative term doesn't
        # spike on the next loss.
        self._reset_wall_follow_state()

        offset = (follow[0] - self.line_target_x) / self.line_image_center_x
        self.line_smoothed_offset = (
            self.line_offset_alpha * offset
            + (1.0 - self.line_offset_alpha) * self.line_smoothed_offset
        )
        if abs(self.line_smoothed_offset) < self.line_offset_deadband:
            self.line_smoothed_offset = 0.0

        d_offset = (self.line_smoothed_offset - self.line_prev_offset) / 0.05
        self.line_prev_offset = self.line_smoothed_offset

        raw = (
            self.line_kp * self.line_smoothed_offset
            + self.line_kd * d_offset
            + self.line_steering_trim
        )
        # tanh saturates softly inside ±line_max_angular instead of clamping.
        target_steer = float(
            self.line_max_angular * math.tanh(raw / self.line_max_angular)
        )
        self.line_last_steer = self._slew_line_steer(target_steer)
        cmd.angular.z = float(self.line_last_steer)
        self.cmd_pub.publish(cmd)
        self._log_mode(
            now_s,
            "line",
            f"offset={self.line_smoothed_offset:+.2f} "
            f"steer={self.line_last_steer:+.2f} v={self.forward_speed:.2f}",
        )

    def _slew_line_steer(self, target):
        delta = max(
            -self.line_max_steer_step,
            min(self.line_max_steer_step, target - self.line_last_steer),
        )
        return self.line_last_steer + delta

    def _publish_wall_follow(self, now) -> bool:
        """PD right-wall fallback when the line is missing.

        Reuses (D_ahead, alpha) from scan_callback's spike-rejected state.
        Returns True if a Twist was published, False if there's no fresh
        wall data and the caller should coast straight instead.
        """
        if (
            self._last_valid_D is None
            or self._last_valid_alpha is None
            or self._last_valid_time is None
        ):
            return False
        now_s = now.nanoseconds * 1e-9
        if (now_s - self._last_valid_time) > self.wall_data_max_age_s:
            return False

        D_ahead = self._last_valid_D
        alpha = self._last_valid_alpha

        # Positive error => too close to right wall => want to steer LEFT.
        # The sign flip at the end inverts to match the rover's wiring.
        error = self.wall_target_distance - D_ahead
        error = max(-self.wall_max_error, min(self.wall_max_error, error))

        if self._wall_prev_time is None:
            d_error = 0.0
        else:
            dt = now_s - self._wall_prev_time
            raw_d = (error - self._wall_prev_error) / dt if dt > 0 else 0.0
            d_error = (
                self.wall_d_error_alpha * raw_d
                + (1.0 - self.wall_d_error_alpha) * self._wall_prev_d_error
            )
        self._wall_prev_error = error
        self._wall_prev_d_error = d_error
        self._wall_prev_time = now_s

        steering = (
            self.wall_kp * error
            + self.wall_kd * d_error
            - self.wall_k_alpha * alpha
            - self.wall_k_yaw * self._yaw_rate
        )
        steering = self.wall_steering_sign * steering
        steering = max(
            -self.wall_max_steering, min(self.wall_max_steering, steering)
        )

        cmd = Twist()
        cmd.linear.x = float(self.line_missing_speed)
        cmd.angular.z = float(steering)
        self.cmd_pub.publish(cmd)
        self._log_mode(
            now_s,
            "wall",
            f"D={D_ahead:.2f}m α={math.degrees(alpha):+.1f}° "
            f"err={error:+.2f} steer={steering:+.2f} "
            f"v={self.line_missing_speed:.2f}",
        )
        return True

    def _reset_wall_follow_state(self):
        self._wall_prev_error = 0.0
        self._wall_prev_d_error = 0.0
        self._wall_prev_time = None

    def _log_mode(self, now_s, name, detail):
        """Emit an immediate transition line on mode change, then a 2 Hz
        throttled status line while the mode persists."""
        if name != self._mode_str:
            prev = self._mode_str or "init"
            self.get_logger().info(f"[mode] {prev} → {name}: {detail}")
            self._mode_str = name
            self._last_mode_log = now_s
            return
        if self._last_mode_log is None or (now_s - self._last_mode_log) >= 0.5:
            self._last_mode_log = now_s
            self.get_logger().info(f"[{name}] {detail}")

    def _extract_line_point(self, msg: PointStamped | None, now):
        if msg is None:
            return None
        age_s = (now - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        if age_s > self.line_goal_timeout_s:
            return None
        return (msg.point.x, msg.point.y)

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _ray_at_angle(self, msg: LaserScan, target_angle: float) -> float:
        # Mean of valid rays within ray_half_window of target_angle. NaN
        # if none — that's the "beam missing" signal the caller relies on.
        target = self._wrap(target_angle)
        readings = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = self._wrap(msg.angle_min + i * msg.angle_increment)
            if abs(self._wrap(angle - target)) <= self.ray_half_window:
                readings.append(r)
        if not readings:
            return float("nan")
        return sum(readings) / len(readings)

    def _right_wall_state(self, msg: LaserScan):
        # F1TENTH two-ray estimator. Returns (D_ahead, alpha) where alpha
        # is the car's heading vs the right wall (0 = parallel) and
        # D_ahead is the perpendicular distance projected `look_ahead`
        # in front. Perp beam (-90°) missing => wall absent.
        a = self._ray_at_angle(msg, math.radians(self.ray_a_deg))
        b = self._ray_at_angle(msg, math.radians(self.ray_b_deg))
        a_ok = math.isfinite(a)
        b_ok = math.isfinite(b)
        if not b_ok:
            return float("nan"), float("nan")
        if not a_ok:
            return b, 0.0
        theta = abs(math.radians(self.ray_b_deg - self.ray_a_deg))
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        D_now = b * math.cos(alpha)
        D_ahead = D_now + self.look_ahead * math.sin(alpha)
        return D_ahead, alpha


def main(args=None):
    rclpy.init(args=args)
    node = WallLineNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
