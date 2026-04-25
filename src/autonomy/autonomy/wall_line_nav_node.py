"""
wall_line_nav_node.py
Inputs:
  /scan                       sensor_msgs/LaserScan
  /line_follow_point          geometry_msgs/PointStamped (from line_detector)

Output:
  /cmd_vel                    geometry_msgs/Twist
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Twist

FRONT_CLEAR_MIN_SAMPLES = 3
RIGHT_TURN_COOLDOWN_S = 2.0


class WallLineNavNode(Node):
    def __init__(self):
        super().__init__("wall_line_nav_node")

        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()

        self.latest_follow_point: PointStamped | None = None
        self.line_visible = False

        self.mode = "forward"
        self.turn_until = None
        self.cooldown_until = None
        self.right_open_scan_run = 0
        self.last_scan_stamp = None
        self.latest_front_clear = False

        # Spike-detector state: last accepted (D_ahead, alpha) and its time.
        self._last_valid_D = None
        self._last_valid_alpha = None
        self._last_valid_time = None
        self._last_debug_log = None

        # Throttle for the "turn active" status log (Hz-rate spam guard).
        self._last_turn_log = None

        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def _setup_parameters(self):
        self.declare_parameter("forward_speed", 0.30)
        self.declare_parameter("turn_linear_speed", 0.40)
        self.declare_parameter("right_turn_steering", 2.0)
        self.declare_parameter("right_turn_duration", 2.0)
        # Two-ray F1TENTH-style estimator borrowed from claude/wall-following.
        # The perpendicular beam vanishing or the projected wall distance
        # exceeding `right_open_distance` IS the right-corner signal.
        self.declare_parameter("right_open_distance", 4.0)
        self.declare_parameter("right_open_required_scans", 8)
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
        self.declare_parameter("front_clear_distance", 0.8)
        self.declare_parameter("debug_log_period_s", 0.5)
        # Line-follow P control on /line_follow_point. Image is 640 wide; the
        # camera is mounted off-centre so a perfectly-centred line lands
        # at column ~400, which is the steering target.
        self.declare_parameter("line_image_width", 640)
        self.declare_parameter("line_target_x", 400.0)
        self.declare_parameter("line_kp", 2.4)
        # Cap below full lock; the scripted right turn handles hard corners.
        self.declare_parameter("line_max_angular", 1.35)
        self.declare_parameter("line_goal_timeout_s", 1.0)

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
        self.front_clear_distance = self.get_parameter("front_clear_distance").value
        self.debug_log_period_s = self.get_parameter("debug_log_period_s").value
        line_image_width = float(self.get_parameter("line_image_width").value)
        self.line_image_center_x = line_image_width / 2.0
        self.line_target_x = self.get_parameter("line_target_x").value
        self.line_kp = self.get_parameter("line_kp").value
        self.line_max_angular = self.get_parameter("line_max_angular").value
        self.line_goal_timeout_s = self.get_parameter("line_goal_timeout_s").value

    def _setup_subscriptions(self):
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )
        self.follow_point_sub = self.create_subscription(
            PointStamped, "/line_follow_point", self.follow_point_callback, 10
        )

    def _setup_publishers(self):
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def scan_callback(self, msg: LaserScan):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_scan_stamp:
            return
        self.last_scan_stamp = stamp

        D_ahead, alpha = self._right_wall_state(msg)
        front_min = self._forward_min(msg)
        front_clear = (
            math.isfinite(front_min) and front_min >= self.front_clear_distance
        )
        self.latest_front_clear = front_clear

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
            f_str = f"{front_min:.2f}" if math.isfinite(front_min) else " inf"
            self.get_logger().info(
                f"right: D={D_str}m a={a_str}deg lost={int(wall_lost_now)} "
                f"spike={int(is_spike)} run={self.right_open_scan_run}/"
                f"{self.right_open_required_scans} fwd={f_str}m "
                f"fclear={int(front_clear)}"
            )

    def follow_point_callback(self, msg: PointStamped):
        if msg.point.x == 0.0 and msg.point.y == 0.0:
            self.line_visible = False
            self.latest_follow_point = None
            return
        self.line_visible = True
        self.latest_follow_point = msg

    def control_loop(self):
        now = self.get_clock().now()
        cmd = Twist()

        if self.mode == "turn_right":
            if self.turn_until is not None and now < self.turn_until:
                now_s = now.nanoseconds * 1e-9
                if self._last_turn_log is None or (now_s - self._last_turn_log) >= 0.5:
                    self._last_turn_log = now_s
                    remaining = (self.turn_until - now).nanoseconds * 1e-9
                    self.get_logger().info(
                        f"[turn] active: {remaining:.2f}s remaining "
                        f"v={self.turn_linear_speed:.2f} "
                        f"steer={self.right_turn_steering:.2f}"
                    )
                cmd.linear.x = self.turn_linear_speed
                cmd.angular.z = self.right_turn_steering
                self.cmd_pub.publish(cmd)
                return

            self.mode = "forward"
            self.turn_until = None
            self.cooldown_until = now + Duration(seconds=RIGHT_TURN_COOLDOWN_S)
            self.right_open_scan_run = 0
            self._last_turn_log = None
            self.get_logger().info(
                f"[turn] complete: handing back to line PD; "
                f"cooldown {RIGHT_TURN_COOLDOWN_S:.1f}s"
            )

        if (
            (self.cooldown_until is None or now >= self.cooldown_until)
            and self.right_open_scan_run >= self.right_open_required_scans
            and self.latest_front_clear
        ):
            self.mode = "turn_right"
            self.turn_until = now + Duration(seconds=self.right_turn_duration)
            self._last_turn_log = None
            self.get_logger().info(
                "[turn] commit: right wall absent — scripted right turn "
                f"v={self.turn_linear_speed:.2f} "
                f"steer={self.right_turn_steering:.2f} "
                f"duration={self.right_turn_duration:.2f}s"
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
        if follow is None:
            self.cmd_pub.publish(cmd)
            return

        offset = (follow[0] - self.line_target_x) / self.line_image_center_x
        raw = self.line_kp * offset
        # tanh saturates softly inside ±line_max_angular instead of clamping.
        cmd.angular.z = float(
            self.line_max_angular * math.tanh(raw / self.line_max_angular)
        )
        self.cmd_pub.publish(cmd)

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

    def _forward_min(self, msg: LaserScan) -> float:
        front_ranges = self._sector_ranges(msg, -15.0, 15.0)
        if len(front_ranges) < FRONT_CLEAR_MIN_SAMPLES:
            return float("inf")
        return min(front_ranges)

    def _sector_ranges(self, scan: LaserScan, start_deg: float, end_deg: float):
        start = math.radians(start_deg)
        end = math.radians(end_deg)
        ranges = []
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if angle < start or angle > end:
                continue
            if math.isinf(r):
                ranges.append(scan.range_max)
            elif math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                ranges.append(r)
        return ranges


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
