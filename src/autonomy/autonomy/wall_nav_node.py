"""
wall_nav_node.py

PD wall-following controller. Subscribes to /scan, uses a two-ray
look-ahead estimator (F1TENTH-style) to compute the car's angle relative
to the right-hand wall and a projected distance a short look-ahead in
front, then drives a Twist on /cmd_vel that holds the car a fixed
distance from that wall.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


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

    def _setup_parameters(self):
        # Tunable live via `ros2 param set /wall_nav_node <name> <value>`.
        # Note: on this robot `cmd_vel.angular.z` is a normalised STEERING
        # command (rover_node maps it as angular.z * 500 clipped to ±1000,
        # so ±2 = full lock). Gains are tuned for that, not for rad/s.
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("kd", 0.15)
        # α-feedback: counteracts the car's yaw toward/away from the
        # wall. Stops the controller from committing to a big approach
        # angle that it can't recover from before it reaches the wall.
        # Final command: sign * (kp·err + kd·dE − k_alpha·α).
        self.declare_parameter("k_alpha", 1.5)
        self.declare_parameter("max_steering", 1.5)  # clamp before publishing
        # Distance error is clipped to ±max_error before PD. Stops the
        # controller from panic-saturating when the estimator briefly
        # reports an absurd distance (window, long recess, beam glitch).
        self.declare_parameter("max_error", 0.4)
        # Anything beyond this is treated as "no wall visible" and the
        # car coasts instead of trying to chase a phantom wall.
        self.declare_parameter("max_plausible_distance", 2.5)
        self.declare_parameter("target_distance", 0.8)
        self.declare_parameter("forward_speed", 0.5)
        # Two-ray look-ahead estimator. Angles measured from the car's
        # forward axis (0°), REP-103 convention: +CCW, so the right wall
        # sits at negative angles.
        self.declare_parameter("ray_a_deg", -45.0)  # forward-right beam
        self.declare_parameter("ray_b_deg", -90.0)  # perpendicular-right beam
        self.declare_parameter("ray_half_window_deg", 2.0)
        self.declare_parameter("look_ahead", 0.3)
        # Slow the car down as the wall-angle |α| grows (corners, juts).
        # Floor at 0.3 m/s — below this the car stalls on its own weight.
        self.declare_parameter("min_forward_speed", 0.3)
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
        # When the wall briefly drops out (window, open doorway), coast
        # straight at min_forward_speed for this long before giving up.
        self.declare_parameter("lost_wall_timeout_s", 1.5)

    def _setup_publishers(self):
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def _setup_subscriptions(self):
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

    @staticmethod
    def _wrap(angle):
        """Wrap an angle to (-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _ray_at_angle(self, msg: LaserScan, target_angle: float, half_window: float) -> float:
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

        if not a_ok and not b_ok:
            return float("nan"), float("nan")

        # Perpendicular beam missing (e.g. window blows out the -90° ray).
        # Use the forward-diagonal beam alone, assuming parallel wall:
        # D ≈ a · sin(|θ_a|).
        if not b_ok:
            theta_a = math.radians(abs(ray_a_deg))
            return a * math.sin(theta_a), 0.0

        # Forward-diagonal missing: perpendicular only, no look-ahead.
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

        v_min = self.get_parameter("min_forward_speed").value
        bias = self.get_parameter("steering_bias").value
        sign = self.get_parameter("steering_sign").value
        max_plausible = self.get_parameter("max_plausible_distance").value

        now = self.get_clock().now().nanoseconds * 1e-9

        # Treat an implausibly far reading as "wall lost" so the PD
        # doesn't saturate trying to chase a phantom wall across a room.
        wall_lost = (not math.isfinite(D_ahead)) or D_ahead > max_plausible

        # Wall lost entirely (e.g. large window / open doorway on the right).
        # Coast straight at the stall-floor speed so we keep moving through
        # the gap. Stop only if the wall stays lost past the timeout.
        if wall_lost:
            if self._lost_since is None:
                self._lost_since = now
            lost_for = now - self._lost_since
            timeout = self.get_parameter("lost_wall_timeout_s").value
            if lost_for > timeout:
                self.get_logger().warn(
                    f"Right wall lost for {lost_for:.1f}s > {timeout:.1f}s; stopping."
                )
                self.cmd_pub.publish(Twist())
                return
            # Coast: reset PD state so we don't spike when the wall returns.
            self._prev_error = 0.0
            self._prev_d_error = 0.0
            self._prev_time = now
            coast = Twist()
            coast.linear.x = float(v_min)
            coast.angular.z = float(bias)
            self.cmd_pub.publish(coast)
            self.get_logger().info(
                f"wall lost ({lost_for:.2f}s); coasting straight v={v_min:.2f}"
            )
            return

        self._lost_since = None

        target = self.get_parameter("target_distance").value
        kp = self.get_parameter("kp").value
        kd = self.get_parameter("kd").value
        k_alpha = self.get_parameter("k_alpha").value
        max_steer = self.get_parameter("max_steering").value
        v_max = self.get_parameter("forward_speed").value
        alpha_scale = math.radians(
            self.get_parameter("speed_alpha_scale_deg").value
        )
        d_alpha = self.get_parameter("d_error_alpha").value
        max_error = self.get_parameter("max_error").value

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

        # PD on distance + α-feedback. α<0 (car yawed toward right wall)
        # pushes `steering` more positive → more "turn left" in REP-103 →
        # actively un-yaws the car while it closes on the target.
        steering = kp * error + kd * d_error - k_alpha * alpha
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

        self.get_logger().info(
            f"D={D_ahead:.2f}m α={math.degrees(alpha):+.1f}° "
            f"err={error:+.2f} dErr={d_error:+.2f} "
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
