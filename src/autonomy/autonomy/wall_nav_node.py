"""
wall_nav_node.py

PD wall-following controller for a closed-loop course (no real hallway
exits). Subscribes to /scan, fits a line by total least squares through
right-side lidar returns to estimate the car's angle α relative to the
wall and its perpendicular distance D, then drives a Twist on /cmd_vel
that holds the car a fixed distance from that wall. Also subscribes to
imu/gyro and uses the measured yaw rate as an additional damping term
on the steering output.

The fit averages over ~30–60 beams per scan, so a single noisy return
barely moves (D, α). On a scan where the fit can't find enough inliers
(or its residual is too high), we hold the previous (D, α) for a short
window and keep running PD — there are no real doorways on this course,
so "wall lost" is always noise or transient occlusion, not a corner to
commit a turn for. Corners are handled naturally: as the wall bends,
α grows, PD + α-feedback steers the car through. The forward-sweep
emergency stop is the only safety net that can override PD.
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
        # Last accepted (D_ahead, α) and its timestamp. On a fit failure
        # we reuse these for up to hold_last_valid_s before coasting.
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
        # on straights AND drives turn-in as the wall bends through a
        # corner. Final command: sign * (kp·err + kd·dE − k_alpha·α − k_yaw·ω).
        self.declare_parameter("k_alpha", 3.5)
        # Clamp the control effort BEFORE bias. ±2.0 maps to full servo
        # lock (rover uses angular.z*500 clipped to ±1000).
        self.declare_parameter("max_steering", 2.0)
        # Distance error is clipped to ±max_error before PD. Prevents
        # panic-saturation on a briefly absurd D_ahead.
        self.declare_parameter("max_error", 0.4)
        # Beams farther than this are dropped before the fit (long returns
        # from a window or the far wall of an intersection shouldn't pull
        # the line).
        self.declare_parameter("max_plausible_distance", 4.0)
        # Forward sweep used ONLY for the emergency stop.
        self.declare_parameter("forward_half_window_deg", 20.0)
        # Hard stop if the forward sweep reads closer than this. Independent
        # safety net — fires regardless of fit state. Sized so the car has
        # several scans of braking room at forward_speed.
        self.declare_parameter("emergency_stop_fwd_m", 0.6)
        self.declare_parameter("target_distance", 0.8)
        self.declare_parameter("forward_speed", 0.4)
        # Right-side angular window for the LSQ wall fit. Narrow the range
        # if the line is getting dragged by beams that don't hit the right
        # wall (e.g. a wall dead ahead at angles close to 0°).
        self.declare_parameter("wall_fit_min_deg", -110.0)
        self.declare_parameter("wall_fit_max_deg", -20.0)
        # Minimum inlier beams before we trust a fit. Two points are enough
        # to define a line but give no noise averaging; ~10 is a generous
        # floor for a healthy lidar on this course.
        self.declare_parameter("wall_fit_min_inliers", 10)
        # RMS perpendicular residual (m) above which we reject the fit as
        # noisy — comfortably exceeds sensor noise on a flat wall (~1–3 cm)
        # while catching fits pulled by off-wall returns.
        self.declare_parameter("wall_fit_max_residual", 0.1)
        # If the fit fails, reuse the last accepted (D, α) for up to this
        # long before coasting with bias. At ~8–10 Hz scan rate, 0.5 s
        # covers 4–5 scans of transient dropout.
        self.declare_parameter("hold_last_valid_s", 0.5)
        # Look-ahead (m) for the predictive distance estimate:
        # D_ahead = D + L·sin(α). Gives PD something anticipatory so it
        # starts turning in before the car has reached the corner.
        self.declare_parameter("look_ahead", 0.5)
        # Slow the car down as the wall-angle |α| grows (corners).
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
        # Damping gain on measured yaw rate (rad/s) from imu/gyro.z. Applied
        # in the REP-103 control frame (before the steering_sign flip), so
        # a positive yaw rate (CCW / left) subtracts from `steering` to
        # oppose current rotation.
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

    def _fit_right_wall(self, msg: LaserScan):
        """
        Total least-squares line fit through right-side lidar returns.
        Returns (D_ahead, alpha):
          alpha   — wall angle in car frame (0 = parallel; α<0 = car
                    yawed right, toward wall). Same sign convention as
                    the two-ray F1TENTH estimator this replaced.
          D_ahead — perpendicular distance to the fitted wall, projected
                    `look_ahead` in front of the car.
        Returns (nan, nan) if too few inliers or residual too large.
        """
        ang_min = math.radians(self.get_parameter("wall_fit_min_deg").value)
        ang_max = math.radians(self.get_parameter("wall_fit_max_deg").value)
        max_range = self.get_parameter("max_plausible_distance").value
        min_inliers = self.get_parameter("wall_fit_min_inliers").value
        max_residual = self.get_parameter("wall_fit_max_residual").value
        L = self.get_parameter("look_ahead").value

        xs, ys = [], []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            if r > max_range:
                continue
            angle = self._wrap(msg.angle_min + i * msg.angle_increment)
            if angle < ang_min or angle > ang_max:
                continue
            xs.append(r * math.cos(angle))
            ys.append(r * math.sin(angle))

        n = len(xs)
        if n < min_inliers:
            return float("nan"), float("nan")

        mx = sum(xs) / n
        my = sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        syy = sum((y - my) ** 2 for y in ys)
        sxy = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))

        # Principal axis (line direction) = eigenvector of the larger
        # eigenvalue of the 2x2 scatter matrix [[sxx, sxy], [sxy, syy]].
        tr = sxx + syy
        det = sxx * syy - sxy * sxy
        disc = max(0.0, (tr * tr) / 4.0 - det)
        lam_max = tr / 2.0 + math.sqrt(disc)
        lam_min = tr / 2.0 - math.sqrt(disc)

        if abs(sxy) > 1e-9:
            ex, ey = sxy, lam_max - sxx
        elif sxx >= syy:
            ex, ey = 1.0, 0.0
        else:
            ex, ey = 0.0, 1.0
        norm = math.hypot(ex, ey)
        if norm < 1e-9:
            return float("nan"), float("nan")
        ex /= norm
        ey /= norm
        # Pick the forward-pointing orientation.
        if ex < 0.0:
            ex, ey = -ex, -ey

        # RMS perpendicular residual of the fit. For the scatter matrix
        # (un-normalized sums), variance along the minor axis is lam_min/n.
        residual = math.sqrt(max(0.0, lam_min / n))
        if residual > max_residual:
            return float("nan"), float("nan")

        # Perpendicular distance from origin (car) to the fitted line.
        D_now = abs(mx * ey - my * ex)
        # α<0 when car yawed right (toward right wall).
        alpha = -math.atan2(ey, ex)
        D_ahead = D_now + L * math.sin(alpha)
        return D_ahead, alpha

    def scan_callback(self, msg: LaserScan):
        D_ahead, alpha = self._fit_right_wall(msg)
        fwd = self._forward_distance(msg)

        v_min = self.get_parameter("min_forward_speed").value
        bias = self.get_parameter("steering_bias").value
        sign = self.get_parameter("steering_sign").value

        now = self.get_clock().now().nanoseconds * 1e-9

        # Emergency stop: forward sweep says we're physically close to a
        # wall. Independent safety net — fires regardless of fit state.
        e_stop_fwd = self.get_parameter("emergency_stop_fwd_m").value
        if math.isfinite(fwd) and fwd < e_stop_fwd:
            self.get_logger().warn(
                f"EMERGENCY STOP: fwd={fwd:.2f}m < {e_stop_fwd:.2f}m"
            )
            self.cmd_pub.publish(Twist())
            return

        fit_ok = math.isfinite(D_ahead) and math.isfinite(alpha)
        hold_s = self.get_parameter("hold_last_valid_s").value

        if not fit_ok:
            # On a closed-loop course the wall never truly ends, so a
            # failed fit is noise or transient occlusion. Reuse the last
            # trusted (D, α) for hold_s; beyond that, coast straight and
            # let emergency-stop catch anything dangerous.
            if (
                self._last_valid_time is not None
                and (now - self._last_valid_time) < hold_s
            ):
                D_ahead = self._last_valid_D
                alpha = self._last_valid_alpha
                self.get_logger().info(
                    f"fit failed — holding last valid "
                    f"(age={(now - self._last_valid_time):.2f}s)"
                )
            else:
                cmd = Twist()
                cmd.linear.x = float(v_min)
                cmd.angular.z = float(bias)
                self.cmd_pub.publish(cmd)
                self._prev_error = 0.0
                self._prev_d_error = 0.0
                self._prev_time = now
                self.get_logger().warn(
                    "fit failed and no recent history — coasting straight"
                )
                return
        else:
            self._last_valid_D = D_ahead
            self._last_valid_alpha = alpha
            self._last_valid_time = now

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

        # Positive error => too close to the wall => steer left (+angular.z).
        # Clip before the PD so a single outlier can't send the gains to
        # full lock.
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
        # left" in REP-103 → un-yaws the car while it closes on the target.
        yaw_rate = self._yaw_rate
        steering = kp * error + kd * d_error - k_alpha * alpha - k_yaw * yaw_rate
        # Sign-flip (if the rover is wired inverted) then clamp, then bias.
        steering = sign * steering
        steering = max(-max_steer, min(max_steer, steering)) + bias

        # Ease off the throttle when the wall is swinging away (corner).
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
