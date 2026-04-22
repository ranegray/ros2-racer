#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped


class LineControlNode(Node):
    def __init__(self):
        super().__init__("line_control_node")
        self.latest_follow = None
        self.latest_turn = None

        self.follow_sub = self.create_subscription(
            PointStamped, "line_follow_point", self._follow_callback, 10
        )
        self.turn_sub = self.create_subscription(
            PointStamped, "line_turn_point", self._turn_callback, 10
        )
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.image_width = 640
        self.image_center_x = self.image_width / 2.0
        self.target_x = 400.0  # pixel column where a centered line appears (camera is mounted off-center)
        self.steering_kp = 1.2  # proportional gain on follow-point offset
        self.steering_kd = 4.0  # derivative gain on follow-point offset
        self.turn_steering_kp = 3.5  # proportional gain on turn-point offset (override)
        self.base_speed = 0.55  # forward speed when following (m/s)
        self.turn_speed = 0.55  # forward speed when turning (m/s)
        self.speed_scale = 1.1  # how much steering reduces speed (0=no reduction, 1=full stop at max steer)
        self.min_speed = (
            0.30  # floor — motors must always get at least this much throttle
        )
        self.steering_trim = 0.1  # positive = right bias; tune to counteract physical left-pull of wheels
        self.goal_timeout = 1.0  # stop if no goal received for this long (s)

        self.prev_offset = 0.0  # previous normalized offset for D term
        self.dt = 0.1  # control loop period (s)
        self.last_known_offset = (
            0.0  # sign of last seen offset; drives recovery steer when line is lost
        )

        # Reverse recovery when the line is lost
        self.reverse_speed = 0.22  # m/s (published as negative linear.x)
        self.reverse_debounce = 0.3  # s line must be missing before reversing
        self.reverse_duration_cap = 1.0  # max time to spend reversing before giving up
        self.reverse_cooldown = (
            0.5  # s to hold still after reversing before another attempt
        )
        self.last_line_seen_sec = None
        self.reverse_start_sec = None
        self.cooldown_end_sec = None

        self.get_logger().info("Line Control Node has started!")

    def _follow_callback(self, msg):
        self.latest_follow = msg

    def _turn_callback(self, msg):
        self.latest_turn = msg

    def _extract(self, msg):
        """Return (x, y) in pixels if msg is fresh and non-empty, else None."""
        if msg is None:
            return None
        now = self.get_clock().now()
        age = (now - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        if age > self.goal_timeout:
            return None
        if msg.point.x == 0.0 and msg.point.y == 0.0:
            return None
        return (msg.point.x, msg.point.y)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def control_loop(self):
        cmd = Twist()
        now = self._now_sec()

        follow = self._extract(self.latest_follow)
        turn = self._extract(self.latest_turn)

        if follow is not None or turn is not None:
            self.last_line_seen_sec = now
            # Re-acquired while reversing — bail out of reverse and resume normal control.
            if self.reverse_start_sec is not None:
                self.reverse_start_sec = None
                self.cooldown_end_sec = None

        if turn is not None:
            # Right-turn override: detector saw a chunk of tape far to the right.
            # Commit to the turn and steer toward it.
            offset = (turn[0] - self.target_x) / self.image_center_x
            steer = self.turn_steering_kp * offset
            speed = self.turn_speed
            self.prev_offset = (
                offset  # keep prev_offset current to avoid D spike on return to follow
            )
            self.last_known_offset = offset
        elif follow is not None:
            offset = (follow[0] - self.target_x) / self.image_center_x
            d_offset = (offset - self.prev_offset) / self.dt
            self.prev_offset = offset
            self.last_known_offset = offset
            steer = self.steering_kp * offset + self.steering_kd * d_offset
            speed = self.base_speed
        else:
            # Line lost. If we've been reversing, keep reversing until the cap, then cool down.
            if self.reverse_start_sec is not None:
                if now - self.reverse_start_sec >= self.reverse_duration_cap:
                    self.reverse_start_sec = None
                    self.cooldown_end_sec = now + self.reverse_cooldown
                    self.publisher_.publish(cmd)
                    return
                cmd.linear.x = -self.reverse_speed
                self.publisher_.publish(cmd)
                return

            in_cooldown = (
                self.cooldown_end_sec is not None and now < self.cooldown_end_sec
            )
            seen_ever = self.last_line_seen_sec is not None
            debounce_passed = (
                seen_ever and (now - self.last_line_seen_sec) >= self.reverse_debounce
            )
            # Only reverse if we've actually seen a line at some point — don't back up at startup.
            if seen_ever and debounce_passed and not in_cooldown:
                self.reverse_start_sec = now
                cmd.linear.x = -self.reverse_speed
                self.publisher_.publish(cmd)
                return

            self.publisher_.publish(cmd)
            return

        # rover_node accepts angular.z in [-2.0, 2.0]; clamping tighter leaves steering on the table.
        max_angular = 2.0
        cmd.angular.z = max(-max_angular, min(max_angular, steer + self.steering_trim))
        # Normalize steering magnitude to [0,1] for the speed scaler so tuning semantics stay the same.
        steer_frac = abs(cmd.angular.z) / max_angular
        cmd.linear.x = max(
            self.min_speed, speed * (1.0 - steer_frac * self.speed_scale)
        )
        self.publisher_.publish(cmd)
        self.get_logger().debug(
            f"speed={cmd.linear.x:.2f} steer={cmd.angular.z:.2f} "
            f"follow={follow} turn={turn}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LineControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Line Control Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
