#!/usr/bin/env python3
"""
Line follower controller, with optional stop-sign obedience.

The stop state machine (DRIVING / BRAKING / STOPPED / RESUMING) wraps the
existing line-follow planner. Steering is recomputed from the latest
follow/turn/lookahead point in *every* state — only linear.x is modulated,
so steering stays hot through the entire stop sequence. Toggleable at
runtime via the obey_stops parameter:

    ros2 param set /line_control_node obey_stops false
"""
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped

from racer_msgs.msg import StopSignEvent


class StopState(Enum):
    DRIVING = 0
    BRAKING = 1
    STOPPED = 2
    RESUMING = 3


class LineControlNode(Node):
    def __init__(self):
        super().__init__("line_control_node")
        self.latest_follow = None
        self.latest_turn = None
        self.latest_lookahead = None
        self.latest_stop_event = None

        self.follow_sub = self.create_subscription(
            PointStamped, "line_follow_point", self._follow_callback, 10
        )
        self.turn_sub = self.create_subscription(
            PointStamped, "line_turn_point", self._turn_callback, 10
        )
        self.lookahead_sub = self.create_subscription(
            PointStamped, "line_lookahead_point", self._lookahead_callback, 10
        )
        self.stop_sub = self.create_subscription(
            StopSignEvent, "/stop_sign/event", self._stop_event_callback, 10
        )
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.image_width = 640
        self.image_center_x = self.image_width / 2.0
        self.target_x = 400.0  # pixel column where a centered line appears (camera is mounted off-center)
        self.steering_kp = 0.8  # proportional gain on follow-point offset
        self.steering_kd = 1.5  # derivative gain on follow-point offset
        self.turn_steering_kp = 3.5  # proportional gain on turn-point offset (override)
        self.base_speed = 0.40  # forward speed when following (m/s)
        self.turn_speed = 0.35  # forward speed when turning (m/s)
        self.speed_scale = 1.0         # how much steering reduces speed (0=no reduction, 1=full stop at max steer)
        self.curvature_speed_scale = 1.5  # how much bend rate (d_offset) reduces speed; tune up to slow earlier
        self.min_speed = 0.35
        self.steering_trim = 0.12  # positive = right bias; tune to counteract physical left-pull of wheels
        self.goal_timeout = 1.0  # stop if no goal received for this long (s)

        self.prev_offset = 0.0      # previous smoothed offset for D term
        self.smoothed_offset = 0.0  # EMA-filtered offset; kills centroid jitter before derivative
        self.offset_alpha = 0.5     # EMA weight on new sample (lower = smoother, more lag)
        self.dt = 0.1  # control loop period (s)
        self.last_known_offset = (
            0.0  # sign of last seen offset; drives recovery steer when line is lost
        )

        # Recovery when the line is lost:
        #   Phase 1 — hard steer in the last known direction for lost_steer_duration
        #   Phase 2 — reverse for up to reverse_duration_cap, then cooldown/stop
        self.lost_steer_duration = 2.0   # s to steer hard before reversing
        self.reverse_speed = 0.22        # m/s (published as negative linear.x)
        self.reverse_duration_cap = 1.0  # max time to spend reversing before giving up
        self.reverse_cooldown = 0.5      # s to hold still after reversing
        self.last_line_seen_sec = None
        self.reverse_start_sec = None
        self.cooldown_end_sec = None

        # Stop sign obedience — state machine wraps the planner output below.
        self.obey_stops = bool(self.declare_parameter(
            "obey_stops", True
        ).get_parameter_value().bool_value)
        self.brake_duration_sec = 1.0
        self.pause_duration_sec = 1.0
        self.resume_duration_sec = 0.5
        self.stop_state = StopState.DRIVING
        self.brake_start_time = None
        self.stop_start_time = None
        self.resume_start_time = None
        self.speed_at_brake_start = 0.0
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info("Line Control Node has started!")

    def _follow_callback(self, msg):
        self.latest_follow = msg

    def _turn_callback(self, msg):
        self.latest_turn = msg

    def _lookahead_callback(self, msg):
        self.latest_lookahead = msg

    def _stop_event_callback(self, msg):
        self.latest_stop_event = msg

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "obey_stops":
                self.obey_stops = bool(p.value)
                self.get_logger().info(f"obey_stops set to {self.obey_stops}")
        return SetParametersResult(successful=True)

    def _stop_event_is_approach(self) -> bool:
        return (self.latest_stop_event is not None and
                self.latest_stop_event.state == StopSignEvent.STATE_APPROACH)

    def _reset_stop_timers(self):
        self.brake_start_time = None
        self.stop_start_time = None
        self.resume_start_time = None

    def _publish_with_stop_machine(self, cmd: Twist):
        """Modulate cmd.linear.x by the stop state machine, then publish.

        Steering (cmd.angular.z) is left untouched — it stays live in every
        state so the rover keeps tracking the line through the brake / pause
        / resume sequence.
        """
        if not self.obey_stops:
            self.stop_state = StopState.DRIVING
            self._reset_stop_timers()
            self.publisher_.publish(cmd)
            return

        now = self._now_sec()

        if self.stop_state == StopState.DRIVING:
            if self._stop_event_is_approach():
                self.stop_state = StopState.BRAKING
                self.brake_start_time = now
                self.speed_at_brake_start = cmd.linear.x
                self.get_logger().info(
                    f"Stop sign APPROACH — entering BRAKING "
                    f"(speed_at_entry={self.speed_at_brake_start:.2f})"
                )

        elif self.stop_state == StopState.BRAKING:
            t = (now - self.brake_start_time) / max(self.brake_duration_sec, 1e-6)
            t = max(0.0, min(1.0, t))
            cmd.linear.x = (1.0 - t) * self.speed_at_brake_start
            if t >= 1.0:
                self.stop_state = StopState.STOPPED
                self.stop_start_time = now
                cmd.linear.x = 0.0
                self.get_logger().info("BRAKING complete — entering STOPPED")

        elif self.stop_state == StopState.STOPPED:
            cmd.linear.x = 0.0
            if (now - self.stop_start_time) >= self.pause_duration_sec:
                self.stop_state = StopState.RESUMING
                self.resume_start_time = now
                self.get_logger().info("Pause complete — entering RESUMING")

        elif self.stop_state == StopState.RESUMING:
            t = (now - self.resume_start_time) / max(self.resume_duration_sec, 1e-6)
            t = max(0.0, min(1.0, t))
            cmd.linear.x = t * cmd.linear.x
            if t >= 1.0:
                self.stop_state = StopState.DRIVING
                self._reset_stop_timers()
                self.get_logger().info("RESUMING complete — back to DRIVING")

        self.publisher_.publish(cmd)

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
        max_angular = 2.0
        d_offset = 0.0

        follow = self._extract(self.latest_follow)
        turn = self._extract(self.latest_turn)
        lookahead = self._extract(self.latest_lookahead)

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
            self.smoothed_offset = self.offset_alpha * offset + (1.0 - self.offset_alpha) * self.smoothed_offset
            d_offset = (self.smoothed_offset - self.prev_offset) / self.dt
            self.prev_offset = self.smoothed_offset
            self.last_known_offset = offset
            steer = self.steering_kp * self.smoothed_offset + self.steering_kd * d_offset
            speed = self.base_speed
        else:
            # Recovery sequence:
            #   Phase 1: hard steer toward last known direction for lost_steer_duration
            #   Phase 2: reverse for up to reverse_duration_cap, then cooldown/stop
            seen_ever = self.last_line_seen_sec is not None

            # Already in reverse phase — keep going, steering opposite to steer phase.
            if self.reverse_start_sec is not None:
                if now - self.reverse_start_sec >= self.reverse_duration_cap:
                    self.reverse_start_sec = None
                    self.cooldown_end_sec = now + self.reverse_cooldown
                    self._publish_with_stop_machine(cmd)
                    return
                cmd.linear.x = -self.reverse_speed
                cmd.angular.z = -max_angular if self.last_known_offset >= 0 else max_angular
                self._publish_with_stop_machine(cmd)
                return

            # Cooldown after reverse — hold still.
            if self.cooldown_end_sec is not None and now < self.cooldown_end_sec:
                self._publish_with_stop_machine(cmd)
                return

            if not seen_ever:
                self._publish_with_stop_machine(cmd)
                return

            lost_for = now - self.last_line_seen_sec

            if lost_for < self.lost_steer_duration:
                # Phase 1: hard steer in the last known direction.
                cmd.angular.z = max_angular if self.last_known_offset >= 0 else -max_angular
                cmd.linear.x = self.min_speed
            else:
                # Phase 2: start reversing.
                self.reverse_start_sec = now
                cmd.linear.x = -self.reverse_speed

            self._publish_with_stop_machine(cmd)
            return

        # rover_node accepts angular.z in [-2.0, 2.0]; clamping tighter leaves steering on the table.
        # tanh soft saturation: linear near center, smoothly limits at large offsets
        # instead of a hard clamp that causes full-lock overshoot.
        cmd.angular.z = max_angular * math.tanh((steer + self.steering_trim) / max_angular)
        # Normalize steering magnitude to [0,1] for the speed scaler so tuning semantics stay the same.
        steer_frac = abs(cmd.angular.z) / max_angular
        # Slow down based on geometric curvature: how far the lookahead x is from the follow x.
        # This fires early — the lookahead sees the turn coming before the robot is in it.
        if lookahead is not None and follow is not None:
            curvature_frac = min(1.0, abs(lookahead[0] - follow[0]) / self.image_center_x * self.curvature_speed_scale)
        else:
            curvature_frac = min(1.0, abs(d_offset) * self.curvature_speed_scale)
        speed_reduction = max(steer_frac, curvature_frac)
        cmd.linear.x = max(
            self.min_speed, speed * (1.0 - speed_reduction * self.speed_scale)
        )
        self._publish_with_stop_machine(cmd)
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
