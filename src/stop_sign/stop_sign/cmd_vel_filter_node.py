#!/usr/bin/env python3
"""
/cmd_vel filter that adds stop-sign obedience to *any* controller.

Sits between any controller publishing `cmd_vel_raw` and `rover_node`
subscribing to `cmd_vel`:

    controller ──► /cmd_vel_raw ──► [cmd_vel_stop_filter] ──► /cmd_vel ──► rover_node
                                          ▲
                       /stop_sign/event ──┘

On STATE_APPROACH it runs the DRIVING / BRAKING / STOPPED / RESUMING state
machine, modulating only `linear.x`. Steering (`angular.z`) passes through
untouched in every state, so any controller's lateral logic stays live
through the brake / pause / resume sequence.

Runtime toggle:
    ros2 param set /cmd_vel_stop_filter obey_stops false

When `obey_stops=false` the filter is a pure pass-through — every Twist
field is forwarded byte-for-byte, no inspection of `/stop_sign/event`,
identical behavior to having no filter in the graph at all.
"""
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from racer_msgs.msg import StopSignEvent


class StopState(Enum):
    DRIVING = 0
    BRAKING = 1
    STOPPED = 2
    RESUMING = 3


class CmdVelStopFilter(Node):
    def __init__(self):
        super().__init__("cmd_vel_stop_filter")

        self.obey_stops = bool(self.declare_parameter(
            "obey_stops", True
        ).get_parameter_value().bool_value)
        self.brake_duration_sec = float(self.declare_parameter(
            "brake_duration_sec", 1.0
        ).get_parameter_value().double_value)
        self.pause_duration_sec = float(self.declare_parameter(
            "pause_duration_sec", 1.0
        ).get_parameter_value().double_value)
        self.resume_duration_sec = float(self.declare_parameter(
            "resume_duration_sec", 0.5
        ).get_parameter_value().double_value)

        self.latest_stop_event = None
        self.stop_state = StopState.DRIVING
        self.brake_start_time = None
        self.stop_start_time = None
        self.resume_start_time = None
        self.speed_at_brake_start = 0.0

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_sub = self.create_subscription(
            Twist, "cmd_vel_raw", self._cmd_callback, 10
        )
        self.stop_sub = self.create_subscription(
            StopSignEvent, "/stop_sign/event", self._stop_event_callback, 10
        )

        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f"cmd_vel_stop_filter started (obey_stops={self.obey_stops}, "
            f"brake={self.brake_duration_sec}s pause={self.pause_duration_sec}s "
            f"resume={self.resume_duration_sec}s)"
        )

    def _stop_event_callback(self, msg):
        self.latest_stop_event = msg

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "obey_stops":
                self.obey_stops = bool(p.value)
                self.get_logger().info(f"obey_stops set to {self.obey_stops}")
            elif p.name in ("brake_duration_sec", "pause_duration_sec", "resume_duration_sec"):
                setattr(self, p.name, float(p.value))
        return SetParametersResult(successful=True)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _stop_event_is_approach(self) -> bool:
        return (self.latest_stop_event is not None and
                self.latest_stop_event.state == StopSignEvent.STATE_APPROACH)

    def _reset_timers(self):
        self.brake_start_time = None
        self.stop_start_time = None
        self.resume_start_time = None

    def _cmd_callback(self, raw: Twist):
        # OFF: pass-through — Twist forwarded byte-for-byte, no state machine.
        if not self.obey_stops:
            self.stop_state = StopState.DRIVING
            self._reset_timers()
            self.cmd_pub.publish(raw)
            return

        cmd = Twist()
        cmd.linear.x = raw.linear.x
        cmd.linear.y = raw.linear.y
        cmd.linear.z = raw.linear.z
        cmd.angular.x = raw.angular.x
        cmd.angular.y = raw.angular.y
        cmd.angular.z = raw.angular.z

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
                self._reset_timers()
                self.get_logger().info("RESUMING complete — back to DRIVING")

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelStopFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down cmd_vel_stop_filter...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
