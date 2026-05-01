#!/usr/bin/env python3
"""
Line-follower controller with stop-sign obedience.

The existing line-follow planner (compute steering from /goal_point, drive at
fixed forward speed) lives in DRIVING state. A stop state machine wraps it:

  DRIVING ──(stop_sign APPROACH and obey_stops)─► BRAKING
  BRAKING ──(brake_timer expired)─────────────────► STOPPED
  STOPPED ──(pause_timer expired)─────────────────► RESUMING
  RESUMING ──(resume_timer expired)───────────────► DRIVING

Steering is recomputed from the latest goal_point in *every* state — only
linear.x is modulated by the stop machine. obey_stops can be toggled at
runtime via `ros2 param set /green_control_node obey_stops <bool>`.
"""
import os
from enum import Enum

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node

from racer_msgs.msg import StopSignEvent


class StopState(Enum):
    DRIVING = 0
    BRAKING = 1
    STOPPED = 2
    RESUMING = 3


class GreenControlNode(Node):
    def __init__(self):
        super().__init__('green_control_node')

        self._declare_and_load_params()

        self.latest_goal = None
        self.latest_stop_event = None

        self.stop_state = StopState.DRIVING
        self.brake_start_time = None
        self.stop_start_time = None
        self.resume_start_time = None
        self.speed_at_brake_start = self.drive_speed
        self.speed_at_resume_start = 0.0

        self.subscriber = self.create_subscription(
            PointStamped, "goal_point", self.goal_callback, 10
        )
        self.stop_sub = self.create_subscription(
            StopSignEvent, "/stop_sign/event", self.stop_event_callback, 10
        )
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.add_on_set_parameters_callback(self._on_param_change)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"Green Control Node started (obey_stops={self.obey_stops}, "
            f"brake={self.brake_duration_sec}s pause={self.pause_duration_sec}s "
            f"resume={self.resume_duration_sec}s)"
        )

    def _declare_and_load_params(self):
        # Optional YAML config in green_control's share dir.
        default_config = os.path.join(
            get_package_share_directory('green_control'),
            'config',
            'green_control.yaml',
        )
        config_path = self.declare_parameter(
            'config_path', default_config
        ).get_parameter_value().string_value

        cfg = {}
        if os.path.isfile(config_path):
            with open(config_path, 'r') as f:
                raw = yaml.safe_load(f) or {}
            cfg = (raw.get('control') or raw.get('green_control', {}).get('control')
                   or raw.get('green_control') or raw)
        else:
            self.get_logger().info(
                f"No control config at '{config_path}'; using built-in defaults."
            )

        # Planner params
        self.steering_kp = float(cfg.get('steering_kp', 5.0))
        self.drive_speed = float(cfg.get('drive_speed', 0.5))
        self.stop_distance = float(cfg.get('stop_distance', 0.25))
        self.goal_timeout = float(cfg.get('goal_timeout', 1.0))

        # Stop state machine params (declared with defaults from cfg).
        self.obey_stops = bool(self.declare_parameter(
            'obey_stops', bool(cfg.get('obey_stops', True))
        ).get_parameter_value().bool_value)
        self.brake_duration_sec = float(cfg.get('brake_duration_sec', 1.0))
        self.pause_duration_sec = float(cfg.get('pause_duration_sec', 1.0))
        self.resume_duration_sec = float(cfg.get('resume_duration_sec', 0.5))
        # TODO: use this once odometry feedback is available — see Step 8 spec note.
        self.stop_speed_threshold = float(cfg.get('stop_speed_threshold', 0.05))

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'obey_stops':
                self.obey_stops = bool(p.value)
                self.get_logger().info(f"obey_stops set to {self.obey_stops}")
        return SetParametersResult(successful=True)

    def goal_callback(self, msg):
        self.latest_goal = msg

    def stop_event_callback(self, msg):
        self.latest_stop_event = msg

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _compute_planner(self):
        """
        Existing line-follow planner, unmodified. Returns (linear_x, angular_z, ok).
        ok=False means the rover should idle (no goal / stale goal / target lost).
        """
        if self.latest_goal is None:
            return 0.0, 0.0, False

        now = self.get_clock().now()
        goal_time = rclpy.time.Time.from_msg(self.latest_goal.header.stamp)
        age = (now - goal_time).nanoseconds / 1e9
        if age > self.goal_timeout:
            self.get_logger().info("Goal point stale, stopping.")
            return 0.0, 0.0, False

        x = self.latest_goal.point.x
        z = self.latest_goal.point.z

        if z == 0.0:
            self.get_logger().info("No target detected, stopping.")
            return 0.0, 0.0, False

        if z < self.stop_distance:
            self.get_logger().info(f"Target close ({z:.2f}m), stopping.")
            return 0.0, self._clamp(self.steering_kp * x, -1.0, 1.0), True

        steer = self.steering_kp * x
        return self.drive_speed, self._clamp(steer, -1.0, 1.0), True

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _reset_stop_timers(self):
        self.brake_start_time = None
        self.stop_start_time = None
        self.resume_start_time = None

    def _stop_event_is_approach(self) -> bool:
        return (self.latest_stop_event is not None and
                self.latest_stop_event.state == StopSignEvent.STATE_APPROACH)

    def control_loop(self):
        cmd = Twist()
        planner_lin, planner_ang, planner_ok = self._compute_planner()

        # Steering stays hot in every state — only linear.x is modulated below.
        cmd.angular.z = planner_ang

        if not self.obey_stops:
            self.stop_state = StopState.DRIVING
            self._reset_stop_timers()
            cmd.linear.x = planner_lin if planner_ok else 0.0
            self.publisher_.publish(cmd)
            return

        now = self._now()

        if self.stop_state == StopState.DRIVING:
            if self._stop_event_is_approach():
                self.stop_state = StopState.BRAKING
                self.brake_start_time = now
                self.speed_at_brake_start = planner_lin if planner_ok else self.drive_speed
                self.get_logger().info(
                    f"Stop sign APPROACH — entering BRAKING "
                    f"(speed_at_entry={self.speed_at_brake_start:.2f})"
                )
                cmd.linear.x = self.speed_at_brake_start
            else:
                cmd.linear.x = planner_lin if planner_ok else 0.0

        elif self.stop_state == StopState.BRAKING:
            t = (now - self.brake_start_time) / max(self.brake_duration_sec, 1e-6)
            t = self._clamp(t, 0.0, 1.0)
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
                self.speed_at_resume_start = planner_lin if planner_ok else self.drive_speed
                self.get_logger().info(
                    f"Pause complete — entering RESUMING (target={self.speed_at_resume_start:.2f})"
                )

        elif self.stop_state == StopState.RESUMING:
            t = (now - self.resume_start_time) / max(self.resume_duration_sec, 1e-6)
            t = self._clamp(t, 0.0, 1.0)
            target = planner_lin if planner_ok else 0.0
            cmd.linear.x = t * target
            if t >= 1.0:
                self.stop_state = StopState.DRIVING
                self._reset_stop_timers()
                cmd.linear.x = target
                self.get_logger().info("RESUMING complete — back to DRIVING")

        self.publisher_.publish(cmd)
        self.get_logger().debug(
            f"state={self.stop_state.name} lin={cmd.linear.x:.2f} ang={cmd.angular.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GreenControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Green Control Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
