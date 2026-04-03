#!/usr/bin/env python3
import atexit
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

from safety_monitor.monitor_state import MonitorState
from safety_monitor.rover_process_manager import RoverProcessManager
from safety_monitor.stale_detector import StaleDetector


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor')

        self.declare_parameter('connection_string', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('watched_topics', [''])
        self.declare_parameter('stale_timeout', 2.0)

        conn_str = self.get_parameter('connection_string').value
        baud_rate = self.get_parameter('baud_rate').value
        raw_topics = self.get_parameter('watched_topics').value
        stale_timeout = self.get_parameter('stale_timeout').value

        watched_topics = [t for t in raw_topics if t]

        self._state = MonitorState.RUNNING
        self._state_lock = threading.Lock()
        self._stale_timeout = stale_timeout

        self._stale_detector = StaleDetector(watched_topics, stale_timeout)
        self._rover = RoverProcessManager(conn_str, baud_rate)

        # Subscribe only to known topic types that appear in watched_topics
        for topic in watched_topics:
            if topic == '/goal_point':
                self.create_subscription(
                    PointStamped, topic, self._make_callback(topic), 10)
            elif topic == '/perception/front_distance':
                self.create_subscription(
                    Float32, topic, self._make_callback(topic), 10)
            else:
                self.get_logger().warn(
                    f'[MONITOR] Unknown topic type for {topic}, skipping staleness watch')

        self.create_timer(0.5, self._check_staleness)
        self.create_timer(1.0, self._check_rover_alive)

        # Set terminal to cbreak mode: single-keypress, no echo, Ctrl+C still works
        self._original_terminal_settings = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        atexit.register(self._restore_terminal)

        self._rover.spawn()
        self.get_logger().info(
            '[MONITOR] Safety monitor started. Press SPACE to e-stop.')

        self._keyboard_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()

    # ------------------------------------------------------------------
    # Subscription callback factory
    # ------------------------------------------------------------------

    def _make_callback(self, topic: str):
        def callback(msg):
            self._stale_detector.record_message(topic)
        return callback

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------

    def _trigger_estop(self, reason: str) -> None:
        with self._state_lock:
            if self._state == MonitorState.ESTOP:
                return
            self._state = MonitorState.ESTOP
        self._rover.kill()
        print(f'\n[E-STOP] {reason}')
        print('[E-STOP] Press Enter to restart robo_rover...')

    def _trigger_recovery(self) -> None:
        with self._state_lock:
            if self._state != MonitorState.ESTOP:
                return
            self._state = MonitorState.RUNNING
        print('[MONITOR] Restarting robo_rover...')
        self._rover.spawn()
        print('[MONITOR] robo_rover restarted.')

    # ------------------------------------------------------------------
    # Timer callbacks
    # ------------------------------------------------------------------

    def _check_staleness(self) -> None:
        with self._state_lock:
            if self._state == MonitorState.ESTOP:
                return
        stale_topic = self._stale_detector.check_stale()
        if stale_topic:
            self._trigger_estop(
                f'{stale_topic} went stale ({self._stale_timeout}s)')

    def _check_rover_alive(self) -> None:
        with self._state_lock:
            if self._state == MonitorState.ESTOP:
                return
        if not self._rover.is_alive():
            self._trigger_estop('robo_rover process died unexpectedly')

    # ------------------------------------------------------------------
    # Keyboard thread
    # ------------------------------------------------------------------

    def _keyboard_loop(self) -> None:
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            ch = sys.stdin.read(1)
            if ch == ' ':
                with self._state_lock:
                    s = self._state
                if s == MonitorState.RUNNING:
                    self._trigger_estop('Manual stop activated')
            elif ch in ('\r', '\n'):
                with self._state_lock:
                    s = self._state
                if s == MonitorState.ESTOP:
                    self._trigger_recovery()

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def _restore_terminal(self) -> None:
        try:
            termios.tcsetattr(
                sys.stdin.fileno(), termios.TCSADRAIN,
                self._original_terminal_settings)
        except Exception:
            pass

    def destroy_node(self) -> None:
        self._rover.kill()
        self._restore_terminal()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
