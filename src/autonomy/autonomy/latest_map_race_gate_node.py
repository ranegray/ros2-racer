#!/usr/bin/env python3
"""
Gate lap-2 racing on an already-saved latest map.

This node is intentionally small: it does not drive the robot and it does not
save anything. It waits until the latest map files, recorded path, and
localization TF are available, publishes "ready", then starts racing
automatically unless auto_start is disabled.
"""

import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
import tf2_ros
import yaml


CONFIRM_CMD = (
    "ros2 topic pub --once /slam_coordinator/confirm std_msgs/msg/Empty '{}'"
)


class LatestMapRaceGateNode(Node):
    def __init__(self):
        super().__init__("latest_map_race_gate_node")

        self.declare_parameter("map_base", "/home/pi/map/track")
        self.declare_parameter(
            "path_file",
            os.path.expanduser("~/.ros/recorded_path.yaml"),
        )
        self.declare_parameter("require_path", True)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("auto_start_delay_sec", 2.0)

        self._map_base = self.get_parameter("map_base").value
        self._path_file = self.get_parameter("path_file").value
        self._require_path = self.get_parameter("require_path").value
        self._auto_start = self.get_parameter("auto_start").value
        self._auto_start_delay_sec = float(
            self.get_parameter("auto_start_delay_sec").value
        )

        self._mode = "localizing"
        self._last_status = ""
        self._ready_since: float | None = None

        self._mode_pub = self.create_publisher(String, "/slam_coordinator/mode", 10)
        self._status_pub = self.create_publisher(
            String, "/telemetry/internal_state", 10
        )
        self.create_subscription(
            Empty, "/slam_coordinator/confirm", self._confirm_cb, 10
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f"Latest-map race gate started - map_base={self._map_base}, "
            f"path_file={self._path_file}, auto_start={self._auto_start}"
        )

    def _confirm_cb(self, _msg: Empty):
        if self._mode != "ready":
            self.get_logger().warn(
                f"Confirm ignored: gate is {self._mode}, not READY yet"
            )
            return

        self._start_racing("operator confirm")

    def _tick(self):
        if self._mode == "racing":
            self._publish_mode()
            return

        ok, status = self._prerequisites_ready()
        if ok:
            if self._mode != "ready":
                self._mode = "ready"
                self._ready_since = time.monotonic()
                self.get_logger().info(
                    "Latest map is localized and ready."
                )
                if self._auto_start:
                    self.get_logger().info(
                        f"Auto-starting racing in {self._auto_start_delay_sec:.1f}s"
                    )
                else:
                    self.get_logger().info(f"Confirm with:\n  {CONFIRM_CMD}")

            self._publish_mode()
            if self._auto_start:
                self._publish_status("READY TO RACE - auto-start pending")
                if (
                    self._ready_since is not None
                    and time.monotonic() - self._ready_since >= self._auto_start_delay_sec
                ):
                    self._start_racing("auto-start")
            else:
                self._publish_status(f"READY TO RACE - confirm: {CONFIRM_CMD}")
            return

        self._mode = "localizing"
        self._ready_since = None
        self._publish_mode()
        self._publish_status(status)
        if status != self._last_status:
            self.get_logger().info(status)
            self._last_status = status

    def _prerequisites_ready(self) -> tuple[bool, str]:
        map_ok, map_status = self._map_files_ready()
        if not map_ok:
            return False, map_status

        if self._require_path and not os.path.exists(self._path_file):
            return False, f"WAITING: path file missing: {self._path_file}"

        try:
            self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return False, "WAITING: localization TF map -> base_link is not ready"

        return True, "READY"

    def _map_files_ready(self) -> tuple[bool, str]:
        yaml_path = f"{self._map_base}.yaml"
        if not os.path.exists(yaml_path):
            return False, f"WAITING: map yaml missing: {yaml_path}"

        image_path = self._image_path_from_yaml(yaml_path)
        if not os.path.exists(image_path):
            return False, f"WAITING: map image missing: {image_path}"

        return True, "map ready"

    def _image_path_from_yaml(self, yaml_path: str) -> str:
        try:
            with open(yaml_path) as f:
                data = yaml.safe_load(f) or {}
            image = data.get("image")
        except Exception:
            image = None

        if not image:
            return f"{self._map_base}.pgm"
        if os.path.isabs(image):
            return image
        return os.path.join(os.path.dirname(yaml_path), image)

    def _publish_mode(self):
        msg = String()
        msg.data = self._mode
        self._mode_pub.publish(msg)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _start_racing(self, reason: str):
        self._mode = "racing"
        self._publish_mode()
        self._publish_status("RACING")
        self.get_logger().info(f"*** MODE SWITCH: RACING ({reason}) ***")


def main(args=None):
    rclpy.init(args=args)
    node = LatestMapRaceGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
