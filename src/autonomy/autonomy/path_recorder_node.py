#!/usr/bin/env python3
"""
path_recorder_node.py

Subscribes to the TF map → base_link and records the robot's pose at ~10 Hz
during Lap 1.  On shutdown (Ctrl-C or mode switch), saves the path to a YAML
file so pure_pursuit_node can replay it in Lap 2.

Default output: ~/.ros/recorded_path.yaml
Override with ROS parameter: output_path
"""

import os
import yaml
import rclpy
from rclpy.node import Node
import tf2_ros


RECORD_HZ = 10.0
MIN_DIST   = 0.05  # m — only store a new pose if we moved this far


class PathRecorderNode(Node):
    def __init__(self):
        super().__init__("path_recorder_node")

        self.declare_parameter(
            "output_path",
            os.path.expanduser("~/.ros/recorded_path.yaml"),
        )
        self._output_path = self.get_parameter("output_path").value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._poses: list[dict] = []
        self._last_x = None
        self._last_y = None

        self.create_timer(1.0 / RECORD_HZ, self._record_tick)
        self.get_logger().info(
            f"Path recorder started — will save to {self._output_path}"
        )

    def _record_tick(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        # Skip if robot hasn't moved enough
        if self._last_x is not None:
            dist = ((x - self._last_x) ** 2 + (y - self._last_y) ** 2) ** 0.5
            if dist < MIN_DIST:
                return

        self._last_x = x
        self._last_y = y
        self._poses.append({"x": float(x), "y": float(y), "qz": float(qz), "qw": float(qw)})

        if len(self._poses) % 50 == 0:
            self.get_logger().info(f"Recorded {len(self._poses)} poses so far")

    def _save(self):
        if not self._poses:
            self.get_logger().warn("No poses recorded — nothing to save")
            return
        os.makedirs(os.path.dirname(self._output_path), exist_ok=True)
        with open(self._output_path, "w") as f:
            yaml.dump({"poses": self._poses}, f)
        self.get_logger().info(
            f"Saved {len(self._poses)} poses to {self._output_path}"
        )

    def destroy_node(self):
        self._save()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
