#!/usr/bin/env python3
"""
path_recorder_node.py

Subscribes to the TF map → base_link and records the robot's pose at ~10 Hz
during Lap 1.  On shutdown (Ctrl-C or mode switch), saves the path to a YAML
file so pure_pursuit_node can replay it in Lap 2.

Default output: ~/.ros/recorded_path.yaml
Override with ROS parameter: output_path
"""

import math
import os
from datetime import datetime
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros


RECORD_HZ = 10.0
MIN_DIST   = 0.15  # m — only store a new pose if we moved this far


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

        self._saved = False
        self._path_saved_pub = self.create_publisher(
            String, "/slam_coordinator/path_saved", 10
        )
        self._path_viz_pub = self.create_publisher(Path, "/recorded_path", 10)
        # Coordinator triggers save explicitly; don't rely on racing mode message
        self.create_subscription(
            String, "/slam_coordinator/save_path", self._save_path_cb, 10
        )
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

        # Publish live path for dashboard visualisation
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for p in self._poses:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = p["x"]
            ps.pose.position.y = p["y"]
            ps.pose.orientation.z = p["qz"]
            ps.pose.orientation.w = p["qw"]
            path_msg.poses.append(ps)
        self._path_viz_pub.publish(path_msg)

        if len(self._poses) % 50 == 0:
            self.get_logger().info(f"Recorded {len(self._poses)} poses so far")

    def _save_path_cb(self, msg: String):
        if self._saved:
            return
        timestamp: str | None = None
        if msg.data.startswith("save:"):
            timestamp = msg.data.split(":", 1)[1] or None
        self._save(timestamp=timestamp)
        pub_msg = String()
        pub_msg.data = "saved"
        self._path_saved_pub.publish(pub_msg)

    def _close_loop(self):
        """Interpolate waypoints between the last and first pose to close the path."""
        if len(self._poses) < 2:
            return
        last  = self._poses[-1]
        first = self._poses[0]
        gap = math.hypot(first['x'] - last['x'], first['y'] - last['y'])
        if gap < MIN_DIST:
            return
        n_steps = max(1, int(gap / MIN_DIST))
        closing = []
        for i in range(1, n_steps):
            t = i / n_steps
            closing.append({
                'x':  float(last['x']  + t * (first['x']  - last['x'])),
                'y':  float(last['y']  + t * (first['y']  - last['y'])),
                'qz': float(last['qz']),
                'qw': float(last['qw']),
            })
        self._poses.extend(closing)
        self.get_logger().info(
            f"Loop closure: added {len(closing)} waypoints to bridge {gap:.2f}m gap"
        )

    def _save(self, timestamp: str | None = None):
        if not self._poses:
            self.get_logger().warn("No poses recorded — nothing to save")
            return
        self._close_loop()

        if timestamp is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        latest_path = self._output_path
        base_dir = os.path.dirname(latest_path)
        stem, ext = os.path.splitext(os.path.basename(latest_path))
        archive_name = f"{stem}_{timestamp}{ext}"
        archive_path = os.path.join(base_dir, archive_name)

        os.makedirs(base_dir, exist_ok=True)
        with open(archive_path, "w") as f:
            yaml.dump({"poses": self._poses}, f)

        # Update the stable symlink (recorded_path.yaml → recorded_path_<ts>.yaml).
        # Remove first so writing through an old symlink can't clobber an archive.
        try:
            if os.path.islink(latest_path) or os.path.exists(latest_path):
                os.remove(latest_path)
            os.symlink(archive_name, latest_path)
        except OSError as e:
            self.get_logger().error(f"Failed to update symlink {latest_path}: {e}")

        self._saved = True
        self.get_logger().info(
            f"Saved {len(self._poses)} poses to {archive_path} (latest → {archive_name})"
        )

    def destroy_node(self):
        if not self._saved:
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
