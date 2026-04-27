#!/usr/bin/env python3
"""
slam_coordinator_node.py

Simple one-way mode state machine.

  Publish "racing" to /slam_coordinator/switch_mode to trigger the switch.

On switch to "racing":
  1. Calls slam_toolbox/save_map to freeze the map to /home/pi/map/track.
  2. Publishes "racing" on /slam_coordinator/mode so wall_follower stops
     and pure_pursuit takes over.

Modes:
  mapping  — Lap 1: wall follower building the map
  racing   — Lap 2+: pure pursuit on frozen map

Usage (to trigger switch from CLI):
  ros2 topic pub --once /slam_coordinator/switch_mode std_msgs/String "data: racing"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from slam_toolbox.srv import SaveMap


MAP_SAVE_PATH = '/home/pi/map/track'


class SlamCoordinatorNode(Node):
    def __init__(self):
        super().__init__("slam_coordinator_node")

        self._mode = "mapping"

        self._mode_pub = self.create_publisher(String, "/slam_coordinator/mode", 10)
        self.create_subscription(
            String, "/slam_coordinator/switch_mode", self._switch_cb, 10
        )
        # Publish current mode at 1 Hz so other nodes can check
        self.create_timer(1.0, self._publish_mode)

        self._save_map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')

        self.get_logger().info("SLAM coordinator started — mode: MAPPING")

    def _switch_cb(self, msg: String):
        requested = msg.data.strip().lower()
        if requested not in ("mapping", "racing"):
            self.get_logger().warn(f"Unknown mode requested: '{msg.data}' — ignored")
            return

        if requested == self._mode:
            self.get_logger().info(f"Already in {self._mode.upper()} mode")
            return

        if self._mode == "racing" and requested == "mapping":
            self.get_logger().warn("Cannot switch back from RACING to MAPPING")
            return

        if requested == "racing":
            self._save_map()

        self._mode = requested
        self.get_logger().info(f"*** MODE SWITCH: {self._mode.upper()} ***")
        self._publish_mode()

    def _save_map(self):
        if not self._save_map_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("slam_toolbox/save_map service not available — map NOT saved")
            return

        req = SaveMap.Request()
        req.name.data = MAP_SAVE_PATH
        future = self._save_map_client.call_async(req)
        future.add_done_callback(self._save_map_done)
        self.get_logger().info(f"Saving map to {MAP_SAVE_PATH}...")

    def _save_map_done(self, future):
        try:
            result = future.result()
            if result.result:
                self.get_logger().info(f"Map saved to {MAP_SAVE_PATH}")
            else:
                self.get_logger().error("Map save returned failure")
        except Exception as e:
            self.get_logger().error(f"Map save failed: {e}")

    def _publish_mode(self):
        msg = String()
        msg.data = self._mode
        self._mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlamCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
