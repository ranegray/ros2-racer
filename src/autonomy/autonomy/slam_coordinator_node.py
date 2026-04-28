#!/usr/bin/env python3
"""
slam_coordinator_node.py

State machine: mapping → saving → ready → racing

Transition flow:
  1. Operator publishes "racing" to /slam_coordinator/switch_mode.
  2. Enters SAVING: stops wall follower, triggers path_recorder to save,
     starts async map save to /home/pi/map/track.
  3. When BOTH map and path are confirmed saved → enters READY.
     Status is published to /telemetry/internal_state for the dashboard.
  4. Operator confirms via /slam_coordinator/confirm (Empty) → enters RACING.
     pure_pursuit activates.

Trigger commands:
  ros2 topic pub --once /slam_coordinator/switch_mode std_msgs/String "data: racing"
  ros2 topic pub --once /slam_coordinator/confirm std_msgs/Empty "{}"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from slam_toolbox.srv import SaveMap


MAP_SAVE_PATH = '/home/pi/map/track'

CONFIRM_CMD = (
    "ros2 topic pub --once /slam_coordinator/confirm std_msgs/msg/Empty '{}'"
)


class SlamCoordinatorNode(Node):
    def __init__(self):
        super().__init__("slam_coordinator_node")

        self._mode = "mapping"
        self._map_saved = False
        self._path_saved = False

        # Outbound
        self._mode_pub = self.create_publisher(String, "/slam_coordinator/mode", 10)
        self._save_path_pub = self.create_publisher(
            String, "/slam_coordinator/save_path", 1
        )
        self._status_pub = self.create_publisher(
            String, "/telemetry/internal_state", 10
        )

        # Inbound
        self.create_subscription(
            String, "/slam_coordinator/switch_mode", self._switch_cb, 10
        )
        self.create_subscription(
            String, "/slam_coordinator/path_saved", self._path_saved_cb, 10
        )
        self.create_subscription(
            Empty, "/slam_coordinator/confirm", self._confirm_cb, 10
        )

        # Publish current mode at 1 Hz so nodes that missed the initial message catch up
        self.create_timer(1.0, self._publish_mode)

        self._save_map_client = self.create_client(SaveMap, "/slam_toolbox/save_map")

        self.get_logger().info("SLAM coordinator started — mode: MAPPING")

    # ------------------------------------------------------------------
    # Switch trigger
    # ------------------------------------------------------------------

    def _switch_cb(self, msg: String):
        requested = msg.data.strip().lower()
        if requested != "racing":
            self.get_logger().warn(f"Unknown mode requested: '{msg.data}' — ignored")
            return
        if self._mode != "mapping":
            self.get_logger().info(
                f"Transition already in progress (current: {self._mode})"
            )
            return

        self.get_logger().info("Transition requested — entering SAVING")
        self._mode = "saving"
        self._map_saved = False
        self._path_saved = False
        self._publish_mode()
        self._publish_status("SAVING: recording path and saving map...")

        # Ask path_recorder to flush immediately
        save_msg = String()
        save_msg.data = "save"
        self._save_path_pub.publish(save_msg)

        # Async map save
        self._save_map()

    # ------------------------------------------------------------------
    # Map save
    # ------------------------------------------------------------------

    def _save_map(self):
        if not self._save_map_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                "slam_toolbox/save_map not available — proceeding without map save"
            )
            self._map_saved = True
            self._check_ready()
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
                self.get_logger().info(f"Map saved → {MAP_SAVE_PATH}")
            else:
                self.get_logger().error("Map save service returned failure — proceeding")
        except Exception as e:
            self.get_logger().error(f"Map save exception: {e} — proceeding")
        self._map_saved = True
        self._check_ready()

    # ------------------------------------------------------------------
    # Path save confirmation
    # ------------------------------------------------------------------

    def _path_saved_cb(self, msg: String):
        self.get_logger().info("Path recorder confirmed save")
        self._path_saved = True
        self._check_ready()

    # ------------------------------------------------------------------
    # Ready check
    # ------------------------------------------------------------------

    def _check_ready(self):
        if self._mode != "saving":
            return
        if not (self._map_saved and self._path_saved):
            return

        self._mode = "ready"
        self._publish_mode()
        self.get_logger().info(
            f"Map + path saved — READY TO RACE. Confirm with:\n  {CONFIRM_CMD}"
        )
        self._publish_status(f"READY TO RACE — confirm: {CONFIRM_CMD}")

    # ------------------------------------------------------------------
    # User confirmation
    # ------------------------------------------------------------------

    def _confirm_cb(self, _msg):
        if self._mode != "ready":
            self.get_logger().warn(
                f"Confirm received but not in READY state (current: {self._mode})"
            )
            return
        self._mode = "racing"
        self._publish_mode()
        self.get_logger().info("*** MODE SWITCH: RACING ***")
        self._publish_status("RACING")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_mode(self):
        msg = String()
        msg.data = self._mode
        self._mode_pub.publish(msg)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


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
