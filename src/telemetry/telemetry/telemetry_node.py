"""
telemetry_node.py

Aggregates high-frequency ROS topics into a single RacerTelemetry message
published at a fixed low rate (default 10 Hz) for the React dashboard.

Subscriptions (all stored as latest-value cache):
  imu/gyro                    → geometry_msgs/Vector3
  imu/accel                   → geometry_msgs/Vector3
  rover/armed                 → std_msgs/Bool
  /scan                       → sensor_msgs/LaserScan
  /perception/front_distance  → std_msgs/Float32
  /camera/color/image_raw     → sensor_msgs/Image   (converted → CompressedImage)

Optional future subscriptions (already wired, just need a publisher):
  /telemetry/heading_error    → std_msgs/Float32
  /telemetry/internal_state   → std_msgs/String
  /telemetry/obstacle         → std_msgs/String

Publication:
  /telemetry/racer            → racer_msgs/RacerTelemetry  (10 Hz)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan, Image, CompressedImage

from racer_msgs.msg import RacerTelemetry

import cv2
import numpy as np
from cv_bridge import CvBridge


# QoS that matches the BEST_EFFORT publishers in rover_node
SENSOR_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('image_quality', 50)   # JPEG quality 0-100

        rate = self.get_parameter('publish_rate').value
        self._jpeg_quality = self.get_parameter('image_quality').value
        self._bridge = CvBridge()

        # ── cached latest values ──────────────────────────────────────────────
        self._gyro: Vector3 | None = None
        self._accel: Vector3 | None = None
        self._armed: bool = False
        self._scan: LaserScan | None = None
        self._front_distance: float = 0.0
        # self._heading_error: float = 0.0
        # self._internal_state: str = ''
        # self._obstacle_detection: str = ''
        self._color_image: CompressedImage | None = None

        # ── subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Vector3, 'imu/gyro',
                                 self._cb_gyro, SENSOR_QOS)
        self.create_subscription(Vector3, 'imu/accel',
                                 self._cb_accel, SENSOR_QOS)
        self.create_subscription(Bool, 'rover/armed',
                                 self._cb_armed, 10)
        self.create_subscription(LaserScan, '/scan',
                                 self._cb_scan, 10)
        self.create_subscription(Float32, '/perception/front_distance',
                                 self._cb_front_distance, 10)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._cb_image, 10)

        # Optional topics — populated by other nodes when available
        # self.create_subscription(Float32, '/telemetry/heading_error',
        #                          self._cb_heading_error, 10)
        # self.create_subscription(String, '/telemetry/internal_state',
        #                          self._cb_internal_state, 10)
        # self.create_subscription(String, '/telemetry/obstacle',
        #                          self._cb_obstacle, 10)

        # ── publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(RacerTelemetry, '/telemetry/racer', 10)

        # ── publish timer ─────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'TelemetryNode started — publishing at {rate} Hz on /telemetry/racer'
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cb_gyro(self, msg: Vector3):
        self._gyro = msg

    def _cb_accel(self, msg: Vector3):
        self._accel = msg

    def _cb_armed(self, msg: Bool):
        self._armed = msg.data

    def _cb_scan(self, msg: LaserScan):
        self._scan = msg

    def _cb_front_distance(self, msg: Float32):
        self._front_distance = msg.data

    def _cb_image(self, msg: Image):
        """Convert raw Image → JPEG CompressedImage to keep bandwidth low."""
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
            ok, buf = cv2.imencode('.jpg', cv_img, encode_params)
            if not ok:
                return
            compressed = CompressedImage()
            compressed.header = msg.header
            compressed.format = 'jpeg'
            compressed.data = buf.tobytes()
            self._color_image = compressed
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}', throttle_duration_sec=5.0)

    def _cb_heading_error(self, msg: Float32):
        self._heading_error = msg.data

    def _cb_internal_state(self, msg: String):
        self._internal_state = msg.data

    def _cb_obstacle(self, msg: String):
        self._obstacle_detection = msg.data

    # ── publish ───────────────────────────────────────────────────────────────

    def _publish(self):
        msg = RacerTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if self._gyro is not None:
            msg.gyro = self._gyro
        if self._accel is not None:
            msg.accel = self._accel

        msg.armed = self._armed
        msg.front_distance = self._front_distance
        msg.heading_error = self._heading_error
        msg.internal_state = self._internal_state
        msg.obstacle_detection = self._obstacle_detection

        if self._scan is not None:
            msg.scan = self._scan
        if self._color_image is not None:
            msg.color_image = self._color_image

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
