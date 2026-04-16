"""
telemetry_node.py

Aggregates high-frequency ROS topics for the React dashboard.

Two publishers, intentionally decoupled so a heavy payload can't backpressure
the scalar stream:

  /telemetry/racer  → racer_msgs/RacerTelemetry    (status + IMU + scalars, 10 Hz)
  /telemetry/camera → sensor_msgs/CompressedImage  (JPEG camera frames,      5 Hz)

Subscriptions (all stored as latest-value cache):
  imu/gyro                    → geometry_msgs/Vector3
  imu/accel                   → geometry_msgs/Vector3
  rover/armed                 → std_msgs/Bool
  /perception/front_distance  → std_msgs/Float32
  /camera/color/image_raw     → sensor_msgs/Image   (converted → CompressedImage)

Optional future subscriptions (already wired, just need a publisher):
  /telemetry/heading_error    → std_msgs/Float32
  /telemetry/internal_state   → std_msgs/String
  /telemetry/obstacle         → std_msgs/String
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image, CompressedImage

from racer_msgs.msg import RacerTelemetry

import cv2
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
        self.declare_parameter('camera_rate', 5.0)
        self.declare_parameter('image_quality', 50)   # JPEG quality 0-100

        rate = self.get_parameter('publish_rate').value
        camera_rate = self.get_parameter('camera_rate').value
        self._jpeg_quality = self.get_parameter('image_quality').value
        self._bridge = CvBridge()

        # ── cached latest values ──────────────────────────────────────────────
        self._gyro: Vector3 | None = None
        self._accel: Vector3 | None = None
        self._armed: bool = False
        self._front_distance: float = 0.0
        self._heading_error: float = 0.0
        self._internal_state: str = ''
        self._obstacle_detection: str = ''
        # Cache the raw Image; encode JPEG only in the camera publish timer so
        # we do one encode per published frame instead of one per incoming
        # frame. A single-threaded rclpy executor gets monopolized otherwise
        # and delays the 5 Hz timer, producing burst-then-stall on the wire.
        self._latest_image: Image | None = None

        # ── subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Vector3, 'imu/gyro',
                                 self._cb_gyro, SENSOR_QOS)
        self.create_subscription(Vector3, 'imu/accel',
                                 self._cb_accel, SENSOR_QOS)
        self.create_subscription(Bool, 'rover/armed',
                                 self._cb_armed, 10)
        self.create_subscription(Float32, '/perception/front_distance',
                                 self._cb_front_distance, 10)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._cb_image, 10)

        # Optional topics — populated by other nodes when available
        self.create_subscription(Float32, '/telemetry/heading_error',
                                 self._cb_heading_error, 10)
        self.create_subscription(String, '/telemetry/internal_state',
                                 self._cb_internal_state, 10)
        self.create_subscription(String, '/telemetry/obstacle',
                                 self._cb_obstacle, 10)

        # ── publishers ────────────────────────────────────────────────────────
        # Scalar telemetry: small, reliable, steady rate.
        self._pub = self.create_publisher(RacerTelemetry, '/telemetry/racer', 10)
        # Camera stream: heavy, its own lane so a slow socket doesn't block
        # scalar telemetry. Depth=1 so a stalled consumer drops old frames
        # rather than queueing.
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cam_pub = self.create_publisher(
            CompressedImage, '/telemetry/camera', camera_qos
        )

        # ── publish timers ────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._publish)
        self.create_timer(1.0 / camera_rate, self._publish_camera)

        self.get_logger().info(
            f'TelemetryNode started — scalars @ {rate} Hz on /telemetry/racer, '
            f'camera @ {camera_rate} Hz on /telemetry/camera'
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cb_gyro(self, msg: Vector3):
        self._gyro = msg

    def _cb_accel(self, msg: Vector3):
        self._accel = msg

    def _cb_armed(self, msg: Bool):
        self._armed = msg.data

    def _cb_front_distance(self, msg: Float32):
        self._front_distance = msg.data

    def _cb_image(self, msg: Image):
        """Latest-value cache only. JPEG encoding happens in the publish timer."""
        self._latest_image = msg

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

        self._pub.publish(msg)

    def _publish_camera(self):
        img = self._latest_image
        if img is None:
            return
        try:
            cv_img = self._bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
            ok, buf = cv2.imencode('.jpg', cv_img, encode_params)
            if not ok:
                return
            compressed = CompressedImage()
            compressed.header = img.header
            compressed.format = 'jpeg'
            compressed.data = buf.tobytes()
            self._cam_pub.publish(compressed)
        except Exception as e:
            self.get_logger().warn(f'Image encode failed: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
