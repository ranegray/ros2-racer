"""
ROS node for stop sign detection.

Subscribes:
  /camera/color/image_raw     (sensor_msgs/Image, bgr8)
  /camera/color/camera_info   (sensor_msgs/CameraInfo) — optional, falls back to config

Publishes:
  /stop_sign/event            (racer_msgs/StopSignEvent)         every frame
  /stop_sign/debug_image      (sensor_msgs/Image, rgb8)          every Nth frame
"""
import os
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

from racer_msgs.msg import StopSignEvent

from stop_sign.algorithm.detection import detect
from stop_sign.algorithm.temporal import (
    StopSignFilter, STATE_NONE, STATE_DETECTED, STATE_APPROACH,
)
from stop_sign.visualization import annotate_frame


class StopSignNode(Node):

    def __init__(self):
        super().__init__('stop_sign_node')

        default_config = os.path.join(
            get_package_share_directory('stop_sign'),
            'config',
            'stop_sign.yaml',
        )
        config_path = self.declare_parameter(
            'config_path', default_config
        ).get_parameter_value().string_value

        try:
            with open(config_path, 'r') as f:
                raw = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().fatal(
                f"Config file not found: '{config_path}'. "
                "Pass the absolute path via: --ros-args -p config_path:=/absolute/path/to/stop_sign.yaml"
            )
            raise

        self.config = raw['stop_sign'] if 'stop_sign' in raw else raw

        self.focal_px = float(self.config.get('fallback_focal_px', 600))
        self.has_camera_info = False
        self.debug_every_n = int(self.config.get('debug_every_n_frames', 5))
        self.frame_idx = 0

        self.filter = StopSignFilter(self.config)

        self.event_pub = self.create_publisher(StopSignEvent, '/stop_sign/event', 10)
        self.debug_pub = self.create_publisher(Image, '/stop_sign/debug_image', 1)

        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 1
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10
        )

        self.get_logger().info(
            f"Stop Sign Node started (focal_px fallback={self.focal_px:.1f}, "
            f"debug_every_n={self.debug_every_n})"
        )

    def camera_info_callback(self, msg: CameraInfo):
        fx = float(msg.k[0])
        if fx > 0:
            self.focal_px = fx
            if not self.has_camera_info:
                self.has_camera_info = True
                self.get_logger().info(f"Got camera_info: focal_px={fx:.1f}")

    def _decode_image(self, msg: Image) -> np.ndarray:
        """Decode sensor_msgs/Image to HxWx3 RGB uint8. No cv_bridge."""
        h, w = msg.height, msg.width
        encoding = msg.encoding.lower() if msg.encoding else 'bgr8'
        if encoding in ('bgr8', 'rgb8'):
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            if encoding == 'bgr8':
                buf = buf[..., ::-1]  # BGR -> RGB
            return buf.copy()
        if encoding == 'mono8':
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            return np.stack([buf, buf, buf], axis=-1).copy()
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    def image_callback(self, msg: Image):
        if not self.config.get('enabled', True):
            return

        try:
            rgb = self._decode_image(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to decode image: {e}")
            return

        want_debug = (self.debug_idx_due() and
                      self.debug_pub.get_subscription_count() > 0)
        try:
            detection, debug_info = detect(
                rgb, self.config, self.focal_px, debug=want_debug
            )
        except Exception as e:
            self.get_logger().error(f"detect() failed: {e}")
            self.frame_idx += 1
            return

        now = time.time()
        filt_out = self.filter.update(detection, now)

        evt = StopSignEvent()
        evt.header.stamp = msg.header.stamp
        evt.header.frame_id = msg.header.frame_id or 'camera_color_optical_frame'
        evt.state = int(filt_out.state)
        if detection is not None:
            evt.distance_m = float(detection.distance_m)
            x, y, w, h = detection.bbox
            evt.bbox_x = int(x)
            evt.bbox_y = int(y)
            evt.bbox_w = int(w)
            evt.bbox_h = int(h)
            evt.confidence = float(detection.confidence)
        # NONE state with no detection leaves zero-defaults — the controller
        # treats state==NONE as "nothing to react to".
        self.event_pub.publish(evt)

        if want_debug:
            try:
                annotated = annotate_frame(
                    rgb, debug_info, filt_out,
                    frame_index=self.frame_idx, fps=0.0,
                )
                self.debug_pub.publish(self._encode_image(annotated, msg.header))
            except Exception as e:
                self.get_logger().warn(f"debug publish failed: {e}")

        self.frame_idx += 1

    def debug_idx_due(self) -> bool:
        if self.debug_every_n <= 0:
            return False
        return (self.frame_idx % self.debug_every_n) == 0

    def _encode_image(self, rgb: np.ndarray, header) -> Image:
        h, w = rgb.shape[:2]
        out = Image()
        out.header = header
        out.height = h
        out.width = w
        out.encoding = 'rgb8'
        out.is_bigendian = 0
        out.step = w * 3
        out.data = rgb.tobytes()
        return out


def main(args=None):
    rclpy.init(args=args)
    node = StopSignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Stop Sign Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
