#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealsenseColorPublisher(Node):
    def __init__(self, width=640, height=480, fps=30):
        super().__init__('realsense_color_publisher')
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.pub = self.create_publisher(Image, '/camera/color/image_raw', sensor_qos)
        self.bridge = CvBridge()

        # Hardware-reset the device to release any stale USB handles
        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
            self.get_logger().info(f'Hardware-resetting RealSense device: {dev.get_info(rs.camera_info.name)}')
            dev.hardware_reset()
        if len(devices) > 0:
            time.sleep(3)  # wait for device to re-enumerate on USB

        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        # Retry pipeline start in case device is still re-enumerating
        for attempt in range(5):
            try:
                self.get_logger().info(f'Starting RealSense pipeline (attempt {attempt + 1}/5)…')
                self.pipe.start(cfg)
                break
            except RuntimeError as e:
                self.get_logger().warn(f'Pipeline start failed: {e}')
                if attempt < 4:
                    time.sleep(2)
                else:
                    raise RuntimeError('Failed to start RealSense pipeline after 5 attempts') from e

        # timer at ~fps
        self.timer = self.create_timer(1.0 / fps, self.capture_and_publish)
        self._fps = fps
        self._frame_count = 0
        self._heartbeat_interval = fps * 10  # log once every 10 seconds

    def capture_and_publish(self):
        try:
            success, frames = self.pipe.try_wait_for_frames(timeout_ms=5)
            if not success:
                return
            color  = frames.get_color_frame()
            if not color:
                return
            img = np.asanyarray(color.get_data())  # BGR uint8
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
            self.pub.publish(msg)
            self._frame_count += 1
            if self._frame_count % self._heartbeat_interval == 0:
                self.get_logger().info(f'RealSense streaming OK ({self._frame_count} frames published)')
        except Exception:
            pass  # transient frame drop; heartbeat will surface if camera goes silent

    def destroy_node(self):
        self.timer.cancel()
        try:
            self.pipe.stop()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = RealsenseColorPublisher()
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

