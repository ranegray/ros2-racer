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
            reliability=ReliabilityPolicy.BEST_EFFORT,
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

    def capture_and_publish(self):
        try:
            frames = rs.composite_frame(rs.frame())
            if not self.pipe.poll_for_frames(frames):
                return
            color = frames.get_color_frame()
            if not color:
                return
            img = np.asanyarray(color.get_data())  # BGR uint8
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Frame skip: {e}')

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()

