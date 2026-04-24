#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealsenseColorPublisher(Node):
    def __init__(self, width=640, height=480, fps=30):
        super().__init__('realsense_color_publisher')
        self.pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.bridge = CvBridge()

        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.get_logger().info('Starting RealSense pipeline…')
        self.pipe.start(cfg)

        # timer at ~fps
        self.timer = self.create_timer(1.0 / fps, self.capture_and_publish)

    def capture_and_publish(self):
        try:
            frames = self.pipe.wait_for_frames(timeout_ms=1000)
            color  = frames.get_color_frame()
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
