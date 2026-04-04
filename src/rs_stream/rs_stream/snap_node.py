#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np


class SnapNode(Node):
    def __init__(self):
        super().__init__('snap_node')
        self.pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.bridge = CvBridge()

        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipe.start(cfg)

        # let auto-exposure warm up
        for _ in range(30):
            pipe.wait_for_frames()

        frames = pipe.wait_for_frames()
        color = frames.get_color_frame()
        pipe.stop()

        if not color:
            self.get_logger().error('No frame captured')
            return

        img = np.asanyarray(color.get_data())
        self.msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = 'camera_color_optical_frame'

        # publish on a short timer to give rqt time to discover and subscribe
        self.count = 0
        self.timer = self.create_timer(0.1, self.publish_and_shutdown)

    def publish_and_shutdown(self):
        self.pub.publish(self.msg)
        self.count += 1
        self.get_logger().info(f'Published snap ({self.count})')
        if self.count >= 150:  # ~15 seconds of republishing
            self.timer.cancel()
            raise SystemExit


def main():
    rclpy.init()
    node = SnapNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
