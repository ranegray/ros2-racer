import yaml
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class GreenVisionNode(Node):
    def __init__(self):
        super().__init__('green_vision')
        self.bridge = CvBridge()
        self.upper_hsv = None
        self.lower_hsv = None
        self.paper_width = None
        self.paper_height = None
        self.latest_depth_image = None
        self.latest_centroid = None

        # Color camera intrinsics
        self.color_fx = None
        self.color_fy = None
        self.color_cx = None
        self.color_cy = None
        self.has_color_info = False

        self._setup_parameters()
        self._setup_subscribers()
        self._setup_publishers()

    def _setup_parameters(self):
        default_config = os.path.join(
            get_package_share_directory("green_vision"),
            "config",
            "rgb_config.yaml",
        )
        config_path = self.declare_parameter(
            "config_path",
            default_config
        ).get_parameter_value().string_value

        try:
            with open(config_path, "r") as f:
                self.config = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().fatal(
                f"Config file not found: '{config_path}'. "
                "Pass the absolute path via: --ros-args -p config_path:=/absolute/path/to/rgb_config.yaml"
            )
            raise
        config = self.config["paper_config"]
        self.lower_hsv = np.array(config["lower_hsv"])
        self.upper_hsv = np.array(config["upper_hsv"])
        self.paper_width = config["paper_width"]
        self.paper_height = config["paper_height"]

    def _setup_publishers(self):
        self.debug_pub = self.create_publisher(Image, "perception/rgb_debug", 10)
        self.point_pub = self.create_publisher(PointStamped, "goal_point", 10)

    def _setup_subscribers(self):
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 1
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback, 1
        )
        self.color_info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.color_info_callback, 10
        )

    def color_info_callback(self, msg):
        self.color_fx = msg.k[0]
        self.color_fy = msg.k[4]
        self.color_cx = msg.k[2]
        self.color_cy = msg.k[5]
        if not self.has_color_info:
            self.has_color_info = True
            self.get_logger().info(
                f"Got color intrinsics: fx={self.color_fx:.1f}, fy={self.color_fy:.1f}, "
                f"cx={self.color_cx:.1f}, cy={self.color_cy:.1f}"
            )

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, self.lower_hsv, self.upper_hsv)
            mask[: int(mask.shape[0] * 2 / 3), :] = 0

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            published = False
            if contours:
                max_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(max_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    self.latest_centroid = (cX, cY)

                    published = self._publish_point(cX, cY, msg.header)
                    cv2.circle(cv_image, (cX, cY), 5, (0, 255, 0), -1)

            if not published:
                no_target = PointStamped()
                no_target.header.stamp = msg.header.stamp
                no_target.header.frame_id = "camera_color_optical_frame"
                self.point_pub.publish(no_target)

            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def _publish_point(self, cX, cY, header):
        if not self.has_color_info:
            self.get_logger().warn("Waiting for camera info, skipping point publish.")
            return False

        if self.latest_depth_image is None:
            self.get_logger().warn("No depth image received yet, skipping point publish.")
            return False

        depth_image = self.latest_depth_image
        h, w = depth_image.shape[:2]

        if not (0 <= cX < w and 0 <= cY < h):
            return False

        raw = float(depth_image[cY, cX])
        if raw <= 0 or np.isnan(raw):
            self.get_logger().warn(f"Invalid depth at ({cX}, {cY}), skipping.")
            return False

        z = raw / 1000.0  # mm to meters
        x = (cX - self.color_cx) * z / self.color_fx
        y = (cY - self.color_cy) * z / self.color_fy

        point_msg = PointStamped()
        point_msg.header.stamp = header.stamp
        point_msg.header.frame_id = "camera_color_optical_frame"
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = z

        self.point_pub.publish(point_msg)
        self.get_logger().info(f"Published point: x={x:.3f}m, y={y:.3f}m, z={z:.3f}m")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = GreenVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Green Vision Node...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
