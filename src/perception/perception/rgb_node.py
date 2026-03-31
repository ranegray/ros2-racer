import yaml
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

"""
RealSense RGB/Depth Cam

-> rover must autonomously follow a green sheet of paper

"""


class RGBNode(Node):
    def __init__(self):
        super().__init__("rgb_node")

        self.bridge = CvBridge()
        self.upper_hsv = None
        self.lower_hsv = None
        self.hsv = None
        self.rgb = None
        self.paper_width = None
        self.paper_height = None

        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()

    def _setup_parameters(self):
        config_path = self.declare_parameter("config_path", "config/rgb_config.yaml")
        try:
            with open(config_path, "r") as f:
                self.config = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().fatal(
                f"Config file not found: '{config_path}'. "
                "Pass the absolute path via: --ros-args -p config_path:=/absolute/path/to/rgb_config.yaml"
            )
            raise
        config = self.config
        self.lower_hsv = np.array(config["lower_hsv"])
        self.upper_hsv = np.array(config["upper_hsv"])
        self.paper_width = config["paper_width"]
        self.paper_height = config["paper_height"]

    def _setup_publishers(self):
        self.debug_pub = self.create_publisher(Image, "perception/rgb_debug", 10)

        # TODO: add pose publisher

    def _setup_subscribers(self):
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 10
        )

    def image_callback(self, msg):

        try:
            # 1. Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 2. convert img to HSV for robustness since lighting messes w RGB
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # 3. color mask
            mask = cv2.inRange(hsv_image, self.lower_hsv, self.upper_hsv)

            # 4. find contours
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )  # detected boundaries of masked areas

            # 5. TODO compute centroid / error

            # 6. TODO publish pose to controller node

            # TODO: implement timeout / fallback beahvior if paper is lost

            # 7. draw debug info
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")
