import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image


class LineDetectorNode(Node):
    def __init__(self):
        super().__init__("line_detector")

        self.color_lower = np.array([90, 80, 60])   # H_min, S_min, V_min
        self.color_upper = np.array([130, 255, 255])  # H_max, S_max, V_max

        self.image_width = 640
        self.image_height = 480
        self.crop_area = (
            360,
            480,
            0,
            self.image_width,
        )  # y_start, y_end, x_start, x_end

        self._setup_subscribers()
        self._setup_publishers()

        self.frame_count = 0
        self.debug_interval = 30  # publish debug image every N frames

    def _setup_publishers(self):
        self.point_pub = self.create_publisher(PointStamped, "line_goal_point", 10)
        self.debug_img_pub = self.create_publisher(Image, "line_detector/debug_image", 1)

    def _setup_subscribers(self):
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 1
        )

    def image_callback(self, msg):
        # 1. Convert ROS Image to numpy array
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )

        # 2. Crop to region of interest
        y0, y1, x0, x1 = self.crop_area
        cropped = image[y0:y1, x0:x1, :]

        # 3. Convert to HSV and filter by color range
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)

        # 4. Find centroid of masked area
        ys, xs = np.where(mask)

        # 5. Publish goal point (or empty point if no match)
        cX, cY = None, None
        if len(xs) > 0:
            cX = int(np.mean(xs)) + x0
            cY = int(np.mean(ys)) + y0
            self._publish_point(cX, cY, msg.header)
        else:
            no_target = PointStamped()
            no_target.header.stamp = msg.header.stamp
            no_target.header.frame_id = "camera_color_optical_frame"
            self.point_pub.publish(no_target)

        # 6. Publish debug image occasionally
        self.frame_count += 1
        if self.frame_count % self.debug_interval == 0:
            self._publish_debug_image(image, mask, y0, y1, x0, x1, cX, cY, msg.header, len(xs))

    def _publish_debug_image(self, image, mask, y0, y1, x0, x1, cX, cY, header, pixel_count):
        debug = image.copy()

        # Draw crop region rectangle
        cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 255, 0), 2)

        # Overlay mask in green on the crop region
        mask_color = np.zeros_like(debug[y0:y1, x0:x1])
        mask_color[:, :, 1] = mask  # green channel
        debug[y0:y1, x0:x1] = cv2.addWeighted(debug[y0:y1, x0:x1], 0.7, mask_color, 0.3, 0)

        # Draw centroid if detected
        if cX is not None:
            cv2.circle(debug, (cX, cY), 10, (0, 0, 255), -1)
            cv2.putText(debug, f"({cX},{cY}) px={pixel_count}", (cX + 15, cY),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            cv2.putText(debug, "NO LINE DETECTED", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Publish as ROS Image
        debug_msg = Image()
        debug_msg.header = header
        debug_msg.height, debug_msg.width = debug.shape[:2]
        debug_msg.encoding = "bgr8"
        debug_msg.step = debug_msg.width * 3
        debug_msg.data = debug.tobytes()
        self.debug_img_pub.publish(debug_msg)
        self.get_logger().info(f"Debug image published (frame {self.frame_count}, mask_pixels={pixel_count})")

    def _publish_point(self, x, y, header):
        point_msg = PointStamped()
        point_msg.header.stamp = header.stamp
        point_msg.header.frame_id = "camera_color_optical_frame"
        point_msg.point.x = float(x)
        point_msg.point.y = float(y)
        point_msg.point.z = 0.0

        self.point_pub.publish(point_msg)
        self.get_logger().info(f"Published point: x={x}, y={y}")


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Green Vision Node...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
