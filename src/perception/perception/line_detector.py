import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image


class LineDetectorNode(Node):
    def __init__(self):
        super().__init__("line_detector")

        # TODO: Sample from real image in basement for an accurate line color range
        self.color_lower = np.array([0, 150, 0])  # R_min, G_min, B_min
        self.color_upper = np.array([80, 255, 80])  # R_max, G_max, B_max

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

    def _setup_publishers(self):
        self.point_pub = self.create_publisher(PointStamped, "line_goal_point", 10)

    def _setup_subscribers(self):
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 1
        )

    def image_callback(self, msg):
        self.get_logger().info("Received image frame", throttle_duration_sec=2.0)
        # 1. Convert ROS Image to numpy array
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )

        # 2. Crop to region of interest
        y0, y1, x0, x1 = self.crop_area
        cropped = image[y0:y1, x0:x1, :]

        # 3. Filter pixels by color range
        mask = np.all(
            (cropped >= self.color_lower) & (cropped <= self.color_upper), axis=2
        )

        # 4. Find centroid of masked area
        ys, xs = np.where(mask)

        # 5. Publish goal point (or empty point if no match)
        if len(xs) > 0:
            cX = int(np.mean(xs)) + x0
            cY = int(np.mean(ys)) + y0

            self._publish_point(cX, cY, msg.header)
        else:
            no_target = PointStamped()
            no_target.header.stamp = msg.header.stamp
            no_target.header.frame_id = "camera_color_optical_frame"
            self.point_pub.publish(no_target)

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
