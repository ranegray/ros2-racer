import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image


def bgr_to_hsv(bgr):
    # Matches cv2.cvtColor(..., COLOR_BGR2HSV) for uint8:
    # H in [0,180), S,V in [0,255].
    bgr = bgr.astype(np.int32)
    b = bgr[..., 0]
    g = bgr[..., 1]
    r = bgr[..., 2]

    cmax = bgr.max(axis=-1)
    cmin = bgr.min(axis=-1)
    delta = cmax - cmin

    v = cmax.astype(np.uint8)

    s = np.zeros_like(cmax)
    nz = cmax > 0
    s[nz] = (255 * delta[nz]) // cmax[nz]
    s = s.astype(np.uint8)

    delta_f = delta.astype(np.float32)
    hf = np.zeros_like(delta_f)
    valid = delta > 0
    rmax = valid & (cmax == r)
    gmax = valid & (cmax == g) & ~rmax
    bmax = valid & (cmax == b) & ~rmax & ~gmax
    hf[rmax] = 30.0 * (g[rmax] - b[rmax]) / delta_f[rmax]
    hf[gmax] = 60.0 + 30.0 * (b[gmax] - r[gmax]) / delta_f[gmax]
    hf[bmax] = 120.0 + 30.0 * (r[bmax] - g[bmax]) / delta_f[bmax]
    hf = hf % 180.0
    h = hf.astype(np.uint8)

    return np.stack([h, s, v], axis=-1)


def draw_rectangle(img, x0, y0, x1, y1, color, thickness=2):
    t = thickness
    img[y0:y0 + t, x0:x1] = color
    img[y1 - t:y1, x0:x1] = color
    img[y0:y1, x0:x0 + t] = color
    img[y0:y1, x1 - t:x1] = color


def draw_filled_circle(img, cx, cy, radius, color):
    h, w = img.shape[:2]
    y0 = max(0, cy - radius)
    y1 = min(h, cy + radius + 1)
    x0 = max(0, cx - radius)
    x1 = min(w, cx + radius + 1)
    if y1 <= y0 or x1 <= x0:
        return
    ys = np.arange(y0, y1)[:, None]
    xs = np.arange(x0, x1)[None, :]
    mask = (xs - cx) ** 2 + (ys - cy) ** 2 <= radius ** 2
    img[y0:y1, x0:x1][mask] = color


class LineDetectorNode(Node):
    def __init__(self):
        super().__init__("line_detector")

        self.color_lower = np.array([100, 150, 80])   # H_min, S_min, V_min
        self.color_upper = np.array([125, 255, 255])  # H_max, S_max, V_max

        self.image_width = 640
        self.image_height = 480

        # Two horizontal ROIs: near band drives steering, far band provides
        # lookahead for curvature-adaptive speed and corner-entry fallback.
        # Format: (y_start, y_end, x_start, x_end)
        self.near_band = (360, 480, 0, self.image_width)
        self.far_band = (140, 300, 0, self.image_width)

        self._setup_subscribers()
        self._setup_publishers()

        self.frame_count = 0
        self.debug_interval = 30  # publish debug image every N frames

    def _setup_publishers(self):
        self.near_point_pub = self.create_publisher(PointStamped, "line_goal_point", 10)
        self.far_point_pub = self.create_publisher(PointStamped, "line_goal_point_far", 10)
        self.debug_img_pub = self.create_publisher(Image, "line_detector/debug_image", 1)

    def _setup_subscribers(self):
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 1
        )

    def image_callback(self, msg):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )

        near_mask, near_centroid = self._detect_in_band(image, self.near_band)
        far_mask, far_centroid = self._detect_in_band(image, self.far_band)

        self._publish_point(self.near_point_pub, near_centroid, msg.header)
        self._publish_point(self.far_point_pub, far_centroid, msg.header)

        self.frame_count += 1
        if self.frame_count % self.debug_interval == 0:
            self._publish_debug_image(
                image, near_mask, far_mask, near_centroid, far_centroid, msg.header
            )

    def _detect_in_band(self, image, band):
        y0, y1, x0, x1 = band
        cropped = image[y0:y1, x0:x1, :]
        hsv = bgr_to_hsv(cropped)
        mask = np.all((hsv >= self.color_lower) & (hsv <= self.color_upper), axis=-1)

        centroid = None
        ys, xs = np.where(mask)
        if len(xs) > 0:
            centroid = (int(np.mean(xs)) + x0, int(np.mean(ys)) + y0)
        return mask, centroid

    def _publish_debug_image(self, image, near_mask, far_mask, near_c, far_c, header):
        debug = image.copy()

        # Near band = yellow, far band = cyan; red dot = near centroid, magenta = far.
        self._overlay_band(debug, self.near_band, near_mask, box_color=(255, 255, 0))
        self._overlay_band(debug, self.far_band, far_mask, box_color=(255, 255, 0))

        if near_c is not None:
            draw_filled_circle(debug, near_c[0], near_c[1], 10, (0, 0, 255))
        if far_c is not None:
            draw_filled_circle(debug, far_c[0], far_c[1], 8, (255, 0, 255))

        debug_msg = Image()
        debug_msg.header = header
        debug_msg.height, debug_msg.width = debug.shape[:2]
        debug_msg.encoding = "bgr8"
        debug_msg.step = debug_msg.width * 3
        debug_msg.data = debug.tobytes()
        self.debug_img_pub.publish(debug_msg)
        self.get_logger().info(
            f"Debug image published (frame {self.frame_count}, "
            f"near_px={int(near_mask.sum())}, far_px={int(far_mask.sum())}, "
            f"near={near_c}, far={far_c})"
        )

    def _overlay_band(self, debug, band, mask, box_color):
        y0, y1, x0, x1 = band
        draw_rectangle(debug, x0, y0, x1, y1, box_color, thickness=2)
        region = debug[y0:y1, x0:x1].astype(np.float32)
        overlay = np.zeros_like(region)
        overlay[..., 1] = mask.astype(np.float32) * 255.0
        blended = region * 0.7 + overlay * 0.3
        debug[y0:y1, x0:x1] = np.clip(blended, 0, 255).astype(np.uint8)

    def _publish_point(self, publisher, centroid, header):
        point_msg = PointStamped()
        point_msg.header.stamp = header.stamp
        point_msg.header.frame_id = "camera_color_optical_frame"
        if centroid is not None:
            point_msg.point.x = float(centroid[0])
            point_msg.point.y = float(centroid[1])
        publisher.publish(point_msg)


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
