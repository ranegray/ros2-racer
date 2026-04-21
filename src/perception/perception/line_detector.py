import rclpy
from rclpy.node import Node
import numpy as np
from scipy.ndimage import label, binary_erosion
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image


def bgr_to_hsv(bgr):
    """OpenCV-compatible HSV: H in 0–180, S and V in 0–255."""
    f = bgr.astype(np.float32) / 255.0
    b, g, r = f[..., 0], f[..., 1], f[..., 2]
    cmax = f.max(axis=-1)
    cmin = f.min(axis=-1)
    delta = cmax - cmin

    v = (cmax * 255).astype(np.uint8)

    s = np.where(cmax > 0, delta / cmax * 255, 0.0).astype(np.uint8)

    h = np.zeros_like(delta)
    valid = delta > 0
    rmax = valid & (cmax == r)
    gmax = valid & (cmax == g) & ~rmax
    bmax = valid & (cmax == b) & ~rmax & ~gmax
    h[rmax] = 60.0 * ((g[rmax] - b[rmax]) / delta[rmax] % 6)
    h[gmax] = 60.0 * ((b[gmax] - r[gmax]) / delta[gmax] + 2)
    h[bmax] = 60.0 * ((r[bmax] - g[bmax]) / delta[bmax] + 4)
    h = (h / 2).astype(np.uint8)

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

        # --- HSV thresholds (OpenCV: H 0-180, S 0-255, V 0-255) ---
        # Derived from 8 real tape samples: H:100.9-103.7, S:129-210, V:106-196
        # High S_min is the key wall rejector — light blue walls are desaturated.
        self.hsv_lower = np.array([100, 125, 100])
        self.hsv_upper = np.array([104, 215, 200])

        # --- RGB thresholds (image arrives as BGR) ---
        # Same 8 samples: B:106-196, B-R:80-136, B-G:34-56
        self.b_min = 90
        self.br_margin = 65
        self.bg_margin = 25

        self.min_blob_pixels = 60

        self.image_width = 640
        self.image_height = 480

        # Detection band (y_start, y_end, x_start, x_end)
        self.band = (140, 380, 0, self.image_width)

        self.turn_right_threshold_x = 500
        self.turn_right_pixel_min = 60

        # Shape filter — only accept blobs that look like a thin straight line.
        # lam1/lam2 is the eigenvalue ratio of the blob's covariance matrix.
        #   Round blob → ratio ≈ 1.  Long thin line → ratio >> 1.
        # 4*sqrt(lam2) approximates the blob's width in the minor direction.
        self.min_elongation_ratio = 5.0
        self.max_tape_width_px = 70

        self._setup_subscribers()
        self._setup_publishers()

        self.frame_count = 0
        self.debug_interval = 30

    def _setup_publishers(self):
        self.follow_point_pub = self.create_publisher(PointStamped, "line_follow_point", 10)
        self.turn_point_pub = self.create_publisher(PointStamped, "line_turn_point", 10)
        self.debug_img_pub = self.create_publisher(Image, "line_detector/debug_image", 1)

    def _setup_subscribers(self):
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 1
        )

    def image_callback(self, msg):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )

        mask, follow_centroid, turn_target = self._detect(image)

        self._publish_point(self.follow_point_pub, follow_centroid, msg.header)
        self._publish_point(self.turn_point_pub, turn_target, msg.header)

        self.frame_count += 1
        if self.frame_count % self.debug_interval == 0:
            self._publish_debug_image(image, mask, follow_centroid, turn_target, msg.header)

    def _detect(self, image):
        y0, y1, x0, x1 = self.band
        cropped = image[y0:y1, x0:x1]

        # 1. HSV filter — tight hue + high saturation rejects pale walls
        hsv = bgr_to_hsv(cropped)
        hsv_mask = np.all(
            (hsv >= self.hsv_lower) & (hsv <= self.hsv_upper), axis=-1
        )

        # 2. RGB filter — B-channel dominance confirms it's genuinely blue
        b = cropped[..., 0].astype(np.int32)
        g = cropped[..., 1].astype(np.int32)
        r = cropped[..., 2].astype(np.int32)
        rgb_mask = (
            (b >= self.b_min) &
            ((b - r) >= self.br_margin) &
            ((b - g) >= self.bg_margin)
        )

        # Combined: must pass both
        mask = hsv_mask & rgb_mask

        # 3. Morphological erosion — kill isolated noise pixels before contouring
        mask = binary_erosion(mask, structure=np.ones((3, 3)))

        all_ys, all_xs = np.where(mask)
        if len(all_xs) == 0:
            return mask, None, None

        all_xs_abs = all_xs + x0
        all_ys_abs = all_ys + y0

        # Largest blob → follow centroid
        labeled, num_features = label(mask)
        if num_features == 0:
            return mask, None, None

        sizes = [labeled[labeled == i].size for i in range(1, num_features + 1)]
        largest_label = int(np.argmax(sizes)) + 1
        if sizes[largest_label - 1] < self.min_blob_pixels:
            return mask, None, None

        blob_ys, blob_xs = np.where(labeled == largest_label)
        blob_xs_abs = blob_xs + x0
        blob_ys_abs = blob_ys + y0
        follow_centroid = (int(np.mean(blob_xs_abs)), int(np.mean(blob_ys_abs)))

        # Right-turn override
        right_mask = all_xs_abs > self.turn_right_threshold_x
        turn_target = None
        if int(right_mask.sum()) >= self.turn_right_pixel_min:
            turn_target = (
                int(np.mean(all_xs_abs[right_mask])),
                int(np.mean(all_ys_abs[right_mask])),
            )

        return mask, follow_centroid, turn_target

    def _publish_debug_image(self, image, mask, follow_c, turn_c, header):
        debug = image.copy()

        self._overlay_band(debug, self.band, mask, box_color=(255, 255, 0))

        y0, y1, _x0, _x1 = self.band
        if turn_c is not None:
            tx = self.turn_right_threshold_x
            draw_rectangle(debug, tx, y0, tx + 2, y1, (255, 0, 255), thickness=2)

        if follow_c is not None:
            draw_filled_circle(debug, follow_c[0], follow_c[1], 10, (0, 0, 255))
        if turn_c is not None:
            draw_filled_circle(debug, turn_c[0], turn_c[1], 10, (255, 0, 255))

        debug_msg = Image()
        debug_msg.header = header
        debug_msg.height, debug_msg.width = debug.shape[:2]
        debug_msg.encoding = "bgr8"
        debug_msg.step = debug_msg.width * 3
        debug_msg.data = debug.tobytes()
        self.debug_img_pub.publish(debug_msg)
        self.get_logger().info(
            f"Debug image published (frame {self.frame_count}, "
            f"px={int(mask.sum())}, follow={follow_c}, turn={turn_c})"
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
