import rclpy
from rclpy.node import Node
import numpy as np
from scipy import ndimage
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image


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

        # RGB thresholds for blue tape (image arrives as BGR).
        # 8 samples taken across tape under varying light → BGR stats:
        #   B channel: 106–196,  B-R: 80–136,  B-G: 34–56
        self.b_min = 90       # floor is 106; 16-unit buffer for darker shadows
        self.br_margin = 65   # floor is  80; 15-unit buffer
        self.bg_margin = 25   # floor is  34;  9-unit buffer
        self.min_blob_pixels = 60  # blobs smaller than this are ignored

        self.image_width = 640
        self.image_height = 480

        # Detection band (y_start, y_end, x_start, x_end)
        self.band = (140, 380, 0, self.image_width)

        # 90° right-turn detection: far-right pixel cluster = horizontal
        # stroke of the L-marker. Checked across ALL detected pixels (not
        # just the largest blob) so the stroke registers even if separated.
        self.turn_right_threshold_x = 500
        self.turn_right_pixel_min = 60

        # Orientation-based early turn detection.
        # The major axis of the tape blob is computed via second-order moments.
        # A vertical (straight) tape has |cos(angle)| ≈ 0.
        # A horizontal (90° turn ahead) tape has |cos(angle)| → 1.
        # When the blob tips past the threshold, steer toward the far (top)
        # portion of the blob so the turn starts well before the L-marker
        # reaches the right edge.
        self.min_pixels_for_orientation = 100
        self.horizontal_threshold = 0.6   # |cos(angle)| above this = turning
        self.min_elongation_ratio = 2.0   # lam1/lam2 must exceed this; rejects
                                          # blob-shaped false positives (tape is
                                          # always elongated, not round)

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
        """RGB threshold → largest connected blob → centroid.

        follow_centroid: centroid of the largest detected blob (the line).
            Using the largest blob instead of all pixels rejects noise and
            false positives automatically — the contour-based approach from
            https://const-toporov.medium.com/line-following-robot-with-opencv-and-contour-based-approach-417b90f2c298
        turn_target: centroid of far-right pixels when the right-turn
            L-marker is visible. Checked on all detected pixels so the
            horizontal stroke registers even if it forms a separate blob.
        """
        y0, y1, x0, x1 = self.band
        cropped = image[y0:y1, x0:x1]

        b = cropped[..., 0].astype(np.int32)
        g = cropped[..., 1].astype(np.int32)
        r = cropped[..., 2].astype(np.int32)

        mask = (
            (b >= self.b_min) &
            ((b - r) >= self.br_margin) &
            ((b - g) >= self.bg_margin)
        )

        # All detected pixels (used for right-turn check)
        all_ys, all_xs = np.where(mask)
        if len(all_xs) == 0:
            return mask, None, None

        all_xs_abs = all_xs + x0
        all_ys_abs = all_ys + y0

        # --- Largest blob → follow centroid ---
        labeled, num_features = ndimage.label(mask)
        if num_features == 0:
            return mask, None, None

        sizes = ndimage.sum(mask, labeled, range(1, num_features + 1))
        largest_label = int(np.argmax(sizes)) + 1
        if sizes[largest_label - 1] < self.min_blob_pixels:
            return mask, None, None

        blob_ys, blob_xs = np.where(labeled == largest_label)
        blob_xs_abs = blob_xs + x0
        blob_ys_abs = blob_ys + y0

        # --- Moments: shape filter + orientation ---
        # Compute second-order moments once; use them for both:
        #   1. Elongation check — rejects blob-shaped false positives.
        #      lam1/lam2 is the eigenvalue ratio of the covariance matrix.
        #      Tape is elongated (ratio >> 1); random blobs are round (ratio ≈ 1).
        #      If the blob fails the shape check, follow_centroid stays None
        #      (the right-turn check below still runs independently).
        #   2. Orientation check — sets turn_target early when tape is horizontal.
        follow_centroid = None
        turn_target = None
        if len(blob_xs) >= self.min_pixels_for_orientation:
            cx_f = float(np.mean(blob_xs_abs))
            cy_f = float(np.mean(blob_ys_abs))
            dx = blob_xs_abs.astype(float) - cx_f
            dy = blob_ys_abs.astype(float) - cy_f
            mu11 = float(np.mean(dx * dy))
            mu20 = float(np.mean(dx * dx))
            mu02 = float(np.mean(dy * dy))

            trace = mu20 + mu02
            det = mu20 * mu02 - mu11 ** 2
            discriminant = max(0.0, (trace / 2) ** 2 - det)
            lam1 = trace / 2 + discriminant ** 0.5
            lam2 = max(trace / 2 - discriminant ** 0.5, 1e-6)

            if lam1 / lam2 >= self.min_elongation_ratio:
                follow_centroid = (int(cx_f), int(cy_f))

                angle = 0.5 * np.arctan2(2.0 * mu11, mu20 - mu02)
                if abs(np.cos(angle)) > self.horizontal_threshold:
                    mid_y = (y0 + y1) // 2
                    far_mask = blob_ys_abs < mid_y
                    if int(far_mask.sum()) >= self.min_pixels_for_orientation // 4:
                        turn_target = (
                            int(np.mean(blob_xs_abs[far_mask])),
                            int(np.mean(blob_ys_abs[far_mask])),
                        )
        else:
            follow_centroid = (int(np.mean(blob_xs_abs)), int(np.mean(blob_ys_abs)))

        # --- 90° right-turn override ---
        # Far-right pixel cluster overrides the orientation-based target.
        right_mask = all_xs_abs > self.turn_right_threshold_x
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
