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

        self.color_lower = np.array([100, 110, 60])   # H_min, S_min, V_min
        self.color_upper = np.array([120, 255, 200])  # H_max, S_max, V_max

        self.image_width = 640
        self.image_height = 480

        # Single detection band. Tall enough to see across large (~1m) gaps in
        # the tape so the centroid finds the next segment rather than going blind.
        # Biased toward the upper (farther) portion so the centroid gives a
        # lookahead signal — anticipates curves instead of reacting once the
        # line has already swept across the near view.
        # Format: (y_start, y_end, x_start, x_end)
        self.band = (140, 380, 0, self.image_width)

        # 90° right-turn detection: we declare a right turn when there is a
        # sizable chunk of tape pixels to the right of turn_right_threshold_x
        # (these come from the horizontal stroke of an L). Robust to gaps
        # because it only requires the stroke, not a stem-plus-stroke pair.
        self.turn_right_threshold_x = 400  # pixels with x > this are "far-right"
        self.turn_right_pixel_min = 60     # min far-right pixel count to call it a turn

        # Horizontal orientation detection: when tape curves it appears more
        # horizontal in the image. We detect this via second-order moments and
        # steer toward the far (top-of-band) centroid to anticipate the turn.
        self.min_pixels_for_orientation = 100  # min tape pixels to compute orientation
        self.horizontal_threshold = 0.4        # |cos(angle)| above this = curving

        self._setup_subscribers()
        self._setup_publishers()

        self.frame_count = 0
        self.debug_interval = 30  # publish debug image every N frames

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
        """Run HSV detection on the band and return (mask, follow_centroid, turn_target).

        follow_centroid: mean of all detected pixels. Drives normal following.
        turn_target: set when a curve is detected. Two sources, in priority order:
            1. Far-right pixel cluster (horizontal stroke of a 90° right-turn L-marker).
            2. Orientation detection — when the tape blob is significantly non-vertical
               (|cos(angle)| > horizontal_threshold), uses the far (top-of-band) centroid
               to anticipate the curve direction early. Works for both left and right curves.
            When non-None, control overrides following and steers toward turn_target.
        """
        y0, y1, x0, x1 = self.band
        cropped = image[y0:y1, x0:x1, :]
        hsv = bgr_to_hsv(cropped)
        mask = np.all((hsv >= self.color_lower) & (hsv <= self.color_upper), axis=-1)

        ys, xs = np.where(mask)
        if len(xs) == 0:
            return mask, None, None

        xs_abs = xs + x0
        ys_abs = ys + y0

        # Weight pixels by distance ahead — higher in the image (lower y) = further
        # ahead = more weight. This biases the centroid toward the upcoming curve
        # without discarding near pixels entirely.
        weights = (y1 - ys_abs).astype(float)
        weights /= weights.sum()
        follow_centroid = (int(np.average(xs_abs, weights=weights)), int(np.average(ys_abs, weights=weights)))

        # --- Orientation-based curve detection ---
        # Compute second-order moments to find the angle of the tape blob's
        # major axis. A straight tape is nearly vertical (angle ≈ ±π/2,
        # |cos| ≈ 0). A curving/horizontal tape has |cos(angle)| > threshold.
        # When curving, use the far (top) half of the band as the turn target
        # so we react to the upcoming turn instead of the tape underfoot.
        turn_target = None
        if len(xs) >= self.min_pixels_for_orientation:
            cx_f = float(np.mean(xs_abs))
            cy_f = float(np.mean(ys_abs))
            dx = xs_abs.astype(float) - cx_f
            dy = ys_abs.astype(float) - cy_f
            mu11 = float(np.mean(dx * dy))
            mu20 = float(np.mean(dx * dx))
            mu02 = float(np.mean(dy * dy))
            angle = 0.5 * np.arctan2(2.0 * mu11, mu20 - mu02)
            if abs(np.cos(angle)) > self.horizontal_threshold:
                mid_y = (y0 + y1) // 2
                far_mask = ys_abs < mid_y
                if int(far_mask.sum()) >= self.min_pixels_for_orientation // 4:
                    turn_target = (
                        int(np.mean(xs_abs[far_mask])),
                        int(np.mean(ys_abs[far_mask])),
                    )

        # --- 90° right-turn override ---
        # Far-right pixel cluster = horizontal stroke of an L-marker.
        # Overrides the orientation-based target when present.
        right_mask = xs_abs > self.turn_right_threshold_x
        if int(right_mask.sum()) >= self.turn_right_pixel_min:
            turn_target = (int(np.mean(xs_abs[right_mask])), int(np.mean(ys_abs[right_mask])))

        return mask, follow_centroid, turn_target

    def _publish_debug_image(self, image, mask, follow_c, turn_c, header):
        debug = image.copy()

        # Yellow band outline, green mask overlay. Red dot = follow centroid.
        # Magenta dot + vertical magenta line at turn_right_threshold_x when a
        # right turn is detected.
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
