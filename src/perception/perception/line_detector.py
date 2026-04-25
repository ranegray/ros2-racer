import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from scipy import ndimage
from sensor_msgs.msg import Image


class LineDetectorNode(Node):
    def __init__(self):
        super().__init__("line_detector")

        # RGB thresholds for blue tape (image arrives as BGR).
        # Tape color ~#405F8A → B=138, G=95, R=64 in BGR.
        self.b_min = 90       # minimum blue channel value
        self.br_margin = 30   # blue must exceed red by at least this
        self.bg_margin = 15   # blue must exceed green by at least this
        # Ground reflections are bright-and-desaturated: G and R both climb past
        # the tape's true values. Cap them so specular highlights / glare are
        # rejected even when blue still dominates by the margins above.
        self.r_max = 120      # reject if red channel exceeds this
        self.g_max = 160      # reject if green channel exceeds this
        self.b_max = 210      # reject specular reflections (too-bright blue)
        self.min_blob_pixels = 60  # blobs smaller than this are ignored

        self._setup_subscribers()
        self._setup_publishers()

        self.frame_count = 0
        self.debug_interval = 1
        self.debug_log_interval = 30

    def _setup_publishers(self):
        self.follow_point_pub = self.create_publisher(PointStamped, "line_follow_point", 10)
        self.lookahead_point_pub = self.create_publisher(PointStamped, "line_lookahead_point", 10)
        self.debug_img_pub = self.create_publisher(Image, "line_detector/debug_image", 1)

    def _setup_subscribers(self):
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 1
        )

    def image_callback(self, msg):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )

        det = self._detect(image)

        self._publish_point(self.follow_point_pub, det["follow_centroid"], msg.header)
        self._publish_point(self.lookahead_point_pub, det["lookahead_centroid"], msg.header)

        self.frame_count += 1
        if self.frame_count % self.debug_interval == 0:
            self._publish_debug_image(image, det, msg.header)

    def _detect(self, image):
        """RGB threshold → largest connected blob → centroid.

        follow_centroid: centroid of the largest detected blob (the line).
            Using the largest blob instead of all pixels rejects noise and
            false positives automatically — the contour-based approach from
            https://const-toporov.medium.com/line-following-robot-with-opencv-and-contour-based-approach-417b90f2c298
        """
        height, width = image.shape[:2]
        # Detection band (y_start, y_end, x_start, x_end). Only use the bottom
        # half of the actual frame so upper-frame artifacts cannot pull the
        # follow point even if camera resolution changes.
        band = (height // 2, height, 0, width)
        y0, y1, x0, x1 = band
        cropped = image[y0:y1, x0:x1]

        b = cropped[..., 0].astype(np.int32)
        g = cropped[..., 1].astype(np.int32)
        r = cropped[..., 2].astype(np.int32)

        mask = (
            (b >= self.b_min) &
            (b <= self.b_max) &
            ((b - r) >= self.br_margin) &
            ((b - g) >= self.bg_margin) &
            (r <= self.r_max) &
            (g <= self.g_max)
        )

        det = {
            "mask": mask,
            "largest_blob_mask": None,
            "num_blobs": 0,
            "largest_blob_size": 0,
            "total_mask_pixels": int(mask.sum()),
            "band": band,
            "follow_centroid": None,
            "lookahead_centroid": None,
            "rejected_small_blob": False,
        }

        if not mask.any():
            return det

        labeled, num_features = ndimage.label(mask)
        det["num_blobs"] = int(num_features)
        if num_features == 0:
            return det

        sizes = ndimage.sum(mask, labeled, range(1, num_features + 1))
        largest_label = int(np.argmax(sizes)) + 1
        largest_size = int(sizes[largest_label - 1])
        det["largest_blob_size"] = largest_size
        det["largest_blob_mask"] = (labeled == largest_label)

        if largest_size < self.min_blob_pixels:
            det["rejected_small_blob"] = True
            return det

        blob_ys, blob_xs = np.where(labeled == largest_label)

        # Near centroid: bottom half of band (close to robot)
        band_height = y1 - y0
        near_mask = blob_ys >= (band_height // 2)
        if near_mask.any():
            follow_x = int(np.mean(blob_xs[near_mask] + x0))
            follow_y = int(np.mean(blob_ys[near_mask] + y0))
        else:
            follow_x = int(np.mean(blob_xs + x0))
            follow_y = int(np.mean(blob_ys + y0))
        det["follow_centroid"] = (follow_x, follow_y)

        # Far centroid: top third of band (lookahead)
        far_mask = blob_ys < (band_height // 3)
        if far_mask.any() and far_mask.sum() >= 10:
            det["lookahead_centroid"] = (
                int(np.mean(blob_xs[far_mask] + x0)),
                int(np.mean(blob_ys[far_mask] + y0)),
            )

        return det

    def _publish_debug_image(self, image, det, header):
        debug = image.copy()

        self._overlay_band(debug, det["band"], det["mask"], det["largest_blob_mask"])
        self._draw_centroids(debug, det["follow_centroid"])
        self._draw_hud(debug, det)

        debug_msg = Image()
        debug_msg.header = header
        debug_msg.height, debug_msg.width = debug.shape[:2]
        debug_msg.encoding = "bgr8"
        debug_msg.step = debug_msg.width * 3
        debug_msg.data = debug.tobytes()
        self.debug_img_pub.publish(debug_msg)
        if self.frame_count % self.debug_log_interval == 0:
            self.get_logger().info(
                f"Debug image published (frame {self.frame_count}, "
                f"px={det['total_mask_pixels']}, blobs={det['num_blobs']}, "
                f"largest={det['largest_blob_size']}, "
                f"follow={det['follow_centroid']})"
            )

    def _overlay_band(self, debug, band, mask, largest_blob_mask):
        y0, y1, x0, x1 = band
        if y0 > 0:
            debug[:y0, :] = (debug[:y0, :].astype(np.float32) * 0.25).astype(np.uint8)
        cv2.rectangle(debug, (x0, y0), (x1 - 1, y1 - 1), (255, 255, 0), 2)

        region = debug[y0:y1, x0:x1].astype(np.float32)

        # Green tint for every masked pixel — shows the raw threshold result.
        green = np.zeros_like(region)
        green[..., 1] = mask.astype(np.float32) * 255.0
        region = region * 0.6 + green * 0.4

        # Yellow tint for the largest blob — shows which component was picked
        # as the follow target. Layered on top so noise pixels stay green.
        if largest_blob_mask is not None:
            yellow = np.zeros_like(region)
            yellow[..., 1] = largest_blob_mask.astype(np.float32) * 255.0
            yellow[..., 2] = largest_blob_mask.astype(np.float32) * 255.0
            region = np.where(
                largest_blob_mask[..., None],
                region * 0.4 + yellow * 0.6,
                region,
            )

        debug[y0:y1, x0:x1] = np.clip(region, 0, 255).astype(np.uint8)

    def _draw_centroids(self, debug, follow_c):
        if follow_c is not None:
            cv2.circle(debug, follow_c, 12, (255, 255, 255), 2)
            cv2.circle(debug, follow_c, 8, (0, 0, 255), -1)

    def _draw_hud(self, debug, det):
        if det["follow_centroid"] is not None:
            status, status_color = "FOLLOW", (0, 255, 0)
        elif det["rejected_small_blob"]:
            status, status_color = "BLOB TOO SMALL", (0, 165, 255)
        else:
            status, status_color = "NO LINE", (0, 0, 255)

        h, w = debug.shape[:2]

        self._text(debug, status, (w - 10, 30), status_color,
                   scale=0.9, thickness=2, align_right=True)

        lines = [
            f"frame {self.frame_count}",
            f"mask px: {det['total_mask_pixels']}",
            f"blobs: {det['num_blobs']}  largest: {det['largest_blob_size']}"
            f" / min {self.min_blob_pixels}",
            f"follow: {self._fmt_pt(det['follow_centroid'])}",
            f"thresh: b>={self.b_min}  b-r>={self.br_margin}"
            f"  b-g>={self.bg_margin}",
            f"  r<={self.r_max}  g<={self.g_max}",
        ]
        y = 22
        for line in lines:
            self._text(debug, line, (10, y), (255, 255, 255),
                       scale=0.5, thickness=1)
            y += 18

    @staticmethod
    def _text(img, text, org, color, scale=0.5, thickness=1, align_right=False):
        font = cv2.FONT_HERSHEY_SIMPLEX
        if align_right:
            (tw, _), _ = cv2.getTextSize(text, font, scale, thickness)
            org = (org[0] - tw, org[1])
        # Black shadow for legibility over any background.
        cv2.putText(img, text, org, font, scale, (0, 0, 0), thickness + 2,
                    cv2.LINE_AA)
        cv2.putText(img, text, org, font, scale, color, thickness, cv2.LINE_AA)

    @staticmethod
    def _fmt_pt(pt):
        return f"({pt[0]},{pt[1]})" if pt is not None else "—"

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
