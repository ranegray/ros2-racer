#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanFilterNode(Node):
    """
    Two outputs from /scan:
      /scan_filtered — gap fill only. Used by slam_toolbox for accurate mapping.
      /scan_nav      — gap fill + angular dilation. Used by wall_follower and
                       pure_pursuit for conservative obstacle avoidance.
    """

    def __init__(self):
        super().__init__('scan_filter_node')

        self.declare_parameter('max_gap_deg', 30.0)
        self.declare_parameter('inflate_deg', 6.5)

        _qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(LaserScan, '/scan', self._scan_cb, _qos)
        self._pub     = self.create_publisher(LaserScan, '/scan_filtered', _qos)
        self._pub_nav = self.create_publisher(LaserScan, '/scan_nav', _qos)

        self.get_logger().info('scan_filter_node ready — /scan_filtered (SLAM), /scan_nav (navigation)')

    def _scan_cb(self, msg: LaserScan):
        try:
            if msg.angle_increment <= 0 or not msg.ranges:
                self._pub.publish(msg)
                self._pub_nav.publish(msg)
                return

            max_gap_rays = int(math.radians(
                self.get_parameter('max_gap_deg').value
            ) / msg.angle_increment)

            raw = list(msg.ranges)
            n = len(raw)

            def bad(r):
                return not math.isfinite(r) or r < msg.range_min or r > msg.range_max

            def make_msg(r):
                out = LaserScan()
                out.header          = msg.header
                out.angle_min       = msg.angle_min
                out.angle_max       = msg.angle_max
                out.angle_increment = msg.angle_increment
                out.time_increment  = msg.time_increment
                out.scan_time       = msg.scan_time
                out.range_min       = msg.range_min
                out.range_max       = msg.range_max
                out.ranges          = r
                out.intensities     = list(msg.intensities)
                return out

            # /scan_filtered — raw ranges only, no gap fill.
            # slam_toolbox handles inf/NaN correctly (no return = free space).
            # Gap-filling here adds phantom walls in glass/open areas that corrupt the map.
            self._pub.publish(make_msg(raw))

            # /scan_nav — gap fill + angular dilation, for wall_follower / pure_pursuit.
            # Conservative: fill short gaps with min(neighbors) so glass doesn't look like open space.
            nav = list(raw)
            i = 0
            while i < n:
                if bad(nav[i]):
                    j = i
                    while j < n and bad(nav[j]):
                        j += 1
                    gap_len = j - i
                    if gap_len <= max_gap_rays:
                        left  = next((nav[k] for k in range(i - 1, -1, -1) if not bad(nav[k])), None)
                        right = next((nav[k] for k in range(j, n)          if not bad(nav[k])), None)
                        candidates = [v for v in (left, right) if v is not None]
                        if candidates:
                            fill = min(candidates)
                            for k in range(i, j):
                                nav[k] = fill
                    i = j
                else:
                    i += 1

            inflate_rays = int(math.radians(self.get_parameter('inflate_deg').value) / msg.angle_increment)
            if inflate_rays > 0:
                arr     = np.array(nav, dtype=np.float32)
                padded  = np.pad(arr, inflate_rays, mode='edge')
                w       = 2 * inflate_rays + 1
                windows = np.lib.stride_tricks.as_strided(
                    padded,
                    shape=(n, w),
                    strides=(padded.strides[0], padded.strides[0]),
                )
                nav = np.min(windows, axis=1).tolist()

            self._pub_nav.publish(make_msg(nav))

        except Exception as e:
            self.get_logger().error(f'Filter error: {e}', throttle_duration_sec=5.0)
            self._pub.publish(msg)
            self._pub_nav.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
