#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanFilterNode(Node):
    """
    fills narrow max-range gaps in the LIDAR scan caused by glass windows/doors/indentations in the hallway.

    Subscribes to /scan, interpolates gaps that are too narrow to be real hallways (< max_gap_deg), and republishes as /scan_filtered.  Both slam_toolbox andwall_follower_node subscribe to /scan_filtered so the map doesn't grow phantom openings where glass is.
    """

    def __init__(self):
        super().__init__('scan_filter_node')

        self.declare_parameter('max_gap_deg', 30.0)

        _qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(LaserScan, '/scan', self._scan_cb, _qos)
        self._pub = self.create_publisher(LaserScan, '/scan_filtered', _qos)

        self.get_logger().info('scan_filter_node ready — publishing /scan_filtered')

    def _scan_cb(self, msg: LaserScan):
        if msg.angle_increment <= 0 or not msg.ranges:
            return

        max_gap_rays = int(math.radians(
            self.get_parameter('max_gap_deg').value
        ) / msg.angle_increment)

        ranges = list(msg.ranges)
        n = len(ranges)

        def bad(r):
            return not math.isfinite(r) or r < msg.range_min or r > msg.range_max

        i = 0
        while i < n:
            if bad(ranges[i]):
                j = i
                while j < n and bad(ranges[j]):
                    j += 1
                gap_len = j - i
                if gap_len <= max_gap_rays:
                    left = next((ranges[k] for k in range(i - 1, -1, -1) if not bad(ranges[k])), None)
                    right = next((ranges[k] for k in range(j, n) if not bad(ranges[k])), None)
                    candidates = [v for v in (left, right) if v is not None]
                    if candidates:
                        fill = min(candidates)
                        for k in range(i, j):
                            ranges[k] = fill
                i = j
            else:
                i += 1

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        out.intensities = list(msg.intensities)
        self._pub.publish(out)


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
