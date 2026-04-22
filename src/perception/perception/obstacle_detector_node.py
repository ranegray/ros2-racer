"""
RPLIDAR depth perception for autonomous detection of wall

sub to LIDAR sensor /scan topic -> publish distance from wall
"""

import math
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__("obstacle_detector_node")

        self._setup_parameters()
        self._setup_subscriptions()
        self._setup_publishers()

    def _setup_parameters(self):
        self.declare_parameter("distance_threshold", 1.0)  # meters
        self.distance_threshold = self.get_parameter("distance_threshold").value
        self.declare_parameter("cone_width_deg", 30)  # degrees
        self.cone_width_deg = self.get_parameter("cone_width_deg").value
        self.declare_parameter("scan_hist", 3)  # counter
        self.scan_hist = self.get_parameter("scan_hist").value
        

    def _setup_publishers(self):
        self.obstacle_in_path = self.create_publisher(
            Bool, "/perception/obstacle_in_path", 10
        )
        # self.obstacle_dist_pub = self.create_publisher(
        #     Float32, "/perception/front_distance", 10
        # )

    def _setup_subscriptions(self):
        """lidar depth sub to LaserScan topic"""
        self.depth_sub = self.create_subscription(
            LaserScan, "/scan", self.cone_slice_callback, 10
        )

    def cone_slice_callback(self, msg):
        """Performs LIDAR cone slicing and publishes whether an obstacle is in the way."""
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return
        
        # Cone Slicing 
        
        center = int(round((0.0 - msg.angle_min) / msg.angle_increment)) % n
        half_window = 5
        # Collect window with wrap-around so we handle scans where 0° lies near
        # the seam between angle_max and angle_min.
        readings = [
            r for i in range(-half_window, half_window + 1)
            for r in (msg.ranges[(center + i) % n],)
            if math.isfinite(r) and r > 0.0
        ]
        theta = (msg.angle_increment * half_window) + msg.angle_min
        avg_dist = sum(readings) / len(readings) if readings else float("inf")
        # Filtering
        
        acc = 0
        obs_msg = Bool()
        if self.distance_threshold > avg_dist:
            acc += 1
        
        if acc >= self.scan_hist:
            obs_msg = True
        elif avg_dist > self.distance_threshold:
            acc = 0
            obs_msg = False
        
        self.obstacle_in_path.publish(obs_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
