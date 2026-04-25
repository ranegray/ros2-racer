#!/usr/bin/env python3
"""
odometry_node.py

Dead-reckoning odometry from cmd_vel using Ackermann kinematics.

  θ_dot = v * tan(δ) / L

where:
  v  = linear.x  (m/s, from cmd_vel)
  δ  = steering angle (rad), derived from angular.z = v * tan(δ) / L
       → tan(δ) = angular.z * L / v   → δ = atan(angular.z * L / v)
  L  = wheelbase (m) — tune with WHEELBASE param

Publishes:
  /odom  (nav_msgs/Odometry)
  TF:    odom → base_link

This will drift without wheel encoders, but slam_toolbox's scan matching
will correct most of the accumulated error.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_node")

        self.declare_parameter("wheelbase", 0.165)  # metres — 6.5 in rear-to-front axle
        self.L = self.get_parameter("wheelbase").value

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_t = None
        self._last_v = 0.0
        self._last_w = 0.0  # angular.z from cmd_vel

        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Twist, "cmd_vel", self._cmd_cb, 10)
        # Integrate at 20 Hz regardless of cmd_vel rate
        self.create_timer(0.05, self._integrate)
        self._publish_pose(self.get_clock().now())

        self.get_logger().info(f"Odometry node started (wheelbase={self.L} m)")

    def _cmd_cb(self, msg: Twist):
        self._last_v = msg.linear.x
        self._last_w = msg.angular.z

    def _integrate(self):
        now = self.get_clock().now()
        if self._last_t is None:
            self._last_t = now
            return

        dt = (now - self._last_t).nanoseconds / 1e9
        self._last_t = now

        v = self._last_v
        w = self._last_w

        # Euler integration of Ackermann kinematics
        dx = v * math.cos(self._yaw) * dt
        dy = v * math.sin(self._yaw) * dt
        dth = w * dt

        self._x += dx
        self._y += dy
        self._yaw += dth

        self._publish_pose(now)

    def _publish_pose(self, now):
        stamp = now.to_msg()

        # Publish TF odom → base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        cy, sy = math.cos(self._yaw / 2), math.sin(self._yaw / 2)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self._tf_broadcaster.sendTransform(t)

        # Publish /odom
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        odom.twist.twist.linear.x = self._last_v
        odom.twist.twist.angular.z = self._last_w
        # Covariance: high uncertainty since we have no wheel encoders
        odom.pose.covariance[0] = 0.1
        odom.pose.covariance[7] = 0.1
        odom.pose.covariance[35] = 0.2
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
