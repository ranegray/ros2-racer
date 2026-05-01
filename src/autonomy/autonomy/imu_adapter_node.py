#!/usr/bin/env python3
"""
imu_adapter_node.py

Converts rover_node's imu/gyro + imu/accel (geometry_msgs/Vector3) into a
sensor_msgs/Imu message that slam_toolbox can consume.

Orientation is left as zero with a "unknown" covariance (-1 diagonal) since
ArduPilot's SCALED_IMU message does not include orientation.  slam_toolbox
can still use the gyro for yaw-rate integration.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

# Match rover_node's BEST_EFFORT sensor QoS exactly
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


class ImuAdapterNode(Node):
    def __init__(self):
        super().__init__("imu_adapter_node")

        self._latest_gyro: Vector3 | None = None
        self._latest_accel: Vector3 | None = None

        self.create_subscription(Vector3, "imu/gyro", self._gyro_cb, _SENSOR_QOS)
        self.create_subscription(Vector3, "imu/accel", self._accel_cb, _SENSOR_QOS)
        self._pub = self.create_publisher(Imu, "/imu", 10)

        self.get_logger().info("IMU adapter node started")

    def _gyro_cb(self, msg: Vector3):
        self._latest_gyro = msg
        self._maybe_publish()

    def _accel_cb(self, msg: Vector3):
        self._latest_accel = msg
        self._maybe_publish()

    def _maybe_publish(self):
        if self._latest_gyro is None or self._latest_accel is None:
            return

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "base_link"

        # Orientation unknown — slam_toolbox ignores it when covariance[0] == -1
        imu.orientation_covariance[0] = -1.0

        imu.angular_velocity.x = self._latest_gyro.x
        imu.angular_velocity.y = self._latest_gyro.y
        imu.angular_velocity.z = self._latest_gyro.z
        # Low-noise MPU-6000 on Pixhawk 4 Mini; conservative covariance
        for i in (0, 4, 8):
            imu.angular_velocity_covariance[i] = 0.01

        imu.linear_acceleration.x = self._latest_accel.x
        imu.linear_acceleration.y = self._latest_accel.y
        imu.linear_acceleration.z = self._latest_accel.z
        for i in (0, 4, 8):
            imu.linear_acceleration_covariance[i] = 0.1

        self._pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = ImuAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
