#!/usr/bin/env python3
"""
slam_mapping_launch.py  —  Lap 1: build the map while wall-following.

Nodes launched:
  rplidar_ros   rplidar_node       /scan
  autonomy      odometry_node      /odom + TF odom→base_link
  autonomy      imu_adapter_node   /imu
  slam_toolbox  async_slam_toolbox /map + TF map→odom
  autonomy      wall_follower_node /cmd_vel
  autonomy      path_recorder_node records map→base_link poses to YAML
  autonomy      slam_coordinator   /slam_coordinator/mode
  robo_rover    rover_node         MAVLink driver

Static TFs (YOU MUST MEASURE these on the actual rover):
  odom → base_link  provided by odometry_node at runtime
  base_link → laser  static: adjust x/y/z to lidar mounting position

After Lap 1, save the map:
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '/home/pi/map/track'}}"
Then kill this launch and run slam_racing_launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    connection_string = LaunchConfiguration("connection_string", default="/dev/ttyACM1")
    baud_rate = LaunchConfiguration("baud_rate", default="115200")
    wheelbase = LaunchConfiguration("wheelbase", default="0.25")

    slam_config = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config",
        "slam_toolbox_mapping.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("connection_string", default_value="/dev/ttyACM1"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument(
                "wheelbase",
                default_value="0.165",
                description="Rover wheelbase in metres",
            ),
            # Scan filter — fills narrow max-range gaps (glass/windows) before
            # slam_toolbox and wall_follower see the data.
            Node(
                package="perception",
                executable="scan_filter_node",
                name="scan_filter_node",
                output="screen",
                parameters=[{"max_gap_deg": 30.0}],
            ),
            # RPLIDAR A1 — publishes /scan with frame_id=laser
            # respawn=True: LIDAR stays in scan mode after Ctrl+C; first attempt
            # sends reset but times out, second attempt succeeds automatically.
            Node(
                package="rplidar_ros",
                executable="rplidar_node",
                name="rplidar_node",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": "/dev/ttyUSB0",
                        "serial_baudrate": 115200,
                        "frame_id": "laser",
                        "inverted": False,
                        "angle_compensate": True,
                    }
                ],
            ),
            # Static TF: base_link → laser
            # base_link = rear axle center at ground level.
            # Lidar: 0.05 m ahead of rear axle, centered, 0.19 m above ground.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_laser",
                arguments=[
                    "0.05",
                    "0.0",
                    "0.19",
                    "0.0",
                    "0.0",
                    "0.0",
                    "base_link",
                    "laser",
                ],
                output="screen",
            ),
            # Odometry: integrates cmd_vel → /odom + TF odom→base_link
            Node(
                package="autonomy",
                executable="odometry_node",
                name="odometry_node",
                output="screen",
                parameters=[{"wheelbase": wheelbase}],
            ),
            # IMU adapter: imu/gyro + imu/accel → /imu
            Node(
                package="autonomy",
                executable="imu_adapter_node",
                name="imu_adapter_node",
                output="screen",
            ),
            # slam_toolbox online async mapping
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_config],
            ),
            # Wall follower — Lap 1 driver
            Node(
                package="autonomy",
                executable="wall_follower_node",
                name="wall_follower_node",
                output="screen",
                parameters=[{"speed_override": 0.5}],
            remappings=[('/scan', '/scan_filtered')],
            ),
            # Path recorder — saves map→base_link poses to YAML on shutdown
            Node(
                package="autonomy",
                executable="path_recorder_node",
                name="path_recorder_node",
                output="screen",
            ),
            # Mode coordinator
            Node(
                package="autonomy",
                executable="slam_coordinator_node",
                name="slam_coordinator_node",
                output="screen",
            ),
            # Rover MAVLink driver
            Node(
                package="robo_rover",
                executable="rover_node",
                name="rover_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "connection_string": connection_string,
                        "baud_rate": baud_rate,
                        "control_frequency": 20.0,
                        "imu_frequency": 20.0,
                    }
                ],
            ),
        ]
    )
