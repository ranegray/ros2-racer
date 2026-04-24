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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    connection_string = LaunchConfiguration("connection_string", default="/dev/ttyACM1")
    baud_rate = LaunchConfiguration("baud_rate", default="115200")
    wheelbase = LaunchConfiguration("wheelbase", default="0.25")
    lidar_port = LaunchConfiguration("lidar_port", default="/dev/ttyUSB0")
    lidar_baudrate = LaunchConfiguration("lidar_baudrate", default="115200")
    telemetry_rate = LaunchConfiguration("telemetry_rate", default="10.0")
    camera_rate = LaunchConfiguration("camera_rate", default="10.0")
    image_quality = LaunchConfiguration("image_quality", default="50")

    slam_config = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "slam_toolbox_mapping.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("connection_string", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("baud_rate", default_value="115200"),
        DeclareLaunchArgument("wheelbase", default_value="0.165",
                              description="Rover wheelbase in metres"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("lidar_baudrate", default_value="115200"),
        DeclareLaunchArgument("telemetry_rate", default_value="10.0"),
        DeclareLaunchArgument("camera_rate", default_value="10.0"),
        DeclareLaunchArgument("image_quality", default_value="50"),

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
            parameters=[{
                "channel_type": "serial",
                "serial_port": lidar_port,
                "serial_baudrate": lidar_baudrate,
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
            }],
        ),

        # Camera stream — keeps the dashboard camera panel alive during mapping.
        Node(
            package="rs_stream",
            executable="rs_stream_node",
            name="rs_stream_node",
            output="screen",
        ),

        # Front-distance perception for telemetry tiles during mapping.
        Node(
            package="perception",
            executable="depth_node",
            name="depth_node",
            output="screen",
        ),

        # Static TF: base_link → laser
        # base_link = rear axle center at ground level.
        # Lidar: 0.05 m ahead of rear axle, centered, 0.19 m above ground.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_laser",
            arguments=["0.05", "0.0", "0.19", "0.0", "0.0", "0.0",
                       "base_link", "laser"],
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
            parameters=[{
                "connection_string": connection_string,
                "baud_rate": baud_rate,
                "control_frequency": 20.0,
                "imu_frequency": 20.0,
            }],
        ),

        # Telemetry aggregator for the dashboard's existing panels.
        Node(
            package="telemetry",
            executable="telemetry_node",
            name="telemetry_node",
            parameters=[{
                "publish_rate": telemetry_rate,
                "camera_rate": camera_rate,
                "image_quality": image_quality,
            }],
            output="screen",
        ),

        # rosbridge exposes /telemetry/*, /scan, and /map to the React dashboard.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("rosbridge_server"),
                    "launch",
                    "rosbridge_websocket_launch.xml",
                )
            )
        ),
    ])
