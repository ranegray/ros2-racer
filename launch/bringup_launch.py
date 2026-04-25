#!/usr/bin/env python3
"""
bringup_launch.py - Bringup for wall+line navigation dependencies.

Starts sensors, SLAM, perception, rover I/O, telemetry, and rosbridge for the
dashboard. It intentionally does not start wall_line_nav_node so the controller
can be launched, stopped, and restarted independently with wl_nav_launch.py.

Nodes launched:
  rplidar_ros   rplidar_node            /scan
  rs_stream     rs_stream_node          /camera/color/image_raw
  perception    depth_node              /perception/front_distance
  perception    line_detector           /line_follow_point, /line_lookahead_point
  tf2_ros       static_transform_publisher  base_link -> laser
  autonomy      odometry_node           /odom + TF odom->base_link
  autonomy      imu_adapter_node        /imu
  slam_toolbox  async_slam_toolbox_node /map + TF map->odom
  robo_rover    rover_node              MAVLink driver
  telemetry     telemetry_node          /telemetry/*
    - republishes /line_detector/debug_image as /telemetry/line_debug
  rosbridge_server  rosbridge WebSocket on :9090

Usage:
  ros2 launch racer_bringup bringup_launch.py
  ros2 launch racer_bringup bringup_launch.py lidar_port:=/dev/ttyUSB0 \
      rover_port:=/dev/ttyACM1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_port = LaunchConfiguration("lidar_port", default="/dev/ttyUSB0")
    lidar_baudrate = LaunchConfiguration("lidar_baudrate", default="115200")
    rover_port = LaunchConfiguration("rover_port", default="/dev/ttyACM1")
    rover_baudrate = LaunchConfiguration("rover_baudrate", default="115200")
    wheelbase = LaunchConfiguration("wheelbase", default="0.165")
    telemetry_rate = LaunchConfiguration("telemetry_rate", default="10.0")
    camera_rate = LaunchConfiguration("camera_rate", default="30.0")
    image_quality = LaunchConfiguration("image_quality", default="50")

    slam_config = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config",
        "slam_toolbox_mapping.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("lidar_baudrate", default_value="115200"),
        DeclareLaunchArgument("rover_port", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("rover_baudrate", default_value="115200"),
        DeclareLaunchArgument(
            "wheelbase",
            default_value="0.165",
            description="Rover wheelbase in metres",
        ),
        DeclareLaunchArgument("telemetry_rate", default_value="10.0"),
        DeclareLaunchArgument("camera_rate", default_value="30.0"),
        DeclareLaunchArgument("image_quality", default_value="50"),

        LogInfo(
            msg="[racer_bringup] wall_line_nav bringup: sensors + SLAM + dashboard."
        ),

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

        Node(
            package="rs_stream",
            executable="rs_stream_node",
            name="rs_stream_node",
            output="screen",
        ),

        Node(
            package="perception",
            executable="depth_node",
            name="depth_node",
            output="screen",
        ),

        Node(
            package="perception",
            executable="line_detector",
            name="line_detector",
            output="screen",
        ),

        # base_link = rear axle center at ground; lidar mount: 0.05 m forward,
        # 0.19 m up. Must match slam_mapping_launch.py exactly.
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

        Node(
            package="autonomy",
            executable="odometry_node",
            name="odometry_node",
            output="screen",
            parameters=[{"wheelbase": wheelbase}],
        ),

        Node(
            package="autonomy",
            executable="imu_adapter_node",
            name="imu_adapter_node",
            output="screen",
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_config],
        ),

        Node(
            package="robo_rover",
            executable="rover_node",
            name="rover_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "connection_string": rover_port,
                "baud_rate": rover_baudrate,
                "control_frequency": 20.0,
                "imu_frequency": 20.0,
            }],
        ),

        Node(
            package="telemetry",
            executable="telemetry_node",
            name="telemetry_node",
            output="screen",
            parameters=[{
                "publish_rate": telemetry_rate,
                "camera_rate": camera_rate,
                "image_quality": image_quality,
            }],
        ),

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
