#!/usr/bin/env python3
"""
slam_racing_launch.py  —  Lap 2+: localize on frozen map and run pure pursuit.

Prerequisites:
  1. Lap 1 complete and map saved to /home/pi/map/track
     (update map_file_name in config/slam_toolbox_localization.yaml)
  2. Path recorded to ~/.ros/recorded_path.yaml

Nodes launched:
  rplidar_ros   rplidar_node       /scan
  autonomy      odometry_node      /odom + TF odom→base_link
  autonomy      imu_adapter_node   /imu
  slam_toolbox  localization_node  TF map→odom only (map frozen)
  autonomy      pure_pursuit_node  /cmd_vel
  robo_rover    rover_node         MAVLink driver
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    connection_string = LaunchConfiguration("connection_string", default="/dev/ttyACM1")
    baud_rate = LaunchConfiguration("baud_rate", default="115200")
    wheelbase = LaunchConfiguration("wheelbase", default="0.25")
    lookahead = LaunchConfiguration("lookahead", default="0.5")
    speed = LaunchConfiguration("speed", default="0.35")

    slam_config = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "slam_toolbox_localization.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("connection_string", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("baud_rate", default_value="115200"),
        DeclareLaunchArgument("wheelbase", default_value="0.165"),
        DeclareLaunchArgument("lookahead", default_value="0.5",
                              description="Pure pursuit lookahead distance (m)"),
        DeclareLaunchArgument("speed", default_value="0.35",
                              description="Racing speed (m/s)"),

        # RPLIDAR A1
        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            output="screen",
            parameters=[{
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 115200,
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
            }],
        ),

        # Static TF: base_link → laser (must match mapping launch exactly)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_laser",
            arguments=["0.05", "0.0", "0.19", "0.0", "0.0", "0.0",
                       "base_link", "laser"],
            output="screen",
        ),

        # Odometry
        Node(
            package="autonomy",
            executable="odometry_node",
            name="odometry_node",
            output="screen",
            parameters=[{"wheelbase": wheelbase}],
        ),

        # IMU adapter
        Node(
            package="autonomy",
            executable="imu_adapter_node",
            name="imu_adapter_node",
            output="screen",
        ),

        # slam_toolbox localization — map is frozen, provides map→odom TF only
        Node(
            package="slam_toolbox",
            executable="localization_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_config],
        ),

        # Pure pursuit path follower
        Node(
            package="autonomy",
            executable="pure_pursuit_node",
            name="pure_pursuit_node",
            output="screen",
            parameters=[{
                "lookahead": lookahead,
                "speed": speed,
                "wheelbase": wheelbase,
            }],
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
    ])
