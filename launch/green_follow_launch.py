#!/usr/bin/env python3
"""
Launch file for green paper following behavior.

Launches: rs_stream -> green_vision -> green_control -> rover_node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Rover args
    connection_string = LaunchConfiguration("connection_string", default="/dev/ttyACM1")
    baud_rate = LaunchConfiguration("baud_rate", default="115200")

    return LaunchDescription([
        # --- Launch arguments ---
        DeclareLaunchArgument(
            "connection_string",
            default_value="/dev/ttyACM1",
            description="MAVLink connection string for the rover",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Baud rate for the rover serial connection",
        ),

        # --- Nodes ---
        # RealSense camera stream (aligned depth + color + camera_info)
        Node(
            package="rs_stream",
            executable="rs_stream_node",
            name="rs_stream_node",
            output="screen",
        ),

        # Green paper detection
        Node(
            package="green_vision",
            executable="green_vision",
            name="green_vision",
            output="screen",
        ),

        # Green paper following controller
        Node(
            package="green_control",
            executable="green_control",
            name="green_control",
            output="screen",
        ),

        # Rover driver
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
