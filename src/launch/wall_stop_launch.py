#!/usr/bin/env python3
"""
Unified launch file for wall-stop behavior.

Launches: rplidar_node -> depth_node -> wall_nav_node -> rover_node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # RPLIDAR args
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    serial_baudrate = LaunchConfiguration("serial_baudrate", default="115200")

    # Rover args
    connection_string = LaunchConfiguration("connection_string", default="/dev/ttyACM1")
    baud_rate = LaunchConfiguration("baud_rate", default="115200")

    return LaunchDescription([
        # --- Launch arguments ---
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="USB port for the RPLIDAR",
        ),
        DeclareLaunchArgument(
            "serial_baudrate",
            default_value="115200",
            description="Baud rate for the RPLIDAR",
        ),
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
        # RPLIDAR
        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[{
                "channel_type": "serial",
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
            }],
            output="screen",
        ),

        # Perception – depth from LIDAR
        Node(
            package="perception",
            executable="depth_node",
            name="depth_node",
            output="screen",
        ),

        # Autonomy – wall nav controller
        Node(
            package="autonomy",
            executable="wall_nav_node",
            name="wall_nav_node",
            output="screen",
        ),

        # Rover driver
        Node(
            executable="python3",
            arguments=["-m", "robo_rover.rover_node"],
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
