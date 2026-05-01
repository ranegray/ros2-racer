#!/usr/bin/env python3
"""
Launch file for line following behavior.

Launches:
    rs_stream -> line_detector -> line_control -> rover_node
              \\-> stop_sign_node (publishes /stop_sign/event for line_control)

Pass obey_stops:=false to ignore stop signs (lap-timing runs):
    ros2 launch ros2-racer line_follow_launch.py obey_stops:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    connection_string = LaunchConfiguration("connection_string", default="/dev/ttyACM1")
    baud_rate = LaunchConfiguration("baud_rate", default="115200")
    obey_stops = LaunchConfiguration("obey_stops", default="true")

    stop_sign_config = os.path.join(
        get_package_share_directory("stop_sign"),
        "config",
        "stop_sign.yaml",
    )

    return LaunchDescription([
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
        DeclareLaunchArgument(
            "obey_stops",
            default_value="true",
            description="Obey stop signs detected by stop_sign_node. "
                        "Set false for lap-timing runs.",
        ),

        # RealSense camera stream
        Node(
            package="rs_stream",
            executable="rs_stream_node",
            name="rs_stream_node",
            output="screen",
        ),

        # Line detector
        Node(
            package="perception",
            executable="line_detector",
            name="line_detector",
            output="screen",
        ),

        # Stop sign detector — publishes /stop_sign/event
        Node(
            package="stop_sign",
            executable="stop_sign_node",
            name="stop_sign_node",
            output="screen",
            parameters=[{"config_path": stop_sign_config}],
        ),

        # Line following controller (subscribes /stop_sign/event)
        Node(
            package="line_control",
            executable="line_control",
            name="line_control",
            output="screen",
            parameters=[{"obey_stops": obey_stops}],
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
