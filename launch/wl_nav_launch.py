#!/usr/bin/env python3
"""
wl_nav_launch.py - Start the wall+line navigation controller.

This launch file only starts wall_line_nav_node. Run
bringup_launch.py first to start sensors, perception, SLAM,
telemetry, rosbridge, and rover I/O.

Usage:
  ros2 launch racer_bringup bringup_launch.py
  ros2 launch racer_bringup wl_nav_launch.py
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="[racer_bringup] Starting wall_line_nav_node controller."),
        Node(
            package="autonomy",
            executable="wall_line_nav_node",
            name="wall_line_nav_node",
            output="screen",
        ),
    ])
