#!/usr/bin/env python3
"""
Barebones launch file for testing vision pipeline.

Launches: rs_stream -> line_detector
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
    ])
