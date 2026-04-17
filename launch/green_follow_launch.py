#!/usr/bin/env python3
"""
Launch file for green paper following behavior.

Launches: rs_stream -> green_vision -> green_control -> safety_monitor
The safety_monitor node owns the rover_node subprocess lifecycle.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    connection_string = LaunchConfiguration('connection_string', default='/dev/ttyACM1')
    baud_rate = LaunchConfiguration('baud_rate', default='115200')

    return LaunchDescription([
        DeclareLaunchArgument(
            'connection_string',
            default_value='/dev/ttyACM1',
            description='MAVLink connection string for the rover',
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for the rover serial connection',
        ),

        # RealSense camera stream
        Node(
            package='rs_stream',
            executable='rs_stream_node',
            name='rs_stream_node',
            output='screen',
        ),

        # Green paper detection
        Node(
            package='green_vision',
            executable='green_vision',
            name='green_vision',
            output='screen',
        ),

        # Green paper following controller
        Node(
            package='green_control',
            executable='green_control',
            name='green_control',
            output='screen',
        ),

        # Safety monitor - owns rover_node subprocess, e-stop, staleness detection
        Node(
            package='safety_monitor',
            executable='safety_monitor_node',
            name='safety_monitor',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'connection_string': connection_string,
                'baud_rate': baud_rate,
                'watched_topics': ['/goal_point'],
                'stale_timeout': 2.0,
            }],
        ),
    ])
