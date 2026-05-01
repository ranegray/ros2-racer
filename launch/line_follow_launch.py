#!/usr/bin/env python3
"""
Launch file for line following behavior with stop-sign obedience.

Pipeline:
    rs_stream → line_detector ──► line_control ──► /cmd_vel_raw ─┐
                              │                                  │
                              └─► stop_sign_node ──/stop_sign/event─┐
                                                                    │
                                            cmd_vel_stop_filter ────┘
                                                  │
                                                  ▼
                                              /cmd_vel ─► rover_node

`line_control` is unchanged — its `cmd_vel` topic is remapped to `cmd_vel_raw`
in the launch wiring. `cmd_vel_stop_filter` adds the DRIVING/BRAKING/STOPPED/
RESUMING state machine on top, modulating `linear.x` only. Steering passes
through untouched.

Pass obey_stops:=false to skip the stop sequence (lap-timing runs):
    ros2 launch ros2-racer line_follow_launch.py obey_stops:=false

When `obey_stops` is false the filter is a pure pass-through: every Twist
field byte-for-byte from `cmd_vel_raw` to `cmd_vel`. Toggle live with:
    ros2 param set /cmd_vel_stop_filter obey_stops false
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
                        "Set false for lap-timing runs (filter becomes pure pass-through).",
        ),

        # RealSense camera stream
        Node(
            package="rs_stream",
            executable="rs_stream_node",
            name="rs_stream_node",
            output="screen",
        ),

        # Line detector — publishes the line follow/turn/lookahead points
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

        # Line following controller — publishes to cmd_vel_raw (remapped)
        Node(
            package="line_control",
            executable="line_control",
            name="line_control",
            output="screen",
            remappings=[("cmd_vel", "cmd_vel_raw")],
        ),

        # cmd_vel filter — applies stop-sign state machine, republishes to cmd_vel
        Node(
            package="stop_sign",
            executable="cmd_vel_stop_filter",
            name="cmd_vel_stop_filter",
            output="screen",
            parameters=[{"obey_stops": obey_stops}],
            respawn=True,
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
