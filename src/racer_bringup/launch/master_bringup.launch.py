#!/usr/bin/env python3
"""
Phase 1 master bringup launch file.

Brings up the shared hardware and infrastructure stack in a single command:
  1. LIDAR           (rplidar_ros  → /scan)
  2. RealSense       (rs_stream    → /camera/*)
  3. Wall perception (depth_node   → /perception/front_distance)
  4. Hardware bridge (robo_rover   → /imu/*, /rover/armed, subscribes /cmd_vel)
  5. Telemetry       (telemetry    → /telemetry/*)
  6. rosbridge       (rosbridge_server WebSocket on port 9090)

Usage:
    ros2 launch racer_bringup master_bringup.launch.py

Override hardware ports as needed:
    ros2 launch racer_bringup master_bringup.launch.py \
        lidar_port:=/dev/ttyUSB0 \
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
    # ── Launch arguments ────────────────────────────────────────────────────
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='USB serial port for the RPLiDAR',
    )
    lidar_baudrate_arg = DeclareLaunchArgument(
        'lidar_baudrate',
        default_value='115200',
        description='Baud rate for the RPLiDAR serial connection',
    )
    rover_port_arg = DeclareLaunchArgument(
        'rover_port',
        default_value='/dev/ttyACM1',
        description='MAVLink serial port for the ArduPilot rover (Pixhawk)',
    )
    rover_baudrate_arg = DeclareLaunchArgument(
        'rover_baudrate',
        default_value='115200',
        description='Baud rate for the rover MAVLink connection',
    )
    telemetry_rate_arg = DeclareLaunchArgument(
        'telemetry_rate',
        default_value='10.0',
        description='Scalar telemetry publish rate in Hz',
    )
    camera_rate_arg = DeclareLaunchArgument(
        'camera_rate',
        default_value='30.0',
        description='Camera frame publish rate in Hz',
    )
    image_quality_arg = DeclareLaunchArgument(
        'image_quality',
        default_value='50',
        description='JPEG compression quality for camera frames (0-100)',
    )

    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    rover_port = LaunchConfiguration('rover_port')
    rover_baudrate = LaunchConfiguration('rover_baudrate')

    # ── 1. LIDAR ─────────────────────────────────────────────────────────────
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_port,
            'serial_baudrate': lidar_baudrate,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen',
    )

    # ── 2. RealSense ─────────────────────────────────────────────────────────
    realsense_node = Node(
        package='rs_stream',
        executable='rs_stream_node',
        name='rs_stream_node',
        output='screen',
    )

    # ── 3. Perception (lidar-derived front distance) ─────────────────────────
    depth_node = Node(
        package='perception',
        executable='depth_node',
        name='depth_node',
        output='screen',
    )

    # ── 4. Hardware Bridge ───────────────────────────────────────────────────
    rover_node = Node(
        package='robo_rover',
        executable='rover_node',
        name='rover_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'connection_string': rover_port,
            'baud_rate': rover_baudrate,
            'control_frequency': 20.0,
            'imu_frequency': 20.0,
        }],
    )

    # ── 5. Telemetry Aggregator ──────────────────────────────────────────────
    telemetry_node = Node(
        package='telemetry',
        executable='telemetry_node',
        name='telemetry_node',
        parameters=[{
            'publish_rate': LaunchConfiguration('telemetry_rate'),
            'camera_rate': LaunchConfiguration('camera_rate'),
            'image_quality': LaunchConfiguration('image_quality'),
        }],
        output='screen',
    )

    # ── 6. rosbridge WebSocket server ────────────────────────────────────────
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml',
            )
        )
    )

    return LaunchDescription([
        # Arguments
        lidar_port_arg,
        lidar_baudrate_arg,
        rover_port_arg,
        rover_baudrate_arg,
        telemetry_rate_arg,
        camera_rate_arg,
        image_quality_arg,

        # Startup banner
        LogInfo(msg='[racer_bringup] Starting shared hardware and infrastructure stack...'),

        # Nodes (sensors first, bridge, then aggregators/infrastructure)
        lidar_node,
        realsense_node,
        depth_node,
        rover_node,
        telemetry_node,
        rosbridge_launch,

        LogInfo(msg='[racer_bringup] Shared stack launched. Select a controller in system_launch.py or publish /cmd_vel manually.'),
    ])
