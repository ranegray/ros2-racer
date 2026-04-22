#!/usr/bin/env python3
"""
Full-system bringup for the ROS2 Racer.

Starts the shared stack from ``master_bringup.launch.py`` and adds the
higher-level perception/controller pieces needed to run the full robot.

Perception brought up by default:
  - wall distance        (perception/depth_node via master bringup)
  - line detection       (perception/line_detector)

Controller selector:
  - none   : no controller
  - line   : line_control
  - wall   : wall_nav_node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _controller_is(name: str):
    return IfCondition(
        PythonExpression(["'", LaunchConfiguration('controller'), "' == '", name, "'"])
    )


def generate_launch_description():
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='none',
        description='Controller to start: none, line, or wall',
    )

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
        description='MAVLink serial port for the rover',
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

    master_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('racer_bringup'),
                'launch',
                'master_bringup.launch.py',
            )
        ),
        launch_arguments={
            'lidar_port': LaunchConfiguration('lidar_port'),
            'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
            'rover_port': LaunchConfiguration('rover_port'),
            'rover_baudrate': LaunchConfiguration('rover_baudrate'),
            'telemetry_rate': LaunchConfiguration('telemetry_rate'),
            'camera_rate': LaunchConfiguration('camera_rate'),
            'image_quality': LaunchConfiguration('image_quality'),
        }.items(),
    )

    line_detector = Node(
        package='perception',
        executable='line_detector',
        name='line_detector',
        output='screen',
    )

    line_control = Node(
        package='line_control',
        executable='line_control',
        name='line_control',
        output='screen',
        condition=_controller_is('line'),
    )

    wall_nav = Node(
        package='autonomy',
        executable='wall_nav_node',
        name='wall_nav_node',
        output='screen',
        condition=_controller_is('wall'),
    )

    return LaunchDescription([
        controller_arg,
        lidar_port_arg,
        lidar_baudrate_arg,
        rover_port_arg,
        rover_baudrate_arg,
        telemetry_rate_arg,
        camera_rate_arg,
        image_quality_arg,
        LogInfo(
            msg=[
                '[racer_bringup] Starting full system. controller:=',
                LaunchConfiguration('controller'),
            ]
        ),
        master_bringup,
        line_detector,
        line_control,
        wall_nav,
        LogInfo(
            msg='[racer_bringup] Full system launched. Supported controllers: none, line, wall.'
        ),
    ])
