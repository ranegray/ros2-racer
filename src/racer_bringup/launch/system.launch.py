#!/usr/bin/env python3
"""
Full-system bringup for the ROS2 Racer.

Starts the shared stack from ``master_bringup.launch.py`` and adds the
higher-level perception/controller pieces needed to run the full robot.

Perception brought up by default:
  - wall distance        (perception/depth_node via master bringup)
  - line detection       (perception/line_detector)

Controller selector:
  - line   : line_control
  - wall   : wall_nav_node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _default_dashboard_dir():
    """Best-effort guess at <repo>/telemetry-dashboard."""
    try:
        launch_dir = os.path.dirname(os.path.realpath(__file__))
        repo_root = os.path.abspath(os.path.join(launch_dir, '..', '..', '..', '..'))
        candidate = os.path.join(repo_root, 'telemetry-dashboard')
        if os.path.isdir(candidate):
            return candidate
    except Exception:
        pass
    return os.path.expanduser('~/ros2-racer/telemetry-dashboard')


def _controller_is(name: str):
    return IfCondition(
        PythonExpression(["'", LaunchConfiguration('line'), "' == '", name, "'"])
    )


def _bool_expr(*parts: str):
    expr = []
    for part in parts:
        expr.append(part)
    return IfCondition(PythonExpression(expr))


def generate_launch_description():
    line_arg = DeclareLaunchArgument(
        'line',
        default_value='line',
        description='Controller to start: line or wall',
    )
    dashboard_arg = DeclareLaunchArgument(
        'dashboard',
        default_value='true',
        description='If true, bring up the telemetry dashboard docker stack',
    )
    dashboard_dir_arg = DeclareLaunchArgument(
        'dashboard_dir',
        default_value=_default_dashboard_dir(),
        description='Directory containing telemetry-dashboard/docker-compose.yml',
    )
    rebuild_dashboard_arg = DeclareLaunchArgument(
        'rebuild_dashboard',
        default_value='false',
        description='If true, run `docker compose up --build` for the dashboard',
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

    dashboard_dir = LaunchConfiguration('dashboard_dir')

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

    dashboard_proc = ExecuteProcess(
        cmd=['docker', 'compose', 'up'],
        cwd=dashboard_dir,
        output='screen',
        shell=False,
        condition=_bool_expr(
            "'", LaunchConfiguration('dashboard'), "' == 'true' and ",
            "'", LaunchConfiguration('rebuild_dashboard'), "' != 'true'",
        ),
    )

    dashboard_proc_build = ExecuteProcess(
        cmd=['docker', 'compose', 'up', '--build'],
        cwd=dashboard_dir,
        output='screen',
        shell=False,
        condition=_bool_expr(
            "'", LaunchConfiguration('dashboard'), "' == 'true' and ",
            "'", LaunchConfiguration('rebuild_dashboard'), "' == 'true'",
        ),
    )

    return LaunchDescription([
        line_arg,
        dashboard_arg,
        dashboard_dir_arg,
        rebuild_dashboard_arg,
        lidar_port_arg,
        lidar_baudrate_arg,
        rover_port_arg,
        rover_baudrate_arg,
        telemetry_rate_arg,
        camera_rate_arg,
        image_quality_arg,

        LogInfo(msg=['[racer_bringup] Starting full system. line:=', LaunchConfiguration('line')]),

        master_bringup,
        line_detector,
        line_control,
        wall_nav,
        dashboard_proc,
        dashboard_proc_build,

        LogInfo(
            msg='[racer_bringup] Full system launched. Supported controllers: line, wall.'
        ),
    ])
