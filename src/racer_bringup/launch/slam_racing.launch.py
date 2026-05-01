#!/usr/bin/env python3
"""
slam_racing.launch.py — Race on a previously-saved map.

Loads /home/pi/map/track.{pgm,yaml} (symlinks point at the latest archive)
in slam_toolbox localization mode and runs pure_pursuit on the saved path
(~/.ros/recorded_path.yaml, also a symlink to the latest).

Mirrors master_bringup's hardware/telemetry stack but:
  - slam_toolbox runs in LOCALIZATION mode (frozen map, never modified)
  - mapping-only nodes are not started: wall_nav, path_recorder, path_planner,
    slam_coordinator

Usage:
    ros2 launch racer_bringup slam_racing.launch.py

After ~5s (let localization converge), trigger pure pursuit:
    ros2 topic pub --once /slam_coordinator/mode std_msgs/msg/String "data: racing"

Optional overrides:
    ros2 launch racer_bringup slam_racing.launch.py speed:=0.5 lookahead:=0.6
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('racer_bringup')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_localization.yaml')

    # ── Launch arguments ────────────────────────────────────────────────────
    lidar_port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0')
    lidar_baudrate_arg = DeclareLaunchArgument('lidar_baudrate', default_value='115200')
    rover_port_arg = DeclareLaunchArgument('rover_port', default_value='/dev/ttyACM1')
    rover_baudrate_arg = DeclareLaunchArgument('rover_baudrate', default_value='115200')
    telemetry_rate_arg = DeclareLaunchArgument('telemetry_rate', default_value='10.0')
    camera_rate_arg = DeclareLaunchArgument('camera_rate', default_value='30.0')
    image_quality_arg = DeclareLaunchArgument('image_quality', default_value='50')
    wheelbase_arg = DeclareLaunchArgument('wheelbase', default_value='0.165')
    lookahead_arg = DeclareLaunchArgument('lookahead', default_value='0.5')
    speed_arg = DeclareLaunchArgument('speed', default_value='0.35')
    path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value=os.path.expanduser('~/.ros/recorded_path.yaml'),
        description='Recorded path YAML (symlink to latest archive by default)',
    )

    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    rover_port = LaunchConfiguration('rover_port')
    rover_baudrate = LaunchConfiguration('rover_baudrate')
    wheelbase = LaunchConfiguration('wheelbase')
    lookahead = LaunchConfiguration('lookahead')
    speed = LaunchConfiguration('speed')
    path_file = LaunchConfiguration('path_file')

    # ── 1. LIDAR ────────────────────────────────────────────────────────────
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

    # ── 2. Scan filter (republishes /scan → /scan_filtered) ─────────────────
    scan_filter_node = Node(
        package='perception',
        executable='scan_filter_node',
        name='scan_filter_node',
        output='screen',
        parameters=[{'max_gap_deg': 30.0}],
    )

    # ── 3. RealSense (camera for telemetry/dashboard) ───────────────────────
    realsense_node = Node(
        package='rs_stream',
        executable='rs_stream_node',
        name='rs_stream_node',
        output='screen',
    )

    # ── 4. Lidar-derived front distance ─────────────────────────────────────
    depth_node = Node(
        package='perception',
        executable='depth_node',
        name='depth_node',
        output='screen',
    )

    # ── 5. Static TF base_link → laser (must match mapping launch) ──────────
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='log',
    )

    # ── 6. rf2o laser odometry (publishes odom → base_link TF) ──────────────
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
        }],
        ros_arguments=['--log-level', 'rf2o_laser_odometry:=FATAL'],
        output='log',
    )

    # ── 7. Rover MAVLink driver ─────────────────────────────────────────────
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
            'use_velocity_feedback': True,
            'kp_speed': 120.0,
            'ki_speed': 60.0,
        }],
    )

    # ── 8. IMU adapter ──────────────────────────────────────────────────────
    imu_adapter_node = Node(
        package='autonomy',
        executable='imu_adapter_node',
        name='imu_adapter_node',
        output='log',
    )

    # ── 9. slam_toolbox in LOCALIZATION mode (loads frozen saved map) ───────
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config],
        output='screen',
    )

    # ── 10. Pure pursuit path follower ──────────────────────────────────────
    pure_pursuit_node = Node(
        package='autonomy',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{
            'lookahead': lookahead,
            'speed': speed,
            'wheelbase': wheelbase,
            'path_file': path_file,
        }],
    )

    # ── 11. Telemetry aggregator (for dashboard) ────────────────────────────
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

    # ── 12. rosbridge WebSocket (dashboard) ─────────────────────────────────
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
        lidar_port_arg,
        lidar_baudrate_arg,
        rover_port_arg,
        rover_baudrate_arg,
        telemetry_rate_arg,
        camera_rate_arg,
        image_quality_arg,
        wheelbase_arg,
        lookahead_arg,
        speed_arg,
        path_file_arg,

        LogInfo(msg='[slam_racing] Loading saved map at /home/pi/map/track.{pgm,yaml} (symlink → latest archive)'),
        LogInfo(msg='[slam_racing] After localization converges (~5s), run:'),
        LogInfo(msg='[slam_racing]   ros2 topic pub --once /slam_coordinator/mode std_msgs/msg/String "data: racing"'),

        lidar_node,
        scan_filter_node,
        realsense_node,
        depth_node,
        static_tf_base_to_laser,
        rf2o_node,
        rover_node,
        imu_adapter_node,
        slam_toolbox_node,
        pure_pursuit_node,
        telemetry_node,
        rosbridge_launch,
    ])
