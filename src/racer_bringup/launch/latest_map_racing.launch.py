#!/usr/bin/env python3
"""
latest_map_racing.launch.py - race using the current latest saved map.

Loads /home/pi/map/track in slam_toolbox localization mode, waits until the
map files, recorded path, and map->base_link localization TF are ready, then
starts racing automatically by default.

Usage:
    ros2 launch racer_bringup latest_map_racing.launch.py

Manual confirmation, when auto_start:=false:
    ros2 topic pub --once /slam_coordinator/confirm std_msgs/msg/Empty '{}'
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('racer_bringup')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_localization.yaml')

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
    min_speed_arg = DeclareLaunchArgument('min_speed', default_value='0.30')
    map_base_arg = DeclareLaunchArgument(
        'map_base',
        default_value='/home/pi/map/track',
        description='Saved map basename without extension',
    )
    path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value=os.path.expanduser('~/.ros/recorded_path.yaml'),
        description='Recorded path YAML used by planner and pure pursuit fallback',
    )
    use_planner_arg = DeclareLaunchArgument(
        'use_planner',
        default_value='true',
        description='Start A* path_planner_node; pure pursuit falls back to path_file if no plan is published',
    )
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Automatically switch to racing once map, path, and localization are ready',
    )
    auto_start_delay_arg = DeclareLaunchArgument(
        'auto_start_delay_sec',
        default_value='2.0',
        description='Seconds to wait in READY before auto-starting racing',
    )

    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    rover_port = LaunchConfiguration('rover_port')
    rover_baudrate = LaunchConfiguration('rover_baudrate')
    wheelbase = LaunchConfiguration('wheelbase')
    lookahead = LaunchConfiguration('lookahead')
    speed = LaunchConfiguration('speed')
    min_speed = LaunchConfiguration('min_speed')
    map_base = LaunchConfiguration('map_base')
    path_file = LaunchConfiguration('path_file')
    use_planner = LaunchConfiguration('use_planner')
    auto_start = LaunchConfiguration('auto_start')
    auto_start_delay_sec = LaunchConfiguration('auto_start_delay_sec')

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

    scan_filter_node = Node(
        package='perception',
        executable='scan_filter_node',
        name='scan_filter_node',
        output='screen',
        parameters=[{'max_gap_deg': 30.0}],
    )

    realsense_node = Node(
        package='rs_stream',
        executable='rs_stream_node',
        name='rs_stream_node',
        output='screen',
    )

    depth_node = Node(
        package='perception',
        executable='depth_node',
        name='depth_node',
        output='screen',
    )

    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='log',
    )

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

    imu_adapter_node = Node(
        package='autonomy',
        executable='imu_adapter_node',
        name='imu_adapter_node',
        output='log',
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config, {'map_file_name': map_base}],
        output='screen',
    )

    path_planner_node = Node(
        package='autonomy',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[{
            'path_file': path_file,
        }],
        condition=IfCondition(use_planner),
    )

    pure_pursuit_node = Node(
        package='autonomy',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{
            'lookahead': lookahead,
            'speed': speed,
            'min_speed': min_speed,
            'wheelbase': wheelbase,
            'path_file': path_file,
        }],
    )

    race_gate_node = Node(
        package='autonomy',
        executable='latest_map_race_gate_node',
        name='latest_map_race_gate_node',
        output='screen',
        parameters=[{
            'map_base': map_base,
            'path_file': path_file,
            'require_path': True,
            'auto_start': ParameterValue(auto_start, value_type=bool),
            'auto_start_delay_sec': ParameterValue(auto_start_delay_sec, value_type=float),
        }],
    )

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
        min_speed_arg,
        map_base_arg,
        path_file_arg,
        use_planner_arg,
        auto_start_arg,
        auto_start_delay_arg,

        LogInfo(msg='[latest_map_racing] Loading latest saved map /home/pi/map/track in localization mode'),
        LogInfo(msg='[latest_map_racing] Full stack will boot: lidar, odom, rover, localization, planner, controller, telemetry, rosbridge.'),
        LogInfo(msg='[latest_map_racing] Default behavior: auto-start racing once map/path/localization are ready. Use auto_start:=false for manual confirm.'),

        lidar_node,
        scan_filter_node,
        realsense_node,
        depth_node,
        static_tf_base_to_laser,
        rf2o_node,
        rover_node,
        imu_adapter_node,
        slam_toolbox_node,
        path_planner_node,
        pure_pursuit_node,
        race_gate_node,
        telemetry_node,
        rosbridge_launch,
    ])
