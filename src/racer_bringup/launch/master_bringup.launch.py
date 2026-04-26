#!/usr/bin/env python3
"""
Phase 1 master bringup launch file.

Brings up the full hardware and infrastructure stack in a single command:
  1. LIDAR        (rplidar_ros  → /scan)
  2. RealSense    (rs_stream    → /camera/color/image_raw)
  3. Hardware Bridge (robo_rover → /imu/*, /rover/armed, subscribes /cmd_vel)
  4. E-Stop       (TODO: not yet implemented)
  5. Telemetry    (telemetry    → /telemetry/racer)
  6. rosbridge    (rosbridge_server WebSocket on port 9090)

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
        description='Telemetry publish rate in Hz',
    )
    image_quality_arg = DeclareLaunchArgument(
        'image_quality',
        default_value='50',
        description='JPEG compression quality for camera frames (0-100)',
    )
    use_velocity_feedback_arg = DeclareLaunchArgument(
        'use_velocity_feedback',
        default_value='false',
        description=(
            'If true, rover_node closes a PI loop on cmd_vel.linear.x using '
            '/odom_rf2o.twist.linear.x as feedback. If false, falls back to '
            'the legacy open-loop throttle map.'
        ),
    )

    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    rover_port = LaunchConfiguration('rover_port')
    rover_baudrate = LaunchConfiguration('rover_baudrate')
    use_velocity_feedback = LaunchConfiguration('use_velocity_feedback')

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

    # ── 3. Hardware Bridge ───────────────────────────────────────────────────
    rover_node = Node(
        executable='python3',
        arguments=['-m', 'robo_rover.rover_node'],
        name='rover_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'connection_string': rover_port,
            'baud_rate': rover_baudrate,
            'control_frequency': 20.0,
            'imu_frequency': 20.0,
            'use_velocity_feedback': use_velocity_feedback,
            'odom_topic': '/odom_rf2o',
        }],
    )

    # ── 3a. Static TF: base_link -> laser ───────────────────────────────────
    # Required by rf2o (it calls lookupTransform on the first scan; without
    # any TF involving 'laser' in the buffer, it throws and rf2o silently
    # drops every scan thereafter). Identity is fine — we only need the
    # velocity scalar from rf2o, not a precise rigid-body offset.
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='log',
    )

    # ── 3b. LIDAR scan-match odometry (rf2o) ────────────────────────────────
    # Publishes /odom_rf2o with twist.linear.x in m/s. Used as the velocity
    # feedback source for rover_node's closed-loop controller.
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='log',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            # MUST be empty — see comment in our wrapper launch file. The
            # source's default is '/base_pose_ground_truth' which silently
            # disables scan processing.
            'init_pose_from_topic': '',
            # Match the lidar's ~7.6 Hz scan rate; faster just spams warnings.
            'freq': 10.0,
        }],
    )

    # ── 4. E-Stop ────────────────────────────────────────────────────────────
    # TODO: E-Stop node — not yet implemented

    # ── 5. Telemetry Aggregator ──────────────────────────────────────────────
    telemetry_node = Node(
        package='telemetry',
        executable='telemetry_node',
        name='telemetry_node',
        parameters=[{
            'publish_rate': LaunchConfiguration('telemetry_rate'),
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
        image_quality_arg,
        use_velocity_feedback_arg,

        # Startup banner
        LogInfo(msg='[racer_bringup] Starting Phase 1 hardware and infrastructure stack...'),

        # Nodes (sensors first, bridge, then aggregators/infrastructure)
        lidar_node,
        realsense_node,
        static_tf_base_to_laser,
        rf2o_node,
        rover_node,
        telemetry_node,
        rosbridge_launch,

        LogInfo(msg='[racer_bringup] All nodes launched. Car is ready.'),
    ])
