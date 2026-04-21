#!/usr/bin/env python3
"""
Unified launch file for SLAM MAPPER.

Launches: rplidar_node -> depth_node -> mapper_node
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # RPLIDAR args
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    serial_baudrate = LaunchConfiguration("serial_baudrate", default="115200")

    # Rover args
    connection_string = LaunchConfiguration("connection_string", default="/dev/ttyACM1")
    baud_rate = LaunchConfiguration("baud_rate", default="115200")

    # Slam_Mapper args
    slam_params_file = LaunchConfiguration('slam_params_file')


    return LaunchDescription([
        # --- Launch arguments ---
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="USB port for the RPLIDAR",
        ),
        DeclareLaunchArgument(
            "serial_baudrate",
            default_value="115200",
            description="Baud rate for the RPLIDAR",
        ),
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
            'slam_params_file',
            default_value=os.path.join(get_package_share_directory("slam_toolbox"),'config', 'mapper_params_online_sync.yaml'),
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
        ),


        # --- Nodes ---
        # RPLIDAR
        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[{
                "channel_type": "serial",
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
            }],
            output="screen",
        ),

        # Perception – depth from LIDAR
        Node(
            package="perception",
            executable="depth_node",
            name="depth_node",
            output="screen",
        ),
        #slam toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
              slam_params_file,
            ]
        ),

        # Rover driver
        Node(
            executable="python3",
            arguments=["-m", "robo_rover.rover_node"],
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
