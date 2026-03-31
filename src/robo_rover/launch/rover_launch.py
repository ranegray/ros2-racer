#!/usr/bin/env python3
"""
ROS2 Launch file for ROBO Rover
Launches the rover node with configurable parameters
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    connection_string_arg = DeclareLaunchArgument(
        'connection_string',
        default_value='/dev/ttyACM1',
        description='MAVLink connection string (serial port or UDP/TCP)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial connection'
    )

    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='20.0',
        description='Control command frequency in Hz'
    )
    
    imu_frequency_arg = DeclareLaunchArgument(
        'imu_frequency',
        default_value='20.0',
        description='IMU data publishing frequency in Hz'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the rover node'
    )
    
    # Rover node
    rover_node = Node(
        # package='robo_rover',
        executable='python3',
        arguments=['-m', 'robo_rover.rover_node'],
        name='rover_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'connection_string': LaunchConfiguration('connection_string'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'imu_frequency': LaunchConfiguration('imu_frequency'),
        }],
        remappings=[
            # You can add topic remappings here if needed
            # ('cmd_vel', 'rover/cmd_vel'),
            # ('imu/data', 'rover/imu/data'),
        ]
    )
    
    # Log info about the launch
    log_info = LogInfo(
        msg=[
            'Launching Rover Node with:\n',
            '  Connection: ', LaunchConfiguration('connection_string'), '\n',
            '  Baud Rate: ', LaunchConfiguration('baud_rate'), '\n',
            '  Control Frequency: ', LaunchConfiguration('control_frequency'), ' Hz\n',
            '  IMU Frequency: ', LaunchConfiguration('imu_frequency'), ' Hz\n',
        ]
    )
    
    return LaunchDescription([
        # Arguments
        connection_string_arg,
        baud_rate_arg,
        control_frequency_arg,
        imu_frequency_arg,
        namespace_arg,
        
        # Log launch info
        log_info,
        
        # Nodes
        rover_node,
    ])
