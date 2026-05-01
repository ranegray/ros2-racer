"""
Line-follower stack with stop-sign obedience.

Launches:
  rs_stream_node      — RealSense color stream
  green_vision        — line / paper detector → /goal_point
  stop_sign_node      — stop sign detector → /stop_sign/event
  green_control_node  — controller; subscribes /goal_point and /stop_sign/event,
                        publishes /cmd_vel

Lap-timing usage (no stops):
    ros2 launch green_control line_follower_launch.py obey_stops:=false

Runtime toggle:
    ros2 param set /green_control_node obey_stops false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    obey_stops_arg = DeclareLaunchArgument(
        'obey_stops',
        default_value='true',
        description='Obey stop signs detected by stop_sign_node. Set false for lap-timing runs.',
    )

    line_follower_config_path = os.path.join(
        get_package_share_directory('green_control'),
        'config',
        'green_control.yaml',
    )
    stop_sign_config_path = os.path.join(
        get_package_share_directory('stop_sign'),
        'config',
        'stop_sign.yaml',
    )

    rs_stream_node = Node(
        package='rs_stream', executable='rs_stream_node',
        name='rs_stream_node', output='screen',
    )

    green_vision_node = Node(
        package='green_vision', executable='green_vision',
        name='green_vision', output='screen',
    )

    stop_sign_node = Node(
        package='stop_sign', executable='stop_sign_node',
        name='stop_sign_node', output='screen',
        parameters=[{'config_path': stop_sign_config_path}],
    )

    green_control_node = Node(
        package='green_control', executable='green_control',
        name='green_control_node', output='screen',
        parameters=[{
            'config_path': line_follower_config_path,
            'obey_stops': LaunchConfiguration('obey_stops'),
        }],
    )

    return LaunchDescription([
        obey_stops_arg,
        rs_stream_node,
        green_vision_node,
        stop_sign_node,
        green_control_node,
    ])
