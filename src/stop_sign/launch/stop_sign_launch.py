from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='rs_stream', executable='rs_stream_node',
             name='rs_stream_node', output='screen'),
        Node(package='stop_sign', executable='stop_sign_node',
             name='stop_sign_node', output='screen'),
    ])
