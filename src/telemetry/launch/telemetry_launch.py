from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('publish_rate', default_value='10.0',
                              description='Telemetry publish rate in Hz'),
        DeclareLaunchArgument('image_quality', default_value='50',
                              description='JPEG compression quality (0-100)'),

        Node(
            package='telemetry',
            executable='telemetry_node',
            name='telemetry_node',
            parameters=[{
                'publish_rate': LaunchConfiguration('publish_rate'),
                'image_quality': LaunchConfiguration('image_quality'),
            }],
            output='screen',
        ),
    ])
