from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('publish_rate', default_value='10.0',
                              description='Scalar telemetry publish rate in Hz'),
        DeclareLaunchArgument('camera_rate', default_value='5.0',
                              description='Camera frame publish rate in Hz'),
        DeclareLaunchArgument('image_quality', default_value='50',
                              description='JPEG compression quality (0-100)'),

        Node(
            package='telemetry',
            executable='telemetry_node',
            name='telemetry_node',
            parameters=[{
                'publish_rate': LaunchConfiguration('publish_rate'),
                'camera_rate': LaunchConfiguration('camera_rate'),
                'image_quality': LaunchConfiguration('image_quality'),
            }],
            output='screen',
        ),
    ])
