from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    rviz2_map = LaunchConfiguration('rviz2_map')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')

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

    return LaunchDescription([
        lidar_port_arg,
        lidar_baudrate_arg,
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('mapping'),
                'launch',
                'online_sync_launch.py'
            ])
        ),
        DeclareLaunchArgument(
            'rviz2_map',
            default_value='rviz_map1'
        ),
        Node(
            package='rviz2',
            namespace=rviz2_map,
            executable='rviz2',
            name='sim'
        ),
        Node(
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
    ])
