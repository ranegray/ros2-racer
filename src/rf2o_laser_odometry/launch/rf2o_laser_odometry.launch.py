import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

            Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom_rf2o',
                    # Don't publish TF — we only want the velocity estimate
                    # for closing the loop in rover_node, not a TF tree fight.
                    'publish_tf' : False,
                    'base_frame_id' : 'base_link',
                    'odom_frame_id' : 'odom',
                    # MUST stay empty: the source's default is
                    # '/base_pose_ground_truth', which nothing publishes; that
                    # leaves GT_pose_initialized=false and rf2o silently drops
                    # every scan despite a successful subscription.
                    'init_pose_from_topic' : '',
                    # Match lidar's actual rate (~7.6 Hz on this rover);
                    # running the timer faster just spams "Waiting for scans".
                    'freq' : 10.0}],
            ),
    ])
