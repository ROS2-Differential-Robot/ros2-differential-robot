from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    lifecycle_nodes = ['map_server']
    return LaunchDescription([

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[{"yaml_filename": "/home/ali/ros2-differential-robot/src/neu_lidar/maps/map.yaml"}],
            output='screen',
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
        ),
    ])
