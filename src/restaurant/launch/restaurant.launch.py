from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    is_sim = LaunchConfiguration("is_sim")
    declare_is_sim = DeclareLaunchArgument(
        'is_sim', default_value='true',
        description='Set to true if running in simulation'
    )

    neu_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('neu_lidar'), 'launch', 'neu_lidar_launch.py')),
        launch_arguments=[('is_sim', is_sim)]
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_test'), 'launch', 'navigation_launch.py')),
        launch_arguments=[('use_sim_time', is_sim)]
    )

    return LaunchDescription([
        declare_is_sim,
        neu_lidar_launch,
        nav2_launch
    ])
