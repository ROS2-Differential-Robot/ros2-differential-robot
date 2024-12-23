from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value="slam_mapping.rviz",
        description="Name of RViz config file",
    )

    neu_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("neu_lidar"),
                "launch",
                "neu_lidar_launch.py",
            )
        ),
        launch_arguments={"rviz_config": LaunchConfiguration("rviz_config")}.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_test"),
                "launch",
                "navigation_launch.py",
            )
        ),
    )

    return LaunchDescription([rviz_config_arg, neu_lidar_launch, navigation_launch])
