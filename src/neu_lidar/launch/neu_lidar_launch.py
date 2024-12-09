from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments=[('gz_args', "-r " + os.path.join(get_package_share_directory('neu_lidar'), 'model', 'world', 'walled_world.sdf'))]
            #launch_arguments=[('gz_args', "-r empty.sdf")]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': os.path.join(get_package_share_directory('neu_lidar'), 'config', 'gz_bridge_config.yaml'),
            }],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', str(os.path.join(get_package_share_directory('neu_lidar'), 'model', 'v3_approx_xacro.urdf'))]), value_type=str
                ),
                'use_sim_time': True,
            }],
            output='screen'
        ),

        Node(
           package='ros_gz_sim',
           executable='create',
           arguments=['-topic', 'robot_description'],
           output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_cont', 'joint_broad']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar', 'adam/base_link/gpu_lidar'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar', 'adam/base_link/imu_sensor'],
            output='screen'
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     arguments=['-d', os.path.join(get_package_share_directory('neu_lidar'), 'config', 'slam_mapping.rviz')],
        #     output='screen'
        # ),

        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[
        #         os.path.join(get_package_share_directory('neu_lidar'), 'config', 'ekf-config.yaml'),
        #         {'use_sim_time': True}
        #     ],
        # ),

        # IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(
        #                 os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        #         launch_arguments=[
        #                 ('use_sim_time', 'true'),
        #                 ('slam_params_file', os.path.join(get_package_share_directory('neu_lidar'), 'config', 'mapper_params_online_async.yaml')),
        #             ]
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #             os.path.join(get_package_share_directory('nav2_test'), 'launch', 'nav2.launch.py'))
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #             os.path.join(get_package_share_directory('nav2_test'), 'launch', 'slam.launch.py'))
        # ),

        Node(
            package='neu_lidar',
            executable='joystick_twist',
            output='screen',
        ),

        Node(
            package='joy',
            executable='joy_node',
            output='screen',
        ),
    ])
