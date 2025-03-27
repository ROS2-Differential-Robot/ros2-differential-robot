from launch import LaunchDescription
from launch.actions.execute_local import TimerAction
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart

import os


def generate_launch_description():
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params = os.path.join(get_package_share_directory("neu_lidar"), "config", "controllers.yaml")
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', 'joint_broad']
    )
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_params],
    ) 

    delayed_controller_manager = TimerAction(
            period=3.0,
            actions=[controller_manager]
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', str(os.path.join(get_package_share_directory('neu_lidar'), 'model', 'v3_approx_xacro.urdf'))]), value_type=str
                ),
            }],
            output='screen'
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

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('neu_lidar'), 'config', 'simple_env.rviz')],
            output='screen'
        ),

        delayed_controller_manager,

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[controller_spawner]
            )
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('neu_lidar'), 'config', 'ekf-config.yaml'),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            launch_arguments=[
                ('slam_params_file', os.path.join(get_package_share_directory('neu_lidar'), 'config', 'mapper_params_online_async.yaml')),
            ],
        ),

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
