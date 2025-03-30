from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.environ.get('GZ_SIM_RESOURCE_PATH', '') + f":{os.path.join(get_package_share_directory('restaurant'), 'models')}"

    is_sim = LaunchConfiguration("is_sim")

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
        parameters=[{'robot_description': robot_description}, controller_params, {"use_sim_time": is_sim}],
    )

    delayed_controller_manager = TimerAction(
            period=3.0,
            actions=[controller_manager]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'is_sim', default_value='true',
            description='Set to true if running in simulation'
        ),

        # IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_test'), 'launch', 'navigation_launch.py')),
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments=[('gz_args', "-r " + os.path.join(get_package_share_directory('restaurant'), 'world', 'restaurant.sdf'))],
            condition=IfCondition(is_sim)
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
                    Command(['xacro ', str(os.path.join(get_package_share_directory('neu_lidar'), 'model', 'v3_approx_xacro.urdf')),  " is_sim:=", is_sim]), value_type=str
                ),
                'use_sim_time': is_sim,
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
            arguments=['-d', os.path.join(get_package_share_directory('restaurant_nav'), 'config', 'map_created.rviz')],
            output='screen'
        ),

        GroupAction(
            actions=[
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
                        {'use_sim_time': False}
                    ],
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
                    launch_arguments=[
                        ('use_sim_time', 'false'),
                        ('slam_params_file', os.path.join(get_package_share_directory('restaurant_nav'), 'config', 'mapper_params_online_async.yaml')),
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

                Node(
                    package="rplidar_ros",
                    executable="rplidar_composition",
                    parameters=[{
                        "serial_port": "/dev/ttyUSB0",
                         "frame_id": "lidar",
                         "angle_compensate": True
                    }]
                ),

                Node(
                    package='restaurant_nav',
                    executable='pose_recorder',
                ),
            ],
            condition=UnlessCondition(is_sim)
        ),

        TimerAction(
            period=35.0,  # Timeout in seconds
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=['-z', '2.0', '-topic', 'robot_description'],
                    output='screen'
                ),

                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_cont', 'joint_broad']
                ),

                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[
                        os.path.join(get_package_share_directory('neu_lidar'), 'config', 'ekf-config.yaml'),
                        {'use_sim_time': True}
                    ],
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
                    launch_arguments=[
                        ('use_sim_time', 'true'),
                        ('slam_params_file', os.path.join(get_package_share_directory('restaurant_nav'), 'config', 'mapper_params_online_async.yaml')),
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

                Node(
                    package='restaurant_nav',
                    executable='pose_recorder',
                ),
            ],
            condition=IfCondition(is_sim)
        ),
    ])

