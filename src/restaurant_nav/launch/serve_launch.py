from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from nav2_common.launch import RewrittenYaml


from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    is_sim = LaunchConfiguration("is_sim")
    declare_is_sim = DeclareLaunchArgument(
        'is_sim', default_value='true',
        description='Set to true if running in simulation'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_test'), 'launch', 'navigation_launch.py')),
        launch_arguments=[('use_sim_time', is_sim)]
    )

    delayed_nav2 = TimerAction(
        period=7.0,
        actions=[nav2_launch]
    )

    goal_sender = Node(
        package='restaurant_nav',
        executable='goal_sender',
    )

    publisher_table_num = Node(
        package='restaurant_nav',
        executable='publisher_table_num',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('neu_lidar'), 'config', 'ekf-config.yaml'),
            {'use_sim_time': False}
        ],
    )

    map_file_path = LaunchConfiguration("map_file_path")
    slam_params = os.path.join(get_package_share_directory("restaurant_nav"), "config", "mapper_params_online_async.yaml")
    
    configured_params = RewrittenYaml(
        source_file=slam_params, root_key="", param_rewrites={"map_file_name": map_file_path}, convert_types=True
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('slam_params_file', configured_params),
        ],
    )

    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_test'), 'launch', 'localization_launch.py')),
        launch_arguments=[
            ("use_sim_time", "false"),
            ("map", get_package_share_directory("nav2_test") + "/maps/ourPlayground.yaml")
        ]
    )

    initial_pose_publisher = Node(
        package='nav2_test',
        executable='initial_pose_publisher',
        output='screen',
    )

    return LaunchDescription([
        declare_is_sim,
        slam_toolbox,
        ekf_node,
        # amcl,
        delayed_nav2,
        goal_sender,
        publisher_table_num,
        initial_pose_publisher
    ])
