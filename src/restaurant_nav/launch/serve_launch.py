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

    is_sim   = LaunchConfiguration("is_sim")
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

    neu_lidar_dir = get_package_share_directory("neu_lidar")
    mpu6050_dir = get_package_share_directory("mpu6050")
    robot_description_path = os.path.join(neu_lidar_dir, "model", "v3_approx_xacro.urdf")
    controller_params_path = os.path.join(neu_lidar_dir, "config", "controllers.yaml")
    robot_description = ParameterValue(Command(['xacro ', robot_description_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen'
    )

    static_tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )

    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar', 'adam/base_link/gpu_lidar'],
        output='screen'
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar', 'adam/base_link/imu_sensor'],
        output='screen'
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            controller_params_path,
            {"use_sim_time": is_sim}
        ],
    )

    controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', 'joint_broad']
    )

    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager_node]
    )

    lidar_driver_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        parameters=[{
            "serial_port": "/dev/ttyUSB0",
            "frame_id": "lidar",
            "angle_compensate": True
        }]
    )

    mpu6050_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mpu6050_dir, 'launch', 'mpu6050.launch.py')),
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
    declare_map_file_path = DeclareLaunchArgument(
        'map_file_path',
        description='Path to map file'
    )

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
            ("map", "/home/abdo/my_map.yaml")
        ]
    )

    initial_pose_publisher = Node(
        package='nav2_test',
        executable='initial_pose_publisher',
        output='screen',
    )

    return LaunchDescription([
        declare_is_sim,
        declare_map_file_path,
        slam_toolbox,
        robot_state_publisher_node,
        ekf_node,
        # amcl,
        mpu6050_driver_node,
        static_tf_base_link,
        static_tf_lidar,
        static_tf_imu,
        lidar_driver_node,
        delayed_controller_manager,
        controller_spawner_node,
        delayed_nav2,
        goal_sender,
        publisher_table_num,
        initial_pose_publisher
    ])
