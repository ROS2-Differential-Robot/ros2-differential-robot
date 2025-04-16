from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Set model path for Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.environ.get('GZ_SIM_RESOURCE_PATH', '') + f":{os.path.join(get_package_share_directory('restaurant'), 'models')}"

    # === Launch Configurations ===
    is_sim = LaunchConfiguration("is_sim")

    # === Declare launch argument for simulation ===
    declare_is_sim = DeclareLaunchArgument(
        'is_sim', default_value='true', description='Set to true if running in simulation'
    )

    # === Paths ===
    neu_lidar_dir = get_package_share_directory("neu_lidar")
    restaurant_dir = get_package_share_directory("restaurant")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")

    robot_description_path = os.path.join(neu_lidar_dir, "model", "v3_approx_xacro.urdf")
    controller_params_path = os.path.join(neu_lidar_dir, "config", "controllers.yaml")
    ekf_config_path = os.path.join(neu_lidar_dir, "config", "ekf-config.yaml")
    rviz_config_path = os.path.join(neu_lidar_dir, "config", "simple_env.rviz")
    slam_params_path = os.path.join(neu_lidar_dir, "config", "mapper_params_online_async.yaml")
    gz_bridge_config_path = os.path.join(neu_lidar_dir, "config", "gz_bridge_config.yaml")
    sdf_world_path = os.path.join(restaurant_dir, "world", "restaurant.sdf")

    # === Shared robot description ===
    robot_description = ParameterValue(Command(['xacro ', robot_description_path]), value_type=str)

    # === Nodes ===
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(is_sim)
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', f"-r {sdf_world_path}")],
        condition=IfCondition(is_sim)
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gz_bridge_config_path}],
        output='screen',
        condition=IfCondition(is_sim)
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

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': is_sim}]
    )

    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments=[
            ('use_sim_time', is_sim),
            ('slam_params_file', slam_params_path),
        ],
    )

    joystick_twist_node = Node(
        package='neu_lidar',
        executable='joystick_twist',
        output='screen',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
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

    # === Group for real hardware ===
    hw_group = GroupAction(
        actions=[
            delayed_controller_manager,
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=controller_manager_node,
                    on_start=[controller_spawner_node]
                )
            ),
            ekf_node,
            slam_toolbox_node,
            joystick_twist_node,
            lidar_driver_node,
        ],
        condition=UnlessCondition(is_sim)
    )

    # === Delayed spawn for simulation ===
    sim_delayed_actions = TimerAction(
        period=35.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-z', '2.0', '-topic', 'robot_description'],
                output='screen'
            ),
            controller_spawner_node,
            ekf_node,
            slam_toolbox_node,
            joystick_twist_node,
            joy_node,
        ],
        condition=IfCondition(is_sim)
    )

    # === Final LaunchDescription ===
    return LaunchDescription([
        declare_is_sim,
        gz_sim,
        gz_bridge_node,
        robot_state_publisher_node,
        static_tf_base_link,
        static_tf_lidar,
        static_tf_imu,
        rviz_node,
        hw_group,
        sim_delayed_actions,
    ])
