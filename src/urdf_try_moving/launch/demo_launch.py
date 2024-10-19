import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'v3_approx_xacro.urdf'
    urdf = os.path.join(
        get_package_share_directory('urdf_try_moving'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        
        Node(
            package='urdf_try_moving',
            executable='state_publisher',
            name='state_publisher',
            output='screen'),
        
    ])

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'urdf_try_moving',
            'urdf_package_path': PathJoinSubstitution(['v3_approx_xacro.urdf'])}.items()
    ))

    return ld

