from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'amr_mtt'
    file_subpath = 'urdf/amr_mtt.xacro'
    xacro_path = os.path.join(get_package_share_directory(pkg_name), file_subpath)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_description_content = Command(['xacro ', xacro_path])
    robot_description = {'robot_description': robot_description_content}

    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    node_jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'views_robot.rviz'
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        node_rsp,
        node_jsp,
        node_rviz
    ])
