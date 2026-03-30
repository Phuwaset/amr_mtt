"""
urdf_tree.launch.py
-------------------
Launch urdf_tree node together with robot_state_publisher so that
robot_description is available even without a full simulation.

Usage:
  ros2 launch tf2_tools urdf_tree.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    xacro_file = PathJoinSubstitution([
        FindPackageShare('amr_mtt_bot'), 'urdf', 'amr_mtt.xacro'
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' camera_enabled:=true',
        ' stereo_camera_enabled:=true',
        ' two_d_lidar_enabled:=true',
        ' sim_ign:=true',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    urdf_tree_node = Node(
        package='tf2_tools',
        executable='urdf_tree',
        name='urdf_tree',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        urdf_tree_node,
    ])
