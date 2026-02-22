import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gripper_pkg = get_package_share_directory('amr_mtt_Gripper')

    # Spawn the gripper controller via controller_manager
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'robotiq_gripper_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen',
    )

    return LaunchDescription([
        gripper_controller_spawner
    ])
