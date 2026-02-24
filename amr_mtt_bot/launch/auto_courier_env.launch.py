import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    amr_mtt_bot_dir = get_package_share_directory('amr_mtt_bot')
    amr_mtt_moveit_dir = get_package_share_directory('amr_mtt_moveit_config')

    # 1. Gazebo & Robot Spawner
    ign_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_mtt_bot_dir, 'launch', 'ign.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'camera_enabled': 'true',
            'position_x': '-1.7',
            'position_y': '-1.3'
        }.items()
    )

    # 2. Nav2 (Delayed by 5 seconds to let Gazebo load)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_mtt_bot_dir, 'launch', 'nav2.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    delayed_nav2 = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg=">>> Starting Nav2 <<<"),
            nav2_launch
        ]
    )

    # 3. MoveIt 2 (Delayed by 10 seconds to let controllers load)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_mtt_moveit_dir, 'launch', 'move_group.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    delayed_moveit = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg=">>> Starting MoveIt 2 <<<"),
            moveit_launch
        ]
    )

    return LaunchDescription([
        LogInfo(msg=">>> Starting Auto Courier Environment (Gazebo -> Nav2 -> MoveIt) <<<"),
        ign_launch,
        delayed_nav2,
        delayed_moveit
    ])
