#!/usr/bin/env python3
"""
nav2.launch.py

Launch Nav2 Navigation Stack + RViz สำหรับ AMR MTT

⚠️ ต้อง launch moveit.launch.py ก่อน (Gazebo + robot TF)

Usage:
  Terminal 1: ros2_nvidia launch amr_mtt_moveit_config moveit.launch.py
  Terminal 2: ros2_nvidia launch amr_mtt_bot nav2.launch.py
  Terminal 3: ros2 run amr_mtt_task_planner task_sequence
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share    = get_package_share_directory('amr_mtt_bot')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    default_map    = os.path.join(pkg_share, 'map', 'amr_mtt_map_v2.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config    = os.path.join(nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    # ── Launch Arguments ───────────────────────────────────────
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to map yaml file'
    )
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 params yaml'
    )
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True'
    )

    map_file     = LaunchConfiguration('map')
    params_file  = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Nav2 Bringup ───────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':          map_file,
            'params_file':  params_file,
            'use_sim_time': use_sim_time,
            'autostart':    'true',
        }.items(),
    )

    # ── RViz (Nav2 view) ───────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ── cmd_vel Relay ──────────────────────────────────────────
    # Nav2 publishes → /cmd_vel
    # diff_drive_controller (ros2_control) subscribes → /diff_drive_controller/cmd_vel_unstamped
    # (Ignition DiffDrive plugin is DISABLED — mobile base uses ros2_control)
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_sim_time,
        nav2,
        rviz,
        cmd_vel_relay,
    ])
