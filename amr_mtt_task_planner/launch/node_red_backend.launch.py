#!/usr/bin/env python3
"""
node_red_backend.launch.py

เปิด backend services ทั้งหมดที่ Node-RED Dashboard ต้องการ:

  rosbridge_websocket  ws://localhost:9090   — WebSocket bridge สำหรับ ROS2 topics
  arm_jog_node         http://localhost:5005 — Cartesian jog + Save/Load positions
  nav_waypoint_server  http://localhost:5006 — Nav2 waypoint management
  map_manager_node     http://localhost:5007 — SLAM start/stop + Save/Load maps

วิธีใช้:
  ros2 launch amr_mtt_task_planner node_red_backend.launch.py
  ros2 launch amr_mtt_task_planner node_red_backend.launch.py use_sim_time:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── rosbridge WebSocket (port 9090) ─────────────────────────────
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml',
            )
        ),
    )

    # ── arm_jog_node (port 5005) ─────────────────────────────────────
    arm_jog_node = Node(
        package='amr_mtt_task_planner',
        executable='arm_jog_node',
        name='arm_jog_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── nav_waypoint_server (port 5006) ──────────────────────────────
    nav_waypoint_server = Node(
        package='amr_mtt_task_planner',
        executable='nav_waypoint_server',
        name='nav_waypoint_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── map_manager_node (port 5007) ─────────────────────────────────
    map_manager_node = Node(
        package='amr_mtt_task_planner',
        executable='map_manager_node',
        name='map_manager_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── task_config_server (port 5008) ───────────────────────────────
    task_config_server = Node(
        package='amr_mtt_task_planner',
        executable='task_config_server',
        name='task_config_server',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='ใช้ sim time (true = Gazebo simulation, false = real robot)',
        ),
        rosbridge_launch,
        arm_jog_node,
        nav_waypoint_server,
        map_manager_node,
        task_config_server,
    ])
