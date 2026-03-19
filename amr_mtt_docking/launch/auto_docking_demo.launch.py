#!/usr/bin/env python3

"""
Auto Docking Demo - Unified Launch File
=======================================
Single entry point for the entire AMR Auto-Docking System.
- ใช้ ign.launch.py เป็น base (เหมือน launch_sim_amr.launch.py)
- Robot spawn position เหมือนกับ launch_sim_amr.launch.py
- ส่งแขนกลไป HOME_POS หลัง 12 วินาที
- Spawn charging_dock เพิ่มเติม
- Launch enhanced_auto_docking_node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, OpaqueFunction,
    TimerAction, ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ============================================================
# HOME POSITION (radians) - อัปเดตค่าตรงนี้เพื่อเปลี่ยน Home Pose ของแขนกล
# [pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
# ค่าปัจจุบัน:  0°,     -90°,      -90°,    -181°,    0°,      0°
HOME_POS = [0.0, -1.5708, -1.5708, -3.159, 0.0, 0.0]

# ตำแหน่งเริ่มต้นของหุ่นยนต์ (เหมือน launch_sim_amr.launch.py)
ROBOT_START_X   = '-1.7'
ROBOT_START_Y   = '-1.3'
ROBOT_START_YAW = '0.0'

# ตำแหน่ง Charging Dock 
DOCK_X   = '-2.3'
DOCK_Y   = '-4.0'
DOCK_YAW = '0.0'  
# ============================================================


def prompt_user(context, *args, **kwargs):
    """Prompt user for sensor configuration (เหมือน launch_sim_amr.launch.py)."""
    print("\n--- Auto Docking Demo Configuration ---")

    def is_true(val):
        return val.lower() in ['y', 'yes', 't', 'true', '1']

    # 1. camera_enabled
    ans_cam = input("Enable Front/Rear Cameras? (y/n) [default: y]: ").strip()
    if not ans_cam:
        ans_cam = 'y'
    camera_enabled = 'true' if is_true(ans_cam) else 'false'

    # 2. stereo_camera_enabled
    ans_stereo = input("Enable Stereo Camera? (y/n) [default: n]: ").strip()
    if not ans_stereo:
        ans_stereo = 'n'
    stereo_camera_enabled = 'true' if is_true(ans_stereo) else 'false'

    # 3. two_d_lidar_enabled
    ans_lidar = input("Enable 2D LiDAR? (y/n) [default: y]: ").strip()
    if not ans_lidar:
        ans_lidar = 'y'
    two_d_lidar_enabled = 'true' if is_true(ans_lidar) else 'false'

    print("---------------------------------------\n")
    print(f"Cameras: {camera_enabled}, Stereo: {stereo_camera_enabled}, LiDAR: {two_d_lidar_enabled}")
    print(f"Robot Start: x={ROBOT_START_X}, y={ROBOT_START_Y}, yaw={ROBOT_START_YAW}")
    print(f"Arm Home Pose: {HOME_POS}")
    print(f"Charging Dock: x={DOCK_X}, y={DOCK_Y}, yaw={DOCK_YAW}\n")

    # Package Paths
    amr_mtt_path    = get_package_share_directory('amr_mtt_bot')
    amr_docking_path = get_package_share_directory('amr_mtt_docking')

    # ==========================================================
    # 1. Gazebo Simulation (ใช้ ign.launch.py เหมือน launch_sim_amr)
    # ==========================================================
    ign_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_mtt_path, 'launch', 'ign.launch.py')
        ),
        launch_arguments={
            'use_sim_time':            'true',
            'position_x':              ROBOT_START_X,
            'position_y':              ROBOT_START_Y,
            'orientation_yaw':         ROBOT_START_YAW,
            'odometry_source':         'world',
            'world_file':              'small_warehouse.sdf',
            'camera_enabled':          camera_enabled,
            'stereo_camera_enabled':   stereo_camera_enabled,
            'two_d_lidar_enabled':     two_d_lidar_enabled,
        }.items()
    )

    # ==========================================================
    # 2. Spawn Charging Dock (รอ Gazebo โหลดเสร็จก่อน)
    # ==========================================================
    dock_sdf_path = os.path.join(amr_mtt_path, 'models', 'charging_dock', 'model.sdf')
    spawn_dock = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'charging_dock',
            '-file', dock_sdf_path,
            '-x', DOCK_X,
            '-y', DOCK_Y,
            '-z', '0.0',
            '-Y', DOCK_YAW,
        ],
        output='screen'
    )
    # รอให้ controllers activate เสร็จก่อน (~8-9s) แล้วค่อย spawn dock
    spawn_dock_delayed = TimerAction(period=10.0, actions=[spawn_dock])

    # ==========================================================
    # 3. ส่งแขนกลไป HOME_POS หลัง 12 วินาที (ไม่ใช้ MoveIt)
    # ==========================================================
    home_pos_str = str(HOME_POS).replace(' ', '')
    # รอให้ controllers ทั้งหมด settle ก่อน (20s) แล้วค่อยส่ง home goal
    send_home_goal = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    '/ur_arm_controller/follow_joint_trajectory',
                    'control_msgs/action/FollowJointTrajectory',
                    (
                        '{trajectory: {'
                        'joint_names: [ur5_shoulder_pan_joint, ur5_shoulder_lift_joint, ur5_elbow_joint, ur5_wrist_1_joint, ur5_wrist_2_joint, ur5_wrist_3_joint], '
                        'points: [{positions: ' + home_pos_str + ', time_from_start: {sec: 3}}]}}'
                    )
                ],
                output='screen'
            )
        ]
    )

    # ==========================================================
    # 4. apriltag_ros node ← แทนที่ cv2.aruco
    #    remap /image_rect + /camera_info → rear_camera topics
    # ==========================================================
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        remappings=[
            ('image_rect',   '/rear_camera/image_raw'),   # sim ไม่ distort → raw ใช้แทน rect ได้
            ('camera_info',  '/rear_camera/camera_info'),
        ],
        parameters=[{
            'family':  'tag36h11',
            'size':    0.2,         # ขนาด tag จริง (m) — ตั้งเหมือน model.sdf
            'max_hamming': 0,       # strict: ไม่ detect tag ที่เสีย
        }]
    )

    # ==========================================================
    # 5. Enhanced Auto-Docking Node
    # ==========================================================
    enhanced_auto_docking_node = Node(
        package='amr_mtt_docking',
        executable='enhanced_docking_node.py',
        name='enhanced_auto_docking_node',
        output='screen',
        parameters=[{
            'use_sim_time':      True,
            'use_global_nav':    True,
            'docking_timeout':   180.0,
            # Dock position (ผ่านค่า constant ด้านบน)
            'dock_x':            float(DOCK_X),
            'dock_y':            float(DOCK_Y),
            'dock_yaw':          float(DOCK_YAW),
            'pre_dock_offset':   1.5,  # ระยะหน้า dock ที่ robot จอดก่อน
        }]
    )

    # ==========================================================
    # 5. Trigger Docking อัตโนมัติ (หลัง 25 วิ)
    #    พิมพ์ /battery/override = 10% → node เริ่ม docking sequence
    # ==========================================================
    trigger_docking = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/battery/override',
                    'std_msgs/msg/Float32',
                    '{data: 10.0}',
                ],
                output='screen'
            )
        ]
    )

    return [
        ign_sim,
        spawn_dock_delayed,
        send_home_goal,
        apriltag_node,
        enhanced_auto_docking_node,
        trigger_docking,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=prompt_user)
    ])
