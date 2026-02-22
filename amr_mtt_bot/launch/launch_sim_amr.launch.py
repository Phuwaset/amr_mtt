import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def prompt_user(context, *args, **kwargs):
    # Prompt the user for parameters
    print("\n--- Simulation Configuration ---")
    
    # helper for boolean parsing
    def is_true(val):
        return val.lower() in ['y', 'yes', 't', 'true', '1']
    
    # 1. camera_enabled
    ans_cam = input("Enable Front/Rear Cameras? (y/n, true/false) [default: y]: ").strip()
    if not ans_cam:
        ans_cam = 'y'
    camera_enabled = 'true' if is_true(ans_cam) else 'false'

    # 2. stereo_camera_enabled
    ans_stereo = input("Enable Stereo Camera? (y/n, true/false) [default: n]: ").strip()
    if not ans_stereo:
        ans_stereo = 'n'
    stereo_camera_enabled = 'true' if is_true(ans_stereo) else 'false'

    # 3. two_d_lidar_enabled
    ans_lidar = input("Enable 2D LiDAR (Front/Rear)? (y/n, true/false) [default: y]: ").strip()
    if not ans_lidar:
        ans_lidar = 'y'
    two_d_lidar_enabled = 'true' if is_true(ans_lidar) else 'false'

    print("--------------------------------\n")
    print(f"Cameras: {camera_enabled}, Stereo: {stereo_camera_enabled}, LiDAR: {two_d_lidar_enabled}\n")

    # Get Package Paths
    amr_mtt_path = get_package_share_directory("amr_mtt_bot")
    
    # Include ign.launch.py with hardcoded values and prompted values
    ign_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_mtt_path, "launch", "ign.launch.py")),
        launch_arguments={
            'use_sim_time': 'true',
            'position_x': '0.0',
            'position_y': '0.0',
            'orientation_yaw': '0.0',
            'odometry_source': 'world',
            'world_file': 'small_warehouse.sdf',
            'camera_enabled': camera_enabled,
            'stereo_camera_enabled': stereo_camera_enabled,
            'two_d_lidar_enabled': two_d_lidar_enabled
        }.items()
    )

    # Include rviz.launch.py
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_mtt_path, "launch", "rviz.launch.py")),
        launch_arguments={
            'use_sim_time': 'true'
            # Note: robotiq and other states are already handled by ign.launch.py
        }.items()
    )

    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz]
    )

    return [ign_sim, delayed_rviz]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=prompt_user)
    ])
