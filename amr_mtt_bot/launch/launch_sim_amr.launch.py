import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

# ============================================================
# HOME POSITION (radians) - อัปเดตค่าตรงนี้เพื่อเปลี่ยน Home Pose ของแขนกล
# [pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
# ค่าปัจจุบัน:  0°,     -90°,      -90°,    -181°,    0°,      0°
HOME_POS = [0.0, -1.5708, -1.5708, -3.159, 0.0, 0.0]

# ตำแหน่งเริ่มต้นของหุ่นยนต์บน world (เหมือน moveit.launch.py)
ROBOT_START_X = '-1.7'
ROBOT_START_Y = '-1.3'
ROBOT_START_YAW = '0.0'
# ============================================================

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
    print(f"Cameras: {camera_enabled}, Stereo: {stereo_camera_enabled}, LiDAR: {two_d_lidar_enabled}")
    print(f"Robot Start Position: x={ROBOT_START_X}, y={ROBOT_START_Y}, yaw={ROBOT_START_YAW}")
    print(f"Arm Home Pose: {HOME_POS}\n")

    # Get Package Paths
    amr_mtt_path = get_package_share_directory("amr_mtt_bot")
    
    # Include ign.launch.py (ตำแหน่ง spawn เหมือน moveit.launch.py)
    ign_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_mtt_path, "launch", "ign.launch.py")),
        launch_arguments={
            'use_sim_time': 'true',
            'position_x': ROBOT_START_X,
            'position_y': ROBOT_START_Y,
            'orientation_yaw': ROBOT_START_YAW,
            'odometry_source': 'world',
            'world_file': 'small_warehouse.sdf',
            'camera_enabled': camera_enabled,
            'stereo_camera_enabled': stereo_camera_enabled,
            'two_d_lidar_enabled': two_d_lidar_enabled
        }.items()
    )

    # ส่งแขนกลไป HOME_POS หลังจาก 20 วินาที (รอให้ Controller activate เสร็จก่อน)
    # ไม่ใช้ MoveIt — ส่งตรงผ่าน /ur_arm_controller/follow_joint_trajectory
    home_pos_str = str(HOME_POS).replace(' ', '')
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

    return [ign_sim, send_home_goal]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=prompt_user)
    ])
