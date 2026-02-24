import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

# ============================================================
# HOME POSITION (radians) - อัปเดตค่าตรงนี้เพื่อเปลี่ยน Home Pose
# [pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
# ค่าปัจจุบัน:  0°,     -90°,      -90°,    -181°,    0°,      0°
HOME_POS = [0.0, -1.5708, -1.5708, -3.159, 0.0, 0.0]
# ============================================================

def generate_launch_description():
    amr_mtt_path = get_package_share_directory('amr_mtt_bot')
    amr_mtt_moveit_path = get_package_share_directory('amr_mtt_moveit_config')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Launch the Simulation (Ignition/Gazebo) which also spawns the robot and controllers
    # We use ign.launch.py as it seems to be the primary one for Humble+Fortress
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_mtt_path, 'launch', 'ign.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'camera_enabled': 'true',
            'stereo_camera_enabled': 'false',
            'two_d_lidar_enabled': 'true',
            'position_x': '-1.7',
            'position_y': '-1.3',
            'orientation_yaw': '0.0',
            'odometry_source': 'world',
            'world_file': 'small_warehouse.sdf' 
        }.items()
    )

    # 2. Launch Move Group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_mtt_moveit_path, 'launch', 'move_group.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Launch RViz
    # We need a custom RViz config for MoveIt usually, but we can start with the default one
    # or create a basic one. For now let's reuse the one from amr_mtt but we might need to add MotionPlanning plugin manually
    rviz_config_file = os.path.join(amr_mtt_path, 'rviz', 'entire_setup.rviz') 
    
    # Load robot_description (URDF)
    robot_description_content = ParameterValue(
        Command(
            [
                'xacro ',
                os.path.join(get_package_share_directory('amr_mtt_bot'), 'urdf', 'amr_mtt.xacro'),
                ' sim_ign:=true',
                ' stereo_camera_enabled:=false',
                ' two_d_lidar_enabled:=true',
                ' camera_enabled:=true'
            ]
        ),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # Load robot_description_semantic (SRDF)
    srdf_file = os.path.join(amr_mtt_moveit_path, 'config', 'amr_mtt.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_config = f.read()
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # Load kinematics.yaml
    kinematics_yaml = os.path.join(amr_mtt_moveit_path, 'config', 'kinematics.yaml')
    with open(kinematics_yaml, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description,
            robot_description_semantic,
            kinematics_config
        ]
    )

    # 4. Auto move arm to HOME_POS after 8 seconds (รอให้ Controller พร้อมก่อน)
    home_pos_str = str(HOME_POS).replace(' ', '')
    send_home_goal = TimerAction(
        period=8.0,
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

    return LaunchDescription([
        simulation_launch,
        move_group_launch,
        rviz_node,
        send_home_goal,
    ])
