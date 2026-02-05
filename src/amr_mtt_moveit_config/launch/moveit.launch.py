import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    amr_mtt_path = get_package_share_directory('amr_mtt')
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
            'position_x': '0.0',
            'position_y': '0.0',
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
    robot_description_content = Command(
        [
            'xacro ',
            os.path.join(get_package_share_directory('amr_mtt'), 'urdf', 'amr_mtt.xacro'),
            ' sim_ign:=true',
            ' stereo_camera_enabled:=false',
            ' two_d_lidar_enabled:=true',
            ' camera_enabled:=true'
        ]
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

    return LaunchDescription([
        simulation_launch,
        move_group_launch,
        rviz_node
    ])
