import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    moveit_config_package = 'amr_mtt_moveit_config'
    robot_description_package = 'amr_mtt'
    
    # Command to convert Xacro to URDF
    robot_description_content = Command(
        [
            'xacro ',
            os.path.join(get_package_share_directory(robot_description_package), 'urdf', 'amr_mtt.xacro'),
            ' sim_ign:=true', # Ensure we use the same flags as the simulation
            ' stereo_camera_enabled:=false',
            ' two_d_lidar_enabled:=true',
            ' camera_enabled:=true'
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}

    # robot_description_semantic_config = load_yaml(moveit_config_package, 'config/amr_mtt.srdf')
    # robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}
    
    srdf_file = os.path.join(get_package_share_directory(moveit_config_package), 'config', 'amr_mtt.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_config = f.read()
    
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml(moveit_config_package, 'config/kinematics.yaml')

    # MoveIt Controllers
    moveit_controllers = load_yaml(moveit_config_package, 'config/moveit_controllers.yaml')    

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(moveit_config_package, 'config/moveit_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # OMPL Planning
    ompl_planning_pipeline_config = load_yaml(moveit_config_package, 'config/ompl_planning.yaml')
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'ompl': ompl_planning_pipeline_config
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            moveit_controllers,
            planning_pipelines,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {'use_sim_time': True} # Important for simulation
        ],
        # remappings=[
        #     ('/joint_states', '/amr_mtt/joint_states')
        # ]
    )

    return LaunchDescription([
        run_move_group_node
    ])
