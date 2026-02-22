import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
import launch_ros.descriptions

def generate_launch_description():

    pkg_amr_mtt_bot = get_package_share_directory('amr_mtt_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    urdf_file = os.path.join(pkg_amr_mtt_bot, 'urdf', 'amr_mtt_mesh_only.xacro')
    empty_world = os.path.join(pkg_amr_mtt_bot, 'worlds', 'empty.sdf')
    # Use empty.rviz if it exists, otherwise use a generic rviz path or do not specify config. 
    # For now omitting 'rvizconfig' to just open an empty rviz.

    robot_description_content = Command(['xacro ', urdf_file])
    robot_description_param = {'robot_description': robot_description_content}

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {empty_world}'}.items()
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': True}]
    )

    # Joint State Publisher (Fake)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(pkg_amr_mtt_bot, 'rviz', 'amr_mtt.rviz')] # Let's try to reuse the existing config
    )

    # Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'amr_mtt_mesh',
            '-allow_renaming', 'true',
            '-z', '0.2'
        ]
    )
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen'
    )


    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        spawn_entity,
        bridge
    ])
