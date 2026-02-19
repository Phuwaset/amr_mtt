#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    isaac_sim = LaunchConfiguration('isaac_sim')

    # Process XACRO
    xacro_path = os.path.join(get_package_share_directory('amr_mtt_bot'), 'urdf', 'amr_mtt.xacro')
    doc = get_xacro_to_doc(xacro_path, {"wheel_odom_topic": "odom"})

    # Nodes
    # robot_state_publisher is already running from ign.launch.py
    # Uncomment this only if running RViz standalone without simulation
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': doc.toxml()}],
        condition=UnlessCondition(isaac_sim)
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('amr_mtt_bot'), 'rviz', 'entire_setup.rviz')]
        # arguments=['-d', os.path.join(get_package_share_directory('amr_mtt_bot'), 'rviz', 'blank_setup.rviz')]

    )

    dual_lidar_merger = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node',
        name='dual_laser_merger',
        output='screen',
        parameters=[{
            'laser_1_topic': '/lidar_front/scan',
            'laser_2_topic': '/lidar_rear/scan',
            'merged_topic': '/merged',               
            'merged_cloud_topic': '/merged_cloud',
            'target_frame': 'base_link',
            'publisher_qos_reliability': 'best_effort',  
            'publish_rate': 100,
            'angle_increment': 0.0043633,
            'scan_time': 0.067,
            'range_min': 0.05,
            'range_max': 25.0,
            'angle_min': -3.141592654,
            'angle_max': 3.141592654,
            'use_inf': False
        }]
    )
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/lidar_front/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/lidar_rear/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'isaac_sim',
            default_value='false',
            description='Set to true when using Isaac Sim'
        ),
        DeclareLaunchArgument(
            'robot_description',
            default_value=doc.toxml(),
            description='Robot description in URDF/XACRO format'
        ),
        # Nodes
        robot_state_publisher,  # Commented out - running from ign.launch.py
        rviz,
        dual_lidar_merger,
        lidar_bridge
    ])