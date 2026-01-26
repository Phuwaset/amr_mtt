import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    pkg_slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    pkg_amr_mtt = get_package_share_directory('amr_mtt')

    # 1. Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    # 2. Process XACRO (เพื่อสร้าง Robot Description สำหรับ TF)
    xacro_path = os.path.join(pkg_amr_mtt, 'urdf', 'amr_mtt.xacro')
    doc = get_xacro_to_doc(xacro_path, {"wheel_odom_topic": "odom"})

    # ---------------------------------------------------------
    # 3. Nodes ที่ขาดหายไป (เอามาจาก rviz.launch.py)
    # ---------------------------------------------------------
    
    # 3.1 Robot State Publisher (สำคัญมากสำหรับ TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': doc.toxml()}]
    )

    # 3.2 Lidar Bridge (รับข้อมูลจาก Gazebo)
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

    # 3.3 Dual Lidar Merger (รวมข้อมูลเป็น /merged)
    dual_lidar_merger = Node(
        package='dual_laser_merger', # ตรวจสอบชื่อ package ให้ตรงกับที่คุณลงไว้
        executable='dual_laser_merger_node',
        name='dual_laser_merger',
        output='screen',
        parameters=[{
            'laser_1_topic': '/lidar_front/scan',
            'laser_2_topic': '/lidar_rear/scan',
            'merged_topic': '/merged',               
            'target_frame': 'base_link',
            'publish_rate': 20, # ปรับความถี่ตามเหมาะสม
            'range_min': 0.05,
            'range_max': 25.0,
            'angle_min': -3.141592654,
            'angle_max': 3.141592654,
            'use_inf': False
        }]
    )

    # ---------------------------------------------------------
    # 4. SLAM & Visualization
    # ---------------------------------------------------------

    # 4.1 SLAM Toolbox
    slam_toolbox_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_amr_mtt, 'config', 'mapper_params_online_async.yaml'),
        }.items()
    )

    # 4.2 RViz2
    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d', os.path.join(pkg_amr_mtt, 'rviz', 'map.rviz')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        
        # รันทุกอย่างพร้อมกัน
        robot_state_publisher,
        lidar_bridge,
        dual_lidar_merger,
        slam_toolbox_launch_cmd,
        rviz_launch_cmd
        
        # ❌ ลบ static_transform_publisher ออกแล้ว เพราะ SLAM จะทำหน้าที่นี้เอง
    ])