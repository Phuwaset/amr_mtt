import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # --- ส่วนกำหนดค่าและไฟล์ ---
    
    # ชื่อ package ของคุณ
    pkg_name = 'amr_mtt'
    
    # ชื่อไฟล์ URDF ของคุณ
    urdf_file_name = 'amr_update.urdf'

    # หาตำแหน่งของ package
    pkg_share = get_package_share_directory(pkg_name)
    
    # ตำแหน่งไฟล์ URDF
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)
    
    # ตำแหน่งไฟล์ RViz config (เราจะสร้างไฟล์นี้ในขั้นตอนถัดไป)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')

    # --- Nodes ที่จะรัน ---

    # 1. Node สำหรับอ่านไฟล์ URDF และ publish topic /robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # 2. Node สำหรับสร้าง GUI สไลเดอร์เพื่อปรับค่า Joint
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 3. Node สำหรับเปิดโปรแกรม RViz2 พร้อมไฟล์ config
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # --- ส่งค่าทั้งหมดให้ LaunchDescription ---
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])

