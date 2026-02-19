import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_bcr = get_package_share_directory('amr_mtt_bot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': os.path.join(pkg_bcr, 'map', 'amr_mtt_map1.yaml'), # Updated to correct map

            'params_file': os.path.join(pkg_bcr, 'config', 'nav2_params.yaml'),
            'package_path': pkg_bcr, 
        }.items()
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d' + os.path.join(
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
            )
        ]
    )
    
    # ใช้ params file เดียวกันกับ nav2_launch_cmd (nav2_params.yaml)
    # เพราะ nav2_params.yaml ปกติมี config ของ amcl อยู่แล้ว
    # amcl_node = Node( ...) # ไม่จำเป็นต้องแยก ถ้า nav2_bringup รัน amcl ให้ (ซึ่งมันทำ)
    # แต่ถ้าต้องรันแยก ให้ใช้ config เดียวกัน

    # map_server รันแยกออกมาจาก nav2_bringup (ถ้า autostart=True ใน bringup มันอาจจะรัน map_server ให้แล้ว แต่ลองดูก่อน)
    # ปกติ nav2_bringup จะรัน lifecycle manager และ nodes ที่จำเป็นให้
    
    # Define static transform publisher (map -> odom)
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # แก้ไข remapper node ให้ใช้ชื่อ package ที่ถูกต้อง
    remapper_node = Node(
        package='amr_mtt_bot', # แก้จาก amr_mtt เป็น amr_mtt_bot
        executable='remapper.py',
        name='remapper',
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(nav2_launch_cmd)
    ld.add_action(rviz_launch_cmd)
    # ld.add_action(amcl_node) # ปิด เพราะ nav2_bringup น่าจะรัน amcl ให้แล้ว หรือถ้าไม่ ก็ต้อง config ให้ถูก
    # ld.add_action(map_server_node) # ปิด เพราะ nav2_bringup น่าจะรัน map_server ให้
    ld.add_action(static_transform_publisher_node)
    ld.add_action(remapper_node)


    return ld

