#!/usr/bin/env python3

"""
Auto Docking Demo - Unified Launch File
=======================================
Single entry point for the entire AMR Auto-Docking System.
Functionality:
- Launches Gazebo Simulation (Warehouse World)
- Spawns AMR Robot and Charging Dock
- Sets up ROS-Gazebo Bridges (Sensors, Cmd_vel, TF)
- Launches Robot State Publisher & Controllers
- Launches RViz2 for visualization
- Launches Auto Docking Node (Logic)
- Launches Battery Monitor GUI (Control)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Package Directories
    pkg_amr_mtt_bot = get_package_share_directory('amr_mtt_bot')
    pkg_amr_mtt_docking = get_package_share_directory('amr_mtt_docking')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths (use share directory for installed packages)
    # For development, these will point to install/share
    # models_path = os.path.join(pkg_amr_mtt_bot, 'models')
    
    # Hardcoded paths for development (update when deployed)
    models_path_src = os.path.join(pkg_amr_mtt_bot, 'models')
    pkg_root_path = os.path.join(pkg_amr_mtt_bot, '..')
    
    # Try to use bcr_bot warehouse world if available, otherwise use our own
    try:
        warehouse_world_path = "/opt/ros/humble/share/bcr_bot/worlds"
    except:
        warehouse_world_path = os.path.join(pkg_amr_mtt_bot, 'worlds')

    # ========== Launch Arguments ==========
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time')

    enable_rviz = LaunchConfiguration('enable_rviz')
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz', default_value='true', description='Launch RViz')

    enable_gui = LaunchConfiguration('enable_gui')
    declare_enable_gui = DeclareLaunchArgument(
        'enable_gui', default_value='true', description='Launch Battery Monitor GUI')

    two_d_lidar_enabled = LaunchConfiguration('two_d_lidar_enabled') == True
    
    
    # ========== Environment Setup ==========
    # Append to existing IGN_GAZEBO_RESOURCE_PATH
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        resource_path_list = [models_path_src, pkg_root_path, warehouse_world_path, os.environ['IGN_GAZEBO_RESOURCE_PATH']]
    else:
        resource_path_list = [models_path_src, pkg_root_path, warehouse_world_path]
    
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join(resource_path_list)
    )

    # ========== 1. Gazebo Simulation ==========
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r small_warehouse.sdf'}.items(),
    )

    # ========== 2. Robot Description & State Publisher ==========
    xacro_file = os.path.join(pkg_amr_mtt_bot, 'urdf', 'amr_mtt.xacro')
    robot_description = Command([
        'xacro ', xacro_file,
        ' sim_ign:=true',
        ' camera_enabled:=true',
        ' two_d_lidar_enabled:=true',
        ' odometry_source:=false' 
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    # ========== 3. Spawn Robot & Dock ==========
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'amr_mtt',
            '-z', '0.3', 
            '-x', '0.0',
            '-y', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )

    dock_sdf_path = os.path.join(models_path_src, 'charging_dock', 'model.sdf')
    spawn_dock = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'charging_dock',
            '-file', dock_sdf_path,
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '3.14159' 
        ],
        output='screen'
    )
    
    # Delayed Spawning
    spawn_robot_delayed = TimerAction(period=5.0, actions=[spawn_robot])
    spawn_dock_delayed = TimerAction(period=7.0, actions=[spawn_dock])

    # ========== 4. Controllers & Bridge ==========
    spawn_ddc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )
    
    spawn_ddc_delayed = TimerAction(period=10.0, actions=[spawn_ddc])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[ 
            '/model/amr_mtt/battery/linear_battery/state@sensor_msgs/msg/BatteryState[ignition.msgs.BatteryState',
             '/kinect_camera@sensor_msgs/msg/Image[ignition.msgs.Image',
             'kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
             '/rear_camera@sensor_msgs/msg/Image[ignition.msgs.Image',
             'rear_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
             '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
             '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
             # '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V', # Disabled: Using diff_drive_controller for TF
             '/world/default/model/amr_mtt/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
             '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
             '/lidar_front/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
             '/lidar_rear/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ],
        remappings=[
            ('/model/amr_mtt/battery/linear_battery/state', '/battery_state'),
            ('/kinect_camera', '/camera/image_raw'),
            ('kinect_camera/camera_info', '/camera/camera_info'),
            ('/rear_camera', '/rear_camera/image_raw'),
            ('rear_camera/camera_info', '/rear_camera/camera_info'),
            ('/world/default/model/amr_mtt/joint_state', '/joint_states')
        ],
        output='screen'
    )

    # ========== 5. RViz ==========
    rviz_node = Node(
        condition=IfCondition(enable_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_amr_mtt_bot, 'rviz', 'entire_setup.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ========== 6. Auto-Docking System (Enhanced) ==========
    # 6.1 Enhanced Docking Logic
    enhanced_auto_docking_node = Node(
        package='amr_mtt_docking',
        executable='enhanced_docking_node.py',
        name='enhanced_auto_docking_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_global_nav': True,
            'docking_timeout': 180.0
        }]
    )

    # 6.2 Battery Predictor (TODO: Not implemented yet)
    # battery_predictor_node = Node(
    #     package='amr_mtt_docking',
    #     executable='battery_predictor_node.py',
    #     name='battery_predictor_node',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'battery_capacity_ah': 20.0,
    #         'prediction_window': 60.0
    #     }]
    # )

    # 6.3 Charging Station Detector (TODO: Not implemented yet)
    # charging_station_detector_node = Node(
    #     package='amr_mtt_docking',
    #     executable='charging_station_detector_node.py',
    #     name='charging_station_detector_node',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'auto_create_stations': True
    #     }]
    # )

    # Battery Monitor GUI (TODO: Not implemented yet)
    # battery_monitor_gui = Node(
    #     condition=IfCondition(enable_gui),
    #     package='amr_mtt_docking',
    #     executable='battery_monitor_gui.py',
    #     name='battery_monitor_gui',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # ========== 7. LiDAR Merger & Cmd_Vel Remapper ==========
    # Remaps /cmd_vel to /diff_drive_controller/cmd_vel_unstamped
    remapper_node = Node(
        package='amr_mtt_bot',
        executable='remapper.py',
        name='remapper_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Dual Lidar Merger (Restored from rviz.launch.py)
    # Merges LIDAR_FRONT and LIDAR_REAR into /merged
    dual_laser_merger = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node',
        name='dual_laser_merger',
        output='screen',
        parameters=[{
            'laser_1_topic': '/lidar_front/scan',
            'laser_2_topic': '/lidar_rear/scan',
            'merged_topic': '/merged', # Original topic requested by user
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

    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_rviz,
        declare_enable_gui,
        ign_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot_delayed,
        spawn_dock_delayed,
        spawn_ddc_delayed,
        bridge,
        rviz_node,
        enhanced_auto_docking_node,
        remapper_node,
        dual_laser_merger
    ])
