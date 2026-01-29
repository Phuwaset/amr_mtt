import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Get Package Paths
    amr_mtt_path = get_package_share_directory("amr_mtt")
    ros_gz_sim_path = get_package_share_directory("ros_gz_sim")
    ur_description_path = get_package_share_directory('ur_description')
    robotiq_description_path = get_package_share_directory('robotiq_description')
    # 2. Configuration Variables (รับค่าจาก Command Line)
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file = LaunchConfiguration("world_file")
    
    # ตัวแปรสำหรับส่งต่อให้ Spawn File
    x_pos = LaunchConfiguration('position_x')
    y_pos = LaunchConfiguration('position_y')
    yaw_rot = LaunchConfiguration('orientation_yaw')
    camera_enabled = LaunchConfiguration('camera_enabled')
    stereo_camera_enabled = LaunchConfiguration('stereo_camera_enabled')
    two_d_lidar_enabled = LaunchConfiguration('two_d_lidar_enabled')
    ur5_enabled = LaunchConfiguration('ur5_enabled', default='true') # เพิ่มตัวนี้เผื่อไว้เปิดปิดแขน

    # 3. Environment Variables
    # รวม Path ของโมเดลทั้งจาก amr_mtt และ ur_description
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(amr_mtt_path, "worlds"), ':',
            os.path.join(amr_mtt_path, "models"), ':',
            os.path.join(amr_mtt_path, '..'), ':',
            os.path.join(ur_description_path, '..', '..')
        ]
    )

    # 4. Launch Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_path, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'-r ", world_file, "'"])
        }.items()
    )

    # 5. Spawn Robot (ส่งต่อค่าพารามิเตอร์ไปที่ไฟล์ spawn)
    spawn_amr_mtt_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr_mtt_path, "launch", "amr_mtt_ign_spawn.launch.py")),
        launch_arguments={
            'position_x': x_pos,
            'position_y': y_pos,
            'orientation_yaw': yaw_rot,
            'camera_enabled': camera_enabled,
            'stereo_camera_enabled': stereo_camera_enabled,
            'two_d_lidar_enabled': two_d_lidar_enabled,
            'ur5_enabled': ur5_enabled,
        }.items()
    )

    # 6. Spawners for UR5 Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    ur_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_arm_controller", "--controller-manager", "/controller_manager"],
    )
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(amr_mtt_path, "worlds"), ':',
            os.path.join(amr_mtt_path, "models"), ':',
            os.path.join(amr_mtt_path, '..'), ':',
            os.path.join(ur_description_path, '..', '..'), ':',
            
            # +++ เพิ่มบรรทัดนี้ครับ (สำคัญ!) +++
            os.path.join(robotiq_description_path, '..'), 
        ]
    )
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        remappings=[
            ('/diff_drive_controller/cmd_vel_unstamped', '/amr_mtt/cmd_vel')
        ]
    )

    return LaunchDescription([
        # Declare Arguments (ประกาศว่าฉันรับค่าพวกนี้นะ)
        DeclareLaunchArgument("use_sim_time", default_value='true'),
        DeclareLaunchArgument("world_file", default_value=os.path.join(amr_mtt_path, "worlds", "small_warehouse.sdf")),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("camera_enabled", default_value="true"),
        DeclareLaunchArgument("stereo_camera_enabled", default_value="false"),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value="true"),
        DeclareLaunchArgument("ur5_enabled", default_value="true"),

        # Start Processes
        set_ign_resource_path,        
        gz_sim, 
        spawn_amr_mtt_node,
        diff_drive_spawner,
        
        # Controllers (รอให้ Spawn เสร็จก่อนถึงจะทำงานจริง แต่ ROS2 Launch จะจัดการให้)
        joint_state_broadcaster_spawner,
        ur_arm_controller_spawner       
    ])