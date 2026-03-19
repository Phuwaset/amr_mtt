import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Get Package Paths
    amr_mtt_path = get_package_share_directory("amr_mtt_bot")
    ros_gz_sim_path = get_package_share_directory("ros_gz_sim")
    ur_description_path = get_package_share_directory('ur_description')
    
    # Try to get robotiq_description (optional)
    try:
        robotiq_description_path = get_package_share_directory('robotiq_description')
        has_robotiq = True
    except:
        robotiq_description_path = None
        has_robotiq = False
        print("[INFO] robotiq_description not found, continuing without gripper package")
    
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
    ur5_enabled = LaunchConfiguration('ur5_enabled', default='false') # เพิ่มตัวนี้เผื่อไว้เปิดปิดแขน

    # 3. Environment Variables
    # รวม Path ของโมเดลทั้งจาก amr_mtt และ ur_description
    resource_paths = [
        os.path.join(amr_mtt_path, "worlds"), ':',
        os.path.join(amr_mtt_path, "models"), ':',
        os.path.join(amr_mtt_path, '..'), ':',
        os.path.join(ur_description_path, '..', '..')
    ]
    
    # Add robotiq path if available
    if has_robotiq:
        resource_paths.extend([':',  os.path.join(robotiq_description_path, '..')])
    
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_paths
    )

    # 4. Launch Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_path, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'-r --render-engine ogre2 ", world_file, "'"])
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
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        remappings=[
            ('/diff_drive_controller/cmd_vel_unstamped', '/amr_mtt/cmd_vel')
        ]
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # หมายเหตุ: โต๊ะและกล่องถูกกำหนดใน small_warehouse.sdf แล้ว
    # ไม่ต้อง spawn แยก (ป้องกัน duplicate) แต่เก็บไว้ในกรณีต้องการรัน World อื่น
    # gz_spawn_desk และ gz_spawn_box ถูก comment ออก

    # หน่วงเวลา 5 วินาที ให้ Gazebo โหลด world เสร็จก่อน spawn robot
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_amr_mtt_node]
    )

    # หน่วงเวลา 8 วินาที ให้ robot spawn เสร็จและ controller_manager พร้อมก่อน
    delayed_controllers = TimerAction(
        period=8.0,
        actions=[
            diff_drive_spawner,
            joint_state_broadcaster_spawner,
            ur_arm_controller_spawner,
            gripper_spawner,
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
        delayed_spawn,        # t+5s: spawn robot
        delayed_controllers,  # t+8s: activate controllers (หลัง controller_manager พร้อม)
        # gz_spawn_desk และ gz_spawn_box ถูก inline ใน small_warehouse.sdf แล้ว

    ])