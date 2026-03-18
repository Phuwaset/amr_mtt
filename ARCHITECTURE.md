<div align="center">

# AMR MTT — System Architecture

**Autonomous Mobile Robot with Manipulation**
*ROS 2 Humble · Ignition Gazebo Fortress · MoveIt 2 · Nav2*

</div>

---

## 1 · System Overview

```mermaid
graph TB
    subgraph SIM["🖥️  Ignition Gazebo Fortress"]
        direction TB
        GZ_WORLD["🌍 small_warehouse.sdf"]
        GZ_DIFF["⚙️ DiffDrive Plugin"]
        GZ_IMU["📡 IMU Plugin\nMicroStrain GX5-10 sim\n200 Hz"]
        GZ_LIDAR["🔴 LiDAR × 2\ngpu_lidar 360°"]
        GZ_CAM["📷 RGBD Camera × 2\nIntel RealSense sim\n640×480 · 30fps"]
        GZ_ARM["🦾 UR5 Arm\nros2_control"]
        GZ_GRIP["🤏 Robotiq 2F-85\nros2_control"]
    end

    subgraph BRIDGE["🌉 ros_gz_bridge"]
        BR["/tf · /clock · /odom\n/imu · /scan · /image\n/depth · /points · /joint_state"]
    end

    subgraph FUSION["🧠 Sensor Fusion"]
        EKF["📊 EKF Node\nrobot_localization\n50 Hz"]
    end

    subgraph NAV["🗺️  Navigation Stack"]
        SLAM["🗺️ SLAM Toolbox\nonline_async"]
        AMCL["📍 AMCL\nLocalization"]
        NAV2["🧭 Nav2\nBehavior Tree"]
        MERGER["🔀 dual_laser_merger\n360° scan · 20 Hz"]
    end

    subgraph MANIP["🦾 Manipulation Stack"]
        MOVEIT["⚡ MoveIt 2\nmove_group"]
        SCENE["🎭 Planning Scene\ncollision objects"]
    end

    subgraph TASK["🎯 Task Planner"]
        COORD["🤖 coordinator_node\nState Machine"]
        TASKSEQ["📋 task_sequence\npick → nav → drop"]
        ARMJOG["🕹️ arm_jog_node\nHTTP :5005"]
        NAVWP["📌 nav_waypoint_server\nHTTP :5006"]
        MAPMGR["🗂️ map_manager_node\nHTTP :5007"]
    end

    subgraph DASH["📱 Node-RED Dashboard"]
        NR["🖥️ Dashboard UI\nlocalhost:1880/ui"]
        RB["🔌 rosbridge\nWebSocket :9090"]
    end

    SIM --> BRIDGE
    BRIDGE --> FUSION
    BRIDGE --> MERGER
    MERGER -->|/amr_mtt/scan| SLAM
    MERGER -->|/amr_mtt/scan| AMCL
    FUSION -->|/odometry/filtered| NAV2
    SLAM -->|/map| NAV2
    AMCL -->|map→odom TF| NAV2
    NAV2 -->|/cmd_vel| SIM
    MOVEIT --> SIM
    SCENE --> MOVEIT
    COORD --> NAV2
    COORD --> MOVEIT
    TASKSEQ --> NAV2
    TASKSEQ --> MOVEIT
    ARMJOG --> MOVEIT
    NAVWP --> NAV2
    MAPMGR --> SLAM
    NR --> RB
    NR --> ARMJOG
    NR --> NAVWP
    NR --> MAPMGR
    RB --> NAV2
    RB --> MOVEIT
```

---

## 2 · TF Transformation Tree

```mermaid
graph TD
    MAP["🗺️ map"]
    ODOM["📍 odom"]
    BF["base_footprint"]
    BL["🤖 base_link"]

    CL["chassis_link"]
    BAT["battery_link"]

    FL_W["front_left_wheel"]
    FR_W["front_right_wheel"]
    ML_W["middle_left_wheel"]
    MR_W["middle_right_wheel"]
    RL_W["rear_left_wheel"]
    RR_W["rear_right_wheel"]

    LF_LIDAR["📡 L_F_Lidar_Link\nFront-Left 360°"]
    RB_LIDAR["📡 R_B_Lidar_Link\nRear-Right 360°"]
    IMU["🧭 imu_frame\nGX5-10 · 200Hz"]

    KC["📷 kinect_camera\nRGBD Front"]
    KC_OPT["kinect_camera_optical"]
    RC["📷 rear_camera\nRGBD Rear"]
    RC_OPT["rear_camera_optical"]
    SC["📷 stereo_camera"]
    SC_OPT["stereo_camera_optical"]

    UR5_BASE["🦾 ur5_base_link"]
    UR5_SH["ur5_shoulder_link"]
    UR5_UA["ur5_upper_arm_link"]
    UR5_FA["ur5_forearm_link"]
    UR5_W1["ur5_wrist_1_link"]
    UR5_W2["ur5_wrist_2_link"]
    UR5_W3["ur5_wrist_3_link"]
    UR5_TOOL["ur5_tool0 / ee_link"]
    RQ_BASE["🤏 robotiq_85_base_link"]
    RQ_L["robotiq_85_left_knuckle"]
    RQ_R["robotiq_85_right_knuckle"]

    MAP -->|"SLAM / AMCL"| ODOM
    ODOM -->|"Gazebo diff_drive"| BF
    BF -->|"base_joint (fixed)"| BL

    BL --> CL
    BL --> BAT
    BL -->|continuous| FL_W
    BL -->|continuous| FR_W
    BL -->|continuous| ML_W
    BL -->|continuous| MR_W
    BL -->|continuous| RL_W
    BL -->|continuous| RR_W
    BL --> LF_LIDAR
    BL --> RB_LIDAR
    BL --> IMU
    BL --> KC
    KC --> KC_OPT
    BL --> RC
    RC --> RC_OPT
    BL --> SC
    SC --> SC_OPT
    BL --> UR5_BASE

    UR5_BASE -->|shoulder_pan_joint| UR5_SH
    UR5_SH -->|shoulder_lift_joint| UR5_UA
    UR5_UA -->|elbow_joint| UR5_FA
    UR5_FA -->|wrist_1_joint| UR5_W1
    UR5_W1 -->|wrist_2_joint| UR5_W2
    UR5_W2 -->|wrist_3_joint| UR5_W3
    UR5_W3 --> UR5_TOOL
    UR5_TOOL --> RQ_BASE
    RQ_BASE --> RQ_L
    RQ_BASE --> RQ_R

    style MAP fill:#2d6a4f,color:#fff,stroke:#1b4332
    style ODOM fill:#40916c,color:#fff,stroke:#1b4332
    style BF fill:#52b788,color:#fff,stroke:#1b4332
    style BL fill:#1d3557,color:#fff,stroke:#0d1b2a
    style UR5_BASE fill:#e63946,color:#fff,stroke:#c1121f
    style UR5_TOOL fill:#e63946,color:#fff,stroke:#c1121f
    style RQ_BASE fill:#c1121f,color:#fff,stroke:#7b0000
    style IMU fill:#7209b7,color:#fff,stroke:#560bad
    style KC fill:#4361ee,color:#fff,stroke:#3a0ca3
    style RC fill:#4361ee,color:#fff,stroke:#3a0ca3
    style LF_LIDAR fill:#f77f00,color:#fff,stroke:#d62828
    style RB_LIDAR fill:#f77f00,color:#fff,stroke:#d62828
```

---

## 3 · Sensor Data Pipeline

```mermaid
flowchart LR
    subgraph SENSORS["📡 Sensors in Gazebo"]
        L1["LiDAR Front\n/lidar_front/scan"]
        L2["LiDAR Rear\n/lidar_rear/scan"]
        IMU_S["IMU GX5-10\n200 Hz"]
        ODOM_S["Wheel Encoder\n/odom"]
        CAM_F["RGBD Front\nimage+depth+points"]
        CAM_R["RGBD Rear\nimage+depth+points"]
    end

    subgraph PROC["⚙️ Processing"]
        MERGE["dual_laser_merger\n360° · 20Hz"]
        EKF_P["EKF Filter\n50Hz"]
    end

    subgraph OUT["📤 ROS 2 Topics"]
        SCAN["📡 /amr_mtt/scan\n360° merged LiDAR"]
        FILTERED["📊 /odometry/filtered\nFused Odom"]
        IMU_T["/amr_mtt/imu"]
        RAW_ODOM["/amr_mtt/odom"]
        KC_IMG["amr_mtt/kinect_camera/\nimage · depth · points"]
        RC_IMG["amr_mtt/rear_camera/\nimage · depth · points"]
    end

    subgraph CONSUMERS["🧠 Consumers"]
        SLAM_C["SLAM Toolbox"]
        NAV2_C["Nav2 Stack"]
        MOVEIT_C["MoveIt 2"]
    end

    L1 --> MERGE
    L2 --> MERGE
    MERGE --> SCAN
    ODOM_S --> RAW_ODOM
    IMU_S --> IMU_T
    RAW_ODOM --> EKF_P
    IMU_T --> EKF_P
    EKF_P --> FILTERED
    CAM_F --> KC_IMG
    CAM_R --> RC_IMG

    SCAN --> SLAM_C
    SCAN --> NAV2_C
    FILTERED --> NAV2_C
    KC_IMG --> MOVEIT_C

    style SCAN fill:#f77f00,color:#fff
    style FILTERED fill:#7209b7,color:#fff
    style EKF_P fill:#7209b7,color:#fff
    style MERGE fill:#f77f00,color:#fff
```

---

## 4 · Navigation Data Flow

```mermaid
flowchart TD
    subgraph INPUT["Input"]
        SCAN_N["/amr_mtt/scan\n360° LiDAR"]
        ODOM_N["/odometry/filtered\nEKF Fused"]
        MAP_N["/map\nOccupancyGrid"]
    end

    subgraph NAV2_STACK["🧭 Nav2 Stack"]
        AMCL_N["AMCL\nLocalization\nmap→odom TF"]
        COSTMAP_G["Global Costmap\nNavfn Planner"]
        COSTMAP_L["Local Costmap\nDWB Controller"]
        BT["Behavior Tree\nbt_navigator"]
    end

    subgraph OUTPUT["Output"]
        CMDVEL["/cmd_vel"]
        RELAY["cmd_vel relay"]
        CTRL["/diff_drive_controller\n/cmd_vel_unstamped"]
    end

    SCAN_N --> AMCL_N
    ODOM_N --> AMCL_N
    AMCL_N --> MAP_N
    MAP_N --> COSTMAP_G
    SCAN_N --> COSTMAP_G
    SCAN_N --> COSTMAP_L
    COSTMAP_G --> BT
    COSTMAP_L --> BT
    BT --> CMDVEL
    CMDVEL --> RELAY
    RELAY --> CTRL

    style BT fill:#1d3557,color:#fff
    style AMCL_N fill:#2d6a4f,color:#fff
    style CMDVEL fill:#e63946,color:#fff
```

---

## 5 · Arm Manipulation Pipeline

```mermaid
flowchart LR
    subgraph PLANNING["🧠 MoveIt 2"]
        MG["move_group\nOMPL RRT-Connect"]
        SCENE_M["Planning Scene\ncollision objects"]
        IK["KDL IK Solver\nur5_arm group"]
    end

    subgraph CTRL["⚙️ ros2_control"]
        ARM_CTRL["ur_arm_controller\nJointTrajectoryController"]
        GRIP_CTRL["gripper_controller\nGripperActionController"]
    end

    subgraph ROBOT["🤖 Robot"]
        J1["J1 shoulder_pan"]
        J2["J2 shoulder_lift"]
        J3["J3 elbow"]
        J4["J4 wrist_1"]
        J5["J5 wrist_2"]
        J6["J6 wrist_3"]
        GR["🤏 Robotiq 2F-85"]
    end

    subgraph API["🌐 HTTP API :5005"]
        JOG["arm_jog_node\n/jog · /preset · /stop"]
        CART["Cartesian Jog\nstep 5–100mm"]
        PRESET["Preset Poses\nHOME · PRE_APPROACH\nLIFT_UP · DROP_ROTATE"]
    end

    SCENE_M --> MG
    IK --> MG
    MG -->|JointTrajectory| ARM_CTRL
    MG -->|GripperCommand| GRIP_CTRL
    ARM_CTRL --> J1 & J2 & J3 & J4 & J5 & J6
    GRIP_CTRL --> GR
    JOG --> MG
    CART --> MG
    PRESET --> MG

    style MG fill:#e63946,color:#fff
    style GR fill:#c1121f,color:#fff
    style JOG fill:#4361ee,color:#fff
```

---

## 6 · Node-RED Dashboard Architecture

```mermaid
flowchart TD
    subgraph UI["🖥️ Dashboard — localhost:1880/ui"]
        S0["Section 0\n🔒 Safety · 🚨 Emergency Stop · ✅ System Status"]
        S1["Section 1\n🎮 Joint Control J1–J6 (rad)"]
        S2["Section 2\n📌 Preset Poses + Sync"]
        S3["Section 3\n📊 Live Joint State"]
        S4["Section 4\n📍 EEF Monitor XYZ · RPY"]
        S5["Section 5\n🕹️ Cartesian Jog ± · Gripper"]
        S6["Section 6\n💾 Save / Load Position × 20"]
        S7["Section 7\n🗺️ 2D SLAM Map + Robot Pose"]
        S8["Section 8\n📌 Nav Waypoints Save/Go/Delete"]
    end

    subgraph BACKENDS["⚙️ Backends"]
        ROS["🔌 rosbridge\nWebSocket :9090"]
        ARM["🦾 arm_jog_node\nHTTP :5005"]
        NAV["🧭 nav_waypoint_server\nHTTP :5006"]
        MAP["🗂️ map_manager_node\nHTTP :5007"]
    end

    subgraph ROS2["🤖 ROS 2"]
        JS["/joint_states"]
        CMD["/move_group\nMoveIt action"]
        MAP_T["/map OccupancyGrid"]
        NAV2_A["Nav2 action server"]
        SLAM_SRV["SLAM Toolbox\nservice"]
    end

    S0 --> ROS & ARM & NAV
    S1 & S2 --> ROS
    S3 --> ROS
    S4 & S5 & S6 --> ARM
    S7 --> ROS & MAP
    S8 --> NAV

    ROS <--> JS
    ROS <--> CMD
    ROS <--> MAP_T
    ARM <--> CMD
    NAV <--> NAV2_A
    MAP <--> SLAM_SRV

    style S0 fill:#d62828,color:#fff
    style ROS fill:#1d3557,color:#fff
    style ARM fill:#e63946,color:#fff
    style NAV fill:#2d6a4f,color:#fff
    style MAP fill:#7209b7,color:#fff
```

---

## 7 · Auto-Docking State Machine

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> GLOBAL_NAV : dock command
    GLOBAL_NAV --> SEARCHING : near dock area
    SEARCHING --> VISUAL_SERVO : AprilTag detected
    VISUAL_SERVO --> FINE_ALIGN : aligned ±5cm
    FINE_ALIGN --> VERIFY_CONNECTION : contact detected
    VERIFY_CONNECTION --> DOCKED : connection confirmed
    VERIFY_CONNECTION --> FINE_ALIGN : retry
    DOCKED --> IDLE : undock command
    SEARCHING --> GLOBAL_NAV : tag lost (timeout)
    VISUAL_SERVO --> SEARCHING : tag lost
```

---

## 8 · Package Dependency Map

```mermaid
graph LR
    subgraph PKGS["📦 Packages"]
        BOT["amr_mtt_bot\nURDF · Launch · Config"]
        MOVEIT_P["amr_mtt_moveit_config\nMoveIt 2 · SRDF · OMPL"]
        TASK["amr_mtt_task_planner\nNodes · HTTP APIs"]
        DOCK["amr_mtt_docking\nAprilTag · PID"]
        GRIP["amr_mtt_gripper\nRobotiq 2F-85"]
    end

    subgraph DEPS["🔧 External Dependencies"]
        ROS2_CTRL["ros2_control"]
        NAV2_D["nav2_bringup"]
        SLAM_D["slam_toolbox"]
        MOVEIT_D["moveit2"]
        APRIL["apriltag_ros"]
        RL["robot_localization"]
        ROSBRIDGE["rosbridge_suite"]
        RQGZ["ros_gz_bridge"]
    end

    BOT --> ROS2_CTRL & NAV2_D & SLAM_D & RL & RQGZ
    MOVEIT_P --> MOVEIT_D & BOT
    TASK --> MOVEIT_P & NAV2_D & BOT
    DOCK --> APRIL & NAV2_D & BOT
    GRIP --> ROS2_CTRL & MOVEIT_P

    style BOT fill:#1d3557,color:#fff
    style MOVEIT_P fill:#e63946,color:#fff
    style TASK fill:#2d6a4f,color:#fff
    style DOCK fill:#7209b7,color:#fff
    style GRIP fill:#c1121f,color:#fff
```

---

<div align="center">

*AMR MTT — Bachelor of Engineering in Mechatronics Technology*
*College of Industrial Technology (CIT), KMUTNB*
*Developer: Phuwaset Sibta (MtT-13)*

</div>
