<div align="center">

# AMR MTT — Project Roadmap

**Autonomous Mobile Robot with Manipulation**
*Final Project — Mechatronics Technology (MtT) | CIT, KMUTNB*

</div>

---

## สถานะโดยรวม / Overview

| Phase | ชื่อ | สถานะ |
|-------|------|--------|
| 1 | Robot Foundation | ✅ เสร็จแล้ว |
| 2 | Mapping & Navigation | ✅ เสร็จแล้ว |
| 3 | MoveIt 2 & Arm Manipulation | 🔄 กำลังพัฒนา |
| 4 | Auto-Docking System | ✅ เสร็จแล้ว |
| 5 | Full Autonomous Pipeline | 📋 วางแผนไว้ |
| 6 | Real Hardware Deployment | 🔮 อนาคต |

---

## Phase 1 — Robot Foundation ✅

> สร้างโมเดลหุ่นยนต์และระบบพื้นฐานในสภาพแวดล้อม Ignition Gazebo

### เสร็จแล้ว
- [x] URDF/Xacro robot model (Mobile Base + UR5 arm + Robotiq 2F-85 gripper)
- [x] Ignition Gazebo Fortress simulation environment (`small_warehouse.sdf`)
- [x] `ros2_control` configuration — diff_drive, ur_arm, gripper controllers
- [x] `ros_gz_bridge` topic bridging (Gazebo ↔ ROS 2)
- [x] Keyboard teleoperation (`teleop_twist_keyboard`)
- [x] Joystick teleoperation (`joyStick.launch.py`)
- [x] RViz 2 visualization configuration
- [x] Arm auto-home on launch (12 seconds after startup)
- [x] Gazebo warehouse models (shelves, boxes, charging dock)

### Package หลัก
- `amr_mtt_bot` — URDF, worlds, controllers, launch files
- `amr_mtt_bot/robotiq_description` — Robotiq 2F-85 gripper model

---

## Phase 2 — Mapping & Navigation ✅

> ระบบนำทางอัตโนมัติโดยใช้ Nav2 stack และ SLAM Toolbox

### เสร็จแล้ว
- [x] SLAM Toolbox — สร้างแผนที่คลังสินค้าด้วย online async mapping
- [x] Pre-generated warehouse map (`amr_mtt_map_v2.yaml`)
- [x] Nav2 stack integration (AMCL localization, costmap, planner)
- [x] `/cmd_vel` relay node (Nav2 → diff_drive_controller)
- [x] `set_initial_pose` node — กำหนด AMCL initial pose
- [x] `nav_sequence` node — waypoint-based navigation
- [x] `save_waypoint` node — บันทึก waypoint แบบ interactive
- [x] Nav2 parameter tuning (`nav2_params.yaml`)

### Package หลัก
- `amr_mtt_bot` — maps, nav2 config, mapping launch
- `amr_mtt_task_planner` — set_initial_pose, nav_sequence, save_waypoint

---

## Phase 3 — MoveIt 2 & Arm Manipulation 🔄

> วางแผนการเคลื่อนที่แขนกล UR5 ด้วย MoveIt 2 และระบบหยิบ-วางอัตโนมัติ

> **Branch ปัจจุบัน:** `moveit-integration`

### เสร็จแล้ว
- [x] MoveIt 2 configuration (SRDF, kinematics.yaml, OMPL planner)
- [x] KDL IK solver configuration
- [x] RRT-Connect motion planning algorithm (`ompl_planning.yaml`)
- [x] Collision matrix setup (`amr_mtt.srdf`)
- [x] `planning_scene_setup` node — เพิ่ม collision objects ใน MoveIt scene
- [x] `pick_ik` node — inverse kinematics สำหรับ pick pose
- [x] `coordinator_node` — Multi-task coordination state machine
- [x] `task_sequence` node — Full autonomous pick → navigate → drop pipeline
- [x] Node-RED dashboard — ควบคุม joint ทีละตัว + gripper control
- [x] `arm_jog_node` — real-time arm jogging

### กำลังพัฒนา / In Progress
- [ ] ปรับจูน pick pose coordinates ให้แม่นยำกับ parcel box (6×6×8 cm)
- [ ] ทดสอบ grasp success rate ใน simulation
- [ ] ปรับ planning scene collision objects ให้ครอบคลุมสภาพแวดล้อมคลังสินค้า
- [ ] เพิ่ม pre-grasp → grasp → post-grasp motion sequence

### Package หลัก
- `amr_mtt_moveit_config` — MoveIt config, move_group launch
- `amr_mtt_task_planner` — task_sequence, coordinator_node, planning_scene_setup
- `amr_mtt_gripper` — gripper controller configuration

---

## Phase 4 — Auto-Docking System ✅

> ระบบ Docking อัตโนมัติโดยใช้ AprilTag และ multi-stage PID control

### เสร็จแล้ว
- [x] AprilTag detection integration (`apriltag_ros`)
- [x] 7-state docking state machine: `IDLE → GLOBAL_NAV → SEARCHING → VISUAL_SERVO → FINE_ALIGN → VERIFY_CONNECTION → DOCKED`
- [x] PID controller implementation (`pid_controller.py`)
- [x] Per-stage PID gain tuning (`enhanced_docking.yaml`)
- [x] Docking metrics logging (`docking_metrics.py`)
- [x] Charging station definitions (`charging_stations.yaml`)
- [x] Custom charging dock Gazebo model

### Package หลัก
- `amr_mtt_docking` — enhanced_docking_node, PID controller, AprilTag configs

---

## Phase 5 — Full Autonomous Pipeline 📋

> รวมทุก subsystem เข้าด้วยกันเป็น end-to-end autonomous operation

### แผนการพัฒนา
- [ ] **Full mission cycle**: Dock → pick parcel → navigate to destination → place parcel → return to dock
- [ ] `coordinator_node` ครอบคลุม docking integration (เชื่อมต่อ Phase 3 + Phase 4)
- [ ] Error recovery state machine (ล้มเหลว pick → retry, lost localization → re-init)
- [ ] Multi-waypoint delivery mission (รับหลาย parcel ต่อรอบ)
- [ ] Node-RED mission dashboard — สั่ง mission จาก web UI
- [ ] ROS 2 action server สำหรับ mission management
- [ ] Performance metrics logging (mission success rate, time per cycle)
- [ ] Simulation benchmark test suite

---

## Phase 6 — Real Hardware Deployment 🔮

> ติดตั้งและทดสอบกับฮาร์ดแวร์จริง

### แผนการพัฒนา (อนาคต)
- [ ] Hardware interface layer (real UR5 + Robotiq gripper)
- [ ] IMU integration — Adafruit BNO085 (ติดตั้งใน URDF แล้ว รอ activate)
- [ ] Real LiDAR sensor calibration และ Nav2 tuning
- [ ] Real camera AprilTag detection latency optimization
- [ ] Hardware-in-the-loop testing
- [ ] Safety stop และ emergency handling
- [ ] Real charging dock hardware interface
- [ ] Field testing ในสภาพแวดล้อมจริง

---

## Tech Stack

| Technology | บทบาท | สถานะ |
|---|---|---|
| **ROS 2 Humble** | Robot middleware | ✅ |
| **Ignition Gazebo Fortress** | Physics simulation | ✅ |
| **Nav2** | Autonomous navigation | ✅ |
| **MoveIt 2** | Arm motion planning | 🔄 |
| **ros2_control** | Hardware interface | ✅ |
| **SLAM Toolbox** | Map generation | ✅ |
| **apriltag_ros** | Visual docking | ✅ |
| **Node-RED** | Web dashboard | ✅ |
| **OMPL (RRT-Connect)** | Motion planning algorithm | ✅ |
| **KDL** | Inverse kinematics solver | ✅ |

---

## Repository Branch Strategy

| Branch | วัตถุประสงค์ |
|--------|-------------|
| `main` | Stable release — production-ready features |
| `moveit-integration` | **Current** — MoveIt 2 + arm manipulation development |

---

<div align="center">

*AMR MTT — Bachelor of Engineering in Mechatronics Technology*
*College of Industrial Technology (CIT), KMUTNB*
*Developer: Phuwaset Sibta (MtT-13)*

</div>
