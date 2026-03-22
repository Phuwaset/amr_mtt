<div align="center">

# AMR MTT

### Autonomous Mobile Robot with Manipulation

*Final Project — Mechatronics Technology (MtT) | CIT, KMUTNB* <br>
*Mr. Phuwaset Sibta MtT-13<br>
Mr. Kittiphong Simak MtT-13<br>
Mr. Phupha Phungadung MtT-13<br>

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros)
![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)
![MoveIt](https://img.shields.io/badge/MoveIt!-2-orange)
![Ignition](https://img.shields.io/badge/Ignition-Fortress-red)
![License](https://img.shields.io/badge/License-Academic-lightgrey)
</div>

---

## Overview

**AMR MTT** is an autonomous mobile robot developed as a final-year engineering project at King Mongkut's University of Technology North Bangkok (KMUTNB). The system integrates mobile navigation, robotic arm manipulation (UR5 + Robotiq 85 Gripper), gripper control, and automatic docking into a unified ROS 2 framework simulated in Ignition Gazebo Fortress.

### Key Capabilities

- 🗺️ **Autonomous Navigation** — map-based path planning and obstacle avoidance via Nav2
- 🦾 **Arm Manipulation** — UR5 6-DOF arm motion planning using MoveIt! 2
- 🤏 **Gripper Control** — pick-and-place operations with Robotiq 2F-85 gripper
- 📦 **Pick Sequence** — automated waypoint-based pick-and-place execution
- 🔌 **Auto Docking** — precision docking to a charging or handoff station

---

## Contributors

* [@Phuwaset](https://github.com/Phuwaset) - Creator & Lead Developer & Leader Group.

* @Kittiphong - CAD amr_robot design

![CAD Design](docs/images/cad_model_v0.jpg)

* @Phupha - Electrical Design Schematic & System Diagram of amr_robot

![Electrical Diagram](docs/images/systemdiagram.png)

---

## Repository Structure

```
amr_mtt/
├── amr_mtt_bot/             # Core robot — URDF/Xacro, launch files, controllers, worlds
│   ├── urdf/                # amr_mtt.xacro — UR5 arm + Robotiq gripper + mobile base
│   ├── worlds/              # small_warehouse.sdf — Ignition Gazebo world
│   ├── models/              # training_box (parcel: 6×6×8 cm, 0.1 kg)
│   └── launch/              # ign.launch.py, nav2.launch.py
├── amr_mtt_moveit_config/   # MoveIt! 2 — SRDF, kinematics, move_group, RViz
├── amr_mtt_task_planner/    # Task execution nodes
│   └── task_sequence.py     # Full Pick → Navigate → Drop pipeline
├── amr_mtt_gripper/         # Gripper driver and control nodes
├── amr_mtt_docking/         # AprilTag-based auto-docking (multi-stage PID)
└── node_red_flows/          # Node-RED dashboard flows (rosbridge integration)
    ├── amr_map_flow.json    # AMR live position map (AMCL → Worldmap)
    └── arm_control_flow.json # UR5 arm joint control dashboard
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| Operating System | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Ignition Gazebo | Fortress (6) |
| MoveIt! 2 | Latest (Humble) |
| Python | 3.10+ |

---

## Installation

```bash
# 1. Create a ROS 2 workspace
mkdir -p ~/amr_mtt/src
cd ~/amr_mtt/src

# 2. Clone this repository
git clone https://github.com/Phuwaset/amr_mtt.git

# 3. Install dependencies
cd ~/amr_mtt
rosdep install --from-paths src --ignore-src -r -y

cd ~/amr_mtt
. /install_dependencies.sh

# 4. Build the workspace
colcon build --symlink-install

# 5. Source the workspace
source install/setup.bash
```

---

## Usage

### 1. Launch Main Simulation (Gazebo + Controllers)
```bash
ros2_nvidia launch amr_mtt_bot launch_sim_amr.launch.py
```
> **หลัก** — เปิด Ignition Gazebo + ros2_control + ส่งแขนกลไป HOME pose อัตโนมัติหลัง 12 วินาที
> จะถามตอน launch: กล้อง / Stereo Camera / LiDAR เปิดหรือปิด
> NVIDIA GPU offload. ใช้ GPU ปกติให้ใช้ `ros2 launch ...`

### 2. Run Navigation (ต่างหน้าต่าง)
```bash
ros2_nvidia launch amr_mtt_bot nav2.launch.py
```

### 3. Run Localization AMCL
```bash
ros2 run amr_mtt_task_planner set_initial_pose
```

### 4. Launch Full Auto Task (Pick → Nav → Drop)
```bash
ros2 run amr_mtt_task_planner task_sequence
```

### 5. Node-RED Arm Control Dashboard
```bash
# Terminal 1: เปิด rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2: เปิด Node-RED
node-red

# Import flow แล้วเปิด Dashboard
# http://localhost:1880/ui
```
> Import: `node_red_flows/arm_control_flow.json` — ควบคุม joint ทีละตัวพร้อม preset poses + gripper


---

---

## Usage For test

### 1. Launch Main Simulation (Gazebo + Controllers)
```bash
ros2_nvidia launch amr_mtt_bot launch_sim_amr.launch.py
```
> **หลัก** — เปิด Ignition Gazebo + ros2_control + ส่งแขนกลไป HOME pose อัตโนมัติหลัง 12 วินาที
> จะถามตอน launch: กล้อง / Stereo Camera / LiDAR เปิดหรือปิด
> NVIDIA GPU offload. ใช้ GPU ปกติให้ใช้ `ros2 launch ...`

### 2. Run Navigation (ต่างหน้าต่าง)
```bash
ros2_nvidia launch amr_mtt_bot nav2.launch.py
```

### 3. Run Localization AMCL
```bash
ros2 run amr_mtt_task_planner set_initial_pose
```

### 4. Launch node_red_backend rosbridge
```bash
ros2 launch amr_mtt_task_planner node_red_backend.launch.py
```

### 5. Node-RED Arm Control Dashboard
```bash
# Terminal เปิด Node-RED
node-red

# Import flow แล้วเปิด Dashboard
# http://localhost:1880/ui
```
> Import: `node_red_flows/arm_control_flow.json` — ควบคุม joint ทีละตัวพร้อม preset poses + gripper


---

## Robot Specifications

| Parameter | Value |
|---|---|
| **Mobile Base** | Differential drive, 2 traction wheels + 4 caster wheels |
| **Chassis** | 0.90 × 0.64 × 0.20 m |
| **Chassis Collision Height** | 0.40 m (from URDF) |
| **Traction Wheel Radius** | 80 mm |
| **Robot Arm** | UR5 (6-DOF), base at 0.72 m above chassis |
| **Gripper** | Robotiq 2F-85, max opening 85 mm |
| **Parcel Box** | 6 × 6 × 8 cm, 0.1 kg |

---

## Tech Stack

| Technology | Role |
|---|---|
| **ROS 2 Humble** | Robot middleware and communication framework |
| **Python / rclpy** | ROS 2 node development |
| **MoveIt! 2** | Arm motion planning and collision-aware control |
| **Ignition Gazebo Fortress** | Physics simulation |
| **Nav2** | Autonomous mobile navigation stack |
| **URDF / Xacro** | Robot model description |
| **ros2_control** | Hardware interface and controller management |
| **CMake / ament** | Build system configuration |

---

## Known Issues & Notes

- **Gripper mimic joints**: Ignition Gazebo does not support URDF `<mimic>` joints natively. All mimic joints on the Robotiq 2F-85 are set to `fixed` type. Gripper closes via `left_knuckle_joint` only.
- **Gripper stall**: When gripping an object, `GripperCommand` action may timeout (3s). This is expected behavior — sequence continues automatically.
- **Simulation startup**: Allow ~5–8 seconds for Gazebo, MoveIt!, and RViz to fully initialize before running pick sequence.

---

## About

This project was developed as a final-year capstone project for the **Bachelor of Engineering in Mechatronics Technology (MtT)** program at the **College of Industrial Technology (CIT), King Mongkut's University of Technology North Bangkok (KMUTNB)**.

**Developer:** Phuwaset Sibta
**Institution:** KMUTNB, Bangkok, Thailand

---

## License

This project is developed for academic and educational purposes as part of a final-year engineering project at KMUTNB. All rights reserved by the author.
