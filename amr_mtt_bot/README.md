# amr_mtt project

<!-- https://github.com/blackcoffeerobotics/amr_mtt/assets/13151010/0fc570a3-c70c-415b-8222-b9573d5911c8 -->

## About

This repository contains a Gazebo and Isaac Sim simulation for a differential drive robot, equipped with an IMU, a depth camera, stereo camera and a 2D LiDAR. The primary contriution of this project is to support multiple ROS and Gazebo distros. Currently, the project supports the following versions - 

<!-- 1. [ROS Noetic + Gazebo Classic 11 (branch ros1)](#noetic--classic-ubuntu-2004) -->
2. [ROS2 Humble + Gazebo Classic 11 (branch ros2)](#humble--classic-ubuntu-2204)
3. [ROS2 Humble + Gazebo Fortress (branch ros2)](#humble--fortress-ubuntu-2204)
4. [ROS2 Humble + Gazebo Harmonic (branch ros2)](#humble--harmonic-ubuntu-2204)
5. [ROS2 Humble + Isaac Sim (branch ros2)](#humble--isaac-sim-ubuntu-2204)

Each of the following sections describes depedencies, build and run instructions for each of the above combinations

<!-- ## Noetic + Classic (Ubuntu 20.04)

### Dependencies

In addition to ROS1 Noetic and Gazebo Classic installations, the dependencies can be installed with [rosdep](http://wiki.ros.org/rosdep)

```bash
# From the root directory of the workspace. This will install everything mentioned in package.xml
rosdep install --from-paths src --ignore-src -r -y
```

### Source Build

```bash
catkin build --packages-select amr_mtt
```

### Binary Install
To install BCR bot in the binaries:

```bash
sudo apt-get install ros-noetic-bcr-bot
```
### Run

To launch the robot in Gazebo,
```bash
roslaunch amr_mtt gazebo.launch
```
To view in rviz,
```bash
roslaunch amr_mtt rviz.launch
```
### Configuration

The launch file accepts multiple launch arguments,
```bash
roslaunch amr_mtt gazebo.launch 
	camera_enabled:=True \
	two_d_lidar_enabled:=True \
	position_x:=0.0 \
	position_y:=0.0 \
	orientation_yaw:=0.0 \
	odometry_source:=world \
	world_file:=small_warehouse.world \
	robot_namespace:="amr_mtt"
```
**Note:** To use stereo_image_proc with the stereo images excute following command: 
```bash
ROS_NAMESPACE=amr_mtt/stereo_camera rosrun stereo_image_proc stereo_image_proc
``` -->

## Humble + Classic (Ubuntu 22.04)

### Dependencies

In addition to ROS2 Humble and Gazebo Classic installations, we need to manually install [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2) (since the same branch supports Classic and Fortress)

```bash
sudo apt-get install ros-humble-gazebo-ros-pkgs
```
Remainder of the dependencies can be installed with [rosdep](http://wiki.ros.org/rosdep)

```bash
# From the root directory of the workspace. This will install everything mentioned in package.xml
rosdep install --from-paths src --ignore-src -r -y
```

### Source Build

```bash
colcon build --packages-select amr_mtt
```

### Binary Install
To install BCR bot in the binaries:

```bash
sudo apt-get install ros-humble-bcr-bot
```

### Run

To launch the robot in Gazebo,
```bash
ros2 launch amr_mtt gazebo.launch.py
```
To view in rviz,
```bash
ros2 launch amr_mtt rviz.launch.py
```
### Configuration

The launch file accepts multiple launch arguments,
```bash
ros2 launch amr_mtt gazebo.launch.py \
	camera_enabled:=True \
	two_d_lidar_enabled:=True \
	stereo_camera_enabled:=False \
	position_x:=0.0 \
	position_y:=0.0 \
	orientation_yaw:=0.0 \
	odometry_source:=world \
	world_file:=small_warehouse.sdf \
	robot_namespace:="amr_mtt"
```
**Note:** To use stereo_image_proc with the stereo images excute following command: 
```bash
ros2 launch stereo_image_proc stereo_image_proc.launch.py left_namespace:=amr_mtt/stereo_camera/left right_namespace:=amr_mtt/stereo_camera/right
```
## Humble + Fortress (Ubuntu 22.04)

### Dependencies

In addition to ROS2 Humble and [Gazebo Fortress installations](https://gazebosim.org/docs/fortress/install_ubuntu), we need to manually install interfaces between ROS2 and Gazebo sim as follows,

```bash
sudo apt-get install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces 
```
Remainder of the dependencies can be installed with [rosdep](http://wiki.ros.org/rosdep)

```bash
# From the root directory of the workspace. This will install everything mentioned in package.xml
rosdep install --from-paths src --ignore-src -r -y
```

### Source Build

```bash
colcon build --packages-select amr_mtt
```

### Binary Install
To install BCR bot in the binaries:

```bash
sudo apt-get install ros-humble-bcr-bot
```

### Run

To launch the robot in Gazebo,
```bash
ros2 launch amr_mtt ign.launch.py
```
To view in rviz,
```bash
ros2 launch amr_mtt rviz.launch.py
```

### Configuration

The launch file accepts multiple launch arguments,
```bash
ros2 launch amr_mtt ign.launch.py \
	camera_enabled:=True \
	stereo_camera_enabled:=False \
	two_d_lidar_enabled:=True \
	position_x:=0.0 \
	position_y:=0.0  \
	orientation_yaw:=0.0 \
	odometry_source:=world \
	world_file:=small_warehouse.sdf
```
**Note:** To use stereo_image_proc with the stereo images excute following command: 
```bash
ros2 launch stereo_image_proc stereo_image_proc.launch.py left_namespace:=amr_mtt/stereo_camera/left right_namespace:=amr_mtt/stereo_camera/right
```

## Humble + Harmonic (Ubuntu 22.04)

### Dependencies

In addition to ROS2 Humble and [Gazebo Harmonic installations](https://gazebosim.org/docs/harmonic/install_ubuntu), we need to manually install interfaces between ROS2 and Gazebo sim as follows,

```bash
sudo apt-get install ros-humble-ros-gzharmonic
```
Remainder of the dependencies can be installed with [rosdep](http://wiki.ros.org/rosdep)

```bash
# From the root directory of the workspace. This will install everything mentioned in package.xml
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
colcon build --packages-select amr_mtt
```

### Run

To launch the robot in Gazebo,
```bash
ros2 launch amr_mtt gz.launch.py
```
To view in rviz,
```bash
ros2 launch amr_mtt rviz.launch.py
```

### Configuration

The launch file accepts multiple launch arguments,
```bash
ros2 launch amr_mtt gz.launch.py \
	camera_enabled:=True \
	stereo_camera_enabled:=False \
	two_d_lidar_enabled:=True \
	position_x:=0.0 \
	position_y:=0.0  \
	orientation_yaw:=0.0 \
	odometry_source:=world \
	world_file:=small_warehouse.sdf
```
**Note:** 
1. To use stereo_image_proc with the stereo images excute following command: 
```bash
ros2 launch stereo_image_proc stereo_image_proc.launch.py left_namespace:=amr_mtt/stereo_camera/left right_namespace:=amr_mtt/stereo_camera/right
```
2. Harmonic support is not available in the amr_mtt binaries yet.

**Warning:**  `gz-harmonic` cannot be installed alongside gazebo-classic (eg. gazebo11) since both use the `gz` command line tool.

### Humble + Isaac Sim (Ubuntu 22.04)

### Dependencies

In addition to ROS2 Humble [Isaac Sim installation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/index.html) with ROS2 extension is required. Remainder of amr_mtt specific dependencies can be installed with [rosdep](http://wiki.ros.org/rosdep)

```bash
# From the root directory of the workspace. This will install everything mentioned in package.xml
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
colcon build --packages-select amr_mtt
```

### Run

To launch the robot in Isaac Sim:
- Open Isaac Sim and load the `warehouse_scene.usd` or `scene.usd` from [here](usd). 
- Add in extra viewports for different camera views.
- Start the Simulation: Run the simulation directly within Isaac Sim.
- The following USDs are included in the package:
	- `warehouse_scene.usd` - Warehouse scene with a robot.
	- `scene.usd` - Scene with a robot in a empty world.
	- `amr_mtt.usd` - Robot model that can be imported into any scene.
	- `ActionGraphFull.usd` - Action graph for the robot to publish all the required topics.

To view in rviz:
```bash
ros2 launch amr_mtt rviz.launch.py
```
NOTE: The command to run mapping and navigation is common between all versions of gazebo and Isaac sim see [here](#mapping-with-slam-toolbox).

### Mapping with SLAM Toolbox

SLAM Toolbox is an open-source package designed to map the environment using laser scans and odometry, generating a map for autonomous navigation.

NOTE: The command to run mapping is common between all versions of gazebo.

To start mapping:
```bash
ros2 launch amr_mtt mapping.launch.py
```

Use the teleop twist keyboard to control the robot and map the area:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/amr_mtt/cmd_vel
```

To save the map:
```bash
cd src/amr_mtt/config
ros2 run nav2_map_server map_saver_cli -f bcr_map
```

### Using Nav2 with amr_mtt

Nav2 is an open-source navigation package that enables a robot to navigate through an environment easily. It takes laser scan and odometry data, along with the map of the environment, as inputs.

NOTE: The command to run navigation is common between all versions of gazebo and Isaac sim.

To run Nav2 on amr_mtt:
```bash
ros2 launch amr_mtt nav2.launch.py
```

### Simulation and Visualization
1. Gz Sim (Ignition Gazebo) (small_warehouse World):
	![](res/gz.jpg)

2. Isaac Sim:
	![](res/isaac.jpg) 

3. Rviz (Depth camera) (small_warehouse World):
	![](res/rviz.jpg) -->














# amr_mtt: Autonomous Mobile Robot Project
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue) ![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange) ![Gazebo](https://img.shields.io/badge/Simulation-Ignition-green)

Project Simulayion ROS2&Gz-sim AMR (Autonomous Mobile Robot) develop by ROS2 Humble and Ignition Gazebo

## Nvdia setup nvidia-smi

### ตรวจสอบสถานะการทำงาน 
```
nvidia-smi
```
### auto update 
``` 
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
```
### Task manager like htop but this's nvidia 
```
sudo apt update
sudo apt install nvtop
```
#### เรียกใช้ 
```
nvitop
nvtop
```

### ros2_nvidia เพื่อเรียกใช้ การ์จอ ในการ simulation เพิ่ม Alias ไว้ในไฟล์ .bashrc
```
nano ~/.bashrc
```
```
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch amr_mtt gz.launch.py
```
### Example how to run ros2_nvidia launch ....
```
ros2_nvidia launch amr_mtt ign.launch.py   
camera_enabled:=True   
stereo_camera_enabled:=False   
two_d_lidar_enabled:=True   
position_x:=0.0  
position_y:=0.0   
orientation_yaw:=0.0   
odometry_source:=world   
world_file:=small_warehouse.sdf
use_sim_time:=true
```

```
ros2_nvidia launch amr_mtt rviz.launch.py
```

## 🛠️ Getting Started 
ก่อนเริ่มใช้งานทุกครั้ง ให้ทำการ Source Environment ของ Workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/amr_mtt/install/setup.bash
```

## launch gz-sim
### Simulation with ROS2 Project AMR_MtT
```
source /opt/ros/humble/setup.bash
source ~/amr_mtt/install/setup.bash

ros2_nvidia launch amr_mtt_bot ign.launch.py \
  camera_enabled:=True \
  stereo_camera_enabled:=False \
  two_d_lidar_enabled:=True \
  position_x:=0.0 \
  position_y:=0.0 \
  orientation_yaw:=0.0 \
  odometry_source:=world \
  world_file:=small_warehouse.sdf
  use_sim_time:=true
```
## teleop_twist_keyboard 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/amr_mtt/cmd_vel
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```
## MAP
```
ros2 launch amr_mtt_bot mapping.launch.py use_sim_time:=true

```
## Save_Map
```
cd src/amr_mtt/config
ros2 run nav2_map_server map_saver_cli -f amr_mtt
```

## localization_launch
```
source /opt/ros/humble/setup.bash
source ~/amr_mtt/install/setup.bash

ros2 launch nav2_bringup localization_launch.py \
map:=$HOME/amr_mtt/src/amr_mtt/map/amr_mtt_map1.yaml \
use_sim_time:=true
```
## Navigation
```
ros2 launch amr_mtt nav2.launch.py use_sim_time:=true
```

## 📐 Robot Physics Configuration & Analysis

### 1. Mass Distribution and Center of Gravity (CG)
* **Total Mass:** ~115.7 kg 
  * Chassis: 60 kg
  * Battery: 20 kg
  * UR5 Arm: ~18.5 kg
  * Wheels (Traction + Trolley): ~17.2 kg
* **Balance Calculation (X-Axis):** The heavy UR5 arm is placed forward (`X = 0.2`) while the heavy battery acts as a counterweight placed backward (`X = -0.2`). This design shifts the Center of Gravity (CG) directly to the center (`X = 0.0`), right above the traction wheels.
* **Why this matters:** Placing the CG squarely over the traction wheels maximizes grip (downward force) and prevents the robot from tipping over (doing a wheelie or nose-dive) during sudden acceleration or emergency braking.
* **Low CG:** Most of the mass (chassis + battery) is located extremely close to the ground (`Z = 0.02` to `0.1`), ensuring the robot remains highly stable during fast turns despite bearing a tall robotic arm.

### 2. Wheel Friction & Contact Physics (`macros.xacro`)
* **Traction Wheels:** `mu1/mu2 = 5.0`
  * *Reason:* Extremely high friction ensures the wheels grip the floor securely, preventing slippage during acceleration and allowing precise wheel odometry mapping.
* **Trolley (Caster) Wheels:** `mu1/mu2 = 0.0`
  * *Reason:* Zero friction allows these omnidirectional supporting wheels to slide completely freely during rotation. This prevents resistance when the robot pivots (turns in place) and avoids Gazebo simulation jitter.
* **Stiffness & Damping (`kp`, `kd`, `minDepth`):** 
  * `kp = 10000000.0` (Stiffness) | `kd = 1.0` (Damping) | `minDepth = 0.001`
  * *Reason:* These parameters dictate the tire hardness and suspension. They prevent Gazebo from overcompensating for micro-collisions between the wheels and the floor, effectively eliminating "shaking and stuttering" visual bugs in RViz.

### 3. Motor Dynamics and Limits (`controllers.yaml`)
* **Limits Calculation:**
  * Linear velocity: `1.0 m/s`, Acceleration: `0.2 m/s²`
  * Angular velocity: `2.0 rad/s`, Acceleration: `0.5 rad/s²`
  * Jerk (Acceleration derivative): Linear `0.5`, Angular `0.5`
* **Why this matters:** Soft acceleration (`0.2`) and low jerk (`0.5`) limits create an "S-curve" motion profile. This simulates an industrial motor driver ramping up its speed gently to protect the UR5 arm's mechanical joints from sudden shocks during start/stop phases.



การกระจายน้ำหนักและจุดสมดุล (Center of Gravity - CG)
น้ำหนักรวม: ประมาณ 115.7 kg (ตัวถัง 60kg, แบตเตอรี่ 20kg, แขนหุ่นยนต์ 18.5kg, ชุดล้อรวม 17.2kg)
การคำนวณจุดสมดุล (แกน X): แขนหุ่นยนต์ (UR5) ที่หนักมากถูกวางไว้ด้านหน้า (X = 0.2) ส่วนแบตเตอรี่ที่เป็นก้อนน้ำหนักถ่วงถูกวางห่างไปด้านหลัง (X = -0.2) ทำให้น้ำหนักทั้งสองฝั่งหักล้างกัน จุดศูนย์ถ่วง (CG) จึงตกลงมาที่ตรงกลางเป๊ะ (X = 0.0) ซึ่งเป็นตำแหน่งเดียวกับล้อขับเคลื่อน (Traction Wheels) พอดี
ทำไมถึงตั้งค่าแบบนี้? การเทน้ำหนักทั้งหมดให้กดลงที่ล้อขับเคลื่อน จะทำให้ล้อมีแรงยึดเกาะพื้นสูงสุด ไม่เกิดอาการล้อฟรีหรือลอย และการมี CG ต่ำมาก (จากโครงสร้างตัวถังและแบตเตอรี่ที่ติดพื้น) ทำให้หุ่นยนต์ไม่ล้มคว่ำหน้า-หลังเวลาเบรกกะทันหัน แม้จะมีแขนหุ่นยนต์ตั้งขึ้นไปถึงครึ่งเมตรก็ตาม
2. ฟิสิกส์การสัมผัสพื้นของล้อ (Wheel Friction & Contact)
ล้อเลี้ยว / ล้อประคอง (Trolley Wheels):
ตั้งค่าแรงเสียดทาน (mu1/mu2) เป็น 0.0
เหตุผล: เพื่อให้ล้อทั้ง 4 มุมสามารถแฉลบหรือหันเลี้ยวได้อย่างอิสระเหมือนลูกปืนลื่นๆ ป้องกันการงัดกับพื้นเวลาหุ่นยนต์หมุนตัว 제자리 (Pivot) ซึ่งช่วยตัดปัญหาหน้าจอค้างหรือกระตุกใน Gazebo
ล้อขับเคลื่อน (Traction Wheels):
ตั้งค่าแรงเสียดทาน (mu1/mu2) เป็น 5.0 (สูงมาก)
เหตุผล: เป็นล้อสองข้างที่ใช้ขับเคลื่อนจริง จึงต้องเกาะติดพื้นแบบไม่ไถลเลย (No Slip) เพื่อให้การอ่านค่าระยะทางและตำแหน่งจากล้อ (Wheel Odometry) แม่นยำที่สุด
ความแข็งของยางและโช้ค (kp, kd, minDepth):
ความแข็ง kp = 10,000,000.0 / ความหนืด kd = 1.0 / การหยุมลงพื้น minDepth = 0.001
เหตุผล: ช่วยจำลองให้ยางมีลักษณะค่อนข้างแข็ง ไม่เด้งเป็นสปริง และช่วยปิดช่องโหว่ของเอนจิ้นฟิสิกส์ Gazebo ที่ส่งผลให้ล้อสั่นเมื่อชนกับระนาบพื้น
3. ขีดจำกัดความเร็วมอเตอร์ขับเคลื่อน (จากไฟล์ controllers.yaml)
ความเร็วเชิงเส้น (วิ่งตรง): ลิมิตสูงสุด 1.0 เมตร/วินาที, อัตราเร่ง 0.2 เมตร/วินาที²
ความเร็วเชิงมุม (หมุนตัว): ลิมิตสูงสุด 2.0 rad/วินาที, อัตราเร่ง 0.5 rad/วินาที²
การกระชาก (Jerk Limit): กำหนดไว้ต่ำมากที่ 0.5
ทำไมถึงตั้งค่าแบบนี้? การกดค่าลิมิต อัตราเร่ง (Acceleration) และการกระชาก (Jerk) ให้น้อย ทำให้เวลาหุ่นยนต์ออกตัว หรือเบรกหยุด จะมีพฤติกรรมโค้งแบบ "S-curve" (ค่อยๆ ออกตัว แล้วค่อยๆ จอด) ช่วยดูดซับแรงสั่นสะเทือนไม่ให้ย้อนขึ้นไปกระแทกและทำลายข้อต่อ (Joints) ของแขน UR5 ที่ด้านบน




สำหรับโปรเจกต์จบระดับปริญญาตรี (Senior Project) ด้านหุ่นยนต์ AMR (Autonomous Mobile Robot) ที่เน้นทำ Simulation ด้วย ROS 2 และ Gazebo / Isaac Sim นั้น การเก็บผลการทดลอง (Experiment Results) ต้องแสดงให้กรรมการเห็นว่า "ระบบที่เราออกแบบมาทำงานได้ดีระดับไหน และมีความน่าเชื่อถือจริงหรือไม่"

นี่คือ หัวข้อการเก็บผลการทดลอง (Metrics) ที่นิยมทำเป็นกราฟ/ตารางในเล่มโปรเจกต์จบครับ แบ่งตามระบบการทำงานหลักๆ ดังนี้ครับ:

1. ระบบสร้างแผนที่ (Mapping & SLAM Performance)
เป้าหมาย: ประเมินว่าหุ่นยนต์สร้างแผนที่ได้ตรงกับสภาพแวดล้อมจริงแค่ไหน

ความแม่นยำของแผนที่ (Map Accuracy / RMSE): ถ่ายภาพ Top-view ของ World ดั้งเดิม (Ground Truth) ใน Gazebo แล้วเอามาซ้อนทับ (Overlay) กับแผนที่ map.yaml ที่ SLAM สร้างขึ้นมา วัดว่าเส้นขอบกำแพงเบี้ยวไปกี่เซนติเมตร
เวลาในการปิด Loop (Loop Closure Time): วัดว่าตอนหุ่นวิ่งวนกลับมาที่เดิม (Loop Closure) อัลกอริทึม SLAM ใช้เวลากี่วินาทีในการแก้ความเบี้ยวของแผนที่ (Map Correction)
ผลของ Noise ต่อแผนที่: ลองปรับค่า Noise ของ LiDAR ใน 

ign.xacro
 (เช่น Gaussian Noise) แล้วเปรียบเทียบว่าแผนที่ที่ออกมาเละแค่ไหนเมื่อตัวเซ็นเซอร์มีคุณภาพต่ำลง
2. ระบบการหาตำแหน่ง (Localization Performance)
เป้าหมาย: ประเมินว่าหุ่นยนต์รู้ว่าตัวเองอยู่ที่ไหนได้แม่นยำแค่ไหน (通常ใช้แพ็กเกจ AMCL หรือ Nav2 AMCL)

Position Error (ความคลาดเคลื่อนเชิงตำแหน่ง): ใน Simulation เราสามารถดึงค่าตำแหน่ง "จริงเป๊ะๆ" (Ground Truth จาก Gazebo odom หรือ tf) มาเทียบกับค่าตำแหน่ง "ที่หุ่นยนต์คิด" (จาก amcl_pose) แล้วพล็อตเป็นกราฟ Error ในแกน X, Y และทิศทาง Yaw
Recovery Time (เวลาในการหาตัวเองเจอ): ลองทดสอบฟังก์ชัน "ลักพาตัวหุ่นยนต์" (Kidnapped Robot Problem) คือสั่งวาร์ปหุ่นยนต์ใน Gazebo ไปที่อื่น แล้วรอดูว่าระบบ Nav2 ใช้เวลาและระยะทางวิ่งกี่เมตรกว่าอนุภาค (Particles) ของ AMCL จะจับกลุ่มถูกจุดอีกครั้ง
3. ระบบนำทางอัตโนมัติ (Navigation & Path Planning)
เป้าหมาย: ประเมินว่าหุ่นยนต์วิ่งฉลาดแค่ไหน หลบหลีกสิ่งกีดขวางได้ดีไหม

Success Rate (อัตราส่วนความสำเร็จ): สั่งรัน Nav2 ไปยังเป้าหมาย (Goal) 20-30 รอบในระยะทางต่างๆ นับว่าสำเร็จกี่เปอร์เซ็นต์ เฟลกี่เปอร์เซ็นต์ (เช่น ชน, หมุนหาทางไม่เจอจนหมดเวลา)
Path Efficiency (ประสิทธิภาพเส้นทาง): เปรียบเทียบ "ระยะทางที่สั้นที่สุดตามทฤษฎี" (Global Path) กับ "ระยะทางที่หุ่นยนต์วิ่งหลบหลีกจริงๆ" (Actual Trajectory) ว่าพุ่งออกนอกเส้นทางไปกี่เปอร์เซ็นต์
Dynamic Obstacle Avoidance (การหลบหลีกวัตถุเคลื่อนที่): สร้างกล่องหรือคนให้เดินตัดหน้าหุ่นยนต์กะทันหันใน Gazebo แล้วอัดวิดีโอ/กราฟความเร็ว ว่าหุ่นยนต์ชะลอความเร็ว (Deceleration) และเบี่ยงตัวหลบ (Local Planner) ใช้กี่วินาที
Clearance Distance (ระยะห่างความปลอดภัย): เก็บค่ากึ่งกลางตัวหุ่นยนต์ เทียบกับสิ่งกีดขวางที่ใกล้ที่สุดตลอดการเดินทาง ว่าหุ่นยนต์วิ่งเฉียดกำแพงหรือสิ่งกีดขวางมากเกินไปไหม (ต่ำกว่ารัศมี robot_radius หรือไม่)
4. ฟิสิกส์และการกินพลังงานจำลอง (Dynamics & Simulated Power - Optional แต่เท่มาก)
เป้าหมาย: โชว์ความเข้าใจในตัวฮาร์ดแวร์

Tracking Error ของล้อ (Command vs. Actual Velocity): พล็อตกราฟเปรียบเทียบความเร็ว cmd_vel ที่ Nav2 สั่ง เทียบกับค่าความเร็ว odom จริงของล้อ ว่ามันตามทันคำสั่งไหม (มี Delay/Overshoot ไหม)
(ระดับสูง) แรงบิดมอเตอร์ (Motor Torque Profiles): ดึงกราฟค่า Effort/Torque ของ diff_drive_controller เวลาที่หุ่นยนต์เริ่มออกตัวพร้อมแบกแขนกล UR5 เพื่อดูว่าโหลดกระชากมอเตอร์เยอะไหม
เครื่องมือที่ใช้เก็บผลการทดลองใน ROS 2
rosbag2 (เดอะแบก): ใช้คำสั่ง ros2 bag record -a เพื่อบันทึกทุก Topic ขณะทดลองวิ่ง แล้วค่อยเอากลับมาเปิดเล่น (Playback) เพื่อดูกราฟทีหลัง
PlotJuggler: โปรแกรมเทพสำหรับดูกราฟ ROS 2 แค่โหลด Bag file เข้าไป ก็พล็อต Position Error (Ground Truth vs AMCL) ออกมาได้อย่างสวยงาม เอาไปลงเล่ม Report ได้เลย
RQt / RViz2: อัดหน้าจอโชว์ Particles กระจายตัว และ Path Planning ให้กรรมการเห็นภาพ
คำแนะนำเพิ่มเติม: สำหรับระดับ ป.ตรี ไม่จำเป็นต้องทำครบทุกข้อครับ แนะนำให้เลือกโฟกัสที่จุดเด่นของโครงงานคุณ เช่น ถ้างบโครงงานทำเรื่อง "การนำทางในโกดัง" ให้เน้นทำผลการทดลองข้อ 3. การนำทาง (Navigation) ให้ละเอียดที่สุดเลยครับ สร้างโกดังแคบๆ มีคนเดินไปมาใน Simuation แล้วจับเวลา/Success rate ทำเป็นกราฟเปรียบเทียบกันครับ


