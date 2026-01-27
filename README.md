# amr_mtt project

<!-- https://github.com/blackcoffeerobotics/amr_mtt/assets/13151010/0fc570a3-c70c-415b-8222-b9573d5911c8 -->

## About

This repository contains a Gazebo and Isaac Sim simulation for a differential drive robot, equipped with an IMU, a depth camera, stereo camera and a 2D LiDAR. The primary contriution of this project is to support multiple ROS and Gazebo distros. Currently, the project supports the following versions - 

1. [ROS Noetic + Gazebo Classic 11 (branch ros1)](#noetic--classic-ubuntu-2004)
2. [ROS2 Humble + Gazebo Classic 11 (branch ros2)](#humble--classic-ubuntu-2204)
3. [ROS2 Humble + Gazebo Fortress (branch ros2)](#humble--fortress-ubuntu-2204)
4. [ROS2 Humble + Gazebo Harmonic (branch ros2)](#humble--harmonic-ubuntu-2204)
5. [ROS2 Humble + Isaac Sim (branch ros2)](#humble--isaac-sim-ubuntu-2204)

Each of the following sections describes depedencies, build and run instructions for each of the above combinations

## Noetic + Classic (Ubuntu 20.04)

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
```

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

ros2_nvidia launch amr_mtt ign.launch.py \
  camera_enabled:=True \
  stereo_camera_enabled:=False \
  two_d_lidar_enabled:=True \
  position_x:=0.0 \
  position_y:=0.0 \
  orientation_yaw:=0.0 \
  odometry_source:=world \
  world_file:=small_warehouse.sdf
```

## MAP
```
ros2 launch amr_mtt mapping.launch.py use_sim_time:=true

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