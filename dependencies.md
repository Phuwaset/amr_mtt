# AMR-MTT Dependency Installation Guide

## System Requirements
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10+
- Gazebo Fortress/Harmonic

---

## Installation Steps

### 1. Install ROS 2 Dependencies

```bash
# Navigate to workspace root
cd ~/amr_mtt

# Install ROS 2 package dependencies using rosdep
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Install Python Dependencies

```bash
# Install Python packages
pip3 install -r requirements.txt

# Or install manually:
pip3 install opencv-python>=4.5.0 opencv-contrib-python>=4.5.0 numpy>=1.19.0
```

### 3. Install Additional System Dependencies

```bash
# OpenCV system dependencies (if needed)
sudo apt install -y \
    python3-opencv \
    libopencv-dev

# Navigation stack (if not already installed)
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox

# Visualization
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins
```

### 4. Verify Installation

```bash
# Check Python dependencies
python3 -c "import cv2; import numpy; print('Python deps OK')"

# Check ROS 2 packages
ros2 pkg list | grep -E "(nav2|slam_toolbox|cv_bridge)"
```

---

## Troubleshooting

### Issue: cv_bridge not found
```bash
sudo apt install ros-humble-cv-bridge
```

### Issue: ArUco detection not working
```bash
# Ensure opencv-contrib is installed
pip3 install --upgrade opencv-contrib-python
```

### Issue: Nav2 packages missing
```bash
sudo apt install ros-humble-navigation2
```

### Issue: Python import errors
```bash
# Ensure correct Python path
echo $PYTHONPATH
source /opt/ros/humble/setup.bash
```

---

## Dependency Summary

| Category | Packages |
|----------|----------|
| **ROS 2 Core** | rclpy, rclcpp, launch, ament |
| **Messages** | sensor_msgs, geometry_msgs, nav2_msgs, visualization_msgs |
| **Vision** | cv_bridge, opencv-python, image_transport |
| **Navigation** | nav2_bringup, slam_toolbox, nav2_map_server |
| **TF** | tf2_ros, tf2_sensor_msgs |
| **Sensors** | dual_laser_merger, laser_geometry |
| **Python** | numpy, opencv-python, PyYAML |

---

## Build Instructions

```bash
# After installing dependencies
cd ~/amr_mtt
colcon build --packages-select amr_mtt
source install/setup.bash
```

---

Generated for AMR-MTT Project v1.0.2
