#!/bin/bash
# Dependency Installation Script for AMR-MTT
# Run this script from the workspace root after cloning:
#   cd ~/amr_mtt
#   bash src/amr_mtt/install_dependencies.sh

set -e  # Exit on error

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "================================================"
echo " AMR-MTT Dependency Installer"
echo "================================================"
echo " Workspace: $WS_ROOT"
echo ""

# ── OS Check ──────────────────────────────────────
if ! grep -q "22.04" /etc/os-release 2>/dev/null; then
    echo -e "${YELLOW}[WARN] This script is designed for Ubuntu 22.04 (ROS 2 Humble).${NC}"
    read -p "         Continue anyway? (y/n) " -n 1 -r
    echo
    [[ $REPLY =~ ^[Yy]$ ]] || exit 1
fi

# ── ROS 2 Humble Check ────────────────────────────
if ! command -v ros2 &>/dev/null; then
    echo -e "${RED}[ERROR] ros2 command not found.${NC}"
    echo "  Please install ROS 2 Humble first:"
    echo "  https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# ── 1. Update apt ─────────────────────────────────
echo -e "${BLUE}[1/7] Updating package lists...${NC}"
sudo apt update

# ── 2. Simulation — Ignition Fortress + ros_gz ────
echo -e "${BLUE}[2/7] Installing Ignition Gazebo (Fortress) + ROS bridge...${NC}"
sudo apt install -y \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ign-ros2-control

# ── 3. Navigation & SLAM ──────────────────────────
echo -e "${BLUE}[3/7] Installing Navigation2, SLAM, Localization...${NC}"
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization

# ── 4. MoveIt 2 & ros2_control ────────────────────
echo -e "${BLUE}[4/7] Installing MoveIt 2 and ros2_control...${NC}"
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager

# ── 5. Robot Description & Sensor packages ────────
echo -e "${BLUE}[5/7] Installing robot description, sensors, and vision...${NC}"
sudo apt install -y \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-tf2-sensor-msgs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-laser-geometry \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    libpcl-dev \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rviz2 \
    python3-opencv

# ── 6. Teleop, Joystick, AprilTag, UR5 ───────────
echo -e "${BLUE}[6/7] Installing teleop, AprilTag, Universal Robots...${NC}"
sudo apt install -y \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-apriltag-ros \
    ros-humble-ur \
    ros-humble-ur-description

# ── Python packages (data analysis / plotting) ────
echo -e "${BLUE}      Installing Python packages...${NC}"
pip3 install --quiet \
    numpy \
    matplotlib \
    Pillow \
    requests \
    scipy

# ── 7. rosdep for remaining deps ──────────────────
echo -e "${BLUE}[7/7] Running rosdep to catch any remaining dependencies...${NC}"
if ! command -v rosdep &>/dev/null; then
    sudo apt install -y python3-rosdep
fi

# Initialize rosdep only if not yet initialized
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

cd "$WS_ROOT"
rosdep install --from-paths src --ignore-src -r -y

# ── Verify ────────────────────────────────────────
echo ""
echo "================================================"
echo " Verifying installation"
echo "================================================"

check() {
    echo -n "  $1: "
    if eval "$2" &>/dev/null; then
        echo -e "${GREEN}OK${NC}"
    else
        echo -e "${RED}FAILED — $3${NC}"
    fi
}

check "Python numpy/matplotlib" \
    "python3 -c 'import numpy, matplotlib'" \
    "run: pip3 install numpy matplotlib"

check "Python opencv" \
    "python3 -c 'import cv2'" \
    "run: pip3 install opencv-python"

check "ROS nav2_bringup" \
    "ros2 pkg list | grep -q nav2_bringup" \
    "re-run step 3"

check "ROS slam_toolbox" \
    "ros2 pkg list | grep -q slam_toolbox" \
    "re-run step 3"

check "ROS moveit" \
    "ros2 pkg list | grep -q moveit_ros_move_group" \
    "re-run step 4"

check "ROS ros_gz_sim" \
    "ros2 pkg list | grep -q ros_gz_sim" \
    "re-run step 2 (Ignition Fortress)"

check "ROS apriltag_ros" \
    "ros2 pkg list | grep -q apriltag_ros" \
    "re-run step 6"

check "ROS ur_description" \
    "ros2 pkg list | grep -q ur_description" \
    "re-run step 6"

# ── Next Steps ────────────────────────────────────
echo ""
echo -e "${GREEN}================================================"
echo " Installation Complete!"
echo "================================================${NC}"
echo ""
echo "Next steps:"
echo ""
echo "  1. Build the workspace:"
echo "     cd $WS_ROOT"
echo "     colcon build --symlink-install"
echo ""
echo "  2. Source the workspace:"
echo "     source install/setup.bash"
echo "     # หรือเพิ่มใน ~/.bashrc (ถาวร):"
echo "     echo 'source $WS_ROOT/install/setup.bash' >> ~/.bashrc"
echo ""
echo "  3. เปิด Simulation (เลือกอย่างใดอย่างหนึ่ง):"
echo ""
echo "     # หุ่น + Ignition Gazebo:"
echo "     ros2 launch amr_mtt_bot ign.launch.py"
echo ""
echo "     # Auto-docking (AprilTag):"
echo "     ros2 launch amr_mtt_docking auto_docking_demo.launch.py"
echo ""
echo "     # MoveIt (แขนกล UR5):"
echo "     ros2 launch amr_mtt_moveit_config moveit.launch.py"
echo ""
echo "     # Navigation2 (ต้อง map ก่อน):"
echo "     ros2 launch amr_mtt_bot nav2.launch.py"
echo ""
