#!/bin/bash
# Quick Dependency Installation Script for AMR-MTT
# Run this after cloning the repository

set -e  # Exit on error

echo "================================================"
echo "amr_mtt Dependency Installation"
echo "================================================"
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running on Ubuntu 22.04
if ! grep -q "22.04" /etc/os-release; then
    echo -e "${YELLOW}Warning: This script is designed for Ubuntu 22.04${NC}"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Update package lists
echo -e "${GREEN}[1/5] Updating package lists...${NC}"
sudo apt update

# Install Python dependencies
echo -e "${GREEN}[2/5] Installing Python dependencies...${NC}"
pip3 install -r requirements.txt

# Install ROS 2 dependencies via rosdep
echo -e "${GREEN}[3/5] Installing ROS 2 dependencies...${NC}"
cd "$(dirname "$0")"  # Navigate to script directory
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install additional system dependencies
echo -e "${GREEN}[4/5] Installing additional system packages...${NC}"
sudo apt install -y \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-rviz2

# Verify installation
echo -e "${GREEN}[5/5] Verifying installation...${NC}"
echo -n "  - Python packages: "
if python3 -c "import cv2, numpy" 2>/dev/null; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAILED${NC}"
fi

echo -n "  - ROS 2 packages: "
if ros2 pkg list | grep -q "nav2" && ros2 pkg list | grep -q "slam_toolbox"; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAILED${NC}"
fi

echo ""
echo -e "${GREEN}================================================"
echo "Installation Complete!"
echo "================================================${NC}"
echo ""
echo "Next steps:"
echo "  1. Build the workspace:"
echo "     colcon build"
echo ""
echo "  2. Source the workspace:"
echo "     source install/setup.bash"
echo ""
echo "  3. Run simulation (choose one):"
echo "     # Basic robot:"
echo "     ros2 launch amr_mtt_bot ign.launch.py"
echo ""
echo "     # Auto-docking:"
echo "     ros2 launch amr_mtt_docking auto_docking_demo.launch.py"
echo ""
echo "     # MoveIt:"
echo "     ros2 launch amr_mtt_moveit_config moveit.launch.py"
echo ""
