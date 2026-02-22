# amr_mtt Quick Start Guide

## ⚡ Quick Launch Commands

```bash
chmod +x install_dependencies.sh
./install_dependencies.sh
``` 
## reset node & topic
```bash
killall -9 ros2
killall -9 rviz2
killall -9 tf
ros2 daemon stop
ros2 daemon start
```
> **Note:** Make sure to source the workspace first: `source ~/amr_mtt/install/setup.bash`

### 1. Basic Simulation (Robot Only)
```bash
source ~/amr_mtt/install/setup.bash
ros2 launch amr_mtt_bot ign.launch.py
```

### 2. Auto-Docking Demo
```bash
source ~/amr_mtt/install/setup.bash
ros2 launch amr_mtt_docking auto_docking_demo.launch.py
```

### 3. MoveIt (with Manipulator)
```bash
source ~/amr_mtt/install/setup.bash
ros2 launch amr_mtt_moveit_config moveit.launch.py use_sim_time:=true
```

### 4. Navigation & SLAM
```bash
source ~/amr_mtt/install/setup.bash
ros2 launch amr_mtt_bot nav2.launch.py use_sim_time:=true
```

### 5. RViz Visualization
```bash
source ~/amr_mtt/install/setup.bash
ros2 launch amr_mtt_bot rviz.launch.py
```

---

## 📦 Package Structure

```
amr_mtt/
├── amr_mtt_bot/              # Base robot (Resources + SLAM & Navigation)
├── amr_mtt_docking/          # Auto-docking logic only
└── amr_mtt_moveit_config/    # MoveIt configuration only
```

**Principle:** 
- `amr_mtt_bot` contains ALL robot resources (meshes, models, worlds, urdf)
- `amr_mtt_docking` and `amr_mtt_moveit_config` are extensions with no duplication

---

## 🔧 First Time Setup

### Automated Installation (Recommended)

Use the provided installation script to install all dependencies automatically:

```bash
cd ~/amr_mtt
./install_dependencies.sh
```

**What it does:**
- ✅ Installs Python packages (OpenCV, NumPy, etc.)
- ✅ Installs ROS 2 dependencies via rosdep
- ✅ Installs system packages (cv_bridge, nav2, slam_toolbox)
- ✅ Verifies installation

### Manual Setup

If you prefer manual installation, see [DEPENDENCIES.md](DEPENDENCIES.md) for detailed instructions.

### Build & Source

After installing dependencies:

```bash
# Build all packages
colcon build

# Source workspace (add to ~/.bashrc for auto-sourcing)
source install/setup.bash
```

---

## 📝 Common Tasks

### Build Specific Package
```bash
colcon build --packages-select amr_mtt_bot
colcon build --packages-select amr_mtt_docking
colcon build --packages-select amr_mtt_moveit_config
```

### Clean Build
```bash
rm -rf build install log
colcon build
```

### Auto-Source on Terminal Start
```bash
echo "source ~/amr_mtt/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🐛 Troubleshooting

### Missing Dependencies

If you encounter missing package errors:

```bash
# Re-run dependency installer
cd ~/amr_mtt
./install_dependencies.sh

# Or manually install missing packages
sudo apt update
sudo apt install ros-humble-<package-name>
```

### Python Package Issues

```bash
# Reinstall Python dependencies
pip3 install -r requirements.txt --upgrade

# Verify OpenCV installation
python3 -c "import cv2; print(cv2.__version__)"
```

### Package not found

```bash
# Make sure workspace is sourced
source ~/amr_mtt/install/setup.bash

# List available packages
ros2 pkg list | grep amr_mtt
# Should show: amr_mtt_bot, amr_mtt_docking, amr_mtt_moveit_config
```

### Build errors

```bash
# Clean and rebuild
rm -rf build install log
colcon build

# Or build specific package
colcon build --packages-select amr_mtt_bot
```

### rosdep errors

```bash
# Update rosdep database
rosdep update

# Install dependencies again
cd ~/amr_mtt
rosdep install --from-paths src --ignore-src -r -y
```

---

## 📚 Documentation

- **Full README:** [README.md](README.md)
- **Dependencies:** [DEPENDENCIES.md](DEPENDENCIES.md)
- **Installation Script:** [install_dependencies.sh](install_dependencies.sh)
- **Physics Analysis:** [src/amr_mtt/amr_mtt_bot/amr_mtt_Physics.md](src/amr_mtt/amr_mtt_bot/amr_mtt_Physics.md)

---

## 🚀 Quick Reference

| Task | Command |
|------|---------|
| **Install Dependencies** | `./install_dependencies.sh` |
| **Build All** | `colcon build` |
| **Build One Package** | `colcon build --packages-select amr_mtt_bot` |
| **Source Workspace** | `source install/setup.bash` |
| **Basic Simulation** | `ros2 launch amr_mtt_bot ign.launch.py` |
| **Auto-Docking** | `ros2 launch amr_mtt_docking auto_docking_demo.launch.py` |
| **MoveIt** | `ros2 launch amr_mtt_moveit_config moveit.launch.py` |
| **List Packages** | `ros2 pkg list \| grep amr_mtt` |
| **Clean Build** | `rm -rf build install log` |

