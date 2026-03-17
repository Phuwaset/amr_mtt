# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

**amr_mtt** is a ROS 2 Humble workspace for a mobile manipulation robot — a differential-drive AMR with a UR5 arm and Robotiq 2F-85 gripper, simulated in Ignition Gazebo Fortress. It integrates Nav2 navigation, MoveIt 2 motion planning, and AprilTag-based auto-docking.

## Build & Test

```bash
# Source environment
source /opt/ros/humble/setup.bash

# Build entire workspace
cd ~/amr_mtt
colcon build --symlink-install

# Source install after build
source install/setup.bash

# Run tests (task_planner is the only Python package with tests)
colcon test --packages-select amr_mtt_task_planner
colcon test-result --verbose

# Lint Python code
python3 -m flake8 src/amr_mtt/amr_mtt_task_planner --max-line-length=100
```

## Launch Workflows

All launch commands require the environment to be sourced first. Use `ros2_nvidia` instead of `ros2` when launching Gazebo to ensure proper GPU acceleration.

**Full simulation with MoveIt + Nav2 (most common):**
```bash
# Terminal 1
ros2_nvidia launch amr_mtt_moveit_config moveit.launch.py
# Terminal 2 (after sim is ready)
ros2_nvidia launch amr_mtt_bot nav2.launch.py
# Terminal 3 (autonomous task)
ros2 run amr_mtt_task_planner task_sequence
```

**Basic simulation only:**
```bash
ros2_nvidia launch amr_mtt_bot launch_sim_amr.launch.py
# Interactive prompts to enable/disable cameras and LiDAR
```

**Mapping:**
```bash
ros2_nvidia launch amr_mtt_bot ign.launch.py
ros2 launch amr_mtt_bot mapping.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/amr_mtt/cmd_vel
# Save map:
ros2 run nav2_map_server map_saver_cli -f ~/amr_mtt/src/amr_mtt/amr_mtt_bot/map/amr_mtt_new
```

**Auto-docking:**
```bash
ros2 launch amr_mtt_docking auto_docking_demo.launch.py
```

## Architecture

### Package Responsibilities

| Package | Role |
|---|---|
| `amr_mtt_bot` | Robot description (URDF/xacro), Gazebo worlds, ros2_control config, Nav2 config, maps |
| `amr_mtt_moveit_config` | MoveIt 2 config: SRDF, kinematics (KDL), OMPL planners, collision matrix |
| `amr_mtt_task_planner` | Python task orchestration: pick→navigate→drop pipelines, AMCL init, scene setup |
| `amr_mtt_docking` | AprilTag-based 7-state docking state machine with per-stage PID control |
| `amr_mtt_gripper` | Gripper controller config and bringup |

### Control Stack

```
Task Planner (Python)
    ├── MoveIt 2 (arm motion planning, RRT-Connect, collision checking)
    └── Nav2 (AMCL localization, costmap, path planning)
            ↓
    ros2_control Controller Manager
        ├── ur_arm_controller       (JointTrajectoryController, position)
        ├── diff_drive_controller   (DiffDriveController, velocity)
        ├── robotiq_gripper_controller (GripperActionController)
        └── joint_state_broadcaster
            ↓
    Ignition Gazebo (gz_ros2_control plugin)
```

Topic bridging between Gazebo and ROS 2 is handled by `ros_gz_bridge` in `amr_mtt_ign_spawn.launch.py`. All nodes use `use_sim_time: true` — time comes from `/clock`.

### Key Design Notes

- **Diff drive plugin disabled in Gazebo** — ros2_control's `diff_drive_controller` is used instead. The Ignition built-in plugin is explicitly suppressed in the spawn launch file.
- **Gripper mimic joints are fixed** — Ignition Gazebo doesn't support URDF `<mimic>` joints. All Robotiq finger joints except `robotiq_85_left_knuckle_joint` are set to `fixed` type in simulation.
- **`/cmd_vel` relay** — Nav2 publishes to `/cmd_vel`; a relay node in `nav2.launch.py` forwards this to `/diff_drive_controller/cmd_vel_unstamped` which the controller expects.
- **Arm auto-homes on launch** — `launch_sim_amr.launch.py` sends the UR5 to HOME pose 12 seconds after startup via a one-shot timer.
- **Map pre-generated** — `amr_mtt_bot/map/amr_mtt_map_v2.yaml` is the production map loaded by Nav2.

### Docking State Machine (amr_mtt_docking)

States: `IDLE → GLOBAL_NAV → SEARCHING → VISUAL_SERVO → FINE_ALIGN → VERIFY_CONNECTION → DOCKED`

- `GLOBAL_NAV`: Nav2 navigates to a pre-defined dock approach point
- `SEARCHING`: Robot rotates to detect AprilTag
- `VISUAL_SERVO`: PID-based alignment at 1–3m range
- `FINE_ALIGN`: High-precision PID alignment at <1m
- `VERIFY_CONNECTION`: Checks charging contact

PID gains are tuned per-stage in `amr_mtt_bot/config/enhanced_docking.yaml`.

### Task Planner Entry Points

Registered in `amr_mtt_task_planner/setup.py`:
- `task_sequence` — Full autonomous pick→navigate→drop
- `pick_sequence` / `pick_drop_sequence` — Arm-only sequences
- `nav_sequence` — Waypoint navigation only
- `coordinator_node` — Multi-task coordination
- `set_initial_pose` — Sets AMCL initial pose (run before navigation)
- `planning_scene_setup` — Adds collision objects to MoveIt scene
- `save_waypoint` — Interactive waypoint recording for the arm
