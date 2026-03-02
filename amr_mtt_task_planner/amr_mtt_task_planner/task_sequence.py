#!/usr/bin/env python3
"""
task_sequence.py (Updated Full Version)

รวม pick_sequence + nav_sequence + drop_sequence เป็น Task เดียว:
  1. 🦾 ARM: หยิบกล่องจากโต๊ะ (PHASE 1: PICK)
  2. 🚗 BASE: ขับไปส่งกล่อง (PHASE 2: NAV)
  3. 🦾 ARM: วางกล่องและจัดการชิ้นงานปลายทาง (PHASE 3: DROP)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Arm + Gripper
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Navigation
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# ============================================================
#  JOINT NAMES
# ============================================================
JOINT_NAMES = [
    'ur5_shoulder_pan_joint',
    'ur5_shoulder_lift_joint',
    'ur5_elbow_joint',
    'ur5_wrist_1_joint',
    'ur5_wrist_2_joint',
    'ur5_wrist_3_joint',
]

# ============================================================
#  ARM WAYPOINTS (PHASE 1: PICK)
# ============================================================
ARM_WAYPOINTS_PICK = [
    ("HOME",          [0.0,     -1.5708,   -1.5708,   -3.159,    0.0,       0.0],     1.5),
    ("PRE_APPROACH",  [-1.3963, -2.2689,   -1.1519,   -1.2217,   1.5708,    0.0],     1.5),
    ("PRE_REACH",     [-1.4312, -2.6354,   -1.1519,   -0.9250,   1.5533,    0.0175],  1.0),
    ("REACH",         [-1.4312, -2.6354,   -1.1519,   -0.9250,   1.5533,    0.0175],  0.5),
    ("HOME_MID",      [0.0,     -1.5708,   -1.5708,   -3.159,    0.0,       0.0],     1.5),
    ("LIFT_UP",       [0.0,     -2.7576,   -1.5708,   -3.3336,   -1.5708,   0.0],     1.2),
    ("POST_PLACE",    [0.0,     -3.0019,   -1.5708,   -3.3336,   -1.5708,   0.0],     1.0),
    ("HOME_RETURN",   [0.0,     -1.5708,   -1.5708,   -3.159,    0.0,       0.0],     1.2),
]

PICK_GRIPPER_BEFORE = { 0: "OPEN", 3: "OPEN" }
PICK_GRIPPER_AFTER  = { 3: "CLOSE", 6: "OPEN" }

# ============================================================
#  ARM WAYPOINTS (PHASE 3: DROP)
# ============================================================
ARM_WAYPOINTS_DROP = [
    ("HOME",          [0.0,     -1.5708, -1.5708, -3.159,  0.0,     0.0],  1.5),
    ("LIFT_UP",       [0.0,     -2.7576, -1.5708, -3.3336, -1.5708, 0.0],  1.2),
    ("POST_PLACE",    [-0.01,   -3.05,   -1.5708, -3.3336, -1.5708, 0.0],  2.0),
    ("STEP_5_ROTATE", [1.5708,  -1.5708, -1.5708, -3.159,  0.0,     0.0],  2.0),
    ("STEP_6_PLACE_1",[1.5708,  -2.8623, -0.2269, -1.0297, 1.5708,  0.0],  1.5),
    ("STEP_6_PLACE_2",[1.5708,  -2.6623, -0.2269, -1.0297, 1.5708,  0.0],  1.5),
    ("HOME_FINAL",    [0.0,     -1.5708, -1.5708, -3.159,  0.0,     0.0],  2.0),
]

DROP_GRIPPER_BEFORE = { 1: "OPEN" }
DROP_GRIPPER_AFTER  = { 2: "CLOSE", 4: "OPEN" }

# ============================================================
#  NAV SETTINGS
# ============================================================
MAP_FRAME = "map"
NAV_TIMEOUT_SEC = 120.0
NAV_WAYPOINTS = [
    ("HOMING", -1.70, -1.30, 0.0),
    ("DEST_2", 3.1721, 1.8188, 1.6),
]

def yaw_to_quaternion(yaw_deg: float):
    yaw_rad = math.radians(yaw_deg)
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)

class TaskSequenceNode(Node):
    def __init__(self):
        super().__init__('task_sequence_node')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/ur_arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_arm(self, positions, duration_sec) -> bool:
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start.sec = int(duration_sec)
        pt.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        traj.points.append(pt)
        goal.trajectory = traj
        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted: return False
        rclpy.spin_until_future_complete(self, gh.get_result_async())
        return True

    def send_gripper(self, open_gripper: bool, wait: bool = True):
        pos = 0.0 if open_gripper else 0.8
        label = 'OPEN' if open_gripper else 'CLOSE'
        self.get_logger().info(f'  >> Gripper: {label} (wait={wait})')
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = 100.0
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        if wait:
            gh = future.result()
            if gh.accepted:
                rclpy.spin_until_future_complete(self, gh.get_result_async(), timeout_sec=2.0)
        return True

    def navigate_to(self, name: str, x: float, y: float, yaw_deg: float) -> bool:
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        qx, qy, qz, qw = yaw_to_quaternion(yaw_deg)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = qx, qy, qz, qw
        goal.pose = pose
        self.get_logger().info(f'  Navigating → {name}')
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted: return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=NAV_TIMEOUT_SEC)
        return result_future.done() and result_future.result().status == 4

    def run(self):
        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server(); self.gripper_client.wait_for_server(); self.nav_client.wait_for_server()

        # ── PHASE 1: PICK ──────────────────────────────────────
        self.get_logger().info('\nPHASE 1 — PICK SEQUENCE')
        for i, (name, positions, duration) in enumerate(ARM_WAYPOINTS_PICK):
            if i in PICK_GRIPPER_BEFORE:
                self.send_gripper(PICK_GRIPPER_BEFORE[i] == "OPEN", wait=True)
            self.get_logger().info(f'  [{i+1}/{len(ARM_WAYPOINTS_PICK)}] {name} ({duration}s)')
            if not self.send_arm(positions, duration): return
            if i in PICK_GRIPPER_AFTER:
                is_post_place = (i == 6)
                self.send_gripper(PICK_GRIPPER_AFTER[i] == "OPEN", wait=not is_post_place)

        # ── PHASE 2: NAVIGATE ──────────────────────────────────
        self.get_logger().info('\nPHASE 2 — NAV SEQUENCE')
        for i, (name, x, y, yaw) in enumerate(NAV_WAYPOINTS):
            if not self.navigate_to(name, x, y, yaw): return

        # ── PHASE 3: DROP ──────────────────────────────────────
        self.get_logger().info('\nPHASE 3 — DROP SEQUENCE')
        for i, (name, positions, duration) in enumerate(ARM_WAYPOINTS_DROP):
            if i in DROP_GRIPPER_BEFORE:
                self.send_gripper(DROP_GRIPPER_BEFORE[i] == "OPEN", wait=True)
            self.get_logger().info(f'  [{i+1}/{len(ARM_WAYPOINTS_DROP)}] {name} ({duration}s)')
            if not self.send_arm(positions, duration): return
            if i in DROP_GRIPPER_AFTER:
                # เร่งความเร็วที่ STEP_6_PLACE (i=4) โดยสั่ง OPEN แล้วไม่ต้องรอหยุดนิ่ง (wait=False)
                is_place_step = (i == 4)
                self.send_gripper(DROP_GRIPPER_AFTER[i] == "OPEN", wait=not is_place_step)

        self.get_logger().info('\n🎉 MISSION COMPLETE: PICK + NAV + DROP SUCCESS')

def main(args=None):
    rclpy.init(args=args)
    node = TaskSequenceNode()
    try:
        node.run()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()