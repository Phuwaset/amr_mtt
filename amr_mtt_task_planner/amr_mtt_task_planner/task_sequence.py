#!/usr/bin/env python3
"""
task_sequence.py (Updated Version)

รวม pick_sequence + nav_sequence เป็น Task เดียว:
  1. 🦾 ARM: หยิบกล่องจากโต๊ะ (ใช้วงรอบที่ปรับความเร็วแล้ว)
  2. 🚗 BASE: ขับไปส่งกล่อง (nav_sequence)
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
#  PICK SEQUENCE — Joint Waypoints (อัปเดตตาม WAYPOINTS ล่าสุด)
# ============================================================
JOINT_NAMES = [
    'ur5_shoulder_pan_joint',
    'ur5_shoulder_lift_joint',
    'ur5_elbow_joint',
    'ur5_wrist_1_joint',
    'ur5_wrist_2_joint',
    'ur5_wrist_3_joint',
]

ARM_WAYPOINTS = [
    # ชื่อ           [pan,     lift,      elbow,     w1,        w2,        w3]       time(s)
    ("HOME",          [0.0,     -1.5708,   -1.5708,   -3.159,    0.0,       0.0],     1.5),
    ("PRE_APPROACH",  [-1.3963, -2.2689,   -1.1519,   -1.2217,   1.5708,    0.0],     1.5),
    ("PRE_REACH",     [-1.4312, -2.6354,   -1.1519,   -0.9250,   1.5533,    0.0175],  1.0),
    ("REACH",         [-1.4312, -2.6354,   -1.1519,   -0.9250,   1.5533,    0.0175],  0.5),
    ("HOME_MID",      [0.0,     -1.5708,   -1.5708,   -3.159,    0.0,       0.0],     1.5),
    # อัปเดต LIFT_UP จากรูป image_35767c.png
    ("LIFT_UP",       [0.0,     -2.7576,   -1.5708,   -3.3336,   -1.5708,   0.0],     1.2),
    # อัปเดต POST_PLACE จากรูป image_35727b.png
    ("POST_PLACE",    [0.0,     -3.0019,   -1.5708,   -3.3336,   -1.5708,   0.0],     1.0),
    ("HOME_RETURN",   [0.0,     -1.5708,   -1.5708,   -3.159,    0.0,       0.0],     1.2),
]

GRIPPER_BEFORE_STEP = {
    0: "OPEN",    # ก่อน HOME
    3: "OPEN",    # ก่อน REACH
}

GRIPPER_AFTER_STEP = {
    3: "CLOSE",   # หลัง REACH -> คีบ (ต้องรอ)
    6: "OPEN",    # หลัง POST_PLACE -> วาง (ไม่รอ/Speed up)
}

# ============================================================
#  NAV SEQUENCE
# ============================================================
MAP_FRAME       = "map"
NAV_TIMEOUT_SEC = 120.0

NAV_WAYPOINTS = [
    ("HOMING",   -1.70,    -1.30,      0.0),
    ("DEST_2",    3.1721,   1.8188,    1.6),
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
        # รองรับค่าทศนิยมใน duration
        pt.time_from_start.sec = int(duration_sec)
        pt.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        traj.points.append(pt)
        goal.trajectory = traj

        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            return False
        rclpy.spin_until_future_complete(self, gh.get_result_async())
        return True

    def send_gripper(self, open_gripper: bool, wait: bool = True):
        pos     = 0.0 if open_gripper else 0.8
        label   = 'OPEN' if open_gripper else 'CLOSE'
        self.get_logger().info(f'  >> Gripper: {label} (wait={wait})')
        
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = 100.0
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        if wait:
            gh = future.result()
            if gh.accepted:
                # ลด timeout สำหรับจังหวะคีบให้เร็วขึ้น
                rclpy.spin_until_future_complete(self, gh.get_result_async(), timeout_sec=2.0)
        return True

    def navigate_to(self, name: str, x: float, y: float, yaw_deg: float) -> bool:
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw_deg)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
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
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.nav_client.wait_for_server()

        # ── PHASE 1: PICK (Optimized) ──────────────────────────
        self.get_logger().info('\nPHASE 1 — PICK SEQUENCE')
        for i, (name, positions, duration) in enumerate(ARM_WAYPOINTS):
            if i in GRIPPER_BEFORE_STEP:
                self.send_gripper(GRIPPER_BEFORE_STEP[i] == "OPEN", wait=True)

            self.get_logger().info(f'  [{i+1}/{len(ARM_WAYPOINTS)}] {name} ({duration}s)')
            if not self.send_arm(positions, duration): return

            if i in GRIPPER_AFTER_STEP:
                # ถ้าเป็น Step 6 (POST_PLACE) ให้ OPEN แบบไม่รอ (wait=False)
                is_post_place = (i == 6)
                self.send_gripper(GRIPPER_AFTER_STEP[i] == "OPEN", wait=not is_post_place)

        # ── PHASE 2: NAVIGATE ──────────────────────────────────
        self.get_logger().info('\nPHASE 2 — NAV SEQUENCE')
        for i, (name, x, y, yaw) in enumerate(NAV_WAYPOINTS):
            if not self.navigate_to(name, x, y, yaw): return

        self.get_logger().info('\n🎉 TASK COMPLETE: PICK + NAV SUCCESS')

def main(args=None):
    rclpy.init(args=args)
    node = TaskSequenceNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()