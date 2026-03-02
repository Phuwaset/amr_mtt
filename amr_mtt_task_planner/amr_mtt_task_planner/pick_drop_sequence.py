#!/usr/bin/env python3
"""
pick_drop_sequence.py (7 Steps Complete)
ลำดับ: HOME -> LIFT_UP -> POST_PLACE (คีบ) -> STEP 5 (Rotate) -> STEP 6 (วาง) -> HOME
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [
    'ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint', 'ur5_elbow_joint',
    'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint',
]

# ============================================================
#  WAYPOINTS — ครบทั้ง 7 ขั้นตอน
# ============================================================
ARM_WAYPOINTS_DROP = [
    # 1. HOME ท่าเริ่มต้น
    ("HOME",          [0.0, -1.5708, -1.5708, -3.159,  0.0, 0.0], 2.0),
    
    # 2. LIFT_UP (รูป image_35767c.png)
    ("LIFT_UP",       [0.0, -2.7576, -1.5708, -3.3336, -1.5708, 0.0], 2.0),
    
    # 3. POST_PLACE (รูป image_35727b.png)
    ("POST_PLACE",    [0.0, -3.0019, -1.5708, -3.3336, -1.5708, 0.0], 1.5),
    
    # 4. STEP 5 (รูป image_342120.png)
    ("STEP_5_ROTATE", [1.5708, -1.5708, -1.5708, -3.159, 0.0, 0.0], 2.5),
    
    # 5. STEP 6 ท่าเตรียมวาง (รูป image_3415db.png)
    ("STEP_6_PLACE",  [1.5708, -2.8623, -0.2269, -1.0297, 1.5708, 0.0], 2.0),
    
    # 6. STEP 7 กลับท่า HOME จบงาน
    ("HOME_FINAL",    [0.0, -1.5708, -1.5708, -3.159,  0.0, 0.0], 2.5),
]

# ============================================================
#  GRIPPER ACTIONS
# ============================================================
GRIPPER_BEFORE_STEP = {
    1: "OPEN",    # ก่อน LIFT_UP -> เปิดเตรียมวางของเก่า (ถ้ามี)
}

GRIPPER_AFTER_STEP = {
    2: "CLOSE",   # หลัง POST_PLACE -> คีบชิ้นงานใหม่
    4: "OPEN",    # หลัง STEP_6_PLACE -> ปล่อยกริปเปอร์เพื่อวางของ
}

class PickDropNode(Node):
    def __init__(self):
        super().__init__('pick_drop_node')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/ur_arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')

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

    def run(self):
        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()

        self.get_logger().info('\n' + '='*50)
        self.get_logger().info(f'  RUNNING 7-STEP SEQUENCE')
        self.get_logger().info('='*50)

        for i, (name, positions, duration) in enumerate(ARM_WAYPOINTS_DROP):
            # --- BEFORE MOVE ---
            if i in GRIPPER_BEFORE_STEP:
                self.send_gripper(GRIPPER_BEFORE_STEP[i] == "OPEN", wait=True)

            self.get_logger().info(f'  [{i+1}/{len(ARM_WAYPOINTS_DROP)}] {name} ({duration}s)')
            if not self.send_arm(positions, duration): return

            # --- AFTER MOVE ---
            if i in GRIPPER_AFTER_STEP:
                # เร่งสปีดจังหวะวาง (Step 5/Index 4) ไม่ต้องรอจนกางสุด
                wait_needed = (i != 4) 
                self.send_gripper(GRIPPER_AFTER_STEP[i] == "OPEN", wait=wait_needed)

        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('  ✅ ALL 7 STEPS COMPLETE')
        self.get_logger().info('='*50)

def main(args=None):
    rclpy.init(args=args)
    node = PickDropNode()
    try:
        node.run()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()