#!/usr/bin/env python3
"""
pick_sequence_optimized.py
เวอร์ชันเร่งสปีด: ลด Duration และปลดล็อคการรอ Gripper ที่นานเกินจำเป็น
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ============================================================
#  WAYPOINTS — ปรับเวลา Duration ให้เหลือ 1-1.5 วินาที ในจุดที่ปลอดภัย
# ============================================================
JOINT_NAMES = [
    'ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint', 'ur5_elbow_joint',
    'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint',
]

WAYPOINTS = [
    # ชื่อ           [pan,     lift,    elbow,   w1,      w2,      w3]    time(s)
    ("HOME",          [0.0,     -1.5708, -1.5708, -3.159,  0.0,     0.0],  1.5),
    ("PRE_APPROACH",  [-1.3963, -2.2689, -1.1519, -1.2217,  1.5708, 0.0],  1.5),
    ("PRE_REACH",     [-1.4312, -2.6354, -1.1519, -0.9250,  1.5533, 0.0175],  1.0),
    ("REACH",         [-1.4312, -2.6354, -1.1519, -0.9250,  1.5533, 0.0175],  0.5), # เร่งจุดนี้
    ("HOME_MID",      [0.0,     -1.5708, -1.5708, -3.159,  0.0,     0.0],  1.5),
    ("LIFT_UP",       [0.0,     -2.7576,   -1.5708,   -3.3336,   -1.5708,   0.0],1.2),
    ("POST_PLACE",    [0.0, -3.0019, -1.5708, -3.3336, -1.5708, 0.0],           1.0),
    ("HOME_RETURN",   [0.0,     -1.5708, -1.5708, -3.159,   0.0,    0.0],  1.2),
]

# แมปการขยับ Gripper กับลำดับ Step (Index)
GRIPPER_BEFORE_STEP = {0: "OPEN", 3: "OPEN"}
# เปลี่ยนมาสั่ง OPEN ที่ Step 6 (POST_PLACE) เพื่อความเร็วสูงสุด
GRIPPER_AFTER_STEP = {
    3: "CLOSE",   # หลัง REACH: ปิดก้ามปู (จะใช้ wait=True เพื่อความชัวร์)
    6: "OPEN",    # หลัง POST_PLACE: สั่งเปิดทันทีแบบ wait=False
}
class PickSequenceNode(Node):
    def __init__(self):
        super().__init__('pick_sequence_node')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/ur_arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')

        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('=== Ready to speed up! ===')

    def send_arm(self, positions, duration_sec):
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
        
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def send_gripper(self, open_gripper: bool, wait: bool = False):
        """
        open_gripper: True = เปิด, False = ปิด
        wait: ถ้า True จะรอจนขยับเสร็จ (ใช้ตอนคีบ), ถ้า False จะสั่งแล้วไปต่อทันที (ใช้ตอนวาง/ถอย)
        """
        pos = 0.0 if open_gripper else 0.8
        label = 'OPEN' if open_gripper else 'CLOSE'
        self.get_logger().info(f'  >> Gripper: {label} (wait={wait})')
        
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = 100.0
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        if wait:
            # รอ Gripper ทำงานเสร็จเฉพาะจุดที่จำเป็น (เช่น ตอนคีบวัตถุ)
            gh = future.result()
            result_future = gh.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=1.5)
        return True

    def run_sequence(self):
        self.get_logger().info(f'STARTING SEQUENCE...')
        prev_pos = None

        for i, (name, positions, duration) in enumerate(WAYPOINTS):
            # --- BEFORE STEP ---
            if i in GRIPPER_BEFORE_STEP:
                self.send_gripper(GRIPPER_BEFORE_STEP[i] == "OPEN", wait=True)

            # --- ARM MOVE ---
            if positions != prev_pos:
                self.get_logger().info(f'[{i+1}/{len(WAYPOINTS)}] {name} ({duration}s)')
                if not self.send_arm(positions, duration): break
                prev_pos = positions

            # --- AFTER STEP ---
            if i in GRIPPER_AFTER_STEP:
                # ตอนปล่อยของ (Step 5) ให้สั่งเปิดแล้ว "ไม่ต้องรอ" (wait=False) เพื่อเริ่ม Step 6 ทันที
                should_wait = (GRIPPER_AFTER_STEP[i] == "CLOSE") 
                self.send_gripper(GRIPPER_AFTER_STEP[i] == "OPEN", wait=should_wait)

        self.get_logger().info('=== FINISHED ===')

def main(args=None):
    rclpy.init(args=args)
    node = PickSequenceNode()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()