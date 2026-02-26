#!/usr/bin/env python3
"""
pick_drop_sequence.py

ทดสอบ PICK_DROP sequence เฉพาะแขนหุ่นยนต์ (ไม่มี navigation)
สำหรับ tune ท่า drop/pick ที่ปลายทางก่อนนำไปใส่ใน task_sequence.py

------------ ต้องรัน Launch Files ก่อน ------------
Terminal 1:
  ros2_nvidia launch amr_mtt_moveit_config moveit.launch.py

Terminal 2 (ไฟล์นี้):
  ros2 run amr_mtt_task_planner pick_drop_sequence
---------------------------------------------------
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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
#  PICK_DROP WAYPOINTS — แก้ท่าได้ที่นี่
#  Joint Order: [pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
# ============================================================
ARM_WAYPOINTS_DROP = [
    # ชื่อ              [pan,     lift,    elbow,   w1,      w2,      w3]     time(s)
    ("HOME",            [0.0,     -1.5708, -1.5708, -3.159,  0.0,     0.0],   3),
    # ^^^ เริ่มต้น HOME → OPEN gripper (วางของที่ขนมา)
    ("APPROACH_DROP",   [-0.0524, -2.8274, -1.5708, -0.7854,  1.5708, 0.0175], 3),
    ("REACH_DROP",      [-0.0524, -3.1, -1.5708, -0.6109,  1.5708, 0.0175], 2),
    
    # ^^^ หลัง REACH_DROP เสร็จ → ✊ CLOSE gripper (หนีบชิ้นงาน)
    ("LIFT_BACK",       [0.0,     -1.5708, -1.5708, -3.159,  0.0,     0.0],   3),
    # ^^^ ยกขึ้น → กลับ HOME
]

# ============================================================
#  GRIPPER ACTIONS
# ============================================================
GRIPPER_BEFORE_STEP = {
    0: "OPEN",    # ก่อน HOME → ปล่อยของที่อยู่ในมือ
}

GRIPPER_AFTER_STEP = {
    2: "CLOSE",   # หลัง REACH_DROP → ✊ หนีบชิ้นงานใหม่
}


# ============================================================
#  NODE
# ============================================================
class PickDropNode(Node):
    def __init__(self):
        super().__init__('pick_drop_node')

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/ur_arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd'
        )

    # ── ARM ──────────────────────────────────────────────────
    def send_arm(self, positions, duration_sec) -> bool:
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start.sec = duration_sec
        traj.points.append(pt)
        goal.trajectory = traj

        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Arm goal REJECTED!')
            return False
        rclpy.spin_until_future_complete(self, gh.get_result_async())
        return True

    # ── GRIPPER ──────────────────────────────────────────────
    def send_gripper(self, open_gripper: bool):
        pos     = 0.0 if open_gripper else 0.8
        label   = 'OPEN' if open_gripper else 'CLOSE'
        timeout = 2.0 if open_gripper else 3.0
        self.get_logger().info(f'  >> Gripper: {label}')
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = 100.0
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done():
            self.get_logger().warn(f'Gripper {label} timeout — continuing')
        return True

    # ── RUN ──────────────────────────────────────────────────
    def run(self):
        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.get_logger().info('  ✅ Arm controller ready')
        self.gripper_client.wait_for_server()
        self.get_logger().info('  ✅ Gripper controller ready')

        self.get_logger().info('\n' + '='*50)
        self.get_logger().info(f'  PICK_DROP SEQUENCE ({len(ARM_WAYPOINTS_DROP)} steps)')
        self.get_logger().info('='*50)

        for i, (name, positions, duration) in enumerate(ARM_WAYPOINTS_DROP):
            if i in GRIPPER_BEFORE_STEP:
                self.send_gripper(GRIPPER_BEFORE_STEP[i] == "OPEN")

            self.get_logger().info(
                f'  [{i+1}/{len(ARM_WAYPOINTS_DROP)}] {name} ({duration}s)'
            )
            if not self.send_arm(positions, duration):
                self.get_logger().error(f'Step "{name}" failed! Aborting.')
                return

            if i in GRIPPER_AFTER_STEP:
                self.send_gripper(GRIPPER_AFTER_STEP[i] == "OPEN")

        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('  ✅ PICK_DROP COMPLETE')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    node = PickDropNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
