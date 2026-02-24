#!/usr/bin/env python3
"""
pick_sequence.py

ทดสอบ Pick-and-Place โดยเล่นตาม Waypoints ที่กำหนดไว้
แก้ไขค่า Joint ในแต่ละ WAYPOINT ด้านล่างเพื่ออัปเดตลำดับการเคลื่อนที่

Usage:
  ros2 run amr_mtt_task_planner pick_sequence
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ============================================================
#  WAYPOINTS — แก้ไขค่า Joint ตรงนี้ (หน่วย: Radian)
#  Joint Order: [pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
# ============================================================

JOINT_NAMES = [
    'ur5_shoulder_pan_joint',
    'ur5_shoulder_lift_joint',
    'ur5_elbow_joint',
    'ur5_wrist_1_joint',
    'ur5_wrist_2_joint',
    'ur5_wrist_3_joint',
]

WAYPOINTS = [
    # -------------------------------------------------------
    # ชื่อ           [pan,     lift,    elbow,   w1,      w2,      w3]    time(s)
    # -------------------------------------------------------
    ("HOME",          [0.0,     -1.5708, -1.5708, -3.159,  0.0,     0.0],  2),
    ("PRE_APPROACH",  [-1.3963, -1.5708, -1.5708, -1.5708, -1.5708, 0.0],  3),
    ("PRE_REACH",     [-1.4312, -2.5133, -1.2566, -0.7505,  1.5708, 0.1222], 3),
    ("REACH",         [-1.4312, -2.5831, -1.2566, -0.7505,  1.5708, 0.0],  2),
    # ^^^ หลัง REACH เสร็จ → ✊ CLOSE gripper ทันที (GRIPPER_AFTER_STEP[3])
    ("LIFT_UP",       [-1.4312, -1.5708, -1.5708, -1.5708,  1.5708, 0.0],  2),
    ("PLACE",         [-0.0175, -3.0718, -1.8500,  0.1571,  1.5708, 0.0],  3),
    # ^^^ หลัง PLACE เสร็จ → 🖐️ OPEN gripper ทันที (GRIPPER_AFTER_STEP[5])
    ("HOME_RETURN",   [0.0,     -1.5708, -1.5708, -3.159,   0.0,    0.0],  3),
]

# ============================================================
#  GRIPPER ACTIONS — ทำก่อน Waypoint นั้น
# ============================================================
GRIPPER_BEFORE_STEP = {
    0: "OPEN",    # ก่อน HOME → เปิด Gripper เตรียมพร้อม
    3: "OPEN",    # ก่อน REACH → เปิด Gripper เพื่อรับกล่อง
}

# ============================================================
#  GRIPPER ACTIONS — ทำหลัง Waypoint นั้นจบ (ประหยัดเวลา ไม่ต้อง GRASP step)
# ============================================================
GRIPPER_AFTER_STEP = {
    3: "CLOSE",   # หลัง REACH จบ → ✊ ปิด Gripper คีบกล่องทันที (ตัด GRASP step ~20s)
    5: "OPEN",    # หลัง PLACE จบ → 🖐️ เปิด Gripper วางกล่องทันที
}


class PickSequenceNode(Node):
    def __init__(self):
        super().__init__('pick_sequence_node')

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/ur_arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd'
        )

        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('=== Action servers ready. Starting sequence ===')

    def send_arm(self, positions, duration_sec):
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
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def send_gripper(self, open_gripper: bool):
        pos = 0.0 if open_gripper else 0.8
        label = 'OPEN' if open_gripper else 'CLOSE'
        timeout = 2.0 if open_gripper else 3.0
        self.get_logger().info(f'  >> Gripper: {label}')
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = 100.0
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Gripper goal REJECTED!')
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done():
            self.get_logger().warn(
                f'Gripper {label} timeout ({timeout}s) — continuing (stalled on object)'
            )
        return True

    def run_sequence(self):
        self.get_logger().info(f'\n{"="*50}')
        self.get_logger().info(f'  PICK SEQUENCE: {len(WAYPOINTS)} steps')
        self.get_logger().info(f'{"="*50}')

        for i, (name, positions, duration) in enumerate(WAYPOINTS):
            # Gripper BEFORE arm move
            if i in GRIPPER_BEFORE_STEP:
                self.send_gripper(open_gripper=(GRIPPER_BEFORE_STEP[i] == "OPEN"))

            # Move arm
            self.get_logger().info(
                f'[{i+1}/{len(WAYPOINTS)}] {name}: {[round(p, 3) for p in positions]} ({duration}s)'
            )
            if not self.send_arm(positions, duration):
                self.get_logger().error(f'Step "{name}" failed! Stopping.')
                return

            # Gripper AFTER arm move
            if i in GRIPPER_AFTER_STEP:
                self.send_gripper(open_gripper=(GRIPPER_AFTER_STEP[i] == "OPEN"))

        self.get_logger().info('=== SEQUENCE COMPLETE ===')


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
