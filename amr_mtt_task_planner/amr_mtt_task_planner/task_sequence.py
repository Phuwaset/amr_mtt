#!/usr/bin/env python3
"""
task_sequence.py

รวม pick_sequence + nav_sequence เป็น Task เดียว:
  1. 🦾 ARM: หยิบกล่องจากโต๊ะ (pick_sequence)
  2. 🚗 BASE: ขับไปส่งกล่อง (nav_sequence)

---------- ต้องรัน Launch Files ก่อน ----------

Terminal 1:
  ros2_nvidia launch amr_mtt_moveit_config moveit.launch.py
  └── Ignition Gazebo + MoveIt (Arm Controller) + RViz

Terminal 2:
  ros2 launch amr_mtt_bot nav2.launch.py
  └── Nav2 + AMCL (Navigation) + Map Server

Terminal 3 (ไฟล์นี้):
  ros2 run amr_mtt_task_planner task_sequence

-------------------------------------------
Usage:
  ros2 run amr_mtt_task_planner task_sequence
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
#  PICK SEQUENCE — Joint Waypoints (Radian)
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

ARM_WAYPOINTS = [
    # ชื่อ           [pan,     lift,    elbow,   w1,      w2,      w3]    time(s)
    ("HOME",          [0.0,     -1.5708, -1.5708, -3.159,  0.0,     0.0],  2),
    ("PRE_APPROACH",  [-1.3963, -1.5708, -1.5708, -1.5708, -1.5708, 0.0],  3),
    ("PRE_REACH",     [-1.4312, -2.5133, -1.2566, -0.7505,  1.5708, 0.1222], 3),
    ("REACH",         [-1.4312, -2.5831, -1.2566, -0.7505,  1.5708, 0.0],  2),
    # ^^^ หลัง REACH เสร็จ → ✊ CLOSE gripper ทันที
    ("LIFT_UP",       [-1.4312, -1.5708, -1.5708, -1.5708,  1.5708, 0.0],  2),
    ("PLACE",         [-0.0175, -3.0718, -1.8500,  0.1571,  1.5708, 0.0],  3),
    # ^^^ หลัง PLACE เสร็จ → 🖐️ OPEN gripper ทันที
    ("HOME_RETURN",   [0.0,     -1.5708, -1.5708, -3.159,   0.0,    0.0],  3),
]

GRIPPER_BEFORE_STEP = {
    0: "OPEN",    # ก่อน HOME → เปิด Gripper เตรียมพร้อม
    3: "OPEN",    # ก่อน REACH → เปิด Gripper เพื่อรับกล่อง
}

GRIPPER_AFTER_STEP = {
    3: "CLOSE",   # หลัง REACH จบ → ✊ ปิด Gripper คีบกล่อง
    5: "OPEN",    # หลัง PLACE จบ → 🖐️ เปิด Gripper วางกล่อง
}

# ============================================================
#  PICK_DROP SEQUENCE — ARM ที่ปลายทาง
#  ลำดับ: HOME → APPROACH → REACH_DROP → (CLOSE) → LIFT_BACK
#  เริ่มด้วย OPEN gripper (ปล่อยของที่ขนมา) แล้วหยิบชิ้นงานใหม่
# ============================================================
ARM_WAYPOINTS_DROP = [
    # ชื่อ              [pan,     lift,    elbow,   w1,      w2,      w3]     time(s)
    ("HOME",            [0.0,     -1.5708, -1.5708, -3.159,  0.0,     0.0],   3),
    # ^^^ เริ่มต้น HOME → OPEN gripper (วางของที่ขนมา)
    ("APPROACH_DROP",   [-0.0524, -2.8274, -1.5708, -0.7854,  1.5708, 0.0175], 3),
    ("REACH_DROP",      [-0.0524, -2.9671, -1.5708, -0.6109,  1.5708, 0.0175], 2),
    # ^^^ หลัง REACH_DROP เสร็จ → ✊ CLOSE gripper (หนีบชิ้นงาน)
    ("LIFT_BACK",       [0.0,     -1.5708, -1.5708, -3.159,  0.0,     0.0],   3),
    # ^^^ ยกขึ้น → กลับ HOME
]

GRIPPER_BEFORE_DROP = {
    0: "OPEN",    # ก่อน HOME → ปล่อยของจาก PICK phase
}

GRIPPER_AFTER_DROP = {
    2: "CLOSE",   # หลัง REACH_DROP → ✊ หนีบชิ้นงานใหม่
}

# ============================================================
#  NAV SEQUENCE — Navigation Waypoints (meter, degree)
#
#  ⚠️ IMPORTANT: ค่าต้องอยู่ใน MAP frame (ไม่ใช่ odom frame)
#  วิธีบันทึกที่ถูกต้อง:
#    1. Launch moveit.launch.py + nav2.launch.py พร้อมกัน
#    2. รอ AMCL set initial pose (~5s)
#    3. ros2 run amr_mtt_task_planner save_waypoint
#       → จะใช้ /amcl_pose (map frame) อัตโนมัติ
# ============================================================
MAP_FRAME       = "map"
NAV_TIMEOUT_SEC = 120.0   # เพิ่มเป็น 120s สำหรับระยะทางไกล

NAV_WAYPOINTS = [
    # ชื่อ          x         y        yaw(°)    หมายเหตุ
    ("HOMING",   -1.70,    -1.30,      0.0),   # map frame ✅
    # ("DEST_1", ?,        ?,          ?),      # ⚠️ ต้อง re-record ด้วย amcl_pose (map frame)
    ("DEST_2",    3.1721,   1.8188,    1.6),   # map frame ✅ (บันทึกด้วย amcl_pose)
]




def yaw_to_quaternion(yaw_deg: float):
    yaw_rad = math.radians(yaw_deg)
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


# ============================================================
#  TASK SEQUENCE NODE
# ============================================================

class TaskSequenceNode(Node):
    def __init__(self):
        super().__init__('task_sequence_node')

        # Arm action client
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/ur_arm_controller/follow_joint_trajectory'
        )
        # Gripper action client
        self.gripper_client = ActionClient(
            self, GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd'
        )
        # Navigation action client
        self.nav_client = ActionClient(
            self, NavigateToPose,
            'navigate_to_pose'
        )

    # ----------------------------------------------------------
    #  ARM
    # ----------------------------------------------------------
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

    # ----------------------------------------------------------
    #  GRIPPER
    # ----------------------------------------------------------
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

    # ----------------------------------------------------------
    #  NAVIGATION
    # ----------------------------------------------------------
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

        self.get_logger().info(f'  Navigating → {name} (x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}°)')
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error(f'Nav goal to {name} REJECTED!')
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=NAV_TIMEOUT_SEC)
        if not result_future.done():
            self.get_logger().warn(f'{name}: timeout — cancelling')
            gh.cancel_goal_async()
            return False
        return result_future.result().status == 4  # SUCCEEDED

    # ----------------------------------------------------------
    #  MAIN TASK
    # ----------------------------------------------------------
    def run(self):
        # ── รอ Action Servers ──────────────────────────────────
        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.get_logger().info('  ✅ Arm controller ready')
        self.gripper_client.wait_for_server()
        self.get_logger().info('  ✅ Gripper controller ready')
        self.nav_client.wait_for_server()
        self.get_logger().info('  ✅ Nav2 ready')

        # ── PHASE 1: PICK ──────────────────────────────────────
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info(f'  PHASE 1 — PICK SEQUENCE ({len(ARM_WAYPOINTS)} steps)')
        self.get_logger().info('='*50)

        for i, (name, positions, duration) in enumerate(ARM_WAYPOINTS):
            if i in GRIPPER_BEFORE_STEP:
                self.send_gripper(GRIPPER_BEFORE_STEP[i] == "OPEN")

            self.get_logger().info(
                f'  [{i+1}/{len(ARM_WAYPOINTS)}] {name} ({duration}s)'
            )
            if not self.send_arm(positions, duration):
                self.get_logger().error(f'Arm step "{name}" failed! Aborting.')
                return

            if i in GRIPPER_AFTER_STEP:
                self.send_gripper(GRIPPER_AFTER_STEP[i] == "OPEN")

        self.get_logger().info('  ✅ PICK COMPLETE')

        # ── PHASE 2: NAVIGATE ──────────────────────────────────
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info(f'  PHASE 2 — NAV SEQUENCE ({len(NAV_WAYPOINTS)} waypoints)')
        self.get_logger().info('='*50)

        for i, (name, x, y, yaw) in enumerate(NAV_WAYPOINTS):
            self.get_logger().info(f'  [{i+1}/{len(NAV_WAYPOINTS)}] → {name}')
            if not self.navigate_to(name, x, y, yaw):
                self.get_logger().error(f'Nav to "{name}" failed! Aborting.')
                return

        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('  ✅ NAV COMPLETE')
        self.get_logger().info('='*50)

        # ── PHASE 3: PICK_DROP ─────────────────────────────────
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info(f'  PHASE 3 — PICK_DROP SEQUENCE ({len(ARM_WAYPOINTS_DROP)} steps)')
        self.get_logger().info('='*50)

        for i, (name, positions, duration) in enumerate(ARM_WAYPOINTS_DROP):
            if i in GRIPPER_BEFORE_DROP:
                self.send_gripper(GRIPPER_BEFORE_DROP[i] == "OPEN")

            self.get_logger().info(
                f'  [{i+1}/{len(ARM_WAYPOINTS_DROP)}] {name} ({duration}s)'
            )
            if not self.send_arm(positions, duration):
                self.get_logger().error(f'Pick-drop step "{name}" failed! Aborting.')
                return

            if i in GRIPPER_AFTER_DROP:
                self.send_gripper(GRIPPER_AFTER_DROP[i] == "OPEN")

        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('  🎉 TASK COMPLETE: PICK + NAV + PICK_DROP SUCCESS')
        self.get_logger().info('='*50)



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
