#!/usr/bin/env python3
"""
nav_sequence.py

ส่ง Navigation Goals ให้ Nav2 เดินตาม Waypoints หลังจาก pick_sequence เสร็จ

Usage:
  ros2 run amr_mtt_task_planner nav_sequence

Waypoint Format:
  ("ชื่อ", x, y, yaw_deg)
  - x, y คือตำแหน่งใน map frame (เมตร)
  - yaw_deg คือมุมหมุน (องศา) 0=ไปหน้า, 90=เลี้ยวซ้าย, -90=เลี้ยวขวา, 180=กลับหลัง
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# ============================================================
#  NAV WAYPOINTS — แก้ไขตำแหน่งตรงนี้ (หน่วย: เมตร, องศา)
#
#  ดูพิกัดจาก RViz:
#    Tools → 2D Pose Estimate → คลิกบนแผนที่ → อ่านค่า x, y จาก topic /clicked_point
#  หรือ:
#    ros2 topic echo /amcl_pose (ขณะ robot อยู่ที่ตำแหน่งที่ต้องการ)
# ============================================================

NAV_WAYPOINTS = [
    # -------------------------------------------------------
    # ชื่อ          x        y       yaw(°)   หมายเหตุ
    # -------------------------------------------------------
    ("HOMING",   -1.70,   -1.30,     0.0),   # ← ตำแหน่ง spawn / ใกล้กล่อง (จุดเริ่มต้น)
    ("DEST_1",   -0.1418, -6.4007, -180.0),  # ← บันทึกจาก save_waypoint (2026-02-26)
    # ("DEST_2",   ?,       ?,         ?),   # ← เพิ่มจุดต่อไป (ถ้ามี)
    ("RETURN",  -1.70,   -1.30,     0.0),  # ← กลับจุดเริ่มต้น
]

# ============================================================
#  SETTINGS
# ============================================================
MAP_FRAME = "map"           # frame สำหรับ navigation goals
GOAL_TIMEOUT_SEC = 60.0     # timeout ต่อ waypoint (วินาที)


def yaw_to_quaternion(yaw_deg: float):
    """แปลง yaw (องศา) → quaternion (x, y, z, w)"""
    yaw_rad = math.radians(yaw_deg)
    return (
        0.0,
        0.0,
        math.sin(yaw_rad / 2.0),
        math.cos(yaw_rad / 2.0),
    )


class NavSequenceNode(Node):
    def __init__(self):
        super().__init__('nav_sequence_node')

        self.nav_client = ActionClient(
            self, NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('=== Nav2 ready. Starting navigation sequence ===')

    def navigate_to(self, name: str, x: float, y: float, yaw_deg: float) -> bool:
        """ส่ง NavigateToPose goal และรอผล"""
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(yaw_deg)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal.pose = pose

        self.get_logger().info(
            f'  Navigating to {name}: '
            f'x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}°'
        )

        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()

        if not gh.accepted:
            self.get_logger().error(f'Goal to {name} REJECTED!')
            return False

        self.get_logger().info(f'  Goal accepted → navigating...')
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=GOAL_TIMEOUT_SEC)

        if not result_future.done():
            self.get_logger().warn(
                f'  {name}: timeout ({GOAL_TIMEOUT_SEC}s) — cancelling goal'
            )
            gh.cancel_goal_async()
            return False

        result = result_future.result()
        if result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info(f' {name}: REACHED')
            return True
        else:
            self.get_logger().error(f' {name}: FAILED (status={result.status})')
            return False

    def run_sequence(self):
        self.get_logger().info(f'\n{"="*50}')
        self.get_logger().info(f'  NAV SEQUENCE: {len(NAV_WAYPOINTS)} waypoints')
        self.get_logger().info(f'{"="*50}')

        for i, (name, x, y, yaw) in enumerate(NAV_WAYPOINTS):
            self.get_logger().info(
                f'[{i+1}/{len(NAV_WAYPOINTS)}] → {name}'
            )
            success = self.navigate_to(name, x, y, yaw)
            if not success:
                self.get_logger().error(f'Navigation to "{name}" failed! Stopping.')
                return

        self.get_logger().info('=== NAV SEQUENCE COMPLETE ===')


def main(args=None):
    rclpy.init(args=args)
    node = NavSequenceNode()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
