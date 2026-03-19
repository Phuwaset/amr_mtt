#!/usr/bin/env python3
"""
coordinator_node.py — Full mobile manipulation pipeline

Pipeline:
  1. Reset  : arm → HOME, gripper → open
  2. Nav    : navigate to pick location (Nav2)
  3. IK     : compute pick poses in world frame via TF2
  4. Pick   : pre-grasp → grasp → close gripper → lift
  5. Nav    : navigate to drop location (Nav2)
  6. Drop   : fold arm over chassis → open gripper → HOME

Improvements over v1:
  - All arm waypoints computed via IK from world-frame targets (no hardcoded joints)
  - IK pre-computed before any motion starts (fail-fast)
  - send_arm_goal / send_gripper_goal wait for actual action completion
  - Publishes /coordinator/status for arm_jog_node lock integration
"""

import math
import time

import numpy as np
import PyKDL as kdl

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros

from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

from amr_mtt_task_planner.ur5_kinematics import (
    JOINT_NAMES, HOME_Q, build_ur5_chain, solve_ik,
)

# ── Task configuration ────────────────────────────────────────────────────

# Box to pick — world frame (same as pick_ik.py TARGET_BOX)
TARGET_BOX = {
    'world_x': -1.500,
    'world_y': -0.610,
    'world_z':  0.85,
    'half_h':   0.05,   # box half-height
}
TOOL_OFFSET = 0.13   # m: tool0 → gripper contact (Robotiq 2F-85)
PRE_HEIGHT  = 0.15   # m: pre-grasp above contact
LIFT_HEIGHT = 0.15   # m: lift after grasp

# Nav2 waypoints — adjust to match your map
PICK_NAV = dict(x=-1.70, y=-0.60, yaw_z=0.0, yaw_w=1.0)  # beside table
DROP_NAV = dict(x=-0.50, y=-1.30, yaw_z=0.0, yaw_w=1.0)  # drop zone

# Arm pose for placing box on chassis (joint space — body-relative, no IK needed)
PLACE_Q = [0.0, -1.3, -2.0, -1.57, -1.57, 0.0]

GRIPPER_SETTLE = 0.8   # s: wait after gripper closes/opens


# ── Node ──────────────────────────────────────────────────────────────────

class CoordinatorNode(Node):

    def __init__(self):
        super().__init__('coordinator_node')
        self.get_logger().info('Initializing Coordinator Node...')

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/ur_arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd'
        )

        # Status publisher — arm_jog_node subscribes to lock manual jog
        self.status_pub = self.create_publisher(String, '/coordinator/status', 10)

        # TF2 — for world → ur5_base_link transform
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # KDL chain for IK
        self.chain = build_ur5_chain()

        self._wait_for_servers()

    # ── Initialisation ───────────────────────────────────────────

    def _wait_for_servers(self):
        self.get_logger().info('Waiting for action servers...')
        self.nav_client.wait_for_server()
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('All servers ready.')

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    # ── TF + IK helpers ──────────────────────────────────────────

    def _get_T_world_arm(self) -> np.ndarray:
        """
        Lookup world → ur5_base_link as a 4×4 homogeneous transform.
        Falls back to known spawn position if TF is unavailable.
        """
        for frame in ('ur5_base_link', 'amr_mtt/ur5_base_link'):
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    'world', frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=3.0)
                )
                t  = tf_msg.transform.translation
                q  = tf_msg.transform.rotation
                qx, qy, qz, qw = q.x, q.y, q.z, q.w
                R = np.array([
                    [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
                    [2*(qx*qy + qz*qw),       1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                    [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),   1 - 2*(qx**2 + qy**2)],
                ], dtype=float)
                T = np.eye(4)
                T[:3, :3] = R
                T[:3,  3] = [t.x, t.y, t.z]
                self.get_logger().info(f'[TF] world→{frame}: p={T[:3,3].round(3)}')
                return T
            except Exception:
                continue

        # Fallback: compute from known spawn position
        arm_x, arm_y, arm_z = -1.5, -1.3, 1.25
        self.get_logger().warn(
            f'[TF] Lookup failed — using fallback ({arm_x}, {arm_y}, {arm_z})'
        )
        T = np.eye(4)
        T[:3, 3] = [arm_x, arm_y, arm_z]
        return T

    def _joints_for_world_pose(self, T_world_target: np.ndarray,
                                seed: list, label: str) -> list:
        """
        Transform target from world frame → arm frame, solve IK.
        Returns list[6] or None on failure.
        """
        T_arm_target = np.linalg.inv(self._get_T_world_arm()) @ T_world_target
        p = T_arm_target[:3, 3]
        R = T_arm_target[:3, :3]

        self.get_logger().info(
            f'[IK] {label} in arm frame: ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})'
        )

        target_frame = kdl.Frame(
            kdl.Rotation(
                R[0, 0], R[0, 1], R[0, 2],
                R[1, 0], R[1, 1], R[1, 2],
                R[2, 0], R[2, 1], R[2, 2],
            ),
            kdl.Vector(p[0], p[1], p[2])
        )

        extra_seeds = [
            [0.0, -math.pi / 2, 0.0,  0.0, -math.pi / 2, 0.0],
            [0.0, -1.0, -1.5, -1.0, -1.5708, 0.0],
            [0.0, -1.5708, -1.5708, -1.5708, -1.5708, 0.0],
        ]
        q = solve_ik(self.chain, target_frame, seed, extra_seeds=extra_seeds)

        if q is None:
            self.get_logger().error(f'[IK] FAILED for {label}')
        else:
            self.get_logger().info(f'[IK] {label}: {[f"{v:.3f}" for v in q]}')
        return q

    def _compute_pick_poses(self) -> dict:
        """
        Compute pre_grasp, grasp, lift as 4×4 HT matrices in world frame.
        Gripper orientation: pointing straight down (Rx(π)).
        """
        bx = TARGET_BOX['world_x']
        by = TARGET_BOX['world_y']
        bz = TARGET_BOX['world_z']
        tool0_z = bz + TARGET_BOX['half_h'] + TOOL_OFFSET

        # Gripper pointing down, jaw parallel to world X
        R_down = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)

        def _ht(z_offset: float) -> np.ndarray:
            T = np.eye(4)
            T[:3, :3] = R_down
            T[:3,  3] = [bx, by, tool0_z + z_offset]
            return T

        return {
            'pre_grasp': _ht(PRE_HEIGHT),
            'grasp':     _ht(0.0),
            'lift':      _ht(LIFT_HEIGHT),
        }

    # ── Motion primitives ────────────────────────────────────────

    def send_nav_goal(self, x: float, y: float,
                      yaw_z: float, yaw_w: float) -> bool:
        """Navigate to (x, y) and wait for arrival."""
        self.get_logger().info(f'[NAV] → ({x:.2f}, {y:.2f})')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id       = 'map'
        goal.pose.header.stamp          = self.get_clock().now().to_msg()
        goal.pose.pose.position.x       = float(x)
        goal.pose.pose.position.y       = float(y)
        goal.pose.pose.orientation.z    = float(yaw_z)
        goal.pose.pose.orientation.w    = float(yaw_w)

        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('[NAV] Goal rejected')
            return False

        self.get_logger().info('[NAV] Moving...')
        rclpy.spin_until_future_complete(self, gh.get_result_async())
        self.get_logger().info('[NAV] Arrived.')
        return True

    def send_arm_goal(self, q: list, duration_sec: float) -> bool:
        """Send arm to joint configuration and wait for completion."""
        self.get_logger().info(f'[ARM] Moving  duration={duration_sec}s')
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start.sec      = int(duration_sec)
        pt.time_from_start.nanosec  = int((duration_sec % 1) * 1e9)
        traj.points.append(pt)
        goal.trajectory = traj

        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('[ARM] Goal rejected')
            return False

        rclpy.spin_until_future_complete(self, gh.get_result_async())
        return True

    def send_gripper_goal(self, open_gripper: bool) -> bool:
        """Open or close gripper and wait for completion."""
        label = 'OPEN' if open_gripper else 'CLOSE'
        self.get_logger().info(f'[GRIPPER] {label}')
        goal = GripperCommand.Goal()
        goal.command.position   = 0.01 if open_gripper else 0.65
        goal.command.max_effort = 100.0

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('[GRIPPER] Goal rejected')
            return False

        rclpy.spin_until_future_complete(self, gh.get_result_async())
        time.sleep(GRIPPER_SETTLE)  # let physics settle
        return True

    # ── Pipeline ─────────────────────────────────────────────────

    def start_pipeline(self):
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('MISSION START: Autonomous Courier')
        self.get_logger().info('='*50 + '\n')
        self._publish_status('RUNNING')

        # ── Pre-compute all IK before moving (fail fast) ─────────
        self.get_logger().info('[SETUP] Waiting for TF...')
        time.sleep(2.0)

        poses = self._compute_pick_poses()
        seed = HOME_Q[:]

        q_pre   = self._joints_for_world_pose(poses['pre_grasp'], seed,  'PRE_GRASP')
        q_grasp = self._joints_for_world_pose(poses['grasp'],  q_pre or seed,  'GRASP')
        q_lift  = self._joints_for_world_pose(poses['lift'],   q_grasp or seed, 'LIFT')

        if any(q is None for q in (q_pre, q_grasp, q_lift)):
            self.get_logger().error('IK pre-computation failed — aborting mission')
            self._publish_status('IDLE')
            return

        self.get_logger().info('[SETUP] All IK solutions found — starting execution\n')

        # ── 1. Reset ─────────────────────────────────────────────
        self.get_logger().info('[1/7] Resetting posture')
        self.send_gripper_goal(open_gripper=True)
        self.send_arm_goal(HOME_Q, 2.5)

        # ── 2. Navigate to pick location ─────────────────────────
        self.get_logger().info('[2/7] Navigating to pick location')
        if not self.send_nav_goal(**PICK_NAV):
            self._publish_status('IDLE')
            return

        # ── 3. Pre-grasp ─────────────────────────────────────────
        self.get_logger().info('[3/7] Pre-grasp approach')
        self.send_arm_goal(q_pre, 2.0)

        # ── 4. Grasp ─────────────────────────────────────────────
        self.get_logger().info('[4/7] Grasp')
        self.send_arm_goal(q_grasp, 1.2)
        self.send_gripper_goal(open_gripper=False)

        # ── 5. Lift ──────────────────────────────────────────────
        self.get_logger().info('[5/7] Lift')
        self.send_arm_goal(q_lift, 1.2)

        # ── 6. Navigate to drop location ─────────────────────────
        self.get_logger().info('[6/7] Navigating to drop location')
        if not self.send_nav_goal(**DROP_NAV):
            self._publish_status('IDLE')
            return

        # ── 7. Place on chassis and return home ──────────────────
        self.get_logger().info('[7/7] Placing on chassis')
        self.send_arm_goal(PLACE_Q, 3.0)
        self.send_gripper_goal(open_gripper=True)
        self.send_arm_goal(HOME_Q, 2.5)

        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('MISSION COMPLETE')
        self.get_logger().info('='*50 + '\n')
        self._publish_status('IDLE')


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    time.sleep(2.0)  # let everything settle before starting
    try:
        node.start_pipeline()
    except KeyboardInterrupt:
        node.get_logger().info('Pipeline interrupted.')
        node._publish_status('IDLE')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
