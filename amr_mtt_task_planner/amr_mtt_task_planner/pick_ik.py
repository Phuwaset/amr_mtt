#!/usr/bin/env python3
"""
pick_ik.py — IK-based pick using PyKDL + Homogeneous Transformation (Approach A)

ขั้นตอน:
  1. สร้าง KDL chain ของ UR5 จาก DH parameters (ur_description/config/ur5)
  2. คำนวณ grasp pose ใน world frame ด้วย Homogeneous Transformation (numpy)
  3. Transform → ur5_base_link frame ผ่าน TF2
  4. แก้ IK ด้วย PyKDL ChainIkSolverPos_NR_JL
  5. ส่ง joint trajectory ไป /ur_arm_controller (JTC)

ไม่ต้องใช้ MoveIt — ทำงานได้กับ launch_sim_amr.launch.py
"""

import math
import numpy as np
import PyKDL as kdl

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — register transforms

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped

from amr_mtt_task_planner.ur5_kinematics import (
    JOINT_NAMES, HOME_Q, build_ur5_chain, fk as fk_check, solve_ik,
)

# ============================================================
#  CONFIG — ปรับค่าตรงนี้เพื่อ tune การหยิบ
# ============================================================
TARGET_BOX = {
    'name':    'parcel_box_f2',
    'world_x': -1.500,
    'world_y': -0.610,
    'world_z':  0.85,       # center of box
    'half_h':   0.05,       # box half-height (H=10cm)
}

TOOL_OFFSET   = 0.13   # m: ระยะ tool0 → จุด contact gripper (ปรับตาม Robotiq 2F-85)
PRE_HEIGHT    = 0.15   # m: ระยะ pre-grasp เหนือ contact
LIFT_HEIGHT   = 0.15   # m: ยกขึ้นหลัง grasp

# Extra IK seeds for downward grasp configurations
_EXTRA_SEEDS = [
    [0.0,  -math.pi / 2, 0.0,  0.0,  -math.pi / 2,  0.0],
    [0.0,  -1.0,  -1.5,  -1.0,  -1.5708,  0.0],
    [0.0,  -1.5708, -1.5708, -1.5708, -1.5708, 0.0],
]


# ============================================================
#  HOMOGENEOUS TRANSFORMATION — คำนวณ grasp poses
# ============================================================

def make_ht(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """สร้าง 4×4 Homogeneous Transformation matrix"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t
    return T


def compute_grasp_poses(box: dict) -> dict:
    """
    คำนวณ pre_grasp, grasp, lift pose ใน world frame
    โดยใช้ Homogeneous Transformation

    Gripper orientation: ชี้ตรงลงมา (tool0 Z = world −Z)
                         jaw เปิดตาม world X (คีบด้าน 7cm)
    กล่อง face X (7cm) → gripper jaw ต้องขนานกับ X

    R_grasp = Rx(π) = [[1, 0, 0],
                        [0,−1, 0],
                        [0, 0,−1]]
    """
    bx = box['world_x']
    by = box['world_y']
    bz = box['world_z']
    top_z = bz + box['half_h']          # z หน้าบนสุดกล่อง

    # tool0 ต้องอยู่สูงกว่าหน้าบนกล่อง ตามระยะ TOOL_OFFSET
    tool0_z = top_z + TOOL_OFFSET

    # Rx(π): gripper ชี้ลง, jaw ขนาน X
    R_down = np.array([
        [ 1,  0,  0],
        [ 0, -1,  0],
        [ 0,  0, -1]
    ], dtype=float)

    return {
        'pre_grasp': make_ht(R_down, [bx, by, tool0_z + PRE_HEIGHT]),
        'grasp':     make_ht(R_down, [bx, by, tool0_z]),
        'lift':      make_ht(R_down, [bx, by, tool0_z + LIFT_HEIGHT]),
    }


def ht_to_kdl_frame(T: np.ndarray) -> kdl.Frame:
    """แปลง 4×4 numpy matrix → kdl.Frame"""
    R = T[:3, :3]
    p = T[:3,  3]
    return kdl.Frame(
        kdl.Rotation(
            R[0, 0], R[0, 1], R[0, 2],
            R[1, 0], R[1, 1], R[1, 2],
            R[2, 0], R[2, 1], R[2, 2]
        ),
        kdl.Vector(p[0], p[1], p[2])
    )


def tf_to_ht(t, q) -> np.ndarray:
    """แปลง TF transform (translation + quaternion) → 4×4 HT"""
    tx, ty, tz = t.x, t.y, t.z
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    # Quaternion → Rotation matrix
    R = np.array([
        [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),      1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),      2*(qy*qz + qx*qw),   1 - 2*(qx**2 + qy**2)]
    ], dtype=float)
    return make_ht(R, [tx, ty, tz])




# ============================================================
#  MAIN NODE
# ============================================================

class PickIKNode(Node):
    def __init__(self):
        super().__init__('pick_ik_node')

        # TF2 listener — รับ world → ur5_base_link transform
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Action clients
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/ur_arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd'
        )

        # Build KDL chain ครั้งเดียว
        self.chain = build_ur5_chain()
        n_joints   = self.chain.getNrOfJoints()
        self.get_logger().info(
            f'[IK] UR5 KDL chain built — {self.chain.getNrOfSegments()} segments, '
            f'{n_joints} joints'
        )
        # FK diagnostic at HOME to verify chain
        fk_home = fk_check(self.chain, HOME_Q)
        self.get_logger().info(
            f'[FK@HOME] p=({fk_home.p.x():.4f}, {fk_home.p.y():.4f}, {fk_home.p.z():.4f})'
        )
        # Extract RPY from rotation
        rz, ry, rx = fk_home.M.GetEulerZYX()
        self.get_logger().info(f'[FK@HOME] rpy=({rx:.3f}, {ry:.3f}, {rz:.3f})')

        self.get_logger().info('[IK] Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('[IK] Ready.')

    # ── TF: world → ur5_base_link ────────────────────────────────
    def get_T_world_arm(self) -> np.ndarray:
        """
        ดึง transform world → ur5_base_link ผ่าน TF2
        ถ้าดึงไม่ได้ → ใช้ค่าจาก spawn position (fallback)
        """
        for frame in ('ur5_base_link', 'amr_mtt/ur5_base_link'):
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    'world', frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=3.0)
                )
                T = tf_to_ht(
                    tf_msg.transform.translation,
                    tf_msg.transform.rotation
                )
                self.get_logger().info(f'[TF] world→{frame}: p={T[:3,3]}')
                return T
            except Exception:
                continue

        # Fallback: คำนวณจาก spawn params
        # base_footprint @ (-1.7, -1.3, 0.28)
        # base_link offset: (0, 0, 0.25)
        # ur5_base_link offset: (0.2, 0, 0.72)
        arm_x = -1.7 + 0.2          # = -1.5
        arm_y = -1.3
        arm_z =  0.28 + 0.25 + 0.72 # = 1.25
        self.get_logger().warn(
            f'[TF] lookup failed — using fallback: ({arm_x}, {arm_y}, {arm_z})'
        )
        return make_ht(np.eye(3), [arm_x, arm_y, arm_z])

    # ── IK: target ใน world frame → joint angles ─────────────────
    def world_to_joints(self, T_world_target: np.ndarray, seed: list, label: str):
        """
        1. Transform target จาก world frame → ur5_base_link frame
        2. แก้ IK
        คืน joint angles (list[6]) หรือ None
        """
        T_world_arm = self.get_T_world_arm()
        T_arm_world = np.linalg.inv(T_world_arm)
        T_arm_target = T_arm_world @ T_world_target

        p = T_arm_target[:3, 3]
        self.get_logger().info(
            f'[IK] {label} in arm frame: '
            f'p=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})'
        )

        target_frame = ht_to_kdl_frame(T_arm_target)
        q = solve_ik(self.chain, target_frame, seed, extra_seeds=_EXTRA_SEEDS)

        if q is None:
            self.get_logger().error(f'[IK] FAILED for {label}')
            return None

        # FK verify
        fk = fk_check(self.chain, q)
        err = math.sqrt(
            (fk.p.x() - T_arm_target[0, 3])**2 +
            (fk.p.y() - T_arm_target[1, 3])**2 +
            (fk.p.z() - T_arm_target[2, 3])**2
        )
        self.get_logger().info(
            f'[IK] {label}: q={[f"{v:.3f}" for v in q]}  FK_err={err*1000:.1f}mm'
        )
        return q

    # ── Arm movement ─────────────────────────────────────────────
    def send_arm(self, q: list, duration_sec: float) -> bool:
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start.sec = int(duration_sec)
        pt.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        traj.points.append(pt)
        goal.trajectory = traj

        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    # ── Gripper ──────────────────────────────────────────────────
    def send_gripper(self, open_gripper: bool, wait: bool = True):
        pos   = 0.0 if open_gripper else 0.8
        label = 'OPEN' if open_gripper else 'CLOSE'
        self.get_logger().info(f'[Gripper] {label}')

        goal = GripperCommand.Goal()
        goal.command.position   = pos
        goal.command.max_effort = 100.0

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        if wait:
            gh = future.result()
            rclpy.spin_until_future_complete(
                self, gh.get_result_async(), timeout_sec=2.0
            )

    # ── Main sequence ─────────────────────────────────────────────
    def run_sequence(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'[IK] Target: {TARGET_BOX["name"]}')
        self.get_logger().info('=' * 50)

        # รอ TF พร้อม
        self.get_logger().info('[IK] Waiting 3s for TF...')
        import time; time.sleep(3.0)

        # ── คำนวณ grasp poses (Homogeneous Transformation) ────────
        poses = compute_grasp_poses(TARGET_BOX)
        self.get_logger().info(
            f'[HT] pre_grasp  p={poses["pre_grasp"][:3,3]}\n'
            f'[HT] grasp      p={poses["grasp"][:3,3]}\n'
            f'[HT] lift       p={poses["lift"][:3,3]}'
        )

        # ── IK: แก้ joint angles ──────────────────────────────────
        seed = HOME_Q[:]

        q_pre = self.world_to_joints(poses['pre_grasp'], seed, 'PRE_GRASP')
        if q_pre is None: return

        q_grasp = self.world_to_joints(poses['grasp'], q_pre, 'GRASP')
        if q_grasp is None: return

        q_lift = self.world_to_joints(poses['lift'], q_grasp, 'LIFT')
        if q_lift is None: return

        self.get_logger().info('[IK] All solutions found — starting execution')

        # ── STEP 1: HOME ──────────────────────────────────────────
        self.get_logger().info('[1/6] HOME')
        self.send_arm(HOME_Q, 2.0)
        self.send_gripper(open_gripper=True)

        # ── STEP 2: PRE_GRASP ─────────────────────────────────────
        self.get_logger().info('[2/6] PRE_GRASP (เข้าใกล้จากบน)')
        self.send_arm(q_pre, 2.0)

        # ── STEP 3: GRASP ─────────────────────────────────────────
        self.get_logger().info('[3/6] GRASP (ลงไปที่กล่อง)')
        self.send_arm(q_grasp, 1.2)

        # ── STEP 4: CLOSE GRIPPER ────────────────────────────────
        self.get_logger().info('[4/6] CLOSE GRIPPER')
        self.send_gripper(open_gripper=False, wait=True)

        # ── STEP 5: LIFT ─────────────────────────────────────────
        self.get_logger().info('[5/6] LIFT (ยกกล่อง)')
        self.send_arm(q_lift, 1.2)

        # ── STEP 6: RETURN HOME ───────────────────────────────────
        self.get_logger().info('[6/6] HOME_RETURN')
        self.send_arm(HOME_Q, 2.0)

        self.get_logger().info('=' * 50)
        self.get_logger().info('[IK] SEQUENCE COMPLETE')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = PickIKNode()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
