#!/usr/bin/env python3
"""
ur5_kinematics.py — Shared UR5 KDL kinematics

Centralises chain construction, FK, IK, and joint-limit checking so that
arm_jog_node and pick_ik don't duplicate DH parameters.

Usage:
    from amr_mtt_task_planner.ur5_kinematics import (
        JOINT_NAMES, HOME_Q, build_ur5_chain, fk, solve_ik, check_joint_limits
    )
"""

import math
import PyKDL as kdl

# ── Constants ──────────────────────────────────────────────────────────────

JOINT_NAMES = [
    'ur5_shoulder_pan_joint',
    'ur5_shoulder_lift_joint',
    'ur5_elbow_joint',
    'ur5_wrist_1_joint',
    'ur5_wrist_2_joint',
    'ur5_wrist_3_joint',
]

HOME_Q = [0.0, -1.5708, -1.5708, -3.159, 0.0, 0.0]

# UR5 joint limits [min, max] in radians — from UR5 technical manual
JOINT_LIMITS = [
    (-2 * math.pi, 2 * math.pi),  # shoulder_pan  ±360°
    (-2 * math.pi, 2 * math.pi),  # shoulder_lift ±360°
    (-math.pi,      math.pi),      # elbow         ±180°
    (-2 * math.pi, 2 * math.pi),  # wrist_1       ±360°
    (-2 * math.pi, 2 * math.pi),  # wrist_2       ±360°
    (-2 * math.pi, 2 * math.pi),  # wrist_3       ±360°
]


# ── KDL chain ─────────────────────────────────────────────────────────────

def build_ur5_chain() -> kdl.Chain:
    """
    Build KDL chain: ur5_base_link → ur5_tool0.
    DH parameters from UR5 technical manual / ur_description default_kinematics.yaml.
    Every joint rotates around local Z (RotZ).
    The leading Rz(π) fixed segment aligns the UR5 base convention.
    """
    pi = math.pi
    chain = kdl.Chain()

    chain.addSegment(kdl.Segment(
        'base', kdl.Joint(kdl.Joint.Fixed),
        kdl.Frame(kdl.Rotation.RotZ(pi), kdl.Vector(0, 0, 0))
    ))

    # (joint_name, a [m], d [m], alpha [rad])
    DH = [
        ('ur5_shoulder_pan_joint',   0,        0.089159,  pi / 2),
        ('ur5_shoulder_lift_joint', -0.425,    0,         0),
        ('ur5_elbow_joint',         -0.39225,  0,         0),
        ('ur5_wrist_1_joint',        0,        0.10915,   pi / 2),
        ('ur5_wrist_2_joint',        0,        0.09465,  -pi / 2),
        ('ur5_wrist_3_joint',        0,        0.0823,    0),
    ]

    for name, a, d, alpha in DH:
        chain.addSegment(kdl.Segment(
            name.replace('joint', 'link'),
            kdl.Joint(name, kdl.Joint.RotZ),
            kdl.Frame(kdl.Rotation.RotX(alpha), kdl.Vector(a, 0, d))
        ))

    return chain


# ── Solvers ───────────────────────────────────────────────────────────────

def fk(chain: kdl.Chain, q: list) -> kdl.Frame:
    """Forward kinematics: joint angles (rad) → EEF kdl.Frame."""
    solver = kdl.ChainFkSolverPos_recursive(chain)
    ja = kdl.JntArray(len(q))
    for i, v in enumerate(q):
        ja[i] = float(v)
    frame = kdl.Frame()
    solver.JntToCart(ja, frame)
    return frame


def solve_ik(chain: kdl.Chain, target: kdl.Frame, q_seed: list,
             extra_seeds: list = None) -> list:
    """
    LMA inverse kinematics (Levenberg-Marquardt).
    Tries q_seed first, then each entry in extra_seeds.
    Returns list[6] of joint angles, or None if no solution found.
    """
    n = chain.getNrOfJoints()
    ik = kdl.ChainIkSolverPos_LMA(chain, eps=1e-4, maxiter=500)
    for seed in [q_seed] + (extra_seeds or []):
        qa = kdl.JntArray(n)
        for i, v in enumerate(seed):
            qa[i] = float(v)
        qo = kdl.JntArray(n)
        if ik.CartToJnt(qa, target, qo) >= 0:
            return [qo[i] for i in range(n)]
    return None


# ── Validation ────────────────────────────────────────────────────────────

def check_joint_limits(q: list) -> tuple:
    """
    Validate joint angles against UR5 limits.
    Returns (ok: bool, violations: list[str]).
    """
    violations = []
    for i, (v, (lo, hi)) in enumerate(zip(q, JOINT_LIMITS)):
        if not (lo <= v <= hi):
            violations.append(
                f'{JOINT_NAMES[i]}: {math.degrees(v):.1f}° outside '
                f'[{math.degrees(lo):.0f}°, {math.degrees(hi):.0f}°]'
            )
    return (len(violations) == 0, violations)
