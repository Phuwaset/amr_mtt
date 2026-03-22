#!/usr/bin/env python3
"""
arm_jog_node.py — Teach Pendant backend for Node-RED dashboard

Dual-thread architecture:
  Flask/SocketIO thread : รับ HTTP REST + push WebSocket status อัตโนมัติทุก 200 ms
  ROS 2 thread          : อ่าน /joint_states + ส่ง JTC / GripperCommand

Dependencies (pip):
  flask flask-socketio

REST Endpoints:
  GET  /status        → joints (deg/rad), EEF pose, gripper state, saved slots, mode
  POST /jog           → {"axis":"x|y|z|rx|ry|wrist", "dir":1|-1, "step":<m or rad>}
  POST /save_pos      → {"slot":"1"-"9"}
  POST /goto_pos      → {"slot":"1"-"9"}
  POST /run_sequence  → {"slots":["1","2","3"], "delay_sec":1.0}
  POST /stop_sequence → {}
  POST /gripper       → {"action":"open|close"}
  POST /home          → {}
  GET  /mode          → {"mode":"MANUAL"|"LOCKED"}
  POST /lock          → {"lock":true|false}

WebSocket (SocketIO on same port):
  Event 'status' emitted every 200 ms — identical payload to GET /status.
  Connect with any SocketIO client (e.g. node-red-contrib-socketio).

Jog axes (KDL arm-base frame after Rz(π)):
  x, y, z  — Cartesian translation, default 20 mm/click
  rx, ry   — EEF local-frame rotation around X / Y, default 5°/click
  wrist    — wrist_3 joint direct, default 5°/click

Coordinator integration:
  Subscribes to /coordinator/status (std_msgs/String).
  When "RUNNING" is received, mode switches to LOCKED (motion commands rejected).
  When "IDLE" is received, mode returns to MANUAL.
  Manual lock/unlock is available via POST /lock.
"""

import math
import json
import logging
import threading
import os
import time

# Suppress werkzeug access logs (GET /status polls every 200 ms — too noisy)
logging.getLogger('werkzeug').setLevel(logging.ERROR)

import numpy as np
import PyKDL as kdl

from flask import Flask, request, jsonify
from flask_socketio import SocketIO
from flask_cors import CORS

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from amr_mtt_task_planner.ur5_kinematics import (
    JOINT_NAMES, HOME_Q,
    build_ur5_chain, fk, solve_ik, check_joint_limits,
)

# ─────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────
HTTP_PORT     = 5005
JOG_STEP_M    = 0.02           # 20 mm per Cartesian click
JOG_STEP_RAD  = math.radians(5)  # 5° per orientation/wrist click
JOG_DURATION  = 0.3            # seconds per jog move
HOME_DURATION = 2.5
GOTO_DURATION = 1.5

POSITIONS_FILE = os.path.expanduser('~/.ros/arm_positions.json')

# Robotiq 2F-85 — the only moving joint in simulation (others are fixed)
GRIPPER_JOINT             = 'robotiq_85_left_knuckle_joint'
GRIPPER_CLOSED_THRESHOLD  = 0.1   # rad; above this → closed


# ─────────────────────────────────────────────────────────────────
#  ROS 2 NODE
# ─────────────────────────────────────────────────────────────────

class ArmJogNode(Node):

    def __init__(self):
        super().__init__('arm_jog_node')

        self.chain = build_ur5_chain()
        self._lock = threading.Lock()

        # State
        self._q              = list(HOME_Q)
        self._gripper_open   = True
        self._gripper_pos    = 0.0          # actual knuckle joint angle [rad]
        self._arm_goal_handle = None
        self._arm_done_event  = threading.Event()
        self._mode           = 'MANUAL'     # 'MANUAL' | 'LOCKED'
        self._seq_running    = False

        # Saved positions — persisted to disk
        self._positions: dict = {}
        if os.path.exists(POSITIONS_FILE):
            try:
                with open(POSITIONS_FILE) as f:
                    self._positions = json.load(f)
                self.get_logger().info(
                    f'[Jog] Loaded {len(self._positions)} saved position(s)'
                )
            except Exception as e:
                self.get_logger().warn(f'[Jog] Could not load positions: {e}')

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(String, '/coordinator/status', self._coordinator_cb, 10)

        # Action clients
        self._arm = ActionClient(
            self, FollowJointTrajectory,
            '/ur_arm_controller/follow_joint_trajectory'
        )
        self._grip = ActionClient(
            self, GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd'
        )

        self.get_logger().info('[Jog] Waiting for action servers...')
        self._arm.wait_for_server()
        self._grip.wait_for_server()
        self.get_logger().info(f'[Jog] Ready — HTTP/WS on port {HTTP_PORT}')

    # ── ROS callbacks ────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        name_map = {n: i for i, n in enumerate(msg.name)}
        q = list(self._q)
        for j, jname in enumerate(JOINT_NAMES):
            if jname in name_map:
                q[j] = msg.position[name_map[jname]]
        with self._lock:
            self._q = q
            # Update gripper state from actual joint position
            if GRIPPER_JOINT in name_map:
                self._gripper_pos = msg.position[name_map[GRIPPER_JOINT]]
                self._gripper_open = self._gripper_pos < GRIPPER_CLOSED_THRESHOLD

    def _coordinator_cb(self, msg: String):
        new_mode = 'LOCKED' if msg.data == 'RUNNING' else 'MANUAL'
        with self._lock:
            self._mode = new_mode
        self.get_logger().info(f'[Jog] Mode → {new_mode} (coordinator: {msg.data})')

    # ── Getters ──────────────────────────────────────────────────

    def current_q(self) -> list:
        with self._lock:
            return list(self._q)

    def current_eef(self, q=None) -> dict:
        if q is None:
            q = self.current_q()
        fr = fk(self.chain, q)
        rz, ry, rx = fr.M.GetEulerZYX()
        return {
            'x':     round(fr.p.x(), 4),
            'y':     round(fr.p.y(), 4),
            'z':     round(fr.p.z(), 4),
            'roll':  round(math.degrees(rx), 1),
            'pitch': round(math.degrees(ry), 1),
            'yaw':   round(math.degrees(rz), 1),
        }

    # ── Motion senders ───────────────────────────────────────────

    def _send_arm(self, q: list, duration: float = JOG_DURATION):
        """Non-blocking arm motion — cancels previous goal first."""
        with self._lock:
            old_gh = self._arm_goal_handle
        if old_gh is not None:
            old_gh.cancel_goal_async()

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1) * 1e9)
        traj.points.append(pt)
        goal.trajectory = traj

        future = self._arm.send_goal_async(goal)
        future.add_done_callback(self._arm_goal_response_cb)

    def _arm_goal_response_cb(self, future):
        gh = future.result()
        if gh.accepted:
            with self._lock:
                self._arm_goal_handle = gh
            gh.get_result_async().add_done_callback(self._arm_result_cb)

    def _arm_result_cb(self, future):
        with self._lock:
            self._arm_goal_handle = None
        self._arm_done_event.set()   # แจ้ง _runner ว่า trajectory เสร็จแล้ว

    def _send_gripper(self, open_g: bool):
        goal = GripperCommand.Goal()
        goal.command.position   = 0.01 if open_g else 0.65
        goal.command.max_effort = 100.0
        self._grip.send_goal_async(goal)

    # ── API handlers ─────────────────────────────────────────────

    def _check_locked(self) -> str | None:
        """Return error string if node is locked, else None."""
        with self._lock:
            mode = self._mode
        if mode == 'LOCKED':
            return 'locked by coordinator — use POST /lock {"lock":false} to override'
        return None

    def do_jog(self, axis: str, direction: int, step: float) -> str:
        err = self._check_locked()
        if err:
            return err

        q = self.current_q()

        # ── Joint-space: wrist_3 ──────────────────────────────
        if axis == 'wrist':
            q_new = list(q)
            q_new[5] += direction * step
            ok, violations = check_joint_limits(q_new)
            if not ok:
                return f'joint limit: {violations[0]}'
            self._send_arm(q_new, JOG_DURATION)
            return 'ok'

        # ── Cartesian translation: x / y / z ─────────────────
        if axis in ('x', 'y', 'z'):
            fr = fk(self.chain, q)
            p  = np.array([fr.p.x(), fr.p.y(), fr.p.z()])
            idx = {'x': 0, 'y': 1, 'z': 2}[axis]
            p[idx] += direction * step
            target = kdl.Frame(fr.M, kdl.Vector(*p))
            q_new  = solve_ik(self.chain, target, q)
            if q_new is None:
                return 'IK failed — target may be out of reach'
            ok, violations = check_joint_limits(q_new)
            if not ok:
                return f'joint limit: {violations[0]}'
            self._send_arm(q_new, JOG_DURATION)
            return 'ok'

        # ── EEF orientation: rotate around local X or Y ──────
        if axis in ('rx', 'ry'):
            fr = fk(self.chain, q)
            delta = (kdl.Rotation.RotX(direction * step) if axis == 'rx'
                     else kdl.Rotation.RotY(direction * step))
            target = kdl.Frame(fr.M * delta, fr.p)
            q_new  = solve_ik(self.chain, target, q)
            if q_new is None:
                return 'IK failed — orientation may be singular'
            ok, violations = check_joint_limits(q_new)
            if not ok:
                return f'joint limit: {violations[0]}'
            self._send_arm(q_new, JOG_DURATION)
            return 'ok'

        return f'unknown axis: {axis}'

    def do_save(self, slot: str) -> str:
        q   = self.current_q()
        eef = self.current_eef(q)
        with self._lock:
            gripper_open = self._gripper_open
        self._positions[slot] = {
            'q':           [round(v, 5) for v in q],
            'eef':         eef,
            'gripper_open': gripper_open,
        }
        os.makedirs(os.path.dirname(POSITIONS_FILE), exist_ok=True)
        with open(POSITIONS_FILE, 'w') as f:
            json.dump(self._positions, f, indent=2)
        self.get_logger().info(f'[Jog] Saved slot {slot}: {eef}')
        return 'ok'

    def do_goto(self, slot: str) -> str:
        err = self._check_locked()
        if err:
            return err
        if slot not in self._positions:
            return f'slot {slot} not saved yet'
        pos = self._positions[slot]
        self._send_arm(pos['q'], GOTO_DURATION)
        if 'gripper_open' in pos:
            self._send_gripper(pos['gripper_open'])
        self.get_logger().info(f'[Jog] Going to slot {slot}')
        return 'ok'

    def do_run_sequence(self, slots: list, delay_sec: float = 1.0) -> str:
        """Play through saved slots in order (non-blocking background thread)."""
        err = self._check_locked()
        if err:
            return err
        if self._seq_running:
            return 'sequence already running — POST /stop_sequence to abort'
        missing = [s for s in slots if s not in self._positions]
        if missing:
            return f'slots not saved: {missing}'

        def _runner():
            self._seq_running = True
            total = len(slots)
            self.get_logger().info(f'[Jog] Sequence start: {slots}')
            try:
                for i, slot in enumerate(slots):
                    if not self._seq_running or self._mode == 'LOCKED':
                        self.get_logger().info('[Jog] Sequence aborted')
                        socketio.emit('seq_progress', {
                            'slot': slot, 'idx': i, 'total': total, 'aborted': True
                        })
                        return
                    pos = self._positions[slot]
                    self._arm_done_event.clear()
                    self._send_arm(pos['q'], GOTO_DURATION)
                    # รอ action result จริง (ไม่ใช่ wall-clock sleep)
                    self._arm_done_event.wait(timeout=GOTO_DURATION * 10)
                    # ส่ง gripper command ตาม state ที่บันทึกไว้
                    if 'gripper_open' in pos:
                        self._send_gripper(pos['gripper_open'])
                    time.sleep(delay_sec)   # หน่วงเล็กน้อยหลังถึงจุด
                    self.get_logger().info(f'[Jog] Slot {slot} done ({i+1}/{total})')
                    socketio.emit('seq_progress', {
                        'slot': slot, 'idx': i + 1, 'total': total, 'aborted': False
                    })
                self.get_logger().info('[Jog] Sequence complete')
                socketio.emit('seq_complete', {'slots': slots})
            finally:
                self._seq_running = False

        threading.Thread(target=_runner, daemon=True).start()
        return f'sequence started: {slots}'

    def do_stop_sequence(self) -> str:
        self._seq_running = False
        return 'stop requested'

    def do_home(self) -> str:
        err = self._check_locked()
        if err:
            return err
        self._send_arm(HOME_Q, HOME_DURATION)
        return 'ok'

    def do_gripper(self, action: str) -> str:
        err = self._check_locked()
        if err:
            return err
        if action == 'open':
            self._send_gripper(True)
        elif action == 'close':
            self._send_gripper(False)
        else:
            return f'unknown action: {action}'
        return 'ok'

    def do_emergency_stop(self) -> str:
        """Cancel active arm goal immediately and lock all motion commands."""
        with self._lock:
            old_gh = self._arm_goal_handle
            self._mode = 'LOCKED'
        self._seq_running = False   # หยุด sequence runner
        if old_gh is not None:
            old_gh.cancel_goal_async()
        self.get_logger().warn('[Jog] *** EMERGENCY STOP activated ***')
        return 'LOCKED'

    def do_move_joint(self, joint_idx: int, angle_deg: float, duration_sec: float = JOG_DURATION) -> str:
        """Move a single joint to an absolute angle (degrees)."""
        err = self._check_locked()
        if err:
            return err
        if not 0 <= joint_idx <= 5:
            return f'invalid joint index: {joint_idx}'
        q_new = self.current_q()
        q_new[joint_idx] = math.radians(angle_deg)
        ok, violations = check_joint_limits(q_new)
        if not ok:
            return f'joint limit: {violations[0]}'
        self._send_arm(q_new, max(0.1, float(duration_sec)))
        return 'ok'

    def do_set_lock(self, locked: bool) -> str:
        with self._lock:
            self._mode = 'LOCKED' if locked else 'MANUAL'
        self.get_logger().info(f'[Jog] Manual lock → {self._mode}')
        return self._mode


# ─────────────────────────────────────────────────────────────────
#  FLASK + SOCKETIO
# ─────────────────────────────────────────────────────────────────

app      = Flask(__name__)
CORS(app)
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins='*')
node: ArmJogNode = None   # set in main()


def _build_status() -> dict:
    q   = node.current_q()
    eef = node.current_eef(q)
    with node._lock:
        mode        = node._mode
        seq_running = node._seq_running
        gripper_open = node._gripper_open
        gripper_pos  = node._gripper_pos
        saved_slots  = list(node._positions.keys())
    return {
        'joints':       [round(math.degrees(v), 2) for v in q],
        'joints_rad':   [round(v, 4) for v in q],
        'eef':          eef,
        'gripper_open': gripper_open,
        'gripper_pos':  round(gripper_pos, 3),
        'saved_slots':  saved_slots,
        'mode':         mode,
        'seq_running':  seq_running,
    }


def _status_pusher():
    """Background task: push status to all WebSocket clients every 200 ms."""
    while True:
        socketio.sleep(0.2)
        if node is not None:
            socketio.emit('status', _build_status())


@socketio.on('connect')
def on_ws_connect():
    if node is not None:
        socketio.emit('status', _build_status())


# ── REST routes ──────────────────────────────────────────────────

@app.route('/status', methods=['GET'])
def status():
    return jsonify(_build_status())


@app.route('/jog', methods=['POST'])
def jog():
    data  = request.get_json(force=True)
    axis  = data.get('axis', 'z')
    dir_  = int(data.get('dir', 1))
    is_rot = axis in ('wrist', 'rx', 'ry')
    step  = float(data.get('step', JOG_STEP_RAD if is_rot else JOG_STEP_M))
    return jsonify({'status': node.do_jog(axis, dir_, step)})


@app.route('/save_pos', methods=['POST'])
def save_pos():
    slot = str(request.get_json(force=True).get('slot', '1'))
    return jsonify({'status': node.do_save(slot)})


@app.route('/goto_pos', methods=['POST'])
def goto_pos():
    slot = str(request.get_json(force=True).get('slot', '1'))
    return jsonify({'status': node.do_goto(slot)})


@app.route('/run_sequence', methods=['POST'])
def run_sequence():
    data      = request.get_json(force=True)
    slots     = [str(s) for s in data.get('slots', [])]
    delay_sec = float(data.get('delay_sec', 0.3))
    if not slots:
        return jsonify({'status': 'no slots provided'}), 400
    return jsonify({'status': node.do_run_sequence(slots, delay_sec)})


@app.route('/stop_sequence', methods=['POST'])
def stop_sequence():
    return jsonify({'status': node.do_stop_sequence()})


@app.route('/gripper', methods=['POST'])
def gripper():
    action = request.get_json(force=True).get('action', 'open')
    return jsonify({'status': node.do_gripper(action)})


@app.route('/home', methods=['POST'])
def home():
    return jsonify({'status': node.do_home()})


@app.route('/mode', methods=['GET'])
def mode():
    with node._lock:
        return jsonify({'mode': node._mode})


@app.route('/estop', methods=['POST'])
def estop():
    return jsonify({'mode': node.do_emergency_stop()})


@app.route('/move_joint', methods=['POST'])
def move_joint():
    data         = request.get_json(force=True)
    joint_idx    = int(data.get('joint', 0))
    angle_deg    = float(data.get('angle_deg', 0.0))
    duration_sec = float(data.get('duration_sec', JOG_DURATION))
    return jsonify({'status': node.do_move_joint(joint_idx, angle_deg, duration_sec)})


@app.route('/lock', methods=['POST'])
def lock():
    locked = bool(request.get_json(force=True).get('lock', True))
    return jsonify({'mode': node.do_set_lock(locked)})


# ─────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────

def main(args=None):
    global node
    rclpy.init(args=args)
    node = ArmJogNode()

    # Start WebSocket status pusher
    socketio.start_background_task(_status_pusher)

    # Flask + SocketIO in daemon thread
    flask_thread = threading.Thread(
        target=lambda: socketio.run(
            app, host='0.0.0.0', port=HTTP_PORT, log_output=False,
            allow_unsafe_werkzeug=True
        ),
        daemon=True,
    )
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
