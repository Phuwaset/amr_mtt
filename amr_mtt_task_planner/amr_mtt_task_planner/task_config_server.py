#!/usr/bin/env python3
"""
task_config_server.py — Task Configuration & Autonomous Execution Orchestrator

HTTP REST API (Flask, port 5008):
  GET  /task_config             → อ่าน config ปัจจุบัน
  GET  /task_status             → สถานะ pipeline
  POST /save_pick_sequence      → {"slots": ["1","2","3","4","5"]}
  POST /save_drop_sequence      → {"slots": ["6","7","8","9","10"]}
  POST /save_pick_location      → บันทึก pose หุ่น ณ จุดหยิบ (เรียก nav_waypoint_server)
  POST /save_drop_location      → บันทึก pose หุ่น ณ จุดวาง
  POST /save_nav_waypoint       → {"name": "A"} บันทึก waypoint ผ่านทาง
  POST /run_task                → เริ่ม autonomous pipeline
  POST /stop_task               → หยุด pipeline

วิธีใช้:
  ros2 launch amr_mtt_task_planner node_red_backend.launch.py
"""

import json
import logging
import os
import threading
import time

import requests
from flask import Flask, jsonify, request
from flask_cors import CORS

logging.getLogger('werkzeug').setLevel(logging.ERROR)

# ─────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────
HTTP_PORT       = 5008
TASK_CONFIG_FILE = os.path.expanduser('~/.ros/task_config.json')

ARM_API = 'http://localhost:5005'
NAV_API = 'http://localhost:5006'

PICK_WP_NAME = 'PICK_SPOT'
DROP_WP_NAME = 'DROP_SPOT'

_DEFAULT_CONFIG = {
    'pick_sequence': [],
    'drop_sequence': [],
    'pick_waypoint': PICK_WP_NAME,
    'nav_path':      [],
    'drop_waypoint': DROP_WP_NAME,
}


# ─────────────────────────────────────────────────────────────────
#  STATE
# ─────────────────────────────────────────────────────────────────
_lock        = threading.Lock()
_task_running = False
_stop_flag    = False
_task_status  = 'idle'   # idle | running | completed | error | stopped


def _load_config() -> dict:
    if os.path.exists(TASK_CONFIG_FILE):
        try:
            with open(TASK_CONFIG_FILE) as f:
                cfg = json.load(f)
            # fill missing keys with defaults
            for k, v in _DEFAULT_CONFIG.items():
                cfg.setdefault(k, v)
            return cfg
        except Exception:
            pass
    return dict(_DEFAULT_CONFIG)


def _save_config(cfg: dict):
    os.makedirs(os.path.dirname(TASK_CONFIG_FILE), exist_ok=True)
    with open(TASK_CONFIG_FILE, 'w') as f:
        json.dump(cfg, f, indent=2)


# ─────────────────────────────────────────────────────────────────
#  PIPELINE HELPERS
# ─────────────────────────────────────────────────────────────────

def _set_status(msg: str):
    global _task_status
    with _lock:
        _task_status = msg
    print(f'[Task] {msg}')


def _arm_run_sequence(slots: list, delay: float = 0.3) -> bool:
    """POST /run_sequence → รอให้ seq_running=True → รอจนเสร็จ"""
    try:
        requests.post(
            f'{ARM_API}/run_sequence',
            json={'slots': slots, 'delay_sec': delay},
            timeout=5,
        )
    except Exception as e:
        _set_status(f'error: arm_jog_node ไม่ตอบสนอง ({e})')
        return False

    # รอให้ arm_jog_node เริ่ม thread (seq_running = True)
    started = False
    for _ in range(20):
        with _lock:
            if _stop_flag:
                return False
        try:
            if requests.get(f'{ARM_API}/status', timeout=3).json().get('seq_running', False):
                started = True
                break
        except Exception:
            pass
        time.sleep(0.5)

    if not started:
        _set_status('error: arm sequence ไม่เริ่ม — ตรวจสอบ arm_jog_node')
        return False

    # รอจนเสร็จ (seq_running = False)
    while True:
        with _lock:
            if _stop_flag:
                return False
        try:
            if not requests.get(f'{ARM_API}/status', timeout=3).json().get('seq_running', True):
                return True
        except Exception:
            pass
        time.sleep(0.5)


def _nav_goto(waypoint_name: str) -> bool:
    """POST /goto_waypoint → รอให้ navigating → รอจนถึงจุดหมาย"""
    try:
        requests.post(
            f'{NAV_API}/goto_waypoint',
            json={'name': waypoint_name},
            timeout=5,
        )
    except Exception as e:
        _set_status(f'error: nav_waypoint_server ไม่ตอบสนอง ({e})')
        return False

    # รอให้ nav_status กลายเป็น 'navigating' ก่อน (ไม่ใช่ค่าเก่า)
    started = False
    for _ in range(20):
        with _lock:
            if _stop_flag:
                try:
                    requests.post(f'{NAV_API}/cancel_nav', timeout=3)
                except Exception:
                    pass
                return False
        try:
            if requests.get(f'{NAV_API}/status', timeout=3).json().get('nav_status') == 'navigating':
                started = True
                break
        except Exception:
            pass
        time.sleep(0.5)

    if not started:
        _set_status(f'error: Nav2 ไม่ตอบสนองสำหรับ {waypoint_name}')
        return False

    # รอจนถึงจุดหมาย
    while True:
        with _lock:
            if _stop_flag:
                try:
                    requests.post(f'{NAV_API}/cancel_nav', timeout=3)
                except Exception:
                    pass
                return False
        try:
            nav = requests.get(f'{NAV_API}/status', timeout=3).json().get('nav_status', '')
            if nav == 'succeeded':
                return True
            if nav == 'failed':
                _set_status(f'error: navigation ไปยัง {waypoint_name} ล้มเหลว')
                return False
        except Exception:
            pass
        time.sleep(1.0)


def _pipeline_runner(cfg: dict):
    global _task_running, _stop_flag

    try:
        # ── 1. Navigate to pick location ─────────────────────────
        _set_status(f'กำลังเดินทางไปยัง {cfg["pick_waypoint"]}...')
        if not _nav_goto(cfg['pick_waypoint']):
            return

        with _lock:
            if _stop_flag:
                _set_status('stopped')
                return

        # ── 2. Execute pick sequence ──────────────────────────────
        slots_str = ', '.join(cfg['pick_sequence'])
        _set_status(f'กำลัง pick → slots [{slots_str}]')
        if not _arm_run_sequence(cfg['pick_sequence']):
            return

        _set_status('pick sequence เสร็จ — รอ 3 วิก่อนเดินทาง...')
        time.sleep(3.0)

        with _lock:
            if _stop_flag:
                _set_status('stopped')
                return

        # ── 3. Navigate through intermediate waypoints ────────────
        for wp in cfg.get('nav_path', []):
            _set_status(f'กำลังเดินทางผ่าน waypoint {wp}...')
            if not _nav_goto(wp):
                return
            with _lock:
                if _stop_flag:
                    _set_status('stopped')
                    return

        # ── 4. Navigate to drop location ──────────────────────────
        _set_status(f'กำลังเดินทางไปยัง {cfg["drop_waypoint"]}...')
        if not _nav_goto(cfg['drop_waypoint']):
            return

        with _lock:
            if _stop_flag:
                _set_status('stopped')
                return

        # ── 5. Execute drop sequence ──────────────────────────────
        slots_str = ', '.join(cfg['drop_sequence'])
        _set_status(f'กำลัง drop → slots [{slots_str}]')
        if not _arm_run_sequence(cfg['drop_sequence']):
            return

        time.sleep(3.0)
        _set_status('completed — ภารกิจเสร็จสิ้น ✅')

    except Exception as e:
        _set_status(f'error: {e}')
    finally:
        with _lock:
            _task_running = False
            _stop_flag    = False


# ─────────────────────────────────────────────────────────────────
#  FLASK APP
# ─────────────────────────────────────────────────────────────────
app = Flask(__name__)
CORS(app)


@app.route('/task_config', methods=['GET'])
def task_config():
    return jsonify(_load_config())


@app.route('/task_status', methods=['GET'])
def task_status():
    with _lock:
        return jsonify({
            'running': _task_running,
            'status':  _task_status,
        })


@app.route('/save_pick_sequence', methods=['POST'])
def save_pick_sequence():
    slots = request.get_json(force=True).get('slots', [])
    cfg = _load_config()
    cfg['pick_sequence'] = [str(s) for s in slots]
    _save_config(cfg)
    return jsonify({'status': 'ok', 'pick_sequence': cfg['pick_sequence']})


@app.route('/save_drop_sequence', methods=['POST'])
def save_drop_sequence():
    slots = request.get_json(force=True).get('slots', [])
    cfg = _load_config()
    cfg['drop_sequence'] = [str(s) for s in slots]
    _save_config(cfg)
    return jsonify({'status': 'ok', 'drop_sequence': cfg['drop_sequence']})


@app.route('/save_pick_location', methods=['POST'])
def save_pick_location():
    """บันทึก pose หุ่น ณ ปัจจุบัน เป็น pick waypoint ผ่าน nav_waypoint_server"""
    try:
        r = requests.post(
            f'{NAV_API}/save_waypoint',
            json={'name': PICK_WP_NAME},
            timeout=5,
        )
        result = r.json().get('status', 'error')
    except Exception as e:
        return jsonify({'status': f'nav_waypoint_server error: {e}'}), 500

    if result == 'ok':
        cfg = _load_config()
        cfg['pick_waypoint'] = PICK_WP_NAME
        _save_config(cfg)

    return jsonify({'status': result, 'waypoint': PICK_WP_NAME})


@app.route('/save_drop_location', methods=['POST'])
def save_drop_location():
    """บันทึก pose หุ่น ณ ปัจจุบัน เป็น drop waypoint"""
    try:
        r = requests.post(
            f'{NAV_API}/save_waypoint',
            json={'name': DROP_WP_NAME},
            timeout=5,
        )
        result = r.json().get('status', 'error')
    except Exception as e:
        return jsonify({'status': f'nav_waypoint_server error: {e}'}), 500

    if result == 'ok':
        cfg = _load_config()
        cfg['drop_waypoint'] = DROP_WP_NAME
        _save_config(cfg)

    return jsonify({'status': result, 'waypoint': DROP_WP_NAME})


@app.route('/save_nav_waypoint', methods=['POST'])
def save_nav_waypoint():
    """บันทึก waypoint ผ่านทาง (A, B, C ...)"""
    data = request.get_json(force=True) or {}
    name = data.get('name', '').strip().upper()
    if not name:
        return jsonify({'status': 'name ไม่ควรว่าง'}), 400

    try:
        r = requests.post(
            f'{NAV_API}/save_waypoint',
            json={'name': name},
            timeout=5,
        )
        result = r.json().get('status', 'error')
    except Exception as e:
        return jsonify({'status': f'nav_waypoint_server error: {e}'}), 500

    if result == 'ok':
        cfg = _load_config()
        path = cfg.get('nav_path', [])
        if name not in path:
            path.append(name)
            cfg['nav_path'] = path
            _save_config(cfg)

    return jsonify({'status': result, 'waypoint': name})


@app.route('/remove_nav_waypoint', methods=['POST'])
def remove_nav_waypoint():
    """ลบ waypoint ผ่านทางออกจาก nav_path"""
    name = (request.get_json(force=True) or {}).get('name', '').strip().upper()
    if not name:
        return jsonify({'status': 'name ไม่ควรว่าง'}), 400
    cfg = _load_config()
    cfg['nav_path'] = [w for w in cfg.get('nav_path', []) if w != name]
    _save_config(cfg)
    return jsonify({'status': 'ok', 'nav_path': cfg['nav_path']})


@app.route('/run_task', methods=['POST'])
def run_task():
    global _task_running, _stop_flag

    with _lock:
        if _task_running:
            return jsonify({'status': 'task กำลังทำงานอยู่แล้ว'}), 409
        cfg = _load_config()

    if not cfg['pick_sequence']:
        return jsonify({'status': 'ยังไม่ได้บันทึก pick sequence'}), 400
    if not cfg['drop_sequence']:
        return jsonify({'status': 'ยังไม่ได้บันทึก drop sequence'}), 400

    with _lock:
        _task_running = True
        _stop_flag    = False

    threading.Thread(target=_pipeline_runner, args=(cfg,), daemon=True).start()
    return jsonify({'status': 'task started'})


@app.route('/stop_task', methods=['POST'])
def stop_task():
    global _stop_flag
    with _lock:
        _stop_flag = True
    # หยุด arm sequence ด้วย
    try:
        requests.post(f'{ARM_API}/stop_sequence', timeout=3)
    except Exception:
        pass
    return jsonify({'status': 'stop requested'})


# ─────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────

def main(args=None):
    print(f'[TaskConfig] HTTP server on port {HTTP_PORT}')
    print(f'[TaskConfig] Config file: {TASK_CONFIG_FILE}')
    app.run(host='0.0.0.0', port=HTTP_PORT, debug=False, threaded=True)


if __name__ == '__main__':
    main()
