#!/usr/bin/env python3
"""
map_manager_node.py — SLAM & Map Manager

HTTP REST API (Flask, port 5007):
  GET  /status        → {slam_running, nav_map, maps}
  POST /start_slam    → start slam_toolbox (online_async)
  POST /stop_slam     → stop slam subprocess
  POST /save_map      → {"name": "my_map"} → map_saver_cli → ~/.ros/slam_maps/
  GET  /maps          → [{name, path, source}]  (package + user maps)
  POST /load_map      → {"name": "my_map"} → /map_server/load_map service
"""

import os
import glob
import json
import threading
import subprocess

from flask import Flask, request, jsonify

import rclpy
from rclpy.node import Node

from nav2_msgs.srv import LoadMap
from ament_index_python.packages import get_package_share_directory

# ─────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────
HTTP_PORT    = 5007
USER_MAP_DIR = os.path.expanduser('~/.ros/slam_maps')

try:
    PKG_MAP_DIR = os.path.join(get_package_share_directory('amr_mtt_bot'), 'map')
except Exception:
    PKG_MAP_DIR = ''


# ─────────────────────────────────────────────────────────────────
#  ROS 2 NODE
# ─────────────────────────────────────────────────────────────────

class MapManagerNode(Node):

    def __init__(self):
        super().__init__('map_manager_node')
        self._lock      = threading.Lock()
        self._slam_proc = None      # subprocess.Popen or None
        self._nav_map   = None      # name of map loaded into Nav2

        os.makedirs(USER_MAP_DIR, exist_ok=True)

        # Service client for Nav2 map_server
        self._load_map_cli = self.create_client(LoadMap, '/map_server/load_map')

        self.get_logger().info(f'[MapMgr] HTTP on port {HTTP_PORT}')
        self.get_logger().info(f'[MapMgr] pkg maps : {PKG_MAP_DIR}')
        self.get_logger().info(f'[MapMgr] user maps: {USER_MAP_DIR}')

    # ── Helpers ───────────────────────────────────────────────────

    def _slam_running(self) -> bool:
        with self._lock:
            return self._slam_proc is not None and self._slam_proc.poll() is None

    def _list_maps(self) -> list:
        maps = []
        # Package maps (read-only, shipped with repo)
        if PKG_MAP_DIR and os.path.isdir(PKG_MAP_DIR):
            for p in sorted(glob.glob(os.path.join(PKG_MAP_DIR, '*.yaml'))):
                maps.append({
                    'name':   os.path.splitext(os.path.basename(p))[0],
                    'path':   p,
                    'source': 'package',
                })
        # User-saved maps
        for p in sorted(glob.glob(os.path.join(USER_MAP_DIR, '*.yaml'))):
            maps.append({
                'name':   os.path.splitext(os.path.basename(p))[0],
                'path':   p,
                'source': 'user',
            })
        return maps

    # ── API handlers ──────────────────────────────────────────────

    def get_status(self) -> dict:
        with self._lock:
            nav_map = self._nav_map
        return {
            'slam_running': self._slam_running(),
            'nav_map':      nav_map,
            'maps':         self._list_maps(),
            'user_map_dir': USER_MAP_DIR,
        }

    def start_slam(self) -> str:
        if self._slam_running():
            return 'SLAM already running'

        slam_params = os.path.join(PKG_MAP_DIR, '..', 'config',
                                   'mapper_params_online_async.yaml')
        slam_params = os.path.normpath(slam_params)

        if not os.path.exists(slam_params):
            return f'params file not found: {slam_params}'

        cmd = [
            'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
            'use_sim_time:=true',
            f'slam_params_file:={slam_params}',
        ]
        env = os.environ.copy()
        with self._lock:
            self._slam_proc = subprocess.Popen(
                cmd, env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        self.get_logger().info(f'[MapMgr] SLAM started (PID {self._slam_proc.pid})')
        return 'ok'

    def stop_slam(self) -> str:
        with self._lock:
            proc = self._slam_proc
        if proc is None or proc.poll() is not None:
            with self._lock:
                self._slam_proc = None
            return 'SLAM not running'

        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

        with self._lock:
            self._slam_proc = None
        self.get_logger().info('[MapMgr] SLAM stopped')
        return 'ok'

    def save_map(self, name: str) -> str:
        safe = ''.join(c for c in name if c.isalnum() or c in '-_').strip('-_')
        if not safe:
            return 'invalid map name — use letters, numbers, - or _'

        out_path = os.path.join(USER_MAP_DIR, safe)
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', out_path]
        self.get_logger().info(f'[MapMgr] Saving map to {out_path}')
        try:
            result = subprocess.run(cmd, timeout=30, capture_output=True, text=True)
        except subprocess.TimeoutExpired:
            return 'timeout — is /map topic publishing?'

        if result.returncode == 0:
            self.get_logger().info(f'[MapMgr] Map saved: {safe}')
            return 'ok'
        # map_saver often returns 0 even on success; check file
        if os.path.exists(out_path + '.yaml'):
            return 'ok'
        return f'map_saver error: {result.stderr[-200:]}'

    def load_map(self, name: str) -> str:
        yaml_path = None
        for m in self._list_maps():
            if m['name'] == name:
                yaml_path = m['path']
                break
        if yaml_path is None:
            return f'map "{name}" not found'

        if not self._load_map_cli.wait_for_service(timeout_sec=3.0):
            return 'map_server service unavailable — is Nav2 running?'

        req = LoadMap.Request()
        req.map_url = yaml_path

        event         = threading.Event()
        result_holder = [None]

        def _cb(future):
            result_holder[0] = future.result()
            event.set()

        future = self._load_map_cli.call_async(req)
        future.add_done_callback(_cb)
        event.wait(timeout=10.0)

        if result_holder[0] is None:
            return 'timeout waiting for map_server'

        r = result_holder[0]
        if r.result == LoadMap.Response.RESULT_SUCCESS:
            with self._lock:
                self._nav_map = name
            self.get_logger().info(f'[MapMgr] Loaded map: {name}')
            return 'ok'
        return f'map_server rejected: result={r.result}'


# ─────────────────────────────────────────────────────────────────
#  FLASK
# ─────────────────────────────────────────────────────────────────

app  = Flask(__name__)
node: MapManagerNode = None   # set in main()


@app.route('/status', methods=['GET'])
def status():
    return jsonify(node.get_status())


@app.route('/maps', methods=['GET'])
def maps():
    return jsonify(node._list_maps())


@app.route('/start_slam', methods=['POST'])
def start_slam():
    return jsonify({'status': node.start_slam()})


@app.route('/stop_slam', methods=['POST'])
def stop_slam():
    return jsonify({'status': node.stop_slam()})


@app.route('/save_map', methods=['POST'])
def save_map():
    data = request.get_json(force=True) or {}
    name = data.get('name', '').strip()
    if not name:
        return jsonify({'status': 'name is required'}), 400
    return jsonify({'status': node.save_map(name)})


@app.route('/load_map', methods=['POST'])
def load_map():
    data = request.get_json(force=True) or {}
    name = data.get('name', '').strip()
    if not name:
        return jsonify({'status': 'name is required'}), 400
    return jsonify({'status': node.load_map(name)})


# ─────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────

def main(args=None):
    global node
    rclpy.init(args=args)
    node = MapManagerNode()

    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=HTTP_PORT, debug=False),
        daemon=True,
    )
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_slam()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
