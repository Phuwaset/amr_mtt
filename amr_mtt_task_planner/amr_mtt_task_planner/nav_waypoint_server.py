#!/usr/bin/env python3
"""
nav_waypoint_server.py — Navigation Waypoint Manager

HTTP REST API (Flask, port 5006) + ROS 2 node
  รับ AMCL pose → บันทึก nav waypoints → ส่ง NavigateToPose ให้ Nav2
  ใช้งานผ่าน Node-RED dashboard

Endpoints:
  GET  /status          → {pose, pose_source, nav_status, nav_target, waypoints}
  POST /save_waypoint   → {"name": "WP1"} → บันทึก pose ปัจจุบัน
  POST /goto_waypoint   → {"name": "WP1"} → navigate ไปยัง waypoint
  POST /cancel_nav      → {}             → ยกเลิก navigation ปัจจุบัน
  DELETE /waypoint      → {"name": "WP1"} → ลบ waypoint
"""

import math
import json
import threading
import os

from flask import Flask, request, jsonify

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose

# ─────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────
HTTP_PORT      = 5006
WAYPOINTS_FILE = os.path.expanduser('~/.ros/nav_waypoints.json')
MAP_FRAME      = 'map'

ODOM_TOPIC = '/amr_mtt/odom'
AMCL_TOPIC = '/amcl_pose'


def _yaw_from_quaternion(qx, qy, qz, qw) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw_rad):
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


# ─────────────────────────────────────────────────────────────────
#  ROS 2 NODE
# ─────────────────────────────────────────────────────────────────

class NavWaypointNode(Node):

    def __init__(self):
        super().__init__('nav_waypoint_server')

        self._lock        = threading.Lock()
        self._pose        = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._pose_source = 'none'
        self._nav_status  = 'idle'   # idle | navigating | succeeded | failed
        self._nav_target  = None
        self._goal_handle = None

        # Load persisted waypoints
        self._waypoints: dict = {}
        if os.path.exists(WAYPOINTS_FILE):
            try:
                with open(WAYPOINTS_FILE) as f:
                    self._waypoints = json.load(f)
                self.get_logger().info(
                    f'[Nav] Loaded {len(self._waypoints)} waypoint(s) from {WAYPOINTS_FILE}'
                )
            except Exception as e:
                self.get_logger().warn(f'[Nav] Could not load waypoints: {e}')

        # Subscribe: prefer AMCL, fallback to odom
        self.create_subscription(
            PoseWithCovarianceStamped, AMCL_TOPIC, self._amcl_cb, 10
        )
        self.create_subscription(
            Odometry, ODOM_TOPIC, self._odom_cb, 10
        )

        # Nav2 action client (non-blocking init)
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        if not self._nav.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                '[Nav] Nav2 not available yet — goto will work once Nav2 starts'
            )
        else:
            self.get_logger().info('[Nav] Nav2 ready')

        self.get_logger().info(f'[Nav] HTTP on port {HTTP_PORT}')

    # ── ROS callbacks ─────────────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        q    = pose.orientation
        with self._lock:
            self._pose = {
                'x':   round(pose.position.x, 4),
                'y':   round(pose.position.y, 4),
                'yaw': round(math.degrees(_yaw_from_quaternion(q.x, q.y, q.z, q.w)), 2),
            }
            self._pose_source = 'amcl'

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            if self._pose_source == 'amcl':
                return   # AMCL takes priority
        pose = msg.pose.pose
        q    = pose.orientation
        with self._lock:
            self._pose = {
                'x':   round(pose.position.x, 4),
                'y':   round(pose.position.y, 4),
                'yaw': round(math.degrees(_yaw_from_quaternion(q.x, q.y, q.z, q.w)), 2),
            }
            self._pose_source = 'odom'

    # ── Persistence ───────────────────────────────────────────────

    def _save_to_disk(self):
        os.makedirs(os.path.dirname(WAYPOINTS_FILE), exist_ok=True)
        with open(WAYPOINTS_FILE, 'w') as f:
            json.dump(self._waypoints, f, indent=2)

    # ── API handlers (called from Flask thread) ───────────────────

    def get_status(self) -> dict:
        with self._lock:
            return {
                'pose':        dict(self._pose),
                'pose_source': self._pose_source,
                'nav_status':  self._nav_status,
                'nav_target':  self._nav_target,
                'waypoints':   dict(self._waypoints),
            }

    def save_waypoint(self, name: str) -> str:
        with self._lock:
            pose   = dict(self._pose)
            source = self._pose_source
        if source == 'none':
            return 'no pose available — is simulation running?'
        self._waypoints[name] = pose
        self._save_to_disk()
        self.get_logger().info(f'[Nav] Saved "{name}": {pose}')
        return 'ok'

    def goto_waypoint(self, name: str) -> str:
        if name not in self._waypoints:
            return f'waypoint "{name}" not found'
        wp = self._waypoints[name]

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wp['x'])
        pose.pose.position.y = float(wp['y'])
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quaternion(math.radians(float(wp['yaw'])))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose

        with self._lock:
            self._nav_status = 'navigating'
            self._nav_target = name

        future = self._nav.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)
        self.get_logger().info(f'[Nav] Navigating to "{name}": {wp}')
        return 'ok'

    def cancel_nav(self) -> str:
        with self._lock:
            gh = self._goal_handle
        if gh is None:
            with self._lock:
                self._nav_status = 'idle'
                self._nav_target = None
            return 'no active navigation'
        gh.cancel_goal_async()
        with self._lock:
            self._nav_status = 'idle'
            self._nav_target = None
            self._goal_handle = None
        return 'cancelled'

    def delete_waypoint(self, name: str) -> str:
        if name not in self._waypoints:
            return f'waypoint "{name}" not found'
        del self._waypoints[name]
        self._save_to_disk()
        self.get_logger().info(f'[Nav] Deleted waypoint "{name}"')
        return 'ok'

    # ── Action callbacks ──────────────────────────────────────────

    def _goal_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('[Nav] Goal rejected by Nav2')
            with self._lock:
                self._nav_status = 'failed'
                self._nav_target = None
            return
        with self._lock:
            self._goal_handle = gh
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        # GoalStatus.STATUS_SUCCEEDED = 4
        status = 'succeeded' if result.status == 4 else 'failed'
        with self._lock:
            self._nav_status  = status
            self._nav_target  = None
            self._goal_handle = None
        self.get_logger().info(f'[Nav] Navigation {status}')

    def _feedback_cb(self, feedback):
        pass  # could log ETA from feedback.feedback.estimated_time_remaining


# ─────────────────────────────────────────────────────────────────
#  FLASK APP
# ─────────────────────────────────────────────────────────────────

app  = Flask(__name__)
node: NavWaypointNode = None   # set in main()


@app.route('/status', methods=['GET'])
def status():
    return jsonify(node.get_status())


@app.route('/save_waypoint', methods=['POST'])
def save_waypoint():
    data = request.get_json(force=True) or {}
    name = data.get('name', '').strip()
    if not name:
        return jsonify({'status': 'name is required'}), 400
    return jsonify({'status': node.save_waypoint(name)})


@app.route('/goto_waypoint', methods=['POST'])
def goto_waypoint():
    data = request.get_json(force=True) or {}
    name = data.get('name', '').strip()
    if not name:
        return jsonify({'status': 'name is required'}), 400
    return jsonify({'status': node.goto_waypoint(name)})


@app.route('/cancel_nav', methods=['POST'])
def cancel_nav():
    return jsonify({'status': node.cancel_nav()})


@app.route('/waypoint', methods=['DELETE'])
def delete_waypoint():
    data = request.get_json(force=True) or {}
    name = data.get('name', '').strip()
    if not name:
        return jsonify({'status': 'name is required'}), 400
    return jsonify({'status': node.delete_waypoint(name)})


# ─────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────

def main(args=None):
    global node
    rclpy.init(args=args)
    node = NavWaypointNode()

    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=HTTP_PORT, debug=False),
        daemon=True
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
