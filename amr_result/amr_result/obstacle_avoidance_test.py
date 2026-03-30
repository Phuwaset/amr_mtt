#!/usr/bin/env python3
"""
obstacle_avoidance_test.py — ทดสอบการหลบหลีกสิ่งกีดขวางในแผนที่เสมือน

สำหรับแต่ละ Waypoint เก็บผล:
  - ผลนำทาง     : สำเร็จ / ล้มเหลว / timeout
  - เวลา (s)    : ตั้งแต่ออกจนถึงเป้าหมาย
  - ระยะจริง (m): เส้นทางที่หุ่นยนต์วิ่งจริง (สะสมจาก odometry)
  - ระยะ Plan (m): เส้นทางที่ Nav2 Global Planner วางแผน
  - Min Clearance (m): ระยะห่างสิ่งกีดขวางที่ใกล้ที่สุด (LiDAR)

วิธีใช้:
  Terminal 1: ros2_nvidia launch amr_mtt_bot ign.launch.py world_file:=test_room.sdf
  Terminal 2: ros2_nvidia launch amr_mtt_bot nav2.launch.py
  Terminal 3: ros2 run amr_result obstacle_avoidance_test
             (ตั้ง 2D Pose Estimate ใน RViz ก่อนรัน)
"""

import math
import time
import os
import csv
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
# กำหนด Waypoints ที่ต้องการทดสอบ (x, y, yaw_deg, ชื่อ)
# ปรับค่าให้ตรงกับแผนที่ test_room.sdf ของคุณ
WAYPOINTS = [
    # (x, y, yaw_deg, label)
    # small_warehouse.sdf — origin ≈ ศูนย์กลางคลังสินค้า
    # หลีกเลี่ยงตำแหน่งใกล้ชั้นวาง ShelfD/E (x≈4.73), ShelfF (x≈-5.8)
    (1.0,  4.5,  0.0, 'WP1'),   # ขึ้นด้านบน (โซน Clutter)
    (1.0,  5.5, 180.0, 'WP2'),   # ซ้ายกลาง (ห่าง ShelfF พอควร)
    (1.0, -3.0, -90.0, 'WP3'),   # ซ้ายล่าง
    (1.5, -4.5,   0.0, 'WP4'),   # ขวาล่าง (ห่างจากชั้นวาง)
    (0.0,  0.0,   0.0, 'WP5'),   # กลับ Origin
]


GOAL_TIMEOUT = 180.0   # วินาที รอสูงสุดต่อ waypoint
OUT_DIR      = os.path.expanduser('~/amr_mtt_results')

plt.rcParams['font.family'] = 'Waree'


# ── Helpers ───────────────────────────────────────────────────────────────────

def _yaw_to_quat(yaw_deg: float):
    half = math.radians(yaw_deg) / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _path_length(poses) -> float:
    """คำนวณความยาวรวมของ Path จาก list of PoseStamped"""
    total = 0.0
    for i in range(1, len(poses)):
        a = poses[i - 1].pose.position
        b = poses[i].pose.position
        total += math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)
    return total


# ── Node ──────────────────────────────────────────────────────────────────────

class ObstacleAvoidanceTest(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_test')

        self._lock = threading.Lock()

        # ── Nav2 Action Client ────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Odometry (ตำแหน่งจริง) ────────────────────────────────
        self._odom_x   = None
        self._odom_y   = None
        self._prev_x   = None
        self._prev_y   = None
        self._actual_dist = 0.0      # สะสมระหว่าง waypoint

        # ── LiDAR (clearance) ─────────────────────────────────────
        self._min_clearance      = float('inf')
        self._clearance_log      = []   # [(time, min_range), ...]
        self._test_start_time    = None

        # ── Nav2 Plan ─────────────────────────────────────────────
        self._plan_length = 0.0

        # ── Trajectory (ทั้งหมด) ──────────────────────────────────
        self._traj_x = []
        self._traj_y = []

        # ── Subscriptions ─────────────────────────────────────────
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(
            LaserScan, '/amr_mtt/scan', self._scan_cb, scan_qos)
        self.create_subscription(
            Path, '/plan', self._plan_cb, 10)

        self.get_logger().info('ObstacleAvoidanceTest พร้อม')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        with self._lock:
            if self._prev_x is not None:
                self._actual_dist += math.sqrt(
                    (x - self._prev_x) ** 2 + (y - self._prev_y) ** 2)
            self._prev_x  = x
            self._prev_y  = y
            self._odom_x  = x
            self._odom_y  = y
            self._traj_x.append(x)
            self._traj_y.append(y)

    def _scan_cb(self, msg: LaserScan):
        ranges = [r for r in msg.ranges
                  if not math.isinf(r) and not math.isnan(r) and r > 0.01]
        if not ranges:
            return
        min_r = min(ranges)
        with self._lock:
            if min_r < self._min_clearance:
                self._min_clearance = min_r
            if self._test_start_time is not None:
                t = time.time() - self._test_start_time
                self._clearance_log.append((t, min_r))

    def _plan_cb(self, msg: Path):
        length = _path_length(msg.poses)
        with self._lock:
            self._plan_length = length

    # ── ส่ง Nav2 Goal และรอผล (blocking) ──────────────────────────────────────

    def navigate_to(self, x: float, y: float, yaw_deg: float) -> str:
        """
        ส่ง goal ไปยัง Nav2 และรอจนเสร็จ
        คืนค่า: 'success' | 'failed' | 'timeout'
        """
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            return 'failed'

        qx, qy, qz, qw = _yaw_to_quat(yaw_deg)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id    = 'map'
        goal_msg.pose.header.stamp       = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x    = float(x)
        goal_msg.pose.pose.position.y    = float(y)
        goal_msg.pose.pose.orientation.x = float(qx)
        goal_msg.pose.pose.orientation.y = float(qy)
        goal_msg.pose.pose.orientation.z = float(qz)
        goal_msg.pose.pose.orientation.w = float(qw)

        done_event  = threading.Event()
        result_box  = [None]

        def _result_cb(future):
            try:
                res = future.result()
                result_box[0] = res.status
            except Exception:
                result_box[0] = GoalStatus.STATUS_ABORTED
            finally:
                done_event.set()

        def _goal_response_cb(future):
            handle = future.result()
            if not handle.accepted:
                result_box[0] = GoalStatus.STATUS_ABORTED
                done_event.set()
                return
            handle.get_result_async().add_done_callback(_result_cb)

        fut = self._nav_client.send_goal_async(goal_msg)
        fut.add_done_callback(_goal_response_cb)
        reached = done_event.wait(timeout=GOAL_TIMEOUT)

        if not reached:
            return 'timeout'
        return 'success' if result_box[0] == GoalStatus.STATUS_SUCCEEDED else 'failed'

    # ── Reset ต่อ waypoint ─────────────────────────────────────────────────────

    def reset_wp_metrics(self):
        with self._lock:
            self._actual_dist  = 0.0
            self._prev_x       = self._odom_x
            self._prev_y       = self._odom_y
            self._min_clearance = float('inf')
            self._plan_length  = 0.0

    def get_wp_metrics(self) -> dict:
        with self._lock:
            return {
                'actual_dist'   : round(self._actual_dist, 3),
                'plan_length'   : round(self._plan_length, 3),
                'min_clearance' : round(self._min_clearance, 3)
                                  if self._min_clearance != float('inf') else None,
            }

    # ── Run ───────────────────────────────────────────────────────────────────

    def run(self):
        os.makedirs(OUT_DIR, exist_ok=True)

        print()
        print('═══════════════════════════════════════════════════════')
        print('   AMR MTT — Obstacle Avoidance Test')
        print(f'   จำนวน Waypoints: {len(WAYPOINTS)}')
        print('═══════════════════════════════════════════════════════')
        print('\n  รอ odometry และ LiDAR 3 วินาที...')
        time.sleep(3.0)

        if self._odom_x is None:
            print('\n  ❌ ไม่ได้รับ odometry — ตรวจสอบว่า nav2.launch.py ทำงานอยู่')
            return

        self._test_start_time = time.time()
        results = []

        for idx, (wx, wy, wyaw, wname) in enumerate(WAYPOINTS):
            print(f'\n  ── Waypoint {idx+1}/{len(WAYPOINTS)}: {wname} '
                  f'(x={wx:.2f}, y={wy:.2f}, yaw={wyaw:.1f}°) ──')

            self.reset_wp_metrics()
            t_start = time.time()

            status = self.navigate_to(wx, wy, wyaw)
            elapsed = round(time.time() - t_start, 2)

            time.sleep(0.5)   # รอให้หุ่นหยุดนิ่ง
            m = self.get_wp_metrics()

            status_th = {'success': 'สำเร็จ', 'failed': 'ล้มเหลว',
                         'timeout': 'timeout'}[status]
            clearance_str = (f'{m["min_clearance"]:.3f}'
                             if m['min_clearance'] is not None else 'N/A')

            print(f'     ผล          : {status_th}')
            print(f'     เวลา         : {elapsed} s')
            print(f'     ระยะจริง     : {m["actual_dist"]:.3f} m')
            print(f'     ระยะ Plan    : {m["plan_length"]:.3f} m')
            print(f'     Min Clearance: {clearance_str} m')

            results.append({
                'waypoint'       : wname,
                'result'         : status_th,
                'time_s'         : elapsed,
                'actual_dist_m'  : m['actual_dist'],
                'plan_dist_m'    : m['plan_length'],
                'min_clearance_m': m['min_clearance'] if m['min_clearance'] else 0.0,
            })

        # ── สรุป ──────────────────────────────────────────────────
        self._print_table(results)
        self._save_csv(results)
        self._plot(results)

    # ── ตาราง ─────────────────────────────────────────────────────────────────

    def _print_table(self, results):
        sep = '╠══════════╬════════════╬══════════╬══════════════╬══════════════╬══════════════════╣'
        print()
        print('╔══════════╦════════════╦══════════╦══════════════╦══════════════╦══════════════════╗')
        print('║ Waypoint ║  ผลนำทาง   ║ เวลา (s) ║ ระยะจริง (m) ║ ระยะ Plan(m) ║ Min Clearance (m)║')
        print(sep)
        for i, r in enumerate(results):
            print(f'║ {r["waypoint"]:<8} ║ {r["result"]:<10} ║ {r["time_s"]:^8.1f} ║'
                  f' {r["actual_dist_m"]:^12.3f} ║ {r["plan_dist_m"]:^12.3f} ║'
                  f' {r["min_clearance_m"]:^16.3f} ║')
            if i < len(results) - 1:
                print(sep)
        print('╚══════════╩════════════╩══════════╩══════════════╩══════════════╩══════════════════╝')

    # ── CSV ───────────────────────────────────────────────────────────────────

    def _save_csv(self, results):
        ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(OUT_DIR, f'obstacle_avoidance_{ts}.csv')
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=results[0].keys())
            writer.writeheader()
            writer.writerows(results)
        print(f'\n  บันทึก CSV → {path}')

        # บันทึก trajectory สำหรับ comparison plot
        traj_path = os.path.join(OUT_DIR, f'obstacle_avoidance_{ts}_traj.csv')
        with self._lock:
            tx = list(self._traj_x)
            ty = list(self._traj_y)
        with open(traj_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            writer.writerows(zip(tx, ty))
        print(f'  บันทึก Trajectory → {traj_path}')

    # ── Plot ──────────────────────────────────────────────────────────────────

    def _plot(self, results):
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        fig.suptitle('AMR MTT — Obstacle Avoidance Test\nผลการทดสอบการหลบหลีกสิ่งกีดขวางในแผนที่เสมือน',
                     fontsize=13, y=1.01)

        names   = [r['waypoint']        for r in results]
        times   = [r['time_s']          for r in results]
        actual  = [r['actual_dist_m']   for r in results]
        planned = [r['plan_dist_m']     for r in results]
        clear   = [r['min_clearance_m'] for r in results]
        colors  = ['#27ae60' if r['result'] == 'สำเร็จ' else '#e74c3c'
                   for r in results]
        x_idx   = np.arange(len(names))

        # ─── Plot 1: Trajectory ──────────────────────────────────
        ax = axes[0]
        with self._lock:
            tx = list(self._traj_x)
            ty = list(self._traj_y)

        if tx:
            ax.plot(tx, ty, color='#2980b9', linewidth=1.5,
                    label='เส้นทางจริง', zorder=2)
            ax.scatter([tx[0]], [ty[0]], color='black', s=100,
                       zorder=5, marker='o', label='จุดเริ่มต้น')

        # วาด waypoints
        wp_colors = {'สำเร็จ': '#27ae60', 'ล้มเหลว': '#e74c3c', 'timeout': '#f39c12'}
        for r in results:
            wx_t = next((w[0] for w in WAYPOINTS if w[3] == r['waypoint']), None)
            wy_t = next((w[1] for w in WAYPOINTS if w[3] == r['waypoint']), None)
            if wx_t is not None:
                c = wp_colors.get(r['result'], '#888888')
                ax.scatter([wx_t], [wy_t], color=c, s=150, zorder=6, marker='*')
                ax.annotate(r['waypoint'], (wx_t, wy_t),
                            textcoords='offset points', xytext=(6, 6),
                            fontsize=9, fontweight='bold', color=c)

        ax.set_aspect('equal')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('เส้นทางที่วิ่งจริง')
        ax.legend(fontsize=8)
        ax.grid(True, linestyle=':', alpha=0.5)

        # ─── Plot 2: ระยะจริง vs Plan ────────────────────────────
        ax = axes[1]
        w = 0.35
        b1 = ax.bar(x_idx - w/2, actual,  w, label='ระยะจริง (m)',
                    color='#2980b9', alpha=0.85)
        b2 = ax.bar(x_idx + w/2, planned, w, label='ระยะ Plan (m)',
                    color='#95a5a6', alpha=0.85)
        for bars in (b1, b2):
            for bar in bars:
                v = bar.get_height()
                if v > 0:
                    ax.text(bar.get_x() + bar.get_width() / 2,
                            v + 0.02, f'{v:.2f}',
                            ha='center', va='bottom', fontsize=8)

        ax.set_xlabel('Waypoint')
        ax.set_ylabel('ระยะทาง (m)')
        ax.set_title('ระยะทางจริง vs Nav2 Plan')
        ax.set_xticks(x_idx)
        ax.set_xticklabels(names)
        ax.legend(fontsize=8)
        ax.grid(True, linestyle=':', alpha=0.5, axis='y')

        # ─── Plot 3: Min Clearance ────────────────────────────────
        ax = axes[2]
        bars = ax.bar(x_idx, clear, color=colors, alpha=0.85, width=0.5)
        for bar, v in zip(bars, clear):
            ax.text(bar.get_x() + bar.get_width() / 2,
                    v + 0.005, f'{v:.3f} m',
                    ha='center', va='bottom', fontsize=9)

        # เส้นอ้างอิงระยะปลอดภัย (inflation radius ของ costmap ≈ 0.3 m)
        ax.axhline(y=0.3, color='#e74c3c', linestyle='--',
                   linewidth=1.2, label='Safety threshold (0.3 m)')

        ax.set_xlabel('Waypoint')
        ax.set_ylabel('Min Clearance (m)')
        ax.set_title('ระยะห่างสิ่งกีดขวางต่ำสุด')
        ax.set_xticks(x_idx)
        ax.set_xticklabels(names)
        ax.legend(fontsize=8)
        ax.grid(True, linestyle=':', alpha=0.5, axis='y')

        plt.tight_layout()
        ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(OUT_DIR, f'obstacle_avoidance_{ts}.png')
        fig.savefig(path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f'  บันทึก Plot → {path}')


# ── Entry Point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceTest()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
