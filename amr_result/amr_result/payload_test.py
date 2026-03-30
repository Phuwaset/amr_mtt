#!/usr/bin/env python3
"""
payload_test.py — ทดสอบการเคลื่อนที่แนวตรง 5 เมตร ภายใต้น้ำหนักบรรทุกต่างกัน

3 สถานการณ์: ไม่มีน้ำหนัก / 50 kg / 100 kg / 150 kg
ความเร็วสั่งเท่ากันทุกรอบ: 1.0 m/s, accel: 1.5 m/s²
ผลต่างที่เห็นมาจากฟิสิกส์น้ำหนักจริงของกล่องใน Gazebo

วิธีใช้:
    Terminal 1: ros2_nvidia launch amr_mtt_bot ign.launch.py world_file:=payload_test.sdf
    Terminal 2: ros2_nvidia launch amr_mtt_bot nav2.launch.py  (ถ้าต้องการ odometry)
    Terminal 3: ros2 run amr_result payload_test

    ก่อนแต่ละรอบ: ลากกล่องน้ำหนักที่ต้องการวางบนรถใน Gazebo
"""

import math
import time
import os
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['font.family'] = 'Waree'

# ── Config ─────────────────────────────────────────────────────────────────────
TARGET_DIST  = 5.0      # เมตร — เดินตรง
CMD_VEL      = 1.0      # m/s  — ความเร็วสั่ง (เท่ากันทุกรอบ)
SETTLE_TIME  = 2.0      # วินาที — รอก่อนเริ่ม
OUT_DIR      = os.path.expanduser('~/amr_mtt_results')
CMD_TOPIC    = '/diff_drive_controller/cmd_vel_unstamped'

SCENARIOS = [
    {'label': 'ไม่มีน้ำหนัก', 'color': '#2980b9'},   # น้ำเงิน
    {'label': '50 kg',        'color': '#27ae60'},   # เขียว
    {'label': '100 kg',       'color': '#e67e22'},   # ส้ม
    {'label': '150 kg',       'color': '#c0392b'},   # แดง
]


# ── Node ───────────────────────────────────────────────────────────────────────

class PayloadTest(Node):

    def __init__(self):
        super().__init__('payload_test')

        self._pub = self.create_publisher(Twist, CMD_TOPIC, 10)

        self._x = self._y = 0.0
        self._px = self._py = None
        self._dist   = 0.0
        self._speed_log = []   # [(elapsed_s, speed_m/s), ...]
        self._traj_x = []
        self._traj_y = []
        self._t0     = None

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/amr_mtt/scan', self._scan_cb, scan_qos)

        self._min_clearance = float('inf')

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        x  = msg.pose.pose.position.x
        y  = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx**2 + vy**2)
        if self._px is not None:
            self._dist += math.sqrt((x-self._px)**2 + (y-self._py)**2)
        self._px, self._py = x, y
        self._x,  self._y  = x, y
        if self._t0 is not None:
            self._speed_log.append((time.time() - self._t0, speed))
        self._traj_x.append(x)
        self._traj_y.append(y)

    def _scan_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid:
            self._min_clearance = min(self._min_clearance, min(valid))

    # ── Driving ────────────────────────────────────────────────────────────────

    def _spin(self, duration):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def _stop(self):
        self._pub.publish(Twist())
        self._spin(0.5)

    def _drive_straight(self, dist):
        """เดินตรง dist เมตร ด้วย CMD_VEL — คืนค่า elapsed_s"""
        # รีเซ็ต tracking
        self._dist      = 0.0
        self._px = self._py = None
        self._speed_log = []
        self._traj_x    = []
        self._traj_y    = []
        self._min_clearance = float('inf')

        # รอให้ odometry stabilize
        self._spin(SETTLE_TIME)
        self._px, self._py = self._x, self._y

        twist = Twist()
        twist.linear.x = CMD_VEL
        self._t0    = time.time()
        next_pub    = self._t0

        while True:
            now = time.time()
            if now >= next_pub:
                self._pub.publish(twist)
                next_pub = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
            if self._dist >= dist:
                break

        self._stop()
        return time.time() - self._t0

    # ── Main ───────────────────────────────────────────────────────────────────

    def run(self):
        os.makedirs(OUT_DIR, exist_ok=True)
        print('\n' + '═'*60)
        print('   AMR MTT — Payload Test (เดินตรง 5 เมตร)')
        print(f'   ความเร็วสั่ง : {CMD_VEL} m/s (เท่ากันทุกรอบ)')
        print(f'   ระยะทาง      : {TARGET_DIST} m')
        print(f'   จำนวนรอบ     : {len(SCENARIOS)} สถานการณ์')
        print('═'*60)

        print('\n  รอ Odometry 3 วินาที...')
        self._spin(3.0)

        all_results = []

        for i, sc in enumerate(SCENARIOS):
            print(f'\n{"─"*55}')
            print(f'  สถานการณ์ {i+1}/{len(SCENARIOS)}: {sc["label"]}')
            print(f'{"─"*55}')
            if i == 0:
                input('  วางหุ่นยนต์ที่จุดเริ่มต้น (0,0) แล้วกด Enter...')
            else:
                input(f'  วางหุ่นยนต์กลับ (0,0) + ลากกล่อง {sc["label"]} ใส่รถ แล้วกด Enter...')

            elapsed = self._drive_straight(TARGET_DIST)

            speeds    = [v[1] for v in self._speed_log]
            avg_speed = sum(speeds) / len(speeds) if speeds else 0.0
            max_speed = max(speeds)                if speeds else 0.0
            clearance = self._min_clearance if self._min_clearance < float('inf') else 0.0

            row = {
                'scenario'        : sc['label'],
                'time_s'          : round(elapsed, 3),
                'actual_dist_m'   : round(self._dist, 3),
                'avg_speed_ms'    : round(avg_speed, 3),
                'max_speed_ms'    : round(max_speed, 3),
                'min_clearance_m' : round(clearance, 3),
                'traj_x'          : list(self._traj_x),
                'traj_y'          : list(self._traj_y),
                'speed_log'       : list(self._speed_log),
            }
            all_results.append(row)

            print(f'     เวลา          : {elapsed:.3f} s')
            print(f'     ระยะจริง      : {self._dist:.3f} m')
            print(f'     ความเร็วเฉลี่ย : {avg_speed:.3f} m/s')
            print(f'     ความเร็วสูงสุด : {max_speed:.3f} m/s')

        self._print_table(all_results)
        self._save_csv(all_results)
        self._plot(all_results)

    # ── Output ─────────────────────────────────────────────────────────────────

    def _print_table(self, results):
        sep = '╠════════════════╬══════════╬══════════╬══════════╬══════════╣'
        print()
        print('╔════════════════╦══════════╦══════════╦══════════╦══════════╗')
        print('║   บรรทุก       ║ เวลา (s) ║ ระยะ (m) ║v_avg(m/s)║v_max(m/s)║')
        print(sep)
        for r in results:
            print(f"║ {r['scenario']:<14} ║ {r['time_s']:^8.3f} ║"
                  f" {r['actual_dist_m']:^8.3f} ║ {r['avg_speed_ms']:^8.3f} ║"
                  f" {r['max_speed_ms']:^8.3f} ║")
            print(sep)
        print('╚════════════════╩══════════╩══════════╩══════════╩══════════╝')

    def _save_csv(self, results):
        ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(OUT_DIR, f'payload_test_{ts}.csv')
        fields = ['scenario', 'time_s', 'actual_dist_m', 'avg_speed_ms', 'max_speed_ms']
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for r in results:
                writer.writerow({k: r[k] for k in fields})
        print(f'\n  บันทึก CSV → {path}')

    def _plot(self, results):
        ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        fig.suptitle(
            f'AMR MTT — Payload Test (เดินตรง {TARGET_DIST} m, v_cmd = {CMD_VEL} m/s)\n'
            'เปรียบเทียบผลกระทบของน้ำหนักบรรทุกต่อการเคลื่อนที่',
            fontsize=13, y=1.01
        )
        colors = [sc['color'] for sc in SCENARIOS]
        labels = [r['scenario'] for r in results]
        x      = np.arange(len(results))
        w      = 0.35

        # ─── Plot 1: Speed Profile ────────────────────────────────────
        ax = axes[0]
        for r, sc in zip(results, SCENARIOS):
            if r['speed_log']:
                ts_arr = [v[0] for v in r['speed_log']]
                sp_arr = [v[1] for v in r['speed_log']]
                ax.plot(ts_arr, sp_arr, color=sc['color'],
                        linewidth=2.0, label=r['scenario'])
        ax.axhline(CMD_VEL, color='gray', linestyle='--',
                   linewidth=1.0, alpha=0.6, label=f'v_cmd={CMD_VEL} m/s')
        ax.set_xlabel('เวลา (s)')
        ax.set_ylabel('ความเร็ว (m/s)')
        ax.set_title('Speed Profile ตลอดการเดินทาง')
        ax.legend(fontsize=8)
        ax.grid(True, linestyle=':', alpha=0.5)

        # ─── Plot 2: เวลา + ความเร็วเฉลี่ย (Grouped Bar) ────────────
        ax = axes[1]
        b1 = ax.bar(x - w/2, [r['time_s'] for r in results],
                    w, color=colors, alpha=0.85, label='เวลา (s)')
        ax2 = ax.twinx()
        b2  = ax2.bar(x + w/2, [r['avg_speed_ms'] for r in results],
                      w, color=colors, alpha=0.40,
                      edgecolor=colors, linewidth=1.5, label='v_avg (m/s)')
        for bar, r in zip(b1, results):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
                    f"{r['time_s']:.2f}s", ha='center', va='bottom', fontsize=8)
        for bar, r in zip(b2, results):
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                     f"{r['avg_speed_ms']:.2f}", ha='center', va='bottom', fontsize=8)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=8)
        ax.set_ylabel('เวลา (s)', color='black')
        ax2.set_ylabel('ความเร็วเฉลี่ย (m/s)', color='gray')
        ax.set_title('เวลา vs ความเร็วเฉลี่ย')
        ax.grid(True, linestyle=':', alpha=0.5, axis='y')

        # ─── Plot 3: Trajectory ───────────────────────────────────────
        ax = axes[2]
        for r, sc in zip(results, SCENARIOS):
            if r['traj_x']:
                ax.plot(r['traj_x'], r['traj_y'], color=sc['color'],
                        linewidth=2.0, label=r['scenario'], alpha=0.85)
        ax.scatter([0], [0], color='green', s=120, zorder=5,
                   marker='o', label='Start')
        ax.scatter([TARGET_DIST], [0], color='red', s=150, zorder=5,
                   marker='*', label='Goal')
        ax.axhline(0, color='gold', linestyle='--', linewidth=1.0, alpha=0.6)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('เส้นทางที่วิ่งจริง')
        ax.legend(fontsize=8)
        ax.grid(True, linestyle=':', alpha=0.5)

        plt.tight_layout()
        path = os.path.join(OUT_DIR, f'payload_test_{ts}.png')
        fig.savefig(path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f'  บันทึก Plot → {path}')


# ── Entry Point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PayloadTest()
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
