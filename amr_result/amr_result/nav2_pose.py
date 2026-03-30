#!/usr/bin/env python3
"""
nav2_pose.py — Nav2 Waypoint Monitor & Auto Navigator

โหมดเลือกตั้งแต่เริ่ม:
  [1] บันทึก Waypoint — กด Enter ที่ตำแหน่ง แล้วตั้งชื่อ (A, B, C)
                        บันทึกลงไฟล์ waypoints.json + CSV + กราฟ
  [2] นำทางอัตโนมัติ  — โหลด waypoints.json แล้วส่ง Nav2 goal A→B→C
                        บันทึกผลลัพธ์ CSV + กราฟอัตโนมัติ

วิธีใช้:
    ros2 run amr_result nav2_pose
"""

import math
import time
import os
import csv
import json
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose

import matplotlib.pyplot as plt
import matplotlib.patheffects as pe

plt.rcParams['font.family'] = 'Waree'

OUT_DIR        = os.path.expanduser('~/amr_mtt_results')
WAYPOINTS_FILE = os.path.join(OUT_DIR, 'waypoints.json')
COLORS         = ['#1565c0', '#2e7d32', '#c62828', '#e65100', '#6a1b9a', '#00838f']


# ── Utility ───────────────────────────────────────────────────────────────

def _quat_to_yaw_deg(qx, qy, qz, qw) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny, cosy))


def _yaw_deg_to_quat(yaw_deg: float):
    """Return (qx, qy, qz, qw) from yaw in degrees."""
    half = math.radians(yaw_deg) / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _ang_diff(a: float, b: float) -> float:
    return abs((a - b + 180) % 360 - 180)


# ── Node ──────────────────────────────────────────────────────────────────

class Nav2PoseMonitor(Node):

    def __init__(self, mode: str):
        super().__init__('nav2_pose_monitor')

        self._mode = mode          # 'save' | 'navigate'
        self._lock = threading.Lock()
        self._x = self._y = self._yaw = None
        self._results = []

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.create_timer(0.5, self._display_cb)

        if mode == 'navigate':
            self._nav_client = ActionClient(
                self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(f'Nav2PoseMonitor โหมด={mode}')

    # ── Callbacks ─────────────────────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        q = p.orientation
        with self._lock:
            self._x   = p.position.x
            self._y   = p.position.y
            self._yaw = _quat_to_yaw_deg(q.x, q.y, q.z, q.w)

    def _display_cb(self):
        if getattr(self, '_display_paused', False):
            return
        with self._lock:
            x, y, yaw = self._x, self._y, self._yaw
        if x is None:
            return
        print(f'\r\033[2K  AMCL: x={x:+8.4f} m   y={y:+8.4f} m   yaw={yaw:+7.2f}°',
              end='', flush=True)

    # ── Pose getter ───────────────────────────────────────────────────

    def get_pose(self):
        """Return (x, y, yaw_deg) or (None, None, None)."""
        with self._lock:
            return self._x, self._y, self._yaw

    # ── Record snapshot ───────────────────────────────────────────────

    def record_snapshot(self, label: str,
                        target_x=None, target_y=None, target_yaw=None):
        ax, ay, ayaw = self.get_pose()
        if ax is None:
            print('  ยังไม่ได้รับ AMCL pose!')
            return False

        tx   = target_x   if target_x   is not None else float('nan')
        ty   = target_y   if target_y   is not None else float('nan')
        tyaw = target_yaw if target_yaw is not None else float('nan')

        if not math.isnan(tx):
            ex   = abs(ax - tx)
            ey   = abs(ay - ty)
            exy  = math.sqrt(ex**2 + ey**2)
            eyaw = _ang_diff(ayaw, tyaw) if not math.isnan(tyaw) else float('nan')
        else:
            ex = ey = exy = eyaw = float('nan')

        def _f4(v): return round(v, 4) if not math.isnan(v) else '-'
        def _f2(v): return round(v, 2) if not math.isnan(v) else '-'

        record = {
            'waypoint'  : label,
            'target_x'  : _f4(tx),   'target_y'  : _f4(ty),
            'target_yaw': _f2(tyaw),
            'actual_x'  : round(ax, 4), 'actual_y': round(ay, 4),
            'actual_yaw': round(ayaw, 2),
            'err_x'     : _f4(ex),   'err_y'  : _f4(ey),
            'err_xy'    : _f4(exy),  'err_yaw': _f2(eyaw),
            'timestamp' : datetime.now().strftime('%H:%M:%S'),
        }
        self._results.append(record)
        print(f'\n  บันทึก [{label}]  x={ax:.4f}  y={ay:.4f}  yaw={ayaw:.2f}°')
        if not math.isnan(exy):
            print(f'           Δx={ex:.4f}  Δy={ey:.4f}  Δxy={exy:.4f}  Δyaw={eyaw:.2f}°')
        return True

    # ── Waypoint file I/O ─────────────────────────────────────────────

    def load_waypoints(self) -> dict:
        """โหลด waypoints จากไฟล์ JSON (เรียงตาม order)."""
        if not os.path.exists(WAYPOINTS_FILE):
            return {}
        with open(WAYPOINTS_FILE, 'r') as f:
            raw = json.load(f)
        # เรียงตาม field 'order' ที่บันทึกไว้
        sorted_items = sorted(raw.items(), key=lambda kv: kv[1].get('order', 0))
        return dict(sorted_items)

    def save_waypoint_to_file(self, name: str, x: float, y: float, yaw: float):
        """บันทึก waypoint ลงไฟล์ JSON."""
        os.makedirs(OUT_DIR, exist_ok=True)
        wps = {}
        if os.path.exists(WAYPOINTS_FILE):
            with open(WAYPOINTS_FILE, 'r') as f:
                wps = json.load(f)
        order = wps[name]['order'] if name in wps else len(wps)
        wps[name] = {
            'x': round(x, 4), 'y': round(y, 4),
            'yaw': round(yaw, 2), 'order': order,
        }
        with open(WAYPOINTS_FILE, 'w') as f:
            json.dump(wps, f, indent=2, ensure_ascii=False)
        print(f'  บันทึก waypoint [{name}] → {WAYPOINTS_FILE}')

    # ── Nav2 Goal ─────────────────────────────────────────────────────

    def send_nav_goal_and_wait(self, x: float, y: float,
                               yaw_deg: float, timeout: float = 180.0) -> bool:
        """ส่ง NavigateToPose goal และรอจนเสร็จ (blocking)."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            print('  Nav2 action server ไม่พร้อม!')
            return False

        qx, qy, qz, qw = _yaw_deg_to_quat(yaw_deg)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id         = 'map'
        goal.pose.header.stamp            = self.get_clock().now().to_msg()
        goal.pose.pose.position.x         = float(x)
        goal.pose.pose.position.y         = float(y)
        goal.pose.pose.orientation.x      = float(qx)
        goal.pose.pose.orientation.y      = float(qy)
        goal.pose.pose.orientation.z      = float(qz)
        goal.pose.pose.orientation.w      = float(qw)

        done_event  = threading.Event()
        succeeded   = [False]

        def _result_cb(future):
            from action_msgs.msg import GoalStatus
            try:
                result = future.result()
                succeeded[0] = (result.status == GoalStatus.STATUS_SUCCEEDED)
            except Exception as e:
                print(f'  result error: {e}')
            finally:
                done_event.set()

        def _goal_response_cb(future):
            handle = future.result()
            if not handle.accepted:
                print('  Goal ถูกปฏิเสธโดย Nav2!')
                done_event.set()
                return
            handle.get_result_async().add_done_callback(_result_cb)

        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(_goal_response_cb)
        done_event.wait(timeout=timeout)
        return succeeded[0]

    # ── Save CSV ──────────────────────────────────────────────────────

    def save_csv(self):
        if not self._results:
            print('ไม่มีข้อมูลที่จะบันทึก')
            return None
        os.makedirs(OUT_DIR, exist_ok=True)
        ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(OUT_DIR, f'nav2_pose_{ts}.csv')
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self._results[0].keys())
            writer.writeheader()
            writer.writerows(self._results)
        print(f'\nบันทึก CSV → {path}')
        return path

    # ── Plot กราฟ ─────────────────────────────────────────────────────

    def plot(self):
        results = self._results
        if not results:
            return

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        os.makedirs(OUT_DIR, exist_ok=True)

        names   = [r['waypoint'] for r in results]
        ax_vals = [float(r['actual_x']) for r in results]
        ay_vals = [float(r['actual_y']) for r in results]

        has_target = all(r['target_x'] != '-' for r in results)

        def _style(a):
            a.set_facecolor('white')
            a.tick_params(colors='#333333')
            a.xaxis.label.set_color('#333333')
            a.yaxis.label.set_color('#333333')
            a.title.set_color('#1a1a1a')
            for sp in a.spines.values():
                sp.set_edgecolor('#cccccc')

        n_plots = 3 if has_target else 1
        fig, axes = plt.subplots(1, n_plots, figsize=(6 * n_plots, 6))
        if n_plots == 1:
            axes = [axes]
        fig.patch.set_facecolor('white')

        # ── Plot 1: XY map ─────────────────────────────────────────
        a = axes[0]
        _style(a)

        all_x = list(ax_vals)
        all_y = list(ay_vals)
        if has_target:
            all_x += [float(r['target_x']) for r in results if r['target_x'] != '-']
            all_y += [float(r['target_y']) for r in results if r['target_y'] != '-']

        # คำนวณขอบเขตที่เหมาะสม (equal aspect + margin)
        x_center = (max(all_x) + min(all_x)) / 2
        y_center = (max(all_y) + min(all_y)) / 2
        span     = max(max(all_x) - min(all_x), max(all_y) - min(all_y))
        margin   = max(span * 0.4, 0.5)   # ≥ 0.5 m margin

        half = span / 2 + margin
        a.set_xlim(x_center - half, x_center + half)
        a.set_ylim(y_center - half, y_center + half)
        a.set_aspect('equal')

        for i, r in enumerate(results):
            c = COLORS[i % len(COLORS)]
            a.scatter(float(r['actual_x']), float(r['actual_y']),
                      color=c, s=120, marker='o', zorder=5,
                      edgecolors='black', linewidths=0.8)
            a.text(float(r['actual_x']) + margin * 0.06,
                   float(r['actual_y']) + margin * 0.06,
                   r['waypoint'], fontsize=10, fontweight='bold', color=c,
                   path_effects=[pe.withStroke(linewidth=2, foreground='white')])
            if has_target and r['target_x'] != '-':
                a.scatter(float(r['target_x']), float(r['target_y']),
                          color=c, s=140, marker='*', zorder=4,
                          label=f'{r["waypoint"]} เป้าหมาย')
                a.annotate('',
                    xy=(float(r['actual_x']), float(r['actual_y'])),
                    xytext=(float(r['target_x']), float(r['target_y'])),
                    arrowprops=dict(arrowstyle='->', color=c,
                                   lw=1.5, linestyle='dashed'))

        # เส้นเชื่อม waypoints (เส้นทาง)
        if len(ax_vals) > 1:
            a.plot(ax_vals, ay_vals, color='#90a4ae', linewidth=1.2,
                   linestyle='--', zorder=3, alpha=0.7)

        a.set_title('ตำแหน่งจริง (AMCL) ของหุ่นยนต์', fontsize=12, fontweight='bold')
        a.set_xlabel('X (m)', fontsize=10)
        a.set_ylabel('Y (m)', fontsize=10)
        a.grid(True, color='#e0e0e0', linewidth=0.8, alpha=0.8)
        if has_target:
            a.legend(fontsize=8, facecolor='white', edgecolor='#cccccc')

        if has_target:
            x_idx   = list(range(len(names)))
            err_x   = [float(r['err_x'])   for r in results]
            err_y   = [float(r['err_y'])   for r in results]
            err_xy  = [float(r['err_xy'])  for r in results]
            err_yaw = [float(r['err_yaw']) if r['err_yaw'] != '-' else 0.0
                       for r in results]

            # ── Plot 2: Error X/Y/XY ──────────────────────────────
            a2 = axes[1]
            _style(a2)
            w = 0.25
            b1 = a2.bar([i - w for i in x_idx], err_x,  width=w,
                        color='#c62828', label='|Δx| (m)', alpha=0.85)
            b2 = a2.bar(x_idx,                  err_y,  width=w,
                        color='#00838f', label='|Δy| (m)', alpha=0.85)
            b3 = a2.bar([i + w for i in x_idx], err_xy, width=w,
                        color='#6a1b9a', label='|Δxy| (m)', alpha=0.85)
            for bars in (b1, b2, b3):
                for bar in bars:
                    h = bar.get_height()
                    a2.text(bar.get_x() + bar.get_width() / 2, h + 0.001,
                            f'{h:.4f}', ha='center', va='bottom',
                            fontsize=7.5, color='#333333')

            avg_xy = sum(err_xy) / len(err_xy)
            a2.text(0.98, 0.97,
                    f'mean Δxy = {avg_xy:.4f} m\nmax  Δxy = {max(err_xy):.4f} m',
                    transform=a2.transAxes, fontsize=9, color='#333333',
                    va='top', ha='right',
                    bbox=dict(facecolor='#f5f5f5', edgecolor='#cccccc',
                              boxstyle='round,pad=0.5', alpha=0.9))
            a2.set_title('ความคลาดเคลื่อน X / Y / XY', fontsize=12, fontweight='bold')
            a2.set_xlabel('Waypoint', fontsize=10)
            a2.set_ylabel('Error (m)', fontsize=10)
            a2.set_xticks(x_idx)
            a2.set_xticklabels(names, fontsize=10)
            a2.legend(fontsize=9, facecolor='white', edgecolor='#cccccc')
            a2.grid(True, axis='y', color='#e0e0e0', linewidth=0.8, alpha=0.8)

            # ── Plot 3: Error Yaw ─────────────────────────────────
            a3 = axes[2]
            _style(a3)
            bar_c = [COLORS[i % len(COLORS)] for i in x_idx]
            by = a3.bar(x_idx, err_yaw, color=bar_c, alpha=0.85,
                        edgecolor='white', linewidth=0.8)
            for bar in by:
                h = bar.get_height()
                a3.text(bar.get_x() + bar.get_width() / 2, h + 0.05,
                        f'{h:.2f}°', ha='center', va='bottom',
                        fontsize=9, color='#333333')

            avg_yaw = sum(err_yaw) / len(err_yaw)
            a3.text(0.98, 0.97,
                    f'mean Δyaw = {avg_yaw:.2f}°\nmax  Δyaw = {max(err_yaw):.2f}°',
                    transform=a3.transAxes, fontsize=9, color='#333333',
                    va='top', ha='right',
                    bbox=dict(facecolor='#f5f5f5', edgecolor='#cccccc',
                              boxstyle='round,pad=0.5', alpha=0.9))
            a3.set_title('ความคลาดเคลื่อนมุม Yaw', fontsize=12, fontweight='bold')
            a3.set_xlabel('Waypoint', fontsize=10)
            a3.set_ylabel('|Δyaw| (°)', fontsize=10)
            a3.set_xticks(x_idx)
            a3.set_xticklabels(names, fontsize=10)
            a3.grid(True, axis='y', color='#e0e0e0', linewidth=0.8, alpha=0.8)

        fig.suptitle('ผลการทดสอบความแม่นยำการนำทาง (Nav2) — AMR MTT',
                     fontsize=14, fontweight='bold', color='#1a1a1a')
        plt.tight_layout()

        out_file = os.path.join(OUT_DIR, f'nav2_pose_{ts}.png')
        plt.savefig(out_file, dpi=180, bbox_inches='tight',
                    facecolor=fig.get_facecolor())
        print(f'บันทึกกราฟ → {out_file}')
        plt.show()

    # ── Print summary ─────────────────────────────────────────────────

    def print_summary(self):
        if not self._results:
            return
        print('\n' + '=' * 65)
        print('  สรุปผล Nav2 Pose Accuracy')
        print('=' * 65)
        has_err = self._results[0]['err_xy'] != '-'
        if has_err:
            print(f'  {"WP":<5} {"x จริง":<10} {"y จริง":<10} {"yaw จริง":<10}'
                  f' {"Δx (m)":<9} {"Δy (m)":<9} {"Δxy (m)":<9} {"Δyaw (°)"}')
            print('  ' + '-' * 63)
            for r in self._results:
                print(f'  {r["waypoint"]:<5} {r["actual_x"]:<10} {r["actual_y"]:<10}'
                      f' {r["actual_yaw"]:<10} {r["err_x"]:<9} {r["err_y"]:<9}'
                      f' {r["err_xy"]:<9} {r["err_yaw"]}')
        else:
            print(f'  {"WP":<5} {"x จริง":<12} {"y จริง":<12} {"yaw จริง":<12} {"เวลา"}')
            print('  ' + '-' * 55)
            for r in self._results:
                print(f'  {r["waypoint"]:<5} {r["actual_x"]:<12} {r["actual_y"]:<12}'
                      f' {r["actual_yaw"]:<12} {r["timestamp"]}')
        print('=' * 65)


# ── Mode 1: บันทึก Waypoint ───────────────────────────────────────────────

def _save_waypoint_loop(node: Nav2PoseMonitor):
    """
    หุ่นยนต์วิ่งปกติ — ผู้ใช้พิมพ์ชื่อ waypoint (A/B/C) ทุกครั้งที่ถึงจุดหมาย
    ระบบบันทึกทันที ไม่มีการ overwrite ข้อมูลในรอบเดียวกัน
    """
    print('\n════════════════════════════════════════════════')
    print('  โหมด 1: บันทึก Waypoint')
    print('════════════════════════════════════════════════')
    print('  • หุ่นยนต์วิ่งได้ปกติ — AMCL แสดงผลตลอดเวลา')
    print('  • เมื่อถึงจุดหมาย พิมพ์ชื่อ เช่น  A  แล้วกด Enter')
    print('  • ชื่อเดิมในรอบนี้จะไม่ถูกบันทึกซ้ำ')
    print('  • Ctrl+C เพื่อจบและบันทึกผล')
    if os.path.exists(WAYPOINTS_FILE):
        wps_exist = node.load_waypoints()
        if wps_exist:
            print(f'  ℹ  waypoints เดิม: {", ".join(wps_exist.keys())}  (จะถูกอัปเดต)')
    print('────────────────────────────────────────────────\n')

    saved_labels: set = set()   # ป้องกัน save ซ้ำในรอบเดียวกัน

    while True:
        # หยุดแสดงผล live ชั่วคราวเพื่อรับ input
        node._display_paused = True
        print('\n', end='', flush=True)   # ขึ้นบรรทัดใหม่ห่างจาก status line
        try:
            raw = input('  ถึงจุดหมาย → พิมพ์ชื่อ waypoint (A/B/C ...) Ctrl+C=จบ: ')
        except (EOFError, KeyboardInterrupt):
            node._display_paused = False
            break

        label = raw.strip().upper()
        node._display_paused = False   # กลับมาแสดงผล live

        if not label:
            continue

        # ป้องกัน save ซ้ำ (ชื่อเดิมในรอบนี้)
        if label in saved_labels:
            print(f'  [!] {label} บันทึกไปแล้วในรอบนี้ — ข้าม')
            print('      หากต้องการ overwrite พิมพ์  {label}!  (ตามด้วย !)')
            continue

        # รองรับ forced overwrite เช่น "A!"
        if label.endswith('!'):
            label = label[:-1]
            if label in saved_labels:
                # ลบ entry เดิมออกก่อน overwrite
                node._results = [r for r in node._results if r['waypoint'] != label]
                saved_labels.discard(label)

        ax, ay, ayaw = node.get_pose()
        if ax is None:
            print('  [!] ยังไม่ได้รับ AMCL pose — รอสักครู่แล้วลองใหม่')
            continue

        node.save_waypoint_to_file(label, ax, ay, ayaw)
        node.record_snapshot(label)          # ไม่มี target → ไม่คำนวณ error
        saved_labels.add(label)
        print(f'  ✓  บันทึก {label} สำเร็จ  (บันทึกแล้ว: {sorted(saved_labels)})')

    print('\n\nกำลังบันทึกผล...')
    wps = node.load_waypoints()
    if wps:
        print(f'  waypoints ในไฟล์: {" → ".join(wps.keys())}')


# ── Mode 2: นำทางอัตโนมัติ ────────────────────────────────────────────────

def _auto_navigate_loop(node: Nav2PoseMonitor):
    """โหลด waypoints และส่ง Nav2 goal ตามลำดับ."""
    print('\n════════════════════════════════════════════════')
    print('  โหมด 2: นำทางอัตโนมัติ')
    print('════════════════════════════════════════════════')

    wps = node.load_waypoints()
    if not wps:
        print(f'  ไม่พบไฟล์ waypoints: {WAYPOINTS_FILE}')
        print('  กรุณารันโหมด 1 เพื่อบันทึก waypoints ก่อน')
        return

    print(f'  โหลด waypoints: {" → ".join(wps.keys())}')
    print()

    # ถามว่าต้องการเลือกเฉพาะ waypoint หรือรันทั้งหมด
    try:
        sel = input('  เลือก waypoints (Enter = ทั้งหมด, หรือพิมพ์ เช่น A B C): ').strip()
    except (EOFError, KeyboardInterrupt):
        return

    if sel:
        selected_names = sel.upper().split()
        route = {k: v for k, v in wps.items() if k.upper() in selected_names}
        if not route:
            print('  ไม่พบ waypoints ที่เลือก')
            return
    else:
        route = wps

    print(f'\n  เส้นทาง: {" → ".join(route.keys())}')
    print('────────────────────────────────────────────────')

    try:
        input('\n  กด Enter เพื่อเริ่มนำทาง (Ctrl+C เพื่อยกเลิก): ')
    except (EOFError, KeyboardInterrupt):
        return

    for i, (name, wp) in enumerate(route.items()):
        x, y, yaw = wp['x'], wp['y'], wp['yaw']
        print(f'\n  [{i+1}/{len(route)}] กำลังนำทางไปยัง {name}  '
              f'(x={x:.4f}, y={y:.4f}, yaw={yaw:.2f}°)')

        ok = node.send_nav_goal_and_wait(x, y, yaw, timeout=180.0)

        if ok:
            print(f'  ✓ ถึง {name} สำเร็จ')
        else:
            print(f'  ✗ ไม่สำเร็จที่ {name} (timeout หรือ goal ถูกยกเลิก)')

        # รอให้หุ่นยนต์หยุดนิ่ง แล้วบันทึก pose
        time.sleep(0.5)
        node.record_snapshot(name, target_x=x, target_y=y, target_yaw=yaw)

        if i < len(route) - 1:
            try:
                cont = input(f'  กด Enter ไปยัง waypoint ถัดไป (หรือ q เพื่อหยุด): ').strip()
                if cont.lower() == 'q':
                    break
            except (EOFError, KeyboardInterrupt):
                break

    print('\n  นำทางเสร็จสิ้น')


# ── Mode selection menu ───────────────────────────────────────────────────

def _select_mode() -> str:
    """แสดงเมนูและรับโหมด."""
    print('\n╔══════════════════════════════════════════════╗')
    print('║   AMR MTT — Nav2 Pose Tool                  ║')
    print('╠══════════════════════════════════════════════╣')
    print('║  [1]  บันทึก Waypoint  (Save mode)          ║')
    print('║       กด Enter ที่ตำแหน่ง → บันทึกชื่อ+พิกัด  ║')
    print('║                                              ║')
    print('║  [2]  นำทางอัตโนมัติ   (Navigate mode)      ║')
    print('║       โหลด waypoints แล้วเคลื่อนที่ A→B→C   ║')
    print('╚══════════════════════════════════════════════╝')

    while True:
        try:
            choice = input('\n  เลือกโหมด [1/2]: ').strip()
        except (EOFError, KeyboardInterrupt):
            raise SystemExit(0)
        if choice == '1':
            return 'save'
        if choice == '2':
            return 'navigate'
        print('  กรุณาพิมพ์ 1 หรือ 2')


# ── Main ──────────────────────────────────────────────────────────────────

def main(args=None):
    mode = _select_mode()

    rclpy.init(args=args)
    node = Nav2PoseMonitor(mode)

    # Spin ใน background thread เพื่อให้ action callbacks ทำงานได้
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if mode == 'save':
            _save_waypoint_loop(node)
        else:
            _auto_navigate_loop(node)
    except KeyboardInterrupt:
        pass

    print('\n\nกำลังบันทึกผล...')
    node.save_csv()
    node.print_summary()
    node.plot()

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
