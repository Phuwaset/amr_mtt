#!/usr/bin/env python3
"""
square_test.py — เปรียบเทียบความแม่นยำ 3 วิธีระบุตำแหน่งเทียบ Gazebo Ground Truth
                  เคลื่อนที่เป็นสี่เหลี่ยม 2×2 เมตร (4 ด้าน)

Ground Truth : /amr_mtt/odom  (Ignition world-frame, odometry_source=world)
วิธีที่เปรียบเทียบ:
  1. Raw Odometry  — /diff_drive_controller/odom
  2. EKF Filtered  — /odometry/filtered
  3. AMCL          — /amcl_pose

Metrics (เทียบกับ GT):
  ATE RMSE    — Root Mean Square position error ตลอด trajectory (m)
  Mean Error  — ค่าเฉลี่ยความคลาดเคลื่อนตำแหน่ง (m)
  Max Error   — ความคลาดเคลื่อนสูงสุด (m)
  Yaw RMSE    — Root Mean Square yaw error (°)
  Final Drift — ความคลาดเคลื่อนที่จุดสิ้นสุด (m)

วิธีใช้:
    Terminal 1: ros2_nvidia launch amr_mtt_bot ign.launch.py world_file:=test_room.sdf
    Terminal 2: ros2_nvidia launch amr_mtt_bot nav2.launch.py
    Terminal 3: ros2 run amr_result square_test
              (ตั้ง 2D Pose Estimate ใน RViz ก่อนรัน)
"""

import math
import time
import os
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
LINEAR_VEL  = 0.5     # m/s
ANGULAR_VEL = 0.3     # rad/s
SIDE_LENGTH = 2.0     # เมตร
SETTLE_TIME = 1.5     # วินาที รอหลังหยุดแต่ละท่า
OUT_DIR     = os.path.expanduser('~/amr_mtt_results')
CMD_TOPIC   = '/diff_drive_controller/cmd_vel_unstamped'

plt.rcParams['font.family'] = 'Waree'


# ── Math Helpers ──────────────────────────────────────────────────────────────

def _quat_to_yaw(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def _normalize_angle(angle):
    while angle >  math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle


def _euclidean(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _normalize_traj(traj):
    """
    Normalize trajectory เพื่อให้ทุกวิธีเริ่มจากตำแหน่งเดียวกัน (0, 0, yaw=0)
    ใช้ SE2 transform: หมุนและเลื่อนให้จุดแรกอยู่ที่ origin
    """
    if not traj:
        return []
    t0, x0, y0, yaw0 = traj[0]
    cos_y = math.cos(-yaw0)
    sin_y = math.sin(-yaw0)
    result = []
    for t, x, y, yaw in traj:
        dx, dy = x - x0, y - y0
        nx =  dx * cos_y - dy * sin_y
        ny =  dx * sin_y + dy * cos_y
        result.append((t, nx, ny, _normalize_angle(yaw - yaw0)))
    return result


def _interp_traj(traj, query_times):
    """
    Interpolate trajectory ที่ query_times ด้วย nearest-neighbor
    คืน list of (t, x, y, yaw) ที่ตรงกับ query_times
    """
    if not traj:
        return [(t, float('nan'), float('nan'), float('nan')) for t in query_times]
    times = np.array([p[0] for p in traj])
    xs    = np.array([p[1] for p in traj])
    ys    = np.array([p[2] for p in traj])
    yaws  = np.array([p[3] for p in traj])
    result = []
    for qt in query_times:
        idx = int(np.argmin(np.abs(times - qt)))
        result.append((qt, float(xs[idx]), float(ys[idx]), float(yaws[idx])))
    return result


def _compute_metrics(gt_traj, est_traj):
    """
    คำนวณ metrics เปรียบเทียบ estimated trajectory กับ ground truth
    ทั้งสอง trajectory ต้อง time-aligned แล้ว (ความยาวเท่ากัน)
    """
    pos_errors = []
    yaw_errors = []
    for (_, gx, gy, gyaw), (_, ex, ey, eyaw) in zip(gt_traj, est_traj):
        if math.isnan(ex) or math.isnan(gx):
            continue
        pos_errors.append(_euclidean(gx, gy, ex, ey))
        yaw_errors.append(abs(math.degrees(_normalize_angle(gyaw - eyaw))))

    if not pos_errors:
        return None

    ate_rmse    = math.sqrt(sum(e * e for e in pos_errors) / len(pos_errors))
    mean_err    = sum(pos_errors) / len(pos_errors)
    max_err     = max(pos_errors)
    yaw_rmse    = math.sqrt(sum(e * e for e in yaw_errors) / len(yaw_errors))
    final_drift = pos_errors[-1]

    return dict(
        ate_rmse    = ate_rmse,
        mean_err    = mean_err,
        max_err     = max_err,
        yaw_rmse    = yaw_rmse,
        final_drift = final_drift,
        pos_errors  = pos_errors,   # เก็บไว้ plot
    )


# ── Node ──────────────────────────────────────────────────────────────────────

class SquareTest(Node):

    def __init__(self):
        super().__init__('square_test')

        self._pub = self.create_publisher(Twist, CMD_TOPIC, 10)

        # ค่าปัจจุบันสำหรับควบคุมการขับ (ใช้ raw odom)
        self._raw_x   = 0.0
        self._raw_y   = 0.0
        self._raw_yaw = 0.0

        # Accumulated yaw สำหรับ rotation tracking
        self._tracking = False
        self._acc_raw  = 0.0
        self._prev_raw_yaw = 0.0

        # Trajectories: list of (timestamp_sec, x, y, yaw)
        self.gt_traj  = []   # Ground Truth  — /amr_mtt/odom
        self.raw_traj = []   # Raw Odometry  — /diff_drive_controller/odom
        self.ekf_traj = []   # EKF Filtered  — /odometry/filtered
        self.amc_traj = []   # AMCL          — /amcl_pose

        self._gt_recv  = False
        self._raw_recv = False
        self._ekf_recv = False
        self._amc_recv = False

        self.create_subscription(Odometry, '/amr_mtt/odom',
                                 self._gt_cb, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom',
                                 self._raw_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._ekf_cb, 10)
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._amc_cb, amcl_qos)

        self.get_logger().info('SquareTest พร้อม — กำลังรอ Odometry...')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _append(self, traj, stamp, x, y, q):
        t   = _stamp_to_sec(stamp)
        yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        traj.append((t, x, y, yaw))

    def _gt_cb(self, msg: Odometry):
        self._gt_recv = True
        self._append(self.gt_traj, msg.header.stamp,
                     msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     msg.pose.pose.orientation)

    def _raw_cb(self, msg: Odometry):
        self._raw_recv = True
        self._raw_x = msg.pose.pose.position.x
        self._raw_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        new_yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        if self._tracking:
            self._acc_raw += _normalize_angle(new_yaw - self._prev_raw_yaw)
        self._prev_raw_yaw = new_yaw
        self._raw_yaw = new_yaw
        self._append(self.raw_traj, msg.header.stamp,
                     msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     msg.pose.pose.orientation)

    def _ekf_cb(self, msg: Odometry):
        self._ekf_recv = True
        self._append(self.ekf_traj, msg.header.stamp,
                     msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     msg.pose.pose.orientation)

    def _amc_cb(self, msg: PoseWithCovarianceStamped):
        self._amc_recv = True
        self._append(self.amc_traj, msg.header.stamp,
                     msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     msg.pose.pose.orientation)

    # ── Driving Helpers ───────────────────────────────────────────────────────

    def _spin_dur(self, duration: float):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _stop(self):
        self._pub.publish(Twist())
        self._spin_dur(SETTLE_TIME)

    def _drive(self, dist: float):
        """เดินตรง dist เมตร (ใช้ raw odom เป็น feedback)"""
        timeout = dist / LINEAR_VEL * 6.0
        twist = Twist()
        twist.linear.x = LINEAR_VEL
        self._spin_dur(0.3)
        sx, sy = self._raw_x, self._raw_y
        t0 = time.time()
        next_pub = t0
        while True:
            now = time.time()
            if now >= next_pub:
                self._pub.publish(twist)
                next_pub = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
            if _euclidean(self._raw_x, self._raw_y, sx, sy) >= dist:
                break
            if now - t0 >= timeout:
                self.get_logger().warn('Drive timeout')
                break
        self._stop()

    def _rotate_90(self):
        """หมุน 90° ซ้าย (ใช้ raw odom สะสมมุม)"""
        target_rad = math.pi / 2.0
        timeout    = target_rad / ANGULAR_VEL * 6.0
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        self._acc_raw  = 0.0
        self._tracking = True
        twist = Twist()
        twist.angular.z = ANGULAR_VEL
        t0 = time.time()
        next_pub = t0
        while True:
            now = time.time()
            if now >= next_pub:
                self._pub.publish(twist)
                next_pub = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
            if abs(self._acc_raw) >= target_rad:
                break
            if now - t0 >= timeout:
                self.get_logger().warn('Rotate timeout')
                break
        self._pub.publish(Twist())
        self._spin_dur(SETTLE_TIME)
        self._tracking = False

    # ── Main ──────────────────────────────────────────────────────────────────

    def run_square(self):
        os.makedirs(OUT_DIR, exist_ok=True)
        print()
        print('═══════════════════════════════════════════════════════════')
        print('   AMR MTT — Square Test  (2×2 m, 4 ด้าน)')
        print(f'   ความเร็วเชิงเส้น  : {LINEAR_VEL} m/s')
        print(f'   ความเร็วเชิงมุม   : {ANGULAR_VEL} rad/s')
        print(f'   ระยะด้านละ        : {SIDE_LENGTH} m')
        print('═══════════════════════════════════════════════════════════')

        print('\n  รอ Odometry 3 วินาที...')
        self._spin_dur(3.0)

        # ── แสดงสถานะ topic ───────────────────────────────────────
        print()
        print(f'  /amr_mtt/odom  (GT)          : {"✅ รับข้อมูลแล้ว" if self._gt_recv  else "❌ ไม่มีข้อมูล"}')
        print(f'  /diff_drive_controller/odom  : {"✅ รับข้อมูลแล้ว" if self._raw_recv else "❌ ไม่มีข้อมูล"}')
        print(f'  /odometry/filtered           : {"✅ รับข้อมูลแล้ว" if self._ekf_recv else "❌ ไม่มีข้อมูล"}')
        print(f'  /amcl_pose                   : {"✅ รับข้อมูลแล้ว" if self._amc_recv else "⚠️  ไม่มีข้อมูล (ต้อง launch nav2 + ตั้ง 2D Pose Estimate)"}')

        if not self._gt_recv or not self._raw_recv or not self._ekf_recv:
            print('\n  ❌ GT / Raw Odom / EKF ไม่พร้อม — ยกเลิก')
            return

        if not self._amc_recv:
            ans = input('\n  AMCL ไม่มีข้อมูล — ดำเนินการต่อโดยไม่มี AMCL? (y/n): ').strip().lower()
            if ans != 'y':
                print('  ยกเลิก')
                return

        # ── เคลียร์ trajectory แล้วเริ่มวิ่ง ─────────────────────
        self.gt_traj.clear()
        self.raw_traj.clear()
        self.ekf_traj.clear()
        self.amc_traj.clear()

        for side in range(1, 5):
            print(f'\n  ด้านที่ {side}/4 — เดินหน้า {SIDE_LENGTH} m ...')
            self._drive(SIDE_LENGTH)
            print(f'           หมุน 90° ...')
            self._rotate_90()

        print('\n  เสร็จสิ้น! กำลังประมวลผล...')
        self._spin_dur(0.5)

        # ── Normalize ให้ทุกวิธีเริ่มจาก (0, 0, 0) ───────────────
        gt_n  = _normalize_traj(self.gt_traj)
        raw_n = _normalize_traj(self.raw_traj)
        ekf_n = _normalize_traj(self.ekf_traj)
        amc_n = _normalize_traj(self.amc_traj) if self.amc_traj else []

        # ── Time-align: interpolate แต่ละวิธีไปที่ timestamp ของ GT
        gt_times = [p[0] for p in gt_n]
        raw_a = _interp_traj(raw_n, gt_times)
        ekf_a = _interp_traj(ekf_n, gt_times)
        amc_a = _interp_traj(amc_n, gt_times) if amc_n else None

        # ── Compute Metrics ───────────────────────────────────────
        m_raw = _compute_metrics(gt_n, raw_a)
        m_ekf = _compute_metrics(gt_n, ekf_a)
        m_amc = _compute_metrics(gt_n, amc_a) if amc_a else None

        # เก็บไว้ให้ multi-run ใช้
        self._last_metrics = {
            'Raw Odometry': m_raw,
            'EKF Filtered': m_ekf,
            'AMCL':         m_amc,
        }

        # ── แสดงตาราง ─────────────────────────────────────────────
        self._print_table(m_raw, m_ekf, m_amc)
        self._save_csv(m_raw, m_ekf, m_amc)
        self._plot(gt_n, raw_n, ekf_n, amc_n,
                   raw_a, ekf_a, amc_a,
                   m_raw, m_ekf, m_amc, gt_times)

    # ── Output ────────────────────────────────────────────────────────────────

    def _print_table(self, m_raw, m_ekf, m_amc):
        sep = '╠══════════════╬══════════════╬══════════════╬══════════════╬══════════════╬══════════════╣'
        print()
        print('╔══════════════╦══════════════╦══════════════╦══════════════╦══════════════╦══════════════╗')
        print('║     วิธี      ║  ATE RMSE(m) ║  Mean Err(m) ║  Max Err (m) ║  Yaw RMSE(°) ║Final Drift(m)║')
        print(sep)

        def fmt_row(label, m):
            if m is None:
                return f'║ {label:<12} ║     N/A      ║     N/A      ║     N/A      ║     N/A      ║     N/A      ║'
            return (f'║ {label:<12} ║ {m["ate_rmse"]:^12.4f} ║ {m["mean_err"]:^12.4f} ║'
                    f' {m["max_err"]:^12.4f} ║ {m["yaw_rmse"]:^12.2f} ║ {m["final_drift"]:^12.4f} ║')

        print(fmt_row('Raw Odometry', m_raw))
        print(sep)
        print(fmt_row('EKF Filtered', m_ekf))
        print(sep)
        print(fmt_row('AMCL        ', m_amc))
        print('╚══════════════╩══════════════╩══════════════╩══════════════╩══════════════╩══════════════╝')

    def _save_csv(self, m_raw, m_ekf, m_amc):
        ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(OUT_DIR, f'square_test_{ts}.csv')
        fields = ['method', 'ate_rmse_m', 'mean_error_m', 'max_error_m',
                  'yaw_rmse_deg', 'final_drift_m']
        rows = []
        for label, m in [('Raw Odometry', m_raw), ('EKF Filtered', m_ekf), ('AMCL', m_amc)]:
            if m:
                rows.append({
                    'method'        : label,
                    'ate_rmse_m'    : round(m['ate_rmse'],    4),
                    'mean_error_m'  : round(m['mean_err'],    4),
                    'max_error_m'   : round(m['max_err'],     4),
                    'yaw_rmse_deg'  : round(m['yaw_rmse'],    2),
                    'final_drift_m' : round(m['final_drift'], 4),
                })
            else:
                rows.append(dict(zip(fields, [label] + ['N/A'] * 5)))

        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        print(f'\n  บันทึก CSV → {path}')

    def _plot(self, gt_n, raw_n, ekf_n, amc_n,
              raw_a, ekf_a, amc_a, m_raw, m_ekf, m_amc, gt_times):

        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        fig.suptitle('AMR MTT — Square Test 2×2 m\nเปรียบเทียบความแม่นยำ 3 วิธีระบุตำแหน่ง vs Gazebo Ground Truth',
                     fontsize=13, y=1.01)

        COLOR_GT  = '#2c3e50'
        COLOR_RAW = '#e74c3c'
        COLOR_EKF = '#2980b9'
        COLOR_AMC = '#27ae60'

        def xy(traj):
            return [p[1] for p in traj], [p[2] for p in traj]

        # ─── Plot 1: Trajectory ──────────────────────────────────
        ax = axes[0]
        # วาด GT ก่อน (zorder ต่ำ) เพื่อให้วิธีอื่นวาดทับได้
        gx, gy = xy(gt_n)
        ax.plot(gx, gy, color=COLOR_GT, linewidth=2.5, linestyle='-',
                label='Ground Truth (Gazebo)', zorder=3)
        if raw_n:
            rx, ry = xy(raw_n)
            ax.plot(rx, ry, color=COLOR_RAW, linewidth=1.5, label='Raw Odometry', zorder=4)
        if ekf_n:
            ex, ey = xy(ekf_n)
            # ใช้เส้นประเพื่อให้มองเห็นแม้จะทับ GT
            ax.plot(ex, ey, color=COLOR_EKF, linewidth=2.0, linestyle='--',
                    label='EKF Filtered', zorder=6)
        if amc_n:
            ax_x, ax_y = xy(amc_n)
            ax.plot(ax_x, ax_y, color=COLOR_AMC, linewidth=1.5, label='AMCL', zorder=5)

        # Ideal square
        L = SIDE_LENGTH
        ax.plot([0, L, L, 0, 0], [0, 0, L, L, 0],
                'k--', linewidth=1.0, alpha=0.35, label='Ideal Square')
        ax.scatter([0], [0], color='black', s=100, zorder=6, marker='o')

        ax.set_aspect('equal')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('เส้นทาง (Normalized)')
        ax.legend(fontsize=8, loc='upper right')
        ax.grid(True, linestyle=':', alpha=0.5)

        # ─── Plot 2: Position Error vs Time ──────────────────────
        ax = axes[1]
        t0 = gt_times[0] if gt_times else 0

        def err_series(gt, est):
            ts, es = [], []
            if est is None:
                return [], []
            for (tg, gx2, gy2, _), (_, ex2, ey2, _) in zip(gt, est):
                if not (math.isnan(ex2) or math.isnan(gx2)):
                    ts.append(tg - t0)
                    es.append(_euclidean(gx2, gy2, ex2, ey2))
            return ts, es

        t_r, e_r = err_series(gt_n, raw_a)
        t_e, e_e = err_series(gt_n, ekf_a)
        t_a, e_a = err_series(gt_n, amc_a) if amc_a else ([], [])

        if t_r: ax.plot(t_r, e_r, color=COLOR_RAW, linewidth=1.5, label='Raw Odometry')
        if t_e: ax.plot(t_e, e_e, color=COLOR_EKF, linewidth=1.5, label='EKF Filtered')
        if t_a: ax.plot(t_a, e_a, color=COLOR_AMC, linewidth=1.5, label='AMCL')

        ax.set_xlabel('เวลา (s)')
        ax.set_ylabel('Position Error (m)')
        ax.set_title('ความคลาดเคลื่อนตำแหน่ง vs เวลา')
        ax.legend(fontsize=8)
        ax.grid(True, linestyle=':', alpha=0.5)

        # ─── Plot 3: Bar Chart (ATE RMSE / Mean / Max) ───────────
        ax = axes[2]
        metric_labels = ['ATE RMSE', 'Mean Error', 'Max Error']
        method_names  = ['Raw\nOdometry', 'EKF\nFiltered', 'AMCL']
        colors        = [COLOR_RAW, COLOR_EKF, COLOR_AMC]
        x = np.arange(len(metric_labels))
        width = 0.25

        for i, (m, color) in enumerate(zip([m_raw, m_ekf, m_amc], colors)):
            vals = ([m['ate_rmse'], m['mean_err'], m['max_err']]
                    if m else [0.0, 0.0, 0.0])
            bars = ax.bar(x + i * width, vals, width,
                          label=method_names[i], color=color, alpha=0.85)
            for bar, v in zip(bars, vals):
                if v > 0:
                    ax.text(bar.get_x() + bar.get_width() / 2,
                            bar.get_height() + 0.002,
                            f'{v:.3f}', ha='center', va='bottom', fontsize=7)

        ax.set_xlabel('Metric')
        ax.set_ylabel('Error (m)')
        ax.set_title('เปรียบเทียบ Metrics')
        ax.set_xticks(x + width)
        ax.set_xticklabels(metric_labels)
        ax.legend(fontsize=8)
        ax.grid(True, linestyle=':', alpha=0.5, axis='y')

        plt.tight_layout()
        ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(OUT_DIR, f'square_test_{ts}.png')
        fig.savefig(path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f'  บันทึก Plot → {path}')


# ── Multi-Run Summary ─────────────────────────────────────────────────────────

def _print_summary(all_results):
    """แสดงค่าเฉลี่ย ± std จากการทดสอบหลายรอบ"""
    import statistics
    methods = ['Raw Odometry', 'EKF Filtered', 'AMCL']
    metrics = ['ate_rmse', 'mean_err', 'max_err', 'yaw_rmse', 'final_drift']
    labels  = ['ATE RMSE(m)', 'Mean Err(m)', 'Max Err (m)', 'Yaw RMSE(°)', 'Final Drift(m)']

    print()
    print('═' * 70)
    print(f'  สรุปผล {len(all_results)} รอบ — ค่าเฉลี่ย ± std')
    print('═' * 70)
    for method in methods:
        print(f'\n  {method}')
        for metric, label in zip(metrics, labels):
            vals = [r[method][metric] for r in all_results
                    if r.get(method) and r[method].get(metric) is not None]
            if vals:
                mean = statistics.mean(vals)
                std  = statistics.stdev(vals) if len(vals) > 1 else 0.0
                print(f'    {label:<16}: {mean:.4f} ± {std:.4f}')
            else:
                print(f'    {label:<16}: N/A')

    # บันทึก summary CSV
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(OUT_DIR, f'square_summary_{ts}.csv')
    fields = ['method', 'runs', 'ate_rmse_mean', 'ate_rmse_std',
              'mean_err_mean', 'mean_err_std', 'max_err_mean', 'max_err_std',
              'yaw_rmse_mean', 'yaw_rmse_std', 'final_drift_mean', 'final_drift_std']
    rows = []
    for method in methods:
        row = {'method': method, 'runs': len(all_results)}
        for metric in metrics:
            vals = [r[method][metric] for r in all_results
                    if r.get(method) and r[method].get(metric) is not None]
            if vals:
                row[f'{metric}_mean'] = round(statistics.mean(vals), 4)
                row[f'{metric}_std']  = round(statistics.stdev(vals) if len(vals) > 1 else 0.0, 4)
            else:
                row[f'{metric}_mean'] = 'N/A'
                row[f'{metric}_std']  = 'N/A'
        rows.append(row)
    with open(path, 'w', newline='') as f:
        csv.DictWriter(f, fieldnames=fields).writeheader()
        csv.DictWriter(f, fieldnames=fields).writerows(rows)
    print(f'\n  บันทึก Summary CSV → {path}')


# ── Entry Point ───────────────────────────────────────────────────────────────

def main(args=None):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs', type=int, default=1,
                        help='จำนวนรอบทดสอบ (default: 1)')
    parsed, remaining = parser.parse_known_args(args)

    rclpy.init(args=remaining)
    n_runs     = parsed.runs
    all_results = []

    for i in range(n_runs):
        if n_runs > 1:
            print(f'\n{"═"*60}')
            print(f'  รอบที่ {i+1}/{n_runs}')
            print(f'{"═"*60}')
            if i > 0:
                input('  กด Enter เพื่อเริ่มรอบถัดไป (ตั้ง 2D Pose Estimate ใน RViz ก่อน)...')

        node = SquareTest()
        try:
            node.run_square()
            # เก็บ metrics ของรอบนี้
            if hasattr(node, '_last_metrics'):
                all_results.append(node._last_metrics)
        except KeyboardInterrupt:
            print('\n  หยุดโดยผู้ใช้')
            break
        finally:
            node.destroy_node()

    if n_runs > 1 and all_results:
        _print_summary(all_results)

    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
