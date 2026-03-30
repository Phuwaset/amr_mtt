#!/usr/bin/env python3
"""
plot_odometry.py — เปรียบเทียบ Raw Odometry vs EKF Filtered Path
บันทึกเส้นทางจาก 2 Topic แล้ว Plot กราฟเมื่อกด Ctrl+C

วิธีใช้:
    ros2 run amr_mtt_task_planner plot_odometry

แล้วขับรถตามเส้นทางที่ต้องการ จากนั้นกด Ctrl+C เพื่อ Plot กราฟ
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
import csv
import os
from datetime import datetime


def _quat_to_yaw(qx, qy, qz, qw):
    """แปลง Quaternion → Yaw (rad)"""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)

plt.rcParams['font.family'] = 'Waree'


class OdometryPlotter(Node):

    def __init__(self):
        super().__init__('odometry_plotter')

        # เก็บ path ของแต่ละ topic
        self.raw_x, self.raw_y, self.raw_yaw = [], [], []
        self.ekf_x, self.ekf_y, self.ekf_yaw = [], [], []

        # เก็บ velocity (m/s, rad/s)
        self.ekf_vx,    self.ekf_vy,    self.ekf_omega    = [], [], []
        self.ekf_speed  = []   # |v| = sqrt(vx²+vy²)
        self.raw_vx,    self.raw_vy,    self.raw_omega    = [], [], []
        self.raw_speed  = []

        self.create_subscription(
            Odometry, '/amr_mtt/odom', self._raw_cb, 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._ekf_cb, 10)

        self.get_logger().info('บันทึกเส้นทาง... กด Ctrl+C เพื่อ Plot กราฟ')

    def _raw_cb(self, msg: Odometry):
        self.raw_x.append(msg.pose.pose.position.x)
        self.raw_y.append(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.raw_yaw.append(_quat_to_yaw(q.x, q.y, q.z, q.w))
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.raw_vx.append(vx)
        self.raw_vy.append(vy)
        self.raw_omega.append(msg.twist.twist.angular.z)
        self.raw_speed.append(math.sqrt(vx**2 + vy**2))

    def _ekf_cb(self, msg: Odometry):
        self.ekf_x.append(msg.pose.pose.position.x)
        self.ekf_y.append(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.ekf_yaw.append(_quat_to_yaw(q.x, q.y, q.z, q.w))
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.ekf_vx.append(vx)
        self.ekf_vy.append(vy)
        self.ekf_omega.append(msg.twist.twist.angular.z)
        self.ekf_speed.append(math.sqrt(vx**2 + vy**2))

    def save_csv(self):
        """บันทึกข้อมูลเป็น CSV"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_dir = os.path.expanduser('~/amr_mtt_results')
        os.makedirs(out_dir, exist_ok=True)

        # Raw Odom
        raw_file = os.path.join(out_dir, f'raw_odom_{timestamp}.csv')
        with open(raw_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for x, y in zip(self.raw_x, self.raw_y):
                writer.writerow([round(x, 4), round(y, 4)])

        # EKF Filtered
        ekf_file = os.path.join(out_dir, f'ekf_odom_{timestamp}.csv')
        with open(ekf_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for x, y in zip(self.ekf_x, self.ekf_y):
                writer.writerow([round(x, 4), round(y, 4)])

        print(f'\nบันทึก CSV:\n  {raw_file}\n  {ekf_file}')
        return out_dir, timestamp

    def _style_ax(self, ax):
        """ตั้งค่า style white-theme ให้กับ subplot"""
        ax.set_facecolor('white')
        ax.tick_params(colors='#333333')
        ax.xaxis.label.set_color('#333333')
        ax.yaxis.label.set_color('#333333')
        ax.title.set_color('#1a1a1a')
        for spine in ax.spines.values():
            spine.set_edgecolor('#cccccc')

    def _plot_reference_vs_actual(self, ax, data_x, data_y, title, source_label):
        """
        Plot กราฟ Reference vs Actual path (เส้นตรงอ้างอิง vs เส้นทางจริง)
        ตาม style ภาพที่ 4-6 ในรายงาน
        """
        self._style_ax(ax)

        # ── Normalize ให้เริ่มที่ (0, 0) ──
        norm_x = [x - data_x[0] for x in data_x]
        norm_y = [y - data_y[0] for y in data_y]

        # ── Reference line: เส้นตรงจาก Start ถึง End (y = 0) ──
        # ปัดระยะจริงเป็นจำนวนเต็ม เช่น 1.05 → 1, -4.98 → -5 (รักษาทิศทาง +/-)
        end_x = round(norm_x[-1]) if abs(norm_x[-1]) > 0.1 else norm_x[-1]
        ax.plot([0, end_x], [0, 0],
                color='#1565c0', linewidth=2.5, linestyle='-',
                label='Reference', zorder=3)

        # ── Actual path: เส้นทางจริงของหุ่นยนต์ ──
        ax.plot(norm_x, norm_y,
                color='#d32f2f', linewidth=1.8, linestyle='--',
                label='Robot moving distance', alpha=0.9, zorder=4)

        # ── จุด Start (วงกลมเหลือง) และ End (ดาวแดง) ──
        ax.scatter(0, 0, color='#fdd835', s=100, zorder=6,
                   edgecolors='#f57f17', linewidths=1.5, label='Start')
        ax.scatter(norm_x[-1], norm_y[-1], color='#d32f2f', s=150,
                   marker='*', zorder=6, edgecolors='#b71c1c',
                   linewidths=0.8, label='End')

        # ── Label Start / End ──
        ax.annotate('Start', (0, 0), color='#1a1a1a', fontsize=9,
                    fontweight='bold', xytext=(8, -18),
                    textcoords='offset points',
                    path_effects=[pe.withStroke(linewidth=2,
                                               foreground='white')])
        ax.annotate('End', (norm_x[-1], norm_y[-1]), color='#1a1a1a',
                    fontsize=9, fontweight='bold', xytext=(-25, -18),
                    textcoords='offset points',
                    path_effects=[pe.withStroke(linewidth=2,
                                               foreground='white')])

        # ── Style ──
        ax.set_title(title, fontsize=11, fontweight='bold')
        ax.set_xlabel('x (m)', fontsize=10)
        ax.set_ylabel('y (m)', fontsize=10)
        ax.legend(fontsize=8, facecolor='white', labelcolor='#333333',
                  edgecolor='#cccccc', loc='upper left')
        ax.grid(True, color='#e0e0e0', linewidth=0.8, alpha=0.8)

        # ── ปรับ y-axis ให้เห็น deviation ชัด ──
        max_dev = max(abs(min(norm_y)), abs(max(norm_y)), 0.005)
        margin = max_dev * 1.5
        ax.set_ylim(-margin, margin)

        # ── สถิติ deviation ──
        avg_dev = sum(abs(y) for y in norm_y) / len(norm_y)
        max_dev_val = max(abs(y) for y in norm_y)
        dist_traveled = abs(end_x)
        stats = (
            f'{source_label}\n'
            f'ระยะทาง: {dist_traveled:.3f} m\n'
            f'Deviation Y: mean={avg_dev:.4f} m\n'
            f'Deviation Y: max={max_dev_val:.4f} m'
        )
        ax.text(0.98, 0.97, stats, transform=ax.transAxes,
                fontsize=8, color='#333333', va='top', ha='right',
                bbox=dict(facecolor='#f5f5f5', edgecolor='#cccccc',
                          boxstyle='round,pad=0.5', alpha=0.9))

    def _plot_2d_speed_map(self, ax):
        """
        แผนที่เสมือน 2D — เส้นทาง EKF ระบายสีตามความเร็ว (m/s)
        พร้อม Raw Odom ประกอบ และลูกศรทิศทางความเร็ว
        """
        self._style_ax(ax)

        if len(self.ekf_x) < 2 or not self.ekf_speed:
            ax.text(0.5, 0.5, 'ข้อมูลไม่เพียงพอ',
                    transform=ax.transAxes, ha='center', va='center')
            return

        # ── Raw Odom (พื้นหลัง) ─────────────────────────────────────
        ax.plot(self.raw_x, self.raw_y,
                color='#e65100', linewidth=1.0, alpha=0.35,
                linestyle='-', label='Raw Odometry', zorder=2)

        # ── EKF path ระบายสีตามความเร็ว ─────────────────────────────
        pts  = np.array([self.ekf_x, self.ekf_y]).T.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        spd  = np.array(self.ekf_speed)
        spd_mid = (spd[:-1] + spd[1:]) / 2

        vmax = max(spd) if max(spd) > 0 else 1.0
        norm = plt.Normalize(vmin=0, vmax=vmax)
        lc   = LineCollection(segs, cmap='RdYlGn_r', norm=norm,
                              linewidth=3.0, zorder=4, alpha=0.95)
        lc.set_array(spd_mid)
        ax.add_collection(lc)

        cbar = plt.colorbar(lc, ax=ax, fraction=0.025, pad=0.02)
        cbar.set_label('ความเร็ว (m/s)', fontsize=9, color='#333333')
        cbar.ax.tick_params(colors='#333333', labelsize=8)

        # ── ลูกศรทิศทาง (sampled) ─────────────────────────────────
        n = len(self.ekf_x)
        step = max(1, n // 20)
        for i in range(0, n - step, step):
            dx = self.ekf_x[i + step] - self.ekf_x[i]
            dy = self.ekf_y[i + step] - self.ekf_y[i]
            d  = math.sqrt(dx**2 + dy**2)
            if d > 0.01:
                s = self.ekf_speed[i]
                c = plt.cm.RdYlGn_r(norm(s))
                ax.annotate('', xy=(self.ekf_x[i] + dx * 0.6,
                                    self.ekf_y[i] + dy * 0.6),
                            xytext=(self.ekf_x[i], self.ekf_y[i]),
                            arrowprops=dict(arrowstyle='->', color=c,
                                           lw=1.2, alpha=0.75))

        # ── จุด Start / End ─────────────────────────────────────────
        ax.scatter(self.ekf_x[0], self.ekf_y[0],
                   color='#fdd835', s=150, zorder=7,
                   edgecolors='#f57f17', linewidths=1.5, label='Start')
        ax.scatter(self.ekf_x[-1], self.ekf_y[-1],
                   color='#d32f2f', s=180, marker='*', zorder=7,
                   edgecolors='#b71c1c', linewidths=0.8, label='End')
        ax.annotate('Start', (self.ekf_x[0], self.ekf_y[0]),
                    fontsize=9, fontweight='bold', color='#1a1a1a',
                    xytext=(8, 8), textcoords='offset points',
                    path_effects=[pe.withStroke(linewidth=2, foreground='white')])
        ax.annotate('End', (self.ekf_x[-1], self.ekf_y[-1]),
                    fontsize=9, fontweight='bold', color='#1a1a1a',
                    xytext=(-30, 8), textcoords='offset points',
                    path_effects=[pe.withStroke(linewidth=2, foreground='white')])

        # ── Stats box ───────────────────────────────────────────────
        mean_v = float(np.mean(spd))
        max_v  = float(np.max(spd))
        ax.text(0.01, 0.98,
                f'ความเร็ว EKF\nmean = {mean_v:.3f} m/s\nmax  = {max_v:.3f} m/s',
                transform=ax.transAxes, fontsize=9.5, color='#333333',
                va='top', ha='left',
                bbox=dict(facecolor='#f5f5f5', edgecolor='#cccccc',
                          boxstyle='round,pad=0.6', alpha=0.92))

        # ── ขอบเขตแกน ───────────────────────────────────────────────
        all_x = self.ekf_x + self.raw_x
        all_y = self.ekf_y + self.raw_y
        cx = (max(all_x) + min(all_x)) / 2
        cy = (max(all_y) + min(all_y)) / 2
        span   = max(max(all_x) - min(all_x), max(all_y) - min(all_y))
        margin = max(span * 0.15, 0.3)
        half   = span / 2 + margin
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)
        ax.set_aspect('equal')

        ax.set_title('เส้นทาง 2D บนแผนที่เสมือน — ระบายสีตามความเร็ว',
                     fontsize=12, fontweight='bold')
        ax.set_xlabel('X (m)', fontsize=10)
        ax.set_ylabel('Y (m)', fontsize=10)
        ax.legend(fontsize=9, facecolor='white', edgecolor='#cccccc')
        ax.grid(True, color='#e0e0e0', linewidth=0.8, alpha=0.7)

    def _plot_speed_profile(self, ax_top, ax_bot):
        """
        ความเร็วตามลำดับเวลา (2 subplot ซ้อนกัน):
          บน : Linear speed EKF vs Raw  +  ความเร็วไปยังเป้าหมาย (approach speed)
          ล่าง: Angular velocity (yaw rate)
        """
        for ax in (ax_top, ax_bot):
            self._style_ax(ax)

        idx_ekf = list(range(len(self.ekf_speed)))
        idx_raw = list(range(len(self.raw_speed)))

        # ── คำนวณ Approach Speed (ความเร็วเข้าหาจุดปลาย) ──────────────
        x_goal, y_goal = self.ekf_x[-1], self.ekf_y[-1]
        approach = []
        for i, (px, py, vx, vy) in enumerate(
                zip(self.ekf_x, self.ekf_y, self.ekf_vx, self.ekf_vy)):
            dx = x_goal - px
            dy = y_goal - py
            d  = math.sqrt(dx**2 + dy**2)
            if d > 1e-6:
                # projecting velocity vector onto direction to goal
                v_approach = (vx * dx + vy * dy) / d
            else:
                v_approach = 0.0
            approach.append(v_approach)

        # ── Plot บน: Linear Speed ────────────────────────────────────
        ax_top.plot(idx_ekf, self.ekf_speed,
                    color='#2e7d32', linewidth=1.8, label='EKF |v| (m/s)', zorder=4)
        ax_top.plot(idx_raw, self.raw_speed,
                    color='#e65100', linewidth=1.2, alpha=0.55,
                    linestyle='--', label='Raw |v| (m/s)', zorder=3)
        ax_top.plot(idx_ekf, approach,
                    color='#1565c0', linewidth=1.5, linestyle='-.',
                    label='Approach speed (m/s)', zorder=5)
        ax_top.axhline(0, color='#999999', linewidth=0.8, linestyle='--')

        mean_v = sum(self.ekf_speed) / len(self.ekf_speed) if self.ekf_speed else 0
        max_v  = max(self.ekf_speed) if self.ekf_speed else 0
        ax_top.text(0.99, 0.97,
                    f'mean={mean_v:.3f} m/s\nmax={max_v:.3f} m/s',
                    transform=ax_top.transAxes, fontsize=8.5,
                    va='top', ha='right', color='#333333',
                    bbox=dict(facecolor='#f5f5f5', edgecolor='#cccccc',
                              boxstyle='round,pad=0.4', alpha=0.9))
        ax_top.set_title('ความเร็วเชิงเส้น + Approach Speed', fontsize=11, fontweight='bold')
        ax_top.set_ylabel('m/s', fontsize=10)
        ax_top.legend(fontsize=8.5, facecolor='white', edgecolor='#cccccc')
        ax_top.grid(True, color='#e0e0e0', linewidth=0.8, alpha=0.7)
        ax_top.tick_params(labelbottom=False)

        # ── Plot ล่าง: Angular Velocity ──────────────────────────────
        ekf_omega_deg = [math.degrees(w) for w in self.ekf_omega]
        raw_omega_deg = [math.degrees(w) for w in self.raw_omega]
        ax_bot.plot(list(range(len(ekf_omega_deg))), ekf_omega_deg,
                    color='#6a1b9a', linewidth=1.8,
                    label='EKF ω (°/s)', zorder=4)
        ax_bot.plot(list(range(len(raw_omega_deg))), raw_omega_deg,
                    color='#ef6c00', linewidth=1.2, alpha=0.55,
                    linestyle='--', label='Raw ω (°/s)', zorder=3)
        ax_bot.axhline(0, color='#999999', linewidth=0.8, linestyle='--')

        ax_bot.set_title('ความเร็วเชิงมุม (Yaw Rate)', fontsize=11, fontweight='bold')
        ax_bot.set_xlabel('Sample', fontsize=10)
        ax_bot.set_ylabel('°/s', fontsize=10)
        ax_bot.legend(fontsize=8.5, facecolor='white', edgecolor='#cccccc')
        ax_bot.grid(True, color='#e0e0e0', linewidth=0.8, alpha=0.7)

    def _plot_2d_map_deviation(self, ax):
        """
        แผนที่เสมือน 2D — แสดงความคลาดเคลื่อนตำแหน่ง (Cross-Track Error)
        เปรียบเทียบเส้นทาง EKF จริง vs เส้นอ้างอิง (Start→End)

        - เส้นทาง EKF ระบายสีตามขนาด CTE (เขียว=น้อย, แดง=มาก)
        - เส้นทาง Raw Odom แสดงประกอบ (สีส้ม)
        - ลูกศรชี้จากตำแหน่งจริง → ตำแหน่งอ้างอิงบนเส้นตรง
        - Colorbar แสดงสเกล error
        """
        self._style_ax(ax)

        if len(self.ekf_x) < 2:
            ax.text(0.5, 0.5, 'ข้อมูลไม่เพียงพอ',
                    transform=ax.transAxes, ha='center', va='center')
            return

        # ── Reference line: เส้นตรง Start → End ─────────────────────────
        x0, y0 = self.ekf_x[0],  self.ekf_y[0]
        x1, y1 = self.ekf_x[-1], self.ekf_y[-1]
        dx = x1 - x0
        dy = y1 - y0
        L  = math.sqrt(dx**2 + dy**2)  # ระยะทางอ้างอิง

        # ── คำนวณ Cross-Track Error (ระยะตั้งฉากจากเส้นอ้างอิง) ──────────
        def _cte(px, py):
            """Signed cross-track error (+ = ซ้าย, - = ขวา)"""
            if L < 1e-9:
                return math.sqrt((px - x0)**2 + (py - y0)**2)
            return ((px - x0) * dy - (py - y0) * dx) / L

        cte_ekf = [_cte(px, py) for px, py in zip(self.ekf_x, self.ekf_y)]
        abs_cte = [abs(c) for c in cte_ekf]
        max_cte = max(abs_cte) if abs_cte else 1e-3
        mean_cte = sum(abs_cte) / len(abs_cte)
        rms_cte  = math.sqrt(sum(c**2 for c in abs_cte) / len(abs_cte))

        # ── วาด Raw Odom (พื้นหลัง) ─────────────────────────────────────
        ax.plot(self.raw_x, self.raw_y,
                color='#e65100', linewidth=1.2, alpha=0.4,
                linestyle='-', label='Raw Odometry', zorder=2)

        # ── วาด EKF path ระบายสีตาม CTE (LineCollection) ─────────────────
        pts     = np.array([self.ekf_x, self.ekf_y]).T.reshape(-1, 1, 2)
        segs    = np.concatenate([pts[:-1], pts[1:]], axis=1)
        cte_mid = np.array(abs_cte[:-1])

        norm = plt.Normalize(vmin=0, vmax=max_cte)
        lc   = LineCollection(segs, cmap='RdYlGn_r', norm=norm,
                              linewidth=2.8, zorder=4, alpha=0.95)
        lc.set_array(cte_mid)
        ax.add_collection(lc)

        cbar = plt.colorbar(lc, ax=ax, fraction=0.025, pad=0.02)
        cbar.set_label('Cross-Track Error (m)', fontsize=9, color='#333333')
        cbar.ax.tick_params(colors='#333333', labelsize=8)

        # ── วาด Reference line ────────────────────────────────────────────
        # ขยายเส้นอ้างอิงเล็กน้อยทั้งสองด้าน
        ext = L * 0.05
        ux, uy = (dx / (L + 1e-9)), (dy / (L + 1e-9))
        ax.annotate('',
            xy    =(x1 + ux * ext, y1 + uy * ext),
            xytext=(x0 - ux * ext, y0 - uy * ext),
            arrowprops=dict(arrowstyle='->', color='#1565c0',
                            lw=2.0, linestyle='dashed'))
        ax.plot([x0 - ux * ext, x1 + ux * ext],
                [y0 - uy * ext, y1 + uy * ext],
                color='#1565c0', linewidth=2.0, linestyle='--',
                label='Reference (เส้นตรงอ้างอิง)', zorder=3, alpha=0.85)

        # ── ลูกศรแสดง CTE ที่จุดสุ่ม ─────────────────────────────────────
        n_pts    = len(self.ekf_x)
        n_arrows = min(20, max(5, n_pts // 30))
        step     = max(1, n_pts // n_arrows)

        for i in range(0, n_pts, step):
            px, py = self.ekf_x[i], self.ekf_y[i]
            if L > 1e-9:
                t  = max(0.0, min(1.0,
                         ((px - x0) * dx + (py - y0) * dy) / (L * L)))
                rx = x0 + t * dx
                ry = y0 + t * dy
            else:
                rx, ry = x0, y0
            err = math.sqrt((px - rx)**2 + (py - ry)**2)
            if err > max_cte * 0.08:   # แสดงเฉพาะลูกศรที่มีนัยสำคัญ
                ax.annotate('',
                    xy    =(rx, ry),
                    xytext=(px, py),
                    arrowprops=dict(arrowstyle='->', color='#37474f',
                                   lw=0.9, alpha=0.55))

        # ── จุด Start / End ───────────────────────────────────────────────
        ax.scatter(x0, y0, color='#fdd835', s=150, zorder=7,
                   edgecolors='#f57f17', linewidths=1.5, label='Start')
        ax.scatter(x1, y1, color='#d32f2f', s=180, marker='*', zorder=7,
                   edgecolors='#b71c1c', linewidths=0.8, label='End (EKF)')
        ax.annotate('Start', (x0, y0), fontsize=9, fontweight='bold',
                    color='#1a1a1a', xytext=(8, 8), textcoords='offset points',
                    path_effects=[pe.withStroke(linewidth=2, foreground='white')])
        ax.annotate('End', (x1, y1), fontsize=9, fontweight='bold',
                    color='#1a1a1a', xytext=(-30, 8), textcoords='offset points',
                    path_effects=[pe.withStroke(linewidth=2, foreground='white')])

        # ── Stats box ────────────────────────────────────────────────────
        ax.text(0.01, 0.98,
                f'Cross-Track Error (CTE)\n'
                f'mean  = {mean_cte:.4f} m\n'
                f'max   = {max_cte:.4f} m\n'
                f'RMS   = {rms_cte:.4f} m\n'
                f'ระยะทางอ้างอิง = {L:.3f} m',
                transform=ax.transAxes, fontsize=9.5, color='#333333',
                va='top', ha='left',
                bbox=dict(facecolor='#f5f5f5', edgecolor='#cccccc',
                          boxstyle='round,pad=0.6', alpha=0.92))

        # ── ขอบเขตแกนที่เหมาะสม (equal aspect + margin) ─────────────────
        all_x = self.ekf_x + self.raw_x
        all_y = self.ekf_y + self.raw_y
        cx = (max(all_x) + min(all_x)) / 2
        cy = (max(all_y) + min(all_y)) / 2
        span   = max(max(all_x) - min(all_x), max(all_y) - min(all_y))
        margin = max(span * 0.15, 0.3)
        half   = span / 2 + margin
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)
        ax.set_aspect('equal')

        ax.set_title('ความคลาดเคลื่อนตำแหน่งบนแผนที่เสมือน 2D\n'
                     '(EKF Path vs Reference — ระบายสีตาม Cross-Track Error)',
                     fontsize=12, fontweight='bold')
        ax.set_xlabel('X (m)', fontsize=10)
        ax.set_ylabel('Y (m)', fontsize=10)
        ax.legend(fontsize=9, facecolor='white', edgecolor='#cccccc',
                  loc='upper right')
        ax.grid(True, color='#e0e0e0', linewidth=0.8, alpha=0.7)

    def plot(self):
        if not self.raw_x or not self.ekf_x:
            print('ไม่มีข้อมูล — ตรวจสอบว่า Topic ถูก Publish อยู่')
            return

        out_dir, timestamp = self.save_csv()

        fig = plt.figure(figsize=(22, 26))
        fig.patch.set_facecolor('white')
        gs = gridspec.GridSpec(4, 3, figure=fig,
                               hspace=0.42, wspace=0.32,
                               height_ratios=[1, 1, 1, 1.2])

        # ══════════════════════════════════════════════════════════
        # Plot 1 (บนซ้าย): เส้นทาง Raw vs EKF
        # ══════════════════════════════════════════════════════════
        ax = fig.add_subplot(gs[0, 0])
        self._style_ax(ax)

        ax.plot(self.raw_x, self.raw_y,
                color='#e65100', linewidth=1.8, alpha=0.85,
                label='Raw Odometry (/amr_mtt/odom)')
        ax.plot(self.ekf_x, self.ekf_y,
                color='#2e7d32', linewidth=2.2, alpha=0.95,
                label='EKF Filtered (/odometry/filtered)')

        # จุดเริ่มต้นและสิ้นสุด
        ax.scatter(self.raw_x[0],  self.raw_y[0],
                   color='#333333', s=80, zorder=5)
        ax.scatter(self.raw_x[-1], self.raw_y[-1],
                   color='#e65100', s=80, zorder=5, marker='x')
        ax.scatter(self.ekf_x[-1], self.ekf_y[-1],
                   color='#2e7d32', s=80, zorder=5, marker='x')

        ax.annotate('Start', (self.raw_x[0], self.raw_y[0]),
                    color='#333333', fontsize=8,
                    xytext=(8, 8), textcoords='offset points')

        ax.set_title('เส้นทางการเคลื่อนที่ของหุ่นยนต์', fontsize=12, fontweight='bold')
        ax.set_xlabel('X (m)', fontsize=10)
        ax.set_ylabel('Y (m)', fontsize=10)
        ax.legend(fontsize=9, facecolor='white', labelcolor='#333333',
                  edgecolor='#cccccc')
        ax.grid(True, color='#e0e0e0', linewidth=0.8, alpha=0.8)
        ax.set_aspect('equal')

        # ══════════════════════════════════════════════════════════
        # Plot 2 (บนกลาง): ความคลาดเคลื่อน X และ Y
        # ══════════════════════════════════════════════════════════
        ax2 = fig.add_subplot(gs[0, 1])
        self._style_ax(ax2)

        # คำนวณ error
        n = min(len(self.raw_x), len(self.ekf_x))
        err_x = [abs(self.raw_x[i] - self.ekf_x[i]) for i in range(n)]
        err_y = [abs(self.raw_y[i] - self.ekf_y[i]) for i in range(n)]
        steps = list(range(n))

        ax2.plot(steps, err_x, color='#c62828', linewidth=1.5,
                 label='|Error X| (m)', alpha=0.9)
        ax2.plot(steps, err_y, color='#00838f', linewidth=1.5,
                 label='|Error Y| (m)', alpha=0.9)

        ax2.set_title('ความคลาดเคลื่อนระหว่าง Raw vs EKF', fontsize=12, fontweight='bold')
        ax2.set_xlabel('Sample', fontsize=10)
        ax2.set_ylabel('Error (m)', fontsize=10)
        ax2.legend(fontsize=9, facecolor='white', labelcolor='#333333',
                   edgecolor='#cccccc')
        ax2.grid(True, color='#e0e0e0', linewidth=0.8, alpha=0.8)

        # สถิติสรุป
        avg_ex = sum(err_x) / len(err_x) if err_x else 0
        avg_ey = sum(err_y) / len(err_y) if err_y else 0
        max_ex = max(err_x) if err_x else 0
        max_ey = max(err_y) if err_y else 0

        stats_text = (
            f'สรุปสถิติ\n'
            f'Error X: mean={avg_ex:.4f} m  max={max_ex:.4f} m\n'
            f'Error Y: mean={avg_ey:.4f} m  max={max_ey:.4f} m\n'
            f'จำนวน Sample: Raw={len(self.raw_x)}  EKF={len(self.ekf_x)}'
        )
        ax2.text(0.02, 0.97, stats_text, transform=ax2.transAxes,
                 fontsize=8.5, color='#333333', va='top',
                 bbox=dict(facecolor='#f5f5f5', edgecolor='#cccccc',
                           boxstyle='round,pad=0.5', alpha=0.9))

        # ══════════════════════════════════════════════════════════
        # Plot 3 (บนขวา): กราฟวงกลม — ทิศทางหัว (Yaw Angle)
        # ══════════════════════════════════════════════════════════
        ax_polar = fig.add_subplot(gs[0, 2], projection='polar')
        ax_polar.set_facecolor('white')
        ax_polar.set_theta_zero_location('E')   # 0° = ทิศตะวันออก (เหมือน ROS)
        ax_polar.set_theta_direction(1)          # เพิ่มทวนเข็ม (CCW = บวก)

        n_raw = len(self.raw_yaw)
        n_ekf = len(self.ekf_yaw)
        r_raw = [i / max(n_raw - 1, 1) for i in range(n_raw)]
        r_ekf = [i / max(n_ekf - 1, 1) for i in range(n_ekf)]

        ax_polar.plot(self.raw_yaw, r_raw,
                      color='#e65100', linewidth=1.6, alpha=0.85,
                      label='Raw Odom')
        ax_polar.plot(self.ekf_yaw, r_ekf,
                      color='#2e7d32', linewidth=2.0, alpha=0.95,
                      label='EKF Filtered')

        # จุดเริ่มต้น (r=0) และจุดสุดท้าย
        ax_polar.scatter(self.raw_yaw[0],  0,   color='#333333',
                         s=60, zorder=5, label='Start')
        ax_polar.scatter(self.raw_yaw[-1], 1.0, color='#e65100',
                         s=80, marker='x', zorder=5)
        ax_polar.scatter(self.ekf_yaw[-1], 1.0, color='#2e7d32',
                         s=80, marker='x', zorder=5)

        ax_polar.set_title('ทิศทางหัวหุ่นยนต์ (Yaw)',
                            fontsize=11, fontweight='bold', pad=12)
        ax_polar.tick_params(colors='#555555', labelsize=8)
        ax_polar.set_rlabel_position(45)
        ax_polar.set_yticklabels(['0%', '25%', '50%', '75%', '100%'],
                                  fontsize=7, color='#777777')
        ax_polar.legend(fontsize=8, loc='lower left',
                         bbox_to_anchor=(-0.1, -0.12),
                         facecolor='white', edgecolor='#cccccc')

        # ══════════════════════════════════════════════════════════
        # Plot 4 (กลางซ้าย): Reference vs Raw Odom
        # ══════════════════════════════════════════════════════════
        self._plot_reference_vs_actual(
            fig.add_subplot(gs[1, 0]), self.raw_x, self.raw_y,
            'เส้นทางการเคลื่อนที่ — Raw Odometry',
            'Raw Odom')

        # ══════════════════════════════════════════════════════════
        # Plot 5 (กลางกลาง): Reference vs EKF Filtered
        # ══════════════════════════════════════════════════════════
        self._plot_reference_vs_actual(
            fig.add_subplot(gs[1, 1]), self.ekf_x, self.ekf_y,
            'เส้นทางการเคลื่อนที่ — EKF Filtered',
            'EKF Filtered')

        # ══════════════════════════════════════════════════════════
        # Plot 6 (กลางขวา): Error มุม Yaw Raw vs EKF
        # ══════════════════════════════════════════════════════════
        ax_yaw_err = fig.add_subplot(gs[1, 2])
        self._style_ax(ax_yaw_err)

        n_yaw = min(n_raw, n_ekf)
        err_yaw = [
            abs(math.degrees(self.raw_yaw[i]) - math.degrees(self.ekf_yaw[i]))
            for i in range(n_yaw)
        ]
        ax_yaw_err.plot(range(n_yaw), err_yaw,
                        color='#7b1fa2', linewidth=1.5, alpha=0.9)
        ax_yaw_err.fill_between(range(n_yaw), err_yaw,
                                 color='#7b1fa2', alpha=0.15)
        avg_yaw_err = sum(err_yaw) / len(err_yaw) if err_yaw else 0
        max_yaw_err = max(err_yaw) if err_yaw else 0
        ax_yaw_err.axhline(avg_yaw_err, color='#7b1fa2', linewidth=1.0,
                            linestyle='--', alpha=0.7,
                            label=f'Mean = {avg_yaw_err:.2f}°')
        ax_yaw_err.set_title('ความคลาดเคลื่อนมุม Yaw (Raw vs EKF)',
                              fontsize=11, fontweight='bold')
        ax_yaw_err.set_xlabel('Sample', fontsize=10)
        ax_yaw_err.set_ylabel('|Error Yaw| (°)', fontsize=10)
        ax_yaw_err.legend(fontsize=8, facecolor='white', labelcolor='#333333',
                          edgecolor='#cccccc')
        ax_yaw_err.grid(True, color='#e0e0e0', linewidth=0.8, alpha=0.8)
        ax_yaw_err.text(0.98, 0.97,
                         f'Mean = {avg_yaw_err:.2f}°\nMax = {max_yaw_err:.2f}°',
                         transform=ax_yaw_err.transAxes,
                         fontsize=8, color='#333333', va='top', ha='right',
                         bbox=dict(facecolor='#f5f5f5', edgecolor='#cccccc',
                                   boxstyle='round,pad=0.5', alpha=0.9))

        # ══════════════════════════════════════════════════════════
        # Plot 7 (row 2 — เต็มความกว้าง): 2D Map Deviation (CTE)
        # ══════════════════════════════════════════════════════════
        ax_map = fig.add_subplot(gs[2, :])
        self._plot_2d_map_deviation(ax_map)

        # ══════════════════════════════════════════════════════════
        # Plot 8-10 (row 3): Speed Map + Speed Profile
        # ══════════════════════════════════════════════════════════
        ax_spd_map = fig.add_subplot(gs[3, 0:2])   # 2D speed map (span 2 cols)
        self._plot_2d_speed_map(ax_spd_map)

        # Speed profile: แบ่ง 1 col ออกเป็น 2 subplot ซ้อนกัน
        gs_inner = gridspec.GridSpecFromSubplotSpec(
            2, 1, subplot_spec=gs[3, 2], hspace=0.35)
        ax_spd_top = fig.add_subplot(gs_inner[0])
        ax_spd_bot = fig.add_subplot(gs_inner[1])
        self._plot_speed_profile(ax_spd_top, ax_spd_bot)

        # ══════════════════════════════════════════════════════════

        fig.suptitle('ผลการเปรียบเทียบ Raw Odometry vs EKF Filtered — AMR MTT',
                     fontsize=14, color='#1a1a1a', fontweight='bold', y=1.01)

        out_file = os.path.join(out_dir, f'odom_comparison_{timestamp}.png')
        plt.savefig(out_file, dpi=180, bbox_inches='tight',
                    facecolor=fig.get_facecolor())
        print(f'บันทึกกราฟ: {out_file}')
        plt.show()

        # พิมพ์สรุป
        print('\n' + '='*50)
        print('สรุปผลการทดสอบ Odometry')
        print('='*50)
        print(f'Raw Odom  — จำนวน Sample : {len(self.raw_x)}')
        print(f'EKF Odom  — จำนวน Sample : {len(self.ekf_x)}')
        print(f'Error X   — mean: {avg_ex:.4f} m  |  max: {max_ex:.4f} m')
        print(f'Error Y   — mean: {avg_ey:.4f} m  |  max: {max_ey:.4f} m')
        print(f'Error Yaw — mean: {avg_yaw_err:.2f}°  |  max: {max_yaw_err:.2f}°')
        print('='*50)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nกำลัง Plot กราฟ...')
        node.plot()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
