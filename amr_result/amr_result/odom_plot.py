#!/usr/bin/env python3
"""
odom_plot.py — วิเคราะห์และพล็อตกราฟ Odometry 5 ประเภทจาก rosbag2
===================================================================
กราฟที่ได้:
  1. X-Y Trajectory       — เส้นทางที่เดินจริงเปรียบเทียบ 3 sources
  2. Position Time Series  — X(t), Y(t) ตามเวลา
  3. Velocity Time Series  — linear.x(t), angular.z(t)
  4. Error Plot            — ความคลาดเคลื่อน diff_drive & EKF vs ground truth
  5. Yaw/Heading Plot      — มุมหัวหุ่นยนต์ตามเวลา

วิธีใช้:
  ros2 run amr_result odom_plot --ros-args -p bag_path:=<path_to_bag>

หรือรันตรง:
  python3 odom_plot.py <path_to_bag>
"""

import sys
import math
import os
from datetime import datetime

import matplotlib
matplotlib.use('Agg')   # ไม่ต้อง display — บันทึกไฟล์อย่างเดียว
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

# ── สี / label ─────────────────────────────────────────────────────────────
SOURCES = {
    '/amr_mtt/odom':               ('Ground Truth (Gazebo)', '#2196F3', '-'),
    '/diff_drive_controller/odom': ('Diff-Drive (Encoder)',  '#F44336', '--'),
    '/odometry/filtered':          ('EKF Filtered',          '#4CAF50', ':'),
}

OUT_DIR = os.path.expanduser('~/amr_mtt_results/plots')


# ── อ่าน bag ──────────────────────────────────────────────────────────────

def read_bag(bag_path: str) -> dict:
    """อ่าน bag แล้วคืน dict ของแต่ละ topic
    key = topic_name
    value = list of dict: {t, x, y, yaw, vx, wz}
    """
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {
        t.name: t.type
        for t in reader.get_all_topics_and_types()
    }

    data = {topic: [] for topic in SOURCES}
    t0 = None   # เวลาเริ่มต้น (normalize ให้เริ่มที่ 0)

    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if topic not in SOURCES:
            continue

        msg: Odometry = deserialize_message(raw, Odometry)
        t_sec = stamp_ns * 1e-9

        if t0 is None:
            t0 = t_sec

        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        data[topic].append({
            't':   t_sec - t0,
            'x':   msg.pose.pose.position.x,
            'y':   msg.pose.pose.position.y,
            'yaw': math.degrees(yaw),
            'vx':  msg.twist.twist.linear.x,
            'wz':  msg.twist.twist.angular.z,
        })

    return data


def to_arrays(records: list):
    """แปลง list of dict → numpy arrays"""
    if not records:
        return [np.array([])] * 6
    t   = np.array([r['t']   for r in records])
    x   = np.array([r['x']   for r in records])
    y   = np.array([r['y']   for r in records])
    yaw = np.array([r['yaw'] for r in records])
    vx  = np.array([r['vx']  for r in records])
    wz  = np.array([r['wz']  for r in records])
    return t, x, y, yaw, vx, wz


# ── Plot 1: X-Y Trajectory ────────────────────────────────────────────────

def plot_trajectory(data: dict, ax):
    ax.set_title('1. X-Y Trajectory', fontweight='bold')
    for topic, (label, color, ls) in SOURCES.items():
        if not data[topic]:
            continue
        _, x, y, *_ = to_arrays(data[topic])
        ax.plot(x, y, color=color, linestyle=ls, linewidth=2, label=label)
        ax.plot(x[0], y[0], 'o', color=color, markersize=8)   # start
        ax.plot(x[-1], y[-1], 's', color=color, markersize=8) # end

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='datalim')
    ax.annotate('Start', xy=(0, 0), fontsize=8, color='gray')


# ── Plot 2: Position Time Series ─────────────────────────────────────────

def plot_position(data: dict, ax_x, ax_y):
    for topic, (label, color, ls) in SOURCES.items():
        if not data[topic]:
            continue
        t, x, y, *_ = to_arrays(data[topic])
        ax_x.plot(t, x, color=color, linestyle=ls, linewidth=1.5, label=label)
        ax_y.plot(t, y, color=color, linestyle=ls, linewidth=1.5, label=label)

    ax_x.set_title('2. Position X(t)', fontweight='bold')
    ax_x.set_ylabel('X (m)')
    ax_x.legend(fontsize=8)
    ax_x.grid(True, alpha=0.3)

    ax_y.set_title('2. Position Y(t)', fontweight='bold')
    ax_y.set_xlabel('Time (s)')
    ax_y.set_ylabel('Y (m)')
    ax_y.legend(fontsize=8)
    ax_y.grid(True, alpha=0.3)


# ── Plot 3: Velocity Time Series ──────────────────────────────────────────

def plot_velocity(data: dict, ax_v, ax_w):
    for topic, (label, color, ls) in SOURCES.items():
        if not data[topic]:
            continue
        t, _, _, _, vx, wz = to_arrays(data[topic])
        ax_v.plot(t, vx, color=color, linestyle=ls, linewidth=1.5, label=label)
        ax_w.plot(t, wz, color=color, linestyle=ls, linewidth=1.5, label=label)

    ax_v.set_title('3. Linear Velocity Vx(t)', fontweight='bold')
    ax_v.set_ylabel('Vx (m/s)')
    ax_v.legend(fontsize=8)
    ax_v.grid(True, alpha=0.3)
    ax_v.axhline(y=0, color='gray', linewidth=0.5)

    ax_w.set_title('3. Angular Velocity Wz(t)', fontweight='bold')
    ax_w.set_xlabel('Time (s)')
    ax_w.set_ylabel('Wz (rad/s)')
    ax_w.legend(fontsize=8)
    ax_w.grid(True, alpha=0.3)
    ax_w.axhline(y=0, color='gray', linewidth=0.5)


# ── Plot 4: Error vs Ground Truth ─────────────────────────────────────────

def plot_error(data: dict, ax):
    gt = data['/amr_mtt/odom']
    if not gt:
        ax.set_title('4. Error (no ground truth)')
        return

    gt_t, gt_x, gt_y, *_ = to_arrays(gt)

    compare = {
        '/diff_drive_controller/odom': SOURCES['/diff_drive_controller/odom'],
        '/odometry/filtered':          SOURCES['/odometry/filtered'],
    }

    for topic, (label, color, ls) in compare.items():
        if not data[topic]:
            continue
        t, x, y, *_ = to_arrays(data[topic])

        # Interpolate ground truth ที่เวลาตรงกัน
        gt_x_interp = np.interp(t, gt_t, gt_x)
        gt_y_interp = np.interp(t, gt_t, gt_y)

        err = np.sqrt((x - gt_x_interp)**2 + (y - gt_y_interp)**2)
        ax.plot(t, err, color=color, linestyle=ls, linewidth=1.5, label=label)
        ax.fill_between(t, err, alpha=0.1, color=color)

    ax.set_title('4. Position Error vs Ground Truth', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (m)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='gray', linewidth=0.5)


# ── Plot 5: Yaw / Heading ─────────────────────────────────────────────────

def plot_yaw(data: dict, ax):
    for topic, (label, color, ls) in SOURCES.items():
        if not data[topic]:
            continue
        t, _, _, yaw, *_ = to_arrays(data[topic])
        ax.plot(t, yaw, color=color, linestyle=ls, linewidth=1.5, label=label)

    ax.set_title('5. Yaw / Heading (t)', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw (deg)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='gray', linewidth=0.5)


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    # รับ path จาก argument หรือ default
    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
    else:
        bag_path = os.path.expanduser(
            '~/amr_mtt/src/amr_mtt/amr_result/amr_bag2_record/odom_comparison')

    if not os.path.exists(bag_path):
        print(f'[ERROR] ไม่พบ bag: {bag_path}')
        sys.exit(1)

    print(f'[INFO] อ่าน bag: {bag_path}')
    data = read_bag(bag_path)

    for topic, records in data.items():
        print(f'  {topic}: {len(records)} messages')

    # ── Layout: 5 กราฟใน 1 figure ────────────────────────────────
    fig = plt.figure(figsize=(18, 14))
    fig.suptitle('AMR-MTT Odometry Comparison', fontsize=14, fontweight='bold')

    gs = gridspec.GridSpec(3, 4, figure=fig, hspace=0.45, wspace=0.35)

    ax_traj = fig.add_subplot(gs[0:2, 0:2])   # กราฟใหญ่ซ้าย
    ax_px   = fig.add_subplot(gs[0, 2:4])
    ax_py   = fig.add_subplot(gs[1, 2:4])
    ax_vx   = fig.add_subplot(gs[2, 0])
    ax_wz   = fig.add_subplot(gs[2, 1])
    ax_err  = fig.add_subplot(gs[2, 2])
    ax_yaw  = fig.add_subplot(gs[2, 3])

    plot_trajectory(data, ax_traj)
    plot_position(data, ax_px, ax_py)
    plot_velocity(data, ax_vx, ax_wz)
    plot_error(data, ax_err)
    plot_yaw(data, ax_yaw)

    # ── บันทึก ────────────────────────────────────────────────────
    os.makedirs(OUT_DIR, exist_ok=True)
    ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path = os.path.join(OUT_DIR, f'odom_comparison_{ts}.png')
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f'\n[INFO] บันทึกกราฟ → {out_path}')

    # แสดงผลด้วยถ้ามี display
    try:
        plt.show()
    except Exception:
        pass


if __name__ == '__main__':
    main()
