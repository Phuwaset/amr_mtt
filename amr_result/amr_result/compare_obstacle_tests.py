#!/usr/bin/env python3
"""
compare_obstacle_tests.py — เปรียบเทียบผล Obstacle Avoidance 2 สถานการณ์
ใช้ข้อมูลจาก PNG และ CSV ที่มีอยู่แล้ว ไม่ต้องรันใหม่

วิธีใช้:
    ros2 run amr_result compare_obstacle_tests \
        --s1 obstacle_avoidance_20260325_173609 \
        --s2 obstacle_avoidance_20260325_180242 \
        --png1 obstacle_avoidance_20260325_173610 \
        --png2 obstacle_avoidance_20260325_180243 \
        --label1 "สถานการณ์ที่ 1" \
        --label2 "สถานการณ์ที่ 2"
"""

import argparse
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from datetime import datetime

plt.rcParams['font.family'] = 'Waree'

OUT_DIR          = os.path.expanduser('~/amr_mtt_results')
COLOR_S1         = '#e74c3c'
COLOR_S2         = '#2980b9'
SAFETY_THRESHOLD = 0.30


def _load_csv(name):
    path = os.path.join(OUT_DIR, f'{name}.csv')
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            rows.append({
                'waypoint'       : row['waypoint'],
                'result'         : row['result'],
                'time_s'         : float(row['time_s']),
                'actual_dist_m'  : float(row['actual_dist_m']),
                'min_clearance_m': float(row['min_clearance_m']),
            })
    return rows


def _crop_trajectory(img, col=0, total_cols=3):
    """Crop subplot ซ้าย (trajectory) จาก PNG ที่มี 3 subplots เท่ากัน"""
    h, w = img.shape[:2]
    col_w = w // total_cols
    x0 = col * col_w
    x1 = x0 + col_w
    return img[:, x0:x1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--s1',     required=True)
    parser.add_argument('--s2',     required=True)
    parser.add_argument('--png1',   default=None,
                        help='ชื่อไฟล์ PNG สถานการณ์ 1 (ถ้าต่างจาก s1)')
    parser.add_argument('--png2',   default=None,
                        help='ชื่อไฟล์ PNG สถานการณ์ 2 (ถ้าต่างจาก s2)')
    parser.add_argument('--label1', default='สถานการณ์ที่ 1')
    parser.add_argument('--label2', default='สถานการณ์ที่ 2')
    args = parser.parse_args()

    png1_name = args.png1 or args.s1
    png2_name = args.png2 or args.s2

    d1 = _load_csv(args.s1)
    d2 = _load_csv(args.s2)

    img1 = mpimg.imread(os.path.join(OUT_DIR, f'{png1_name}.png'))
    img2 = mpimg.imread(os.path.join(OUT_DIR, f'{png2_name}.png'))

    traj1 = _crop_trajectory(img1, col=0)
    traj2 = _crop_trajectory(img2, col=0)

    wp_names = [r['waypoint'] for r in d1]
    x        = np.arange(len(wp_names))
    width    = 0.35

    # ── Layout: 2 แถว ──────────────────────────────────────────────────────
    # แถวบน: trajectory ทั้ง 2 สถานการณ์ side-by-side (crop จาก PNG เดิม)
    # แถวล่าง: grouped bar เวลา | grouped bar ระยะทาง | clearance
    fig = plt.figure(figsize=(20, 12))
    fig.suptitle(
        f'AMR MTT — เปรียบเทียบ Obstacle Avoidance\n{args.label1} vs {args.label2}',
        fontsize=14, y=0.98
    )

    gs = fig.add_gridspec(2, 3, hspace=0.40, wspace=0.35,
                          height_ratios=[1.1, 1.0])

    # ─── แถวบน: Trajectory ───────────────────────────────────────────────
    ax_t1 = fig.add_subplot(gs[0, 0])
    ax_t1.imshow(traj1)
    ax_t1.axis('off')
    ax_t1.set_title(f'เส้นทาง — {args.label1}', fontsize=11,
                    color=COLOR_S1, fontweight='bold')

    ax_t2 = fig.add_subplot(gs[0, 1])
    ax_t2.imshow(traj2)
    ax_t2.axis('off')
    ax_t2.set_title(f'เส้นทาง — {args.label2}', fontsize=11,
                    color=COLOR_S2, fontweight='bold')

    # Success rate summary (แถวบนขวา)
    ax_sr = fig.add_subplot(gs[0, 2])
    ok1   = sum(1 for r in d1 if r['result'] == 'สำเร็จ')
    ok2   = sum(1 for r in d2 if r['result'] == 'สำเร็จ')
    n     = len(d1)
    bars  = ax_sr.bar(
        [args.label1, args.label2],
        [ok1/n*100, ok2/n*100],
        color=[COLOR_S1, COLOR_S2], alpha=0.85, width=0.4
    )
    for bar, ok in zip(bars, [ok1, ok2]):
        ax_sr.text(bar.get_x() + bar.get_width()/2,
                   bar.get_height() + 1,
                   f'{ok}/{n}\n({ok/n*100:.0f}%)',
                   ha='center', va='bottom', fontsize=11, fontweight='bold')
    ax_sr.set_ylim(0, 120)
    ax_sr.set_ylabel('อัตราความสำเร็จ (%)')
    ax_sr.set_title('อัตราความสำเร็จ', fontsize=11)
    ax_sr.axhline(100, color='gray', linestyle='--', alpha=0.4)
    ax_sr.grid(True, linestyle=':', alpha=0.5, axis='y')

    # ─── แถวล่าง: Bar Charts ────────────────────────────────────────────
    # เวลา
    ax_time = fig.add_subplot(gs[1, 0])
    t1 = [r['time_s'] for r in d1]
    t2 = [r['time_s'] for r in d2]
    b1 = ax_time.bar(x - width/2, t1, width, color=COLOR_S1,
                     alpha=0.85, label=args.label1)
    b2 = ax_time.bar(x + width/2, t2, width, color=COLOR_S2,
                     alpha=0.85, label=args.label2)
    for bar in list(b1) + list(b2):
        h = bar.get_height()
        ax_time.text(bar.get_x() + bar.get_width()/2, h + 1,
                     f'{h:.0f}', ha='center', va='bottom', fontsize=7)
    ax_time.set_xlabel('Waypoint')
    ax_time.set_ylabel('เวลา (s)')
    ax_time.set_title('เวลาที่ใช้ต่อ Waypoint')
    ax_time.set_xticks(x)
    ax_time.set_xticklabels(wp_names)
    ax_time.legend(fontsize=8)
    ax_time.grid(True, linestyle=':', alpha=0.5, axis='y')

    # ระยะทาง
    ax_dist = fig.add_subplot(gs[1, 1])
    a1 = [r['actual_dist_m'] for r in d1]
    a2 = [r['actual_dist_m'] for r in d2]
    b1 = ax_dist.bar(x - width/2, a1, width, color=COLOR_S1,
                     alpha=0.85, label=args.label1)
    b2 = ax_dist.bar(x + width/2, a2, width, color=COLOR_S2,
                     alpha=0.85, label=args.label2)
    for bar in list(b1) + list(b2):
        h = bar.get_height()
        ax_dist.text(bar.get_x() + bar.get_width()/2, h + 0.05,
                     f'{h:.2f}', ha='center', va='bottom', fontsize=7)
    ax_dist.set_xlabel('Waypoint')
    ax_dist.set_ylabel('ระยะทาง (m)')
    ax_dist.set_title('ระยะทางจริงต่อ Waypoint')
    ax_dist.set_xticks(x)
    ax_dist.set_xticklabels(wp_names)
    ax_dist.legend(fontsize=8)
    ax_dist.grid(True, linestyle=':', alpha=0.5, axis='y')

    # Min Clearance
    ax_clr = fig.add_subplot(gs[1, 2])
    c1 = [r['min_clearance_m'] for r in d1]
    c2 = [r['min_clearance_m'] for r in d2]
    b1 = ax_clr.bar(x - width/2, c1, width, color=COLOR_S1,
                    alpha=0.85, label=args.label1)
    b2 = ax_clr.bar(x + width/2, c2, width, color=COLOR_S2,
                    alpha=0.85, label=args.label2)
    for bar in list(b1) + list(b2):
        h = bar.get_height()
        if h > 0:
            ax_clr.text(bar.get_x() + bar.get_width()/2, h + 0.003,
                        f'{h:.3f}', ha='center', va='bottom', fontsize=7)
    ax_clr.axhline(SAFETY_THRESHOLD, color='red', linestyle='--',
                   linewidth=1.2, label=f'Safety threshold ({SAFETY_THRESHOLD} m)')
    ax_clr.set_xlabel('Waypoint')
    ax_clr.set_ylabel('Min Clearance (m)')
    ax_clr.set_title('ระยะห่างสิ่งกีดขวางต่ำสุด')
    ax_clr.set_xticks(x)
    ax_clr.set_xticklabels(wp_names)
    ax_clr.legend(fontsize=7)
    ax_clr.grid(True, linestyle=':', alpha=0.5, axis='y')

    # ── Save ────────────────────────────────────────────────────────────────
    ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
    out = os.path.join(OUT_DIR, f'obstacle_compare_{ts}.png')
    fig.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'บันทึก → {out}')

    # ── Summary ─────────────────────────────────────────────────────────────
    for label, data in [(args.label1, d1), (args.label2, d2)]:
        ok      = sum(1 for r in data if r['result'] == 'สำเร็จ')
        total_t = sum(r['time_s'] for r in data if r['result'] == 'สำเร็จ')
        total_d = sum(r['actual_dist_m'] for r in data)
        print(f'{label}: {ok}/{len(data)} สำเร็จ | เวลารวม {total_t:.1f}s | ระยะรวม {total_d:.3f}m')


if __name__ == '__main__':
    main()
