#!/usr/bin/env python3
"""
auto_system_test.py — Monitor & บันทึกผลระบบอัตโนมัติ AMR MTT หัวข้อ 4.7
  - ไม่ trigger task เอง — ให้ Node-RED กด Start ตามปกติ
  - แค่ poll /task_status แล้วจับเวลาแต่ละ phase
  - รัน N trial แล้วสร้าง CSV + กราฟ

วิธีใช้:
  Terminal: ros2 run amr_result auto_system_test
  จากนั้น กด Start ใน Node-RED ตามปกติ (script จะตรวจจับและเริ่มจับเวลาเอง)
"""

import os
import csv
import time
from datetime import datetime

import requests
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['font.family'] = 'Waree'

# ── Config ────────────────────────────────────────────────────────────────────
TASK_API  = 'http://localhost:5008'
N_TRIALS  = 1
POLL_SEC  = 0.5
TIMEOUT   = 700.0
OUT_DIR   = os.path.expanduser('~/amr_mtt_results')

PHASE_COLORS = {
    'NAV_PICK': '#3498db',
    'PICK':     '#27ae60',
    'NAV_DROP': '#e67e22',
    'DROP':     '#c0392b',
}

PHASE_KEYWORDS = [
    ('NAV_PICK', ['กำลังเดินทางไปยัง PICK', 'PICK_SPOT']),
    ('PICK',     ['กำลัง pick', 'pick →']),
    ('NAV_DROP', ['pick sequence เสร็จ', 'รอ 3', 'กำลังเดินทางผ่าน', 'กำลังเดินทางไปยัง DROP', 'DROP_SPOT']),
    ('DROP',     ['กำลัง drop', 'drop →']),
]
TERMINAL_KEYWORDS = ['completed', 'error', 'stopped']


# ── helpers ───────────────────────────────────────────────────────────────────
def get_status() -> dict:
    try:
        return requests.get(f'{TASK_API}/task_status', timeout=3).json()
    except Exception:
        return {'running': False, 'status': 'connection_error'}


def detect_phase(s: str) -> str:
    for phase, kws in PHASE_KEYWORDS:
        if any(k in s for k in kws):
            return phase
    return ''


# ── monitor one trial ─────────────────────────────────────────────────────────
def monitor_trial(trial_no: int) -> dict:
    result = {
        'trial':        trial_no,
        'NAV_PICK':     0.0,
        'PICK':         0.0,
        'NAV_DROP':     0.0,
        'DROP':         0.0,
        'total_time':   0.0,
        'overall_ok':   False,
        'final_status': '',
    }

    # รอให้ running = True (Node-RED กด Start)
    print(f'\n  [Trial {trial_no}] รอ Node-RED กด Start...')
    while True:
        s = get_status()
        if s.get('status') == 'connection_error':
            print('  ERROR: task_config_server ไม่ตอบสนอง')
            return result
        if s.get('running'):
            print(f'  ตรวจพบ task กำลังทำงาน! เริ่มจับเวลา...')
            break
        time.sleep(POLL_SEC)

    total_t0    = time.time()
    phase_t0    = total_t0
    cur_phase   = ''
    phase_times = {p: 0.0 for p in PHASE_COLORS}

    while True:
        elapsed = time.time() - total_t0
        if elapsed > TIMEOUT:
            print('  TIMEOUT')
            result['final_status'] = 'timeout'
            break

        s          = get_status()
        status_str = s.get('status', '')

        # detect phase เปลี่ยน
        new_phase = detect_phase(status_str)
        if new_phase and new_phase != cur_phase:
            if cur_phase in phase_times:
                phase_times[cur_phase] += time.time() - phase_t0
            cur_phase = new_phase
            phase_t0  = time.time()
            print(f'    --> [{new_phase}]  {status_str[:70]}')

        # terminal
        if any(kw in status_str for kw in TERMINAL_KEYWORDS):
            if cur_phase in phase_times:
                phase_times[cur_phase] += time.time() - phase_t0
            result['overall_ok']   = 'completed' in status_str
            result['final_status'] = status_str
            ok = 'SUCCESS' if result['overall_ok'] else 'FAILED'
            print(f'\n  Trial {trial_no} --> {ok}')
            break

        # task หยุดทำงาน (running=False) โดยไม่มี terminal keyword
        if not s.get('running', True) and cur_phase:
            if cur_phase in phase_times:
                phase_times[cur_phase] += time.time() - phase_t0
            result['final_status'] = status_str
            result['overall_ok']   = False
            print(f'\n  Trial {trial_no} --> task หยุดโดยไม่ทราบสาเหตุ')
            break

        time.sleep(POLL_SEC)

    result['total_time'] = round(time.time() - total_t0, 2)
    for p in PHASE_COLORS:
        result[p] = round(phase_times[p], 2)
    return result


# ── main ──────────────────────────────────────────────────────────────────────
def run():
    os.makedirs(OUT_DIR, exist_ok=True)

    # ตรวจ server
    if get_status().get('status') == 'connection_error':
        print('\n  ERROR: task_config_server ไม่ตอบสนอง (port 5008)')
        print('  ตรวจสอบว่า node_red_backend.launch.py รันอยู่')
        return

    print('\n' + '═'*65)
    print('   AMR MTT — Autonomous System Monitor  (หัวข้อ 4.7)')
    print(f'   จำนวน Trial  : {N_TRIALS}')
    print('   Pipeline     : NAV_PICK → PICK → NAV_DROP → DROP')
    print('   วิธีใช้       : กด Start ใน Node-RED ได้เลย script จะจับเวลาเอง')
    print('═'*65)

    all_results = []

    for t in range(1, N_TRIALS + 1):
        print(f'\n{"─"*65}')
        print(f'  Trial {t}/{N_TRIALS}  — กด Start ใน Node-RED เพื่อเริ่ม')
        print(f'{"─"*65}')
        r = monitor_trial(t)
        all_results.append(r)

        if t < N_TRIALS:
            # รอให้ running กลับ False ก่อน trial ถัดไป
            print('  รอ task เสร็จสมบูรณ์...')
            for _ in range(40):
                if not get_status().get('running', True):
                    break
                time.sleep(1.0)
            print(f'  พร้อม Trial {t+1} แล้ว\n')

    _print_table(all_results)
    _save_csv(all_results)
    _plot(all_results)


# ── print table ───────────────────────────────────────────────────────────────
def _print_table(results):
    sep = '╠═══════╬══════════╬══════════╬══════════╬══════════╬══════════╬══════════╣'
    print('\n' + '╔═══════╦══════════╦══════════╦══════════╦══════════╦══════════╦══════════╗')
    print('║ Trial ║NAV_PICK  ║  PICK    ║NAV_DROP  ║  DROP    ║  Total   ║  Result  ║')
    print(sep)
    for r in results:
        ok = 'SUCCESS' if r['overall_ok'] else 'FAILED'
        print(f"║  {r['trial']:^5}║ {r['NAV_PICK']:^8.2f} ║ {r['PICK']:^8.2f} ║"
              f" {r['NAV_DROP']:^8.2f} ║ {r['DROP']:^8.2f} ║"
              f" {r['total_time']:^8.2f} ║ {ok:^8} ║")
        print(sep)
    if results:
        avg   = lambda k: sum(r[k] for r in results) / len(results)
        ok_n  = sum(1 for r in results if r['overall_ok'])
        print(f"║ {'AVG':^5} ║ {avg('NAV_PICK'):^8.2f} ║ {avg('PICK'):^8.2f} ║"
              f" {avg('NAV_DROP'):^8.2f} ║ {avg('DROP'):^8.2f} ║"
              f" {avg('total_time'):^8.2f} ║ {ok_n}/{len(results)} OK   ║")
    print('╚═══════╩══════════╩══════════╩══════════╩══════════╩══════════╩══════════╝')


# ── save csv ──────────────────────────────────────────────────────────────────
def _save_csv(results):
    ts    = datetime.now().strftime('%Y%m%d_%H%M%S')
    path  = os.path.join(OUT_DIR, f'auto_system_test_{ts}.csv')
    fields = ['trial', 'NAV_PICK', 'PICK', 'NAV_DROP', 'DROP',
              'total_time', 'overall_ok', 'final_status']
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in results:
            w.writerow({k: r[k] for k in fields})
    print(f'\n  บันทึก CSV --> {path}')


# ── plot ──────────────────────────────────────────────────────────────────────
def _plot(results):
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    fig, axes = plt.subplots(1, 3, figsize=(17, 6))
    fig.suptitle(
        'AMR MTT — Autonomous System Test (หัวข้อ 4.7)\n'
        'ผลการทดสอบระบบอัตโนมัติ  NAV_PICK > PICK > NAV_DROP > DROP',
        fontsize=12, y=1.01
    )

    phases = list(PHASE_COLORS.keys())
    colors = list(PHASE_COLORS.values())
    x      = np.arange(len(results))
    w      = 0.18
    labels = [f"Trial {r['trial']}" for r in results]

    # (A) Grouped bar
    ax = axes[0]
    for j, (ph, c) in enumerate(zip(phases, colors)):
        vals   = [r[ph] for r in results]
        offset = (j - 1.5) * w
        bars   = ax.bar(x + offset, vals, w, label=ph, color=c, alpha=0.85)
        for bar in bars:
            if bar.get_height() > 0.5:
                ax.text(bar.get_x() + bar.get_width()/2,
                        bar.get_height() + 0.3,
                        f'{bar.get_height():.1f}',
                        ha='center', va='bottom', fontsize=7)
    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel('เวลา (s)', fontsize=10)
    ax.set_title('(A)  เวลาแต่ละ Phase ต่อ Trial', fontsize=10, pad=8)
    ax.legend(fontsize=8, framealpha=0.8)
    ax.grid(True, axis='y', linestyle=':', alpha=0.45)

    # (B) Stacked bar
    ax      = axes[1]
    bottoms = np.zeros(len(results))
    for ph, c in zip(phases, colors):
        vals = np.array([r[ph] for r in results])
        ax.bar(x, vals, 0.5, bottom=bottoms, color=c, alpha=0.85, label=ph)
        bottoms += vals
    max_total = max(r['total_time'] for r in results) if results else 10
    for i, r in enumerate(results):
        ax.text(x[i], r['total_time'] + max_total * 0.02,
                f"{r['total_time']:.1f}s",
                ha='center', va='bottom', fontsize=9, fontweight='bold')
        ok_str = 'SUCCESS' if r['overall_ok'] else 'FAILED'
        ok_clr = '#27ae60' if r['overall_ok'] else '#e74c3c'
        ax.text(x[i], max_total * 0.05, ok_str, ha='center', va='bottom',
                fontsize=9, color=ok_clr, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.7))
    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylim(0, max_total * 1.18)
    ax.set_ylabel('เวลารวม (s)', fontsize=10)
    ax.set_title('(B)  เวลารวม + ผลลัพธ์', fontsize=10, pad=8)
    ax.legend(fontsize=8, framealpha=0.8)
    ax.grid(True, axis='y', linestyle=':', alpha=0.45)

    # (C) Pie
    ax        = axes[2]
    avg_times = [sum(r[ph] for r in results)/len(results) for ph in phases]
    pie_labels= [f'{ph}\n{v:.1f}s' for ph, v in zip(phases, avg_times)]
    _, _, autotexts = ax.pie(
        avg_times, labels=pie_labels, colors=colors,
        autopct='%1.1f%%', startangle=90,
        wedgeprops=dict(edgecolor='white', linewidth=1.5),
        textprops={'fontsize': 8.5}
    )
    for at in autotexts:
        at.set_fontsize(9); at.set_fontweight('bold')
    ok_n      = sum(1 for r in results if r['overall_ok'])
    avg_total = sum(r['total_time'] for r in results) / len(results)
    ax.set_title(
        f'(C)  สัดส่วนเวลาเฉลี่ย (รวม {avg_total:.1f}s)\n'
        f'Success rate: {ok_n}/{len(results)} trials',
        fontsize=10, pad=8
    )

    plt.tight_layout()
    path = os.path.join(OUT_DIR, f'auto_system_test_{ts}.png')
    fig.savefig(path, dpi=180, bbox_inches='tight')
    plt.close(fig)
    print(f'  บันทึก Plot  --> {path}')


def main(args=None):
    run()


if __name__ == '__main__':
    main()
