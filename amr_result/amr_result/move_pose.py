#!/usr/bin/env python3
"""
move_pose.py — ทดสอบความแม่นยำการเคลื่อนที่แบบ Interactive (cmd_vel)

รองรับ 2 โหมด:
  1. เคลื่อนที่เป็นเส้นตรง (1-5 เมตร, เดินหน้า/ถอยหลัง)
  2. หมุนอยู่กับที่ (45°, 90°, 180°, 270°, 315°)

เมื่อถึงเป้าแสดงผลเปรียบเทียบ Raw Odom vs Diff-Drive vs EKF Filtered

วิธีใช้:
    ros2 run amr_result move_pose
"""

import math
import time
import os
import csv
import subprocess
import signal
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ── Config ───────────────────────────────────────────────────────
LINEAR_VEL   = 0.8       # m/s
ANGULAR_VEL  = 0.5       # rad/s
SETTLE_TIME  = 1.5       # วินาที รอหลังหยุด
VALID_DIST   = [1, 2, 3, 4, 5]
VALID_ANGLES = [45, 90, 180, 270, 315]
OUT_DIR      = os.path.expanduser('~/amr_mtt_results')
CMD_TOPIC    = '/diff_drive_controller/cmd_vel_unstamped'
BAG_DIR      = os.path.expanduser(
    '~/amr_mtt/src/amr_mtt/amr_result/amr_bag2_record')
BAG_TOPICS   = [
    '/amr_mtt/odom',
    '/odometry/filtered',
    '/diff_drive_controller/odom',
]


def euclidean(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def quaternion_to_yaw(qx, qy, qz, qw):
    """แปลง Quaternion เป็นมุม Yaw (rad)"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """ทำให้มุมอยู่ในช่วง [-π, π]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ── Node ─────────────────────────────────────────────────────────
class MovePoseCmdVel(Node):

    def __init__(self):
        super().__init__('move_pose_cmdvel')

        # Publisher
        self._cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)

        # ── Odom Storage (Position) ──────────────────────────────
        self._raw_x = self._raw_y = 0.0
        self._dc_x  = self._dc_y  = 0.0
        self._ekf_x = self._ekf_y = 0.0

        # ── Odom Storage (Yaw) ───────────────────────────────────
        self._raw_yaw = 0.0
        self._dc_yaw  = 0.0
        self._ekf_yaw = 0.0

        # ── Accumulated Rotation Tracking ────────────────────────
        self._tracking_rotation = False
        self._accum_raw_yaw = 0.0
        self._accum_dc_yaw  = 0.0
        self._accum_ekf_yaw = 0.0

        # ตำแหน่งเริ่มต้นของแต่ละรอบ (reset ก่อนวิ่งทุกครั้ง)
        self._start_raw_x = self._start_raw_y = 0.0
        self._start_dc_x  = self._start_dc_y  = 0.0
        self._start_ekf_x = self._start_ekf_y = 0.0

        self.create_subscription(
            Odometry, '/amr_mtt/odom', self._raw_cb, 10)
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self._dc_cb, 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._ekf_cb, 10)

        self._results = []
        self._rotation_results = []

        # ── Rosbag process ───────────────────────────────────────
        self._bag_proc = None
        self._bag_path = None

        self.get_logger().info('พร้อมรับคำสั่ง')

    # ── Callbacks ────────────────────────────────────────────────

    def _raw_cb(self, msg: Odometry):
        self._raw_x = msg.pose.pose.position.x
        self._raw_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        new_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        if self._tracking_rotation:
            delta = normalize_angle(new_yaw - self._raw_yaw)
            self._accum_raw_yaw += delta
        self._raw_yaw = new_yaw

    def _dc_cb(self, msg: Odometry):
        self._dc_x = msg.pose.pose.position.x
        self._dc_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        new_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        if self._tracking_rotation:
            delta = normalize_angle(new_yaw - self._dc_yaw)
            self._accum_dc_yaw += delta
        self._dc_yaw = new_yaw

    def _ekf_cb(self, msg: Odometry):
        self._ekf_x = msg.pose.pose.position.x
        self._ekf_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        new_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        if self._tracking_rotation:
            delta = normalize_angle(new_yaw - self._ekf_yaw)
            self._accum_ekf_yaw += delta
        self._ekf_yaw = new_yaw

    # ── Motion ───────────────────────────────────────────────────

    def _sim_time(self) -> float:
        """ROS sim time (วินาที) — ถูกต้องแม้ Gazebo RTF < 1.0"""
        return self.get_clock().now().nanoseconds * 1e-9

    def _spin(self, duration: float):
        """spin_once ตลอดช่วงเวลา duration วินาที"""
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _stop(self):
        self._cmd_pub.publish(Twist())
        self._spin(SETTLE_TIME)

    def _record_start(self):
        """บันทึกตำแหน่งเริ่มต้นก่อนวิ่ง"""
        self._spin(0.3)
        self._start_raw_x = self._raw_x
        self._start_raw_y = self._raw_y
        self._start_dc_x  = self._dc_x
        self._start_dc_y  = self._dc_y
        self._start_ekf_x = self._ekf_x
        self._start_ekf_y = self._ekf_y

    # ── เคลื่อนที่เป็นเส้นตรง ────────────────────────────────────

    def _drive(self, target_dist: float, reverse: bool = False) -> float:
        """
        วิ่งตรงจนระยะ Odom ครบ target_dist แล้วหยุด
        publish cmd_vel ทุก 0.05s (20 Hz) พร้อม spin รับ odom
        """
        direction = -1.0 if reverse else 1.0
        timeout   = target_dist / LINEAR_VEL * 6.0   # safety timeout x6

        twist = Twist()
        twist.linear.x = direction * LINEAR_VEL
        stop  = Twist()  # linear.x = 0

        # ใช้ตำแหน่งเริ่มต้นที่ _record_start() บันทึกไว้ (สอดคล้องกับ _show_result)
        start_x = self._start_raw_x
        start_y = self._start_raw_y

        print(f'  วิ่งจนครบ {target_dist} m  (timeout {timeout:.1f}s sim)')

        t0_sim        = self._sim_time()   # sim time — ใช้วัด elapsed
        t0_wall       = time.time()        # wall time — ใช้ควบคุม publish rate
        next_pub_time = t0_wall

        while True:
            now_wall = time.time()
            now_sim  = self._sim_time()

            # Publish ทุก 0.05s real time (20 Hz)
            if now_wall >= next_pub_time:
                self._cmd_pub.publish(twist)
                next_pub_time = now_wall + 0.05

            # รับ Odom
            rclpy.spin_once(self, timeout_sec=0.01)

            traveled = euclidean(self._raw_x, self._raw_y, start_x, start_y)

            if traveled >= target_dist:
                break
            if (now_sim - t0_sim) >= timeout:
                print('  ⚠  Timeout — หยุดกลางทาง')
                break

        drive_elapsed = self._sim_time() - t0_sim   # sim time elapsed
        self._cmd_pub.publish(stop)
        self._spin(SETTLE_TIME)
        return drive_elapsed

    # ── หมุนอยู่กับที่ ────────────────────────────────────────────

    def _rotate(self, target_deg: float) -> float:
        """
        หมุนตัวอยู่กับที่จนครบ target_deg องศา (ทวนเข็ม = บวก)
        ใช้ accumulated yaw จาก callbacks เพื่อวัดมุมที่หมุนจริง
        """
        target_rad = math.radians(abs(target_deg))
        timeout    = target_rad / ANGULAR_VEL * 6.0   # safety timeout x6

        twist = Twist()
        twist.angular.z = ANGULAR_VEL   # หมุนทวนเข็มนาฬิกา (CCW)
        stop = Twist()

        # รอ Odom อัปเดต
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        # Reset accumulated rotation & เริ่ม tracking
        self._accum_raw_yaw = 0.0
        self._accum_dc_yaw  = 0.0
        self._accum_ekf_yaw = 0.0
        self._tracking_rotation = True

        print(f'  หมุน {target_deg}°  (timeout {timeout:.1f}s sim)')

        t0_sim        = self._sim_time()
        t0_wall       = time.time()
        next_pub_time = t0_wall

        while True:
            now_wall = time.time()
            now_sim  = self._sim_time()

            if now_wall >= next_pub_time:
                self._cmd_pub.publish(twist)
                next_pub_time = now_wall + 0.05

            rclpy.spin_once(self, timeout_sec=0.01)

            if abs(self._accum_raw_yaw) >= target_rad:
                break
            if (now_sim - t0_sim) >= timeout:
                print('  ⚠  Timeout — หยุดกลางทาง')
                break

        drive_elapsed = self._sim_time() - t0_sim
        self._cmd_pub.publish(stop)
        self._spin(SETTLE_TIME)   # ยังคง tracking ระหว่าง settle

        self._tracking_rotation = False
        return drive_elapsed

    # ── แสดงผลเคลื่อนที่เส้นตรง ──────────────────────────────────

    def _show_result(self, target_dist: float, elapsed: float,
                     trial: int, reverse: bool):
        # ระยะที่วิ่งจริง (เทียบจากจุดเริ่มต้น)
        actual_raw = euclidean(
            self._raw_x, self._raw_y,
            self._start_raw_x, self._start_raw_y)
        actual_dc  = euclidean(
            self._dc_x,  self._dc_y,
            self._start_dc_x,  self._start_dc_y)
        actual_ekf = euclidean(
            self._ekf_x, self._ekf_y,
            self._start_ekf_x, self._start_ekf_y)

        theory = LINEAR_VEL * elapsed   # d = v × t

        err_raw = abs(actual_raw - target_dist)
        err_dc  = abs(actual_dc  - target_dist)
        err_ekf = abs(actual_ekf - target_dist)

        direction = 'ถอยหลัง' if reverse else 'เดินหน้า'

        print()
        print('╔════════════════════════════════════════════════════════════╗')
        print(f'║  ผลการทดสอบ  {direction}  เป้าหมาย {target_dist:.0f} m  ครั้งที่ {trial}')
        print('╠════════════════════════════════════════════════════════════╣')
        print(f'║  ความเร็ว      : {LINEAR_VEL} m/s')
        print(f'║  เวลาจริง      : {elapsed:.3f} s')
        print(f'║  d = v×t       : {theory:.4f} m  (ค่าทางทฤษฎี)')
        print('╠════════════════════════════════════════════════════════════╣')
        print(f'║  {"Topic":<30} {"ระยะจริง (m)":<14} {"Error (m)"}')
        print('║  ' + '─' * 56)
        print(f'║  /amr_mtt/odom            {actual_raw:<14.4f} {err_raw:.4f}')
        print(f'║  /diff_drive_controller   {actual_dc:<14.4f} {err_dc:.4f}')
        print(f'║  /odometry/filtered (EKF) {actual_ekf:<14.4f} {err_ekf:.4f}')
        print('╚════════════════════════════════════════════════════════════╝')

        self._results.append({
            'trial'      : trial,
            'direction'  : direction,
            'target_m'   : target_dist,
            'velocity'   : LINEAR_VEL,
            'time_sec'   : round(elapsed, 3),
            'theory_m'   : round(theory, 4),
            'raw_dist'   : round(actual_raw, 4),
            'err_raw'    : round(err_raw,  4),
            'dc_dist'    : round(actual_dc,  4),
            'err_dc'     : round(err_dc,   4),
            'ekf_dist'   : round(actual_ekf, 4),
            'err_ekf'    : round(err_ekf,  4),
        })

    # ── แสดงผลหมุนอยู่กับที่ ──────────────────────────────────────

    def _show_rotation_result(self, target_deg: float, elapsed: float,
                              trial: int):
        # มุมที่หมุนจริง (จาก accumulated tracking) แปลงเป็นองศา
        actual_raw_deg = math.degrees(abs(self._accum_raw_yaw))
        actual_dc_deg  = math.degrees(abs(self._accum_dc_yaw))
        actual_ekf_deg = math.degrees(abs(self._accum_ekf_yaw))

        theory_deg = math.degrees(ANGULAR_VEL * elapsed)  # θ = ω × t

        err_raw = abs(actual_raw_deg - target_deg)
        err_dc  = abs(actual_dc_deg  - target_deg)
        err_ekf = abs(actual_ekf_deg - target_deg)

        print()
        print('╔════════════════════════════════════════════════════════════╗')
        print(f'║  ผลการทดสอบ  หมุนอยู่กับที่  เป้าหมาย {target_deg:.0f}°  ครั้งที่ {trial}')
        print('╠════════════════════════════════════════════════════════════╣')
        print(f'║  ความเร็วเชิงมุม : {ANGULAR_VEL} rad/s')
        print(f'║  เวลาจริง        : {elapsed:.3f} s')
        print(f'║  θ = ω×t         : {theory_deg:.2f}°  (ค่าทางทฤษฎี)')
        print('╠════════════════════════════════════════════════════════════╣')
        print(f'║  {"Topic":<30} {"มุมจริง (°)":<14} {"Error (°)"}')
        print('║  ' + '─' * 56)
        print(f'║  /amr_mtt/odom            {actual_raw_deg:<14.2f} {err_raw:.2f}')
        print(f'║  /diff_drive_controller   {actual_dc_deg:<14.2f} {err_dc:.2f}')
        print(f'║  /odometry/filtered (EKF) {actual_ekf_deg:<14.2f} {err_ekf:.2f}')
        print('╚════════════════════════════════════════════════════════════╝')

        self._rotation_results.append({
            'trial'         : trial,
            'target_deg'    : target_deg,
            'angular_vel'   : ANGULAR_VEL,
            'time_sec'      : round(elapsed, 3),
            'theory_deg'    : round(theory_deg, 2),
            'raw_deg'       : round(actual_raw_deg, 2),
            'err_raw_deg'   : round(err_raw, 2),
            'dc_deg'        : round(actual_dc_deg, 2),
            'err_dc_deg'    : round(err_dc, 2),
            'ekf_deg'       : round(actual_ekf_deg, 2),
            'err_ekf_deg'   : round(err_ekf, 2),
        })

    # ── Rosbag ───────────────────────────────────────────────────

    def _start_bag(self):
        """เริ่ม ros2 bag record ใน background"""
        os.makedirs(BAG_DIR, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._bag_path = os.path.join(BAG_DIR, f'odom_{ts}')
        cmd = ['ros2', 'bag', 'record', '-o', self._bag_path] + BAG_TOPICS
        self._bag_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print(f'\n  [BAG] บันทึกเริ่มแล้ว → {self._bag_path}')
        print(f'  [BAG] Topics: {", ".join(BAG_TOPICS)}')

    def _stop_bag(self):
        """หยุด rosbag recording"""
        if self._bag_proc and self._bag_proc.poll() is None:
            self._bag_proc.send_signal(signal.SIGINT)
            self._bag_proc.wait(timeout=5)
            print(f'\n  [BAG] บันทึกหยุดแล้ว → {self._bag_path}')
        self._bag_proc = None

    # ── Loop หลัก ────────────────────────────────────────────────

    def run(self):
        os.makedirs(OUT_DIR, exist_ok=True)
        linear_trial   = 0
        rotation_trial = 0

        print()
        print('═══════════════════════════════════════════════════')
        print('   AMR MTT — Move Pose Test (cmd_vel)')
        print(f'   ความเร็วเชิงเส้น : {LINEAR_VEL} m/s')
        print(f'   ความเร็วเชิงมุม  : {ANGULAR_VEL} rad/s')
        print()
        print('   ── เคลื่อนที่เส้นตรง ──')
        print('   กรอก :  1-5         เดินหน้า N เมตร')
        print('           -1 ถึง -5   ถอยหลัง N เมตร')
        print()
        print('   ── หมุนอยู่กับที่ ──')
        print(f'   กรอก :  r45 r90 r180 r270 r315')
        print()
        print('   ── อื่นๆ ──')
        print('           bag         เริ่ม/หยุด บันทึก rosbag')
        print('           save        บันทึก CSV')
        print('           q           ออก')
        print('═══════════════════════════════════════════════════')

        while True:
            try:
                raw = input('\n>> กรอกคำสั่ง : ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if raw == 'q':
                self._stop_bag()
                break

            if raw == 'bag':
                if self._bag_proc and self._bag_proc.poll() is None:
                    self._stop_bag()
                else:
                    self._start_bag()
                continue

            if raw == 'save':
                self._save_csv()
                self._save_rotation_csv()
                continue

            # ── ตรวจสอบคำสั่งหมุน (r45, r90, ...) ────────────────
            if raw.startswith('r'):
                try:
                    angle = int(raw[1:])
                except ValueError:
                    print(f'  ⚠  รูปแบบไม่ถูก — ใช้: r45 r90 r180 r270 r315')
                    continue

                if angle not in VALID_ANGLES:
                    print(f'  ⚠  รองรับเฉพาะ {VALID_ANGLES} องศา')
                    continue

                rotation_trial += 1
                print(f'\n  [หมุน ครั้งที่ {rotation_trial}] หมุน {angle}° ...')

                elapsed = self._rotate(float(angle))
                self._show_rotation_result(float(angle), elapsed,
                                           rotation_trial)
                continue

            # ── ตรวจสอบคำสั่งเคลื่อนที่เส้นตรง ───────────────────
            try:
                dist_input = int(raw)
            except ValueError:
                print(f'  ⚠  กรอกตัวเลข {VALID_DIST} หรือ r45/r90/r180/r270/r315')
                continue

            abs_dist = abs(dist_input)
            if abs_dist not in VALID_DIST:
                print(f'  ⚠  รองรับเฉพาะ {VALID_DIST} เมตร (ใส่ - นำหน้าสำหรับถอยหลัง)')
                continue

            reverse = dist_input < 0
            linear_trial += 1
            direction_txt = 'ถอยหลัง' if reverse else 'เดินหน้า'

            print(f'\n  [ครั้งที่ {linear_trial}] {direction_txt} {abs_dist} m ...')

            self._record_start()
            elapsed = self._drive(float(abs_dist), reverse=reverse)
            self._show_result(float(abs_dist), elapsed, linear_trial, reverse)

        if self._results:
            self._save_csv()
        if self._rotation_results:
            self._save_rotation_csv()

    # ── บันทึก CSV เคลื่อนที่เส้นตรง ─────────────────────────────

    def _save_csv(self):
        if not self._results:
            print('  ยังไม่มีข้อมูลเคลื่อนที่เส้นตรง')
            return
        ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(OUT_DIR, f'move_pose_{ts}.csv')
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self._results[0].keys())
            writer.writeheader()
            writer.writerows(self._results)
        print(f'\n  บันทึก CSV → {path}')
        print(f'  จำนวนการทดสอบเส้นตรง : {len(self._results)} ครั้ง')

    # ── บันทึก CSV หมุนอยู่กับที่ ─────────────────────────────────

    def _save_rotation_csv(self):
        if not self._rotation_results:
            print('  ยังไม่มีข้อมูลหมุนอยู่กับที่')
            return
        ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(OUT_DIR, f'rotation_test_{ts}.csv')
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(
                f, fieldnames=self._rotation_results[0].keys())
            writer.writeheader()
            writer.writerows(self._rotation_results)
        print(f'\n  บันทึก CSV → {path}')
        print(f'  จำนวนการทดสอบหมุน : {len(self._rotation_results)} ครั้ง')


# ── Main ─────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MovePoseCmdVel()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_bag()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
