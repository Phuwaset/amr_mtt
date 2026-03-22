# Node-RED Architecture — AMR MTT

> **หมายเหตุ:** เอกสารนี้เขียนจากการวิเคราะห์ `amr_mtt_flows.json` โดยตรง (อัปเดตล่าสุด 2026-03-20)

---

## 1. ภาพรวมระบบ (System Overview)

Dashboard มี **1 UI Tab** ชื่อ **"🤖 AMR Control"** (`http://localhost:1880/ui`)
ภายในแบ่งเป็น **9 Section** เรียงตามลำดับ

```
Browser Dashboard (localhost:1880/ui)
         │
         ├── rosbridge WebSocket (ws://localhost:9090)   ← Section 0,1,2,4 + Map
         ├── arm_jog_node   HTTP REST (localhost:5005)   ← Section 5,6,7,8
         ├── nav_waypoint_server HTTP REST (localhost:5006) ← Section 9
         └── map_manager_node   HTTP REST (localhost:5007) ← Section SLAM & Map Manager
```

---

## 2. Flow Tab Structure

| Flow Tab (editor) | หน้าที่ |
|---|---|
| `amr-map-tab` (AMR Map View) | Subscribe `/map` + `/amcl_pose`, render SLAM canvas, Nav Waypoints |
| `7f25b3b6ca73783f` (🦾 UR5 Arm Control) | Safety, Joint control, Presets, Live feedback, Cartesian jog, Save/Load |

UI groups ทั้งหมดเชื่อมไปที่ UI tab เดียว (`arm-ui-tab` = "🤖 AMR Control")

---

## 3. Backend Systems (3 ระบบ)

| ระบบ | Port | Protocol | ใช้กับ |
|---|---|---|---|
| **rosbridge_suite** | 9090 | WebSocket | Joint sliders, Preset poses, `/joint_states` feedback, `/map`, `/amcl_pose` |
| **arm_jog_node** | 5005 | HTTP REST | EEF monitor, Cartesian jog, Save/Load 20 slots, HOME, Gripper (jog section) |
| **nav_waypoint_server** | 5006 | HTTP REST | บันทึก/ไปยัง/ลบ waypoint, cancel navigation |
| **map_manager_node** | 5007 | HTTP REST | Start/Stop SLAM, Save/Load map files |

---

## 4. Section Layout (ลำดับบน Dashboard)

### Section 0 — ⚠️ Safety

**กลุ่ม:** `arm-ui-group-safety` | **กว้าง:** 12 คอลัมน์

| Widget | ชื่อ Label | หน้าที่ |
|---|---|---|
| `ui_switch` | 🔒 ENABLE ARM CONTROL | เปิด/ปิด permission ส่งคำสั่งแขน — ต้องเปิดก่อน |
| `ui_text` | Status | แสดงสถานะปัจจุบัน (Arm enabled/disabled) |
| `ui_button` | 🚨 EMERGENCY STOP | ตัดทั้งระบบทันที — bypass guard, ส่ง STOP arm + cancel nav |
| `ui_button` | 🔓 RESET EMERGENCY | ล้าง emergency state — arm ยังปิด ต้อง Enable ใหม่ |
| `ui_template` | System Status / สถานะระบบ | Banner แสดง NORMAL (เขียว) หรือ EMERGENCY STOP ACTIVE (แดง กะพริบ) |

**Safety Logic:**
```
ARM GUARD function node ตรวจสอบก่อนส่งทุก command:
  1. if emergency_stop == true  → block (🚨 ทุก command ถูกตัด)
  2. if arm_enabled == false     → block (ARM DISABLED)
  3. else                        → pass through

ข้อยกเว้น: STOP และ EMERGENCY STOP bypass guard โดยตรง (ทำงานได้เสมอ)
```

**Emergency Stop Flow:**
```
🚨 Button pressed
    ↓ emg-stop-func (sets emergency_stop=true, arm_enabled=false)
    ├── Output 1 → rosbridge → STOP arm (hold current position)
    ├── Output 2 → POST localhost:5005/stop (arm_jog_node)
    ├── Output 3 → POST localhost:5006/cancel_nav (nav_waypoint_server)
    └── Output 4 → emg-alert-banner (แสดง banner แดงกะพริบ)

🔓 RESET pressed
    ↓ emg-reset-func (sets emergency_stop=false, arm_enabled ยัง false)
    └── Output 1 → emg-alert-banner (แสดง banner เขียว)
    (ต้องกด ENABLE ARM ใหม่เองเพื่อใช้งานต่อ)
```

---

### Section 1 — Joint Control (rad)

**กลุ่ม:** `arm-ui-group-joints` | **Protocol:** rosbridge WebSocket

| Widget | ชื่อ Label | หน้าที่ |
|---|---|---|
| `ui_slider` J1 | J1 Shoulder Pan | `ur5_shoulder_pan_joint` ±π rad |
| `ui_slider` J2 | J2 Shoulder Lift | `ur5_shoulder_lift_joint` ±π rad |
| `ui_slider` J3 | J3 Elbow | `ur5_elbow_joint` ±π rad |
| `ui_slider` J4 | J4 Wrist 1 | `ur5_wrist_1_joint` ±π rad |
| `ui_slider` J5 | J5 Wrist 2 | `ur5_wrist_2_joint` ±π rad |
| `ui_slider` J6 | J6 Wrist 3 | `ur5_wrist_3_joint` ±π rad |
| `ui_text` | Target Joint Positions (rad) | แสดง J1–J6 ที่ตั้งไว้ก่อนส่ง |
| `ui_button` | ▶ SEND TO ARM | ส่ง JointTrajectory ไปที่ `/ur_arm_controller/joint_trajectory` |
| `ui_button` | ⛔ STOP | หยุดแขน ณ ตำแหน่งปัจจุบัน (bypass guard) |
| `ui_slider` | Duration (sec) | กำหนดเวลาเคลื่อนที่ 0.5–5 วินาที |

**Flow:**
```
Slider (J1–J6) → Collect Joint Values → เก็บใน flow context
Duration Slider → Set Duration → เก็บใน flow context
SEND button → Build JointTrajectory → ARM GUARD → rosbridge → /ur_arm_controller/joint_trajectory
STOP button → Build STOP Command (hold position) → rosbridge (bypass guard)
```

---

### Section 2 — Preset Poses

**กลุ่ม:** `arm-ui-group-presets` | **Protocol:** rosbridge WebSocket

| ปุ่ม | Joint Values (rad) | Duration | ใช้เมื่อ |
|---|---|---|---|
| 🏠 HOME | `[0, -1.5708, -1.5708, -3.159, 0, 0]` | 2s | เริ่มต้น / reset |
| 📦 PRE_APPROACH | `[-1.3963, -2.2689, -1.1519, -1.2217, 1.5708, 0]` | 3s | เข้าหาวัตถุก่อน pick |
| ⬆ LIFT_UP | `[0, -2.7576, -1.5708, -3.3336, -1.5708, 0]` | 2s | หลัง grip แล้วยกขึ้น |
| 🔄 DROP_ROTATE | `[1.5708, -1.5708, -1.5708, -3.159, 0, 0]` | 2s | หมุนไปวางของ |
| ♻ SYNC SLIDERS | — | — | อัปเดต slider ให้ตรงกับ robot จริง |

กดปุ่ม preset → สร้าง JointTrajectory → ARM GUARD → rosbridge (ส่งทันที ไม่ต้องกด SEND)

**ลำดับ Pick & Place:**
```
HOME → PRE_APPROACH → [Gripper CLOSE] → LIFT_UP → DROP_ROTATE → [Gripper OPEN] → HOME
```
*(Gripper control อยู่ใน Cartesian Jog section)*

---

### Section 3 — Live Joint State

**กลุ่ม:** `arm-ui-group-feedback` | **Protocol:** rosbridge WebSocket (Subscribe)

- Subscribe `/joint_states` จาก rosbridge (auto-subscribe เมื่อ deploy)
- Parse ดึงเฉพาะ 6 joints ของ UR5
- แสดงค่า J1–J6 real-time (rad)
- เก็บ `feedback_positions` ไว้ใน flow context สำหรับ STOP command

---

### Section 4 — 🎯 EEF Position Monitor

**กลุ่ม:** `jog-group-monitor` | **Protocol:** HTTP GET localhost:5005/status ทุก **200ms**

แสดงข้อมูล End-Effector จาก `arm_jog_node`:
- **Position:** X / Y / Z (เมตร, arm frame)
- **Rotation:** Roll / Pitch / Yaw (องศา)
- **Joints:** J1–J6 (องศา)
- **Gripper:** 🟢 OPEN / 🔴 CLOSED + มุม (องศา)
- **Mode:** 🟢 MANUAL หรือ 🔴 LOCKED (+ แสดงเมื่อ sequence กำลังรัน)

---

### Section 5 — 🕹️ Cartesian Jog

**กลุ่ม:** `jog-group-jog` | **Protocol:** HTTP POST localhost:5005/jog

ควบคุมแขนแบบ Cartesian (เคลื่อน End-Effector ทีละ step):

| ปุ่ม | Payload | หน้าที่ |
|---|---|---|
| ⬆ Z+ Up | `{axis:'z', dir:1, step}` | ยกขึ้น |
| ⬇ Z- Down | `{axis:'z', dir:-1, step}` | กดลง |
| ▶ Y+ Fwd | `{axis:'y', dir:1, step}` | เดินหน้า |
| ◀ Y- Back | `{axis:'y', dir:-1, step}` | ถอยหลัง |
| X+ → | `{axis:'x', dir:1, step}` | เลื่อนขวา |
| ← X- | `{axis:'x', dir:-1, step}` | เลื่อนซ้าย |
| ↻ Wrist+ | `{axis:'wrist', dir:1, step:0.0873}` | หมุน wrist +5° |
| ↺ Wrist- | `{axis:'wrist', dir:-1, step:0.0873}` | หมุน wrist -5° |
| 🏠 HOME | POST /home | ส่งแขนกลับ HOME pose |
| ✋ Open | POST /gripper `{action:'open'}` | เปิด gripper |
| ✊ Close | POST /gripper `{action:'close'}` | ปิด gripper |

**Step Size:** dropdown เลือกได้ 5 / 10 / 20 / 50 / 100 mm

---

### Section 6 — 💾 Save / Load Position

**กลุ่ม:** `jog-group-saveload` | **Protocol:** HTTP localhost:5005

มี **20 slot** (Position 1–20) สำหรับจำตำแหน่ง joint:

| ปุ่ม | Endpoint | หน้าที่ |
|---|---|---|
| 💾 Save | POST /save_pos `{slot}` | บันทึก joint ปัจจุบันลง slot ที่เลือก |
| ▶ Go To | POST /goto_pos `{slot}` | ส่งแขนไปยัง slot ที่เลือก |

---

### Section 7 — 🗺️ 2D SLAM Map

**กลุ่ม:** `amr-map-group` | **Protocol:** rosbridge WebSocket

- Subscribe `/map` (nav_msgs/OccupancyGrid) — อัปเดตเมื่อแผนที่เปลี่ยน
- Subscribe `/amcl_pose` — อัปเดตตำแหน่ง robot real-time
- วาดบน **HTML5 Canvas** (ไม่ใช้ external map tiles)
- สี: ⬜ Free (220,220,220) / ⬛ Obstacle (30,30,40) / ▪ Unknown (128,128,128)
- Overlay: จุดสีเขียว + ลูกศรบอกทิศของ robot
- Overlay: waypoints จาก nav_waypoint_server (สีฟ้า/เหลือง)
- ปุ่ม 🔄 Reload Map: re-subscribe `/map`

---

### Section 8 — 📍 Nav Waypoints

**กลุ่ม:** `nav-wp-group` | **Protocol:** HTTP localhost:5006, poll ทุก **2 วินาที**

Panel HTML ภายใน:

| ปุ่ม/Widget | Endpoint | หน้าที่ |
|---|---|---|
| Input + Save Waypoint | POST /save_waypoint `{name}` | บันทึกตำแหน่ง AMCL ปัจจุบันพร้อมชื่อ |
| Go (ต่อ waypoint) | POST /goto_waypoint `{name}` | สั่ง Nav2 นำทางไปยัง waypoint นั้น |
| Del | DELETE /waypoint `{name}` | ลบ waypoint |
| Cancel Nav | POST /cancel_nav | ยกเลิก navigation ที่กำลังทำอยู่ |
| ตาราง waypoints | GET /status | แสดง X / Y / Yaw ของทุก waypoint |

แสดงสถานะ: `idle` / `navigating → <ชื่อ>` / `succeeded` / `failed`
ไฮไลต์แถวที่กำลังเดินไปสีเขียว

---

## 5. Data Flow Diagrams

### Flow A — Joint Control (rosbridge)

```
Sliders J1–J6 ──→ Collect Joint Values (flow context)
Duration Slider ──→ Set Duration (flow context)
SEND button ──→ Build JointTrajectory ──→ ARM GUARD ──→ WebSocket out ──→ /ur_arm_controller/joint_trajectory
STOP button ──→ Build STOP Command ──────────────────→ WebSocket out (bypass guard)
🚨 EMRG STOP ──→ emg-stop-func ──────────────────────→ WebSocket out (bypass guard)

WebSocket in (/joint_states) ──→ Parse /joint_states ──→ Live Joint State display
                                                       └──→ SYNC → update sliders
```

### Flow B — SLAM Map (rosbridge)

```
Deploy/Start (5s delay) ──→ Subscribe /map ──→ WebSocket out
                        └──→ Subscribe /amcl_pose

WebSocket in ──→ Route by Topic ──→ /map      → buildMapImage() → render() on Canvas
                               └──→ /amcl_pose → update robot.x/y/yaw → render()

Poll (3s) ──→ GET localhost:5006/status ──→ Format Waypoints ──→ overlay on Canvas
```

### Flow C — Cartesian Jog (arm_jog_node)

```
Step dropdown ──→ store jog_step in flow context
Jog buttons   ──→ {axis, dir, step} ──→ POST localhost:5005/jog ──→ arm_jog_node

Poll (200ms)  ──→ GET localhost:5005/status ──→ parse status ──→ EEF display widgets
```

---

## 6. Preset Poses Reference

| ท่า | J1 | J2 | J3 | J4 | J5 | J6 | Duration |
|---|---|---|---|---|---|---|---|
| HOME | 0.0 | -1.5708 | -1.5708 | -3.159 | 0.0 | 0.0 | 2s |
| PRE_APPROACH | -1.3963 | -2.2689 | -1.1519 | -1.2217 | 1.5708 | 0.0 | 3s |
| LIFT_UP | 0.0 | -2.7576 | -1.5708 | -3.3336 | -1.5708 | 0.0 | 2s |
| DROP_ROTATE | 1.5708 | -1.5708 | -1.5708 | -3.159 | 0.0 | 0.0 | 2s |

---

## 7. ROS 2 Topics ที่ Dashboard ใช้งาน

| Topic | Type | Direction | ใช้กับ |
|---|---|---|---|
| `/ur_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Publish | Joint control, Preset, STOP, Emergency |
| `/joint_states` | `sensor_msgs/JointState` | Subscribe | Live feedback, SYNC sliders |
| `/map` | `nav_msgs/OccupancyGrid` | Subscribe | SLAM Map canvas |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Subscribe | Robot position on map |

---

## 8. วิธีเปิดใช้งาน

### วิธีที่ 1 — คำสั่งเดียว (แนะนำ)

```bash
# เปิด backend ทั้งหมดพร้อมกัน (rosbridge + arm_jog + nav_waypoint + map_manager)
ros2 launch amr_mtt_task_planner node_red_backend.launch.py

# สำหรับ real robot (ไม่ใช้ sim time)
ros2 launch amr_mtt_task_planner node_red_backend.launch.py use_sim_time:=false
```

### วิธีที่ 2 — แยก terminal

```bash
# Terminal 1: rosbridge (สำหรับ joint control + map)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2: arm_jog_node (สำหรับ Cartesian jog + EEF monitor)
ros2 run amr_mtt_task_planner arm_jog_node

# Terminal 3: nav_waypoint_server (สำหรับ waypoints)
ros2 run amr_mtt_task_planner nav_waypoint_server

# Terminal 4: map_manager_node (สำหรับ SLAM + Save/Load map)
ros2 run amr_mtt_task_planner map_manager_node
```

```bash
# Terminal 5: Node-RED
node-red

# Browser
# http://localhost:1880      ← แก้ flow (import amr_mtt_flows.json ครั้งแรก)
# http://localhost:1880/ui   ← ใช้ dashboard
```

---

## 9. ลำดับการใช้งานที่แนะนำ (Startup Sequence)

### ลำดับปกติ (Navigation + Arm Control)
```
1. เปิด Gazebo simulation
2. ros2 launch amr_mtt_task_planner node_red_backend.launch.py
3. เปิด Node-RED → http://localhost:1880/ui
4. กด ♻ SYNC SLIDERS (sync slider ให้ตรงกับ robot จริง)
5. กด 🔒 ENABLE ARM CONTROL (เปิด permission)
6. ใช้งาน Preset Poses / Cartesian Jog ตามต้องการ
7. กด 🚨 EMERGENCY STOP เมื่อต้องการหยุดทันที
8. หลัง emergency: กด 🔓 RESET EMERGENCY → กด ENABLE ARM ใหม่
```

### ลำดับสำหรับสร้างแผนที่ใหม่ (SLAM Mapping)
```
1. เปิด Gazebo simulation
2. ros2 launch amr_mtt_task_planner node_red_backend.launch.py
3. เปิด Node-RED → http://localhost:1880/ui → กลุ่ม "🗺️ SLAM & Map Manager"
4. กด ▶ Start SLAM  (map_manager_node เปิด slam_toolbox อัตโนมัติ)
5. ขับหุ่นเพื่อสำรวจพื้นที่ — แผนที่จะปรากฏบน SLAM Map Canvas
6. พิมพ์ชื่อ map เช่น "warehouse_v1" → กด 💾 Save Map
   (ไฟล์ถูกบันทึกที่ ~/.ros/slam_maps/warehouse_v1.yaml)
7. กด ⏹ Stop SLAM เมื่อสร้างแผนที่เสร็จแล้ว
8. เปิด Nav2: ros2 launch amr_mtt_bot nav2.launch.py
9. เลือก map จาก dropdown → กด 📂 Load Map (โหลดเข้า Nav2 ทันที)
```

### Map Manager API (port 5007)
```
GET  /status               → { slam_running, nav_map, maps[] }
POST /start_slam           → เปิด slam_toolbox
POST /stop_slam            → ปิด slam_toolbox
POST /save_map  {name}     → บันทึก map → ~/.ros/slam_maps/<name>.yaml
GET  /maps                 → รายการ map (package + user-saved)
POST /load_map  {name}     → โหลด map เข้า Nav2 ผ่าน /map_server/load_map
```
