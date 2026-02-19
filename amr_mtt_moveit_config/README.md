# AMR-MTT MoveIt Configuration Package
## Mechatronics Engineering Analysis & Operational Guide

### 1. System Overview (บทนำและหลักการทำงาน)
Package **`amr_mtt_moveit_config`** นี้ทำหน้าที่เป็น **High-Level Motion Planning Framework** สำหรับหุ่นยนต์ Autonomous Mobile Robot (AMR) ที่ติดตั้งแขนกล 6-DOF (UR5) โดยใช้ **MoveIt 2** บน ROS 2 Humble

หลักการทำงานของระบบเป็นการบูรณาการระหว่างศาสตร์ทาง **Robotics**, **Control Theory**, และ **Trajectroy Optimization** โดยมีโครงสร้างดังนี้:

*   **Robotic Manipulator:** UR5 (6 Degrees of Freedom)
*   **Planning Pipeline:** OMPL (Open Motion Planning Library)
*   **Controller Interface:** ROS 2 Control (Joint Trajectory Controller)
*   **Collision Avoidance:** FCL (Flexible Collision Library)

---

### 2. Control Architecture (สถาปัตยกรรมการควบคุม)
ระบบใช้สถาปัตยกรรมแบบ **Hierarchical Control System** แบ่งเป็น 3 ระดับ:

1.  **High-Level Planning (MoveIt 2 / MoveGroup Node):**
    *   รับคำสั่ง Goal Pose (Position $x,y,z$ + Orientation $roll,pitch,yaw$)
    *   คำนวณ **Inverse Kinematics (IK)** เพื่อหาค่า Joint Angles $(\theta_1, ..., \theta_6)$
    *   ตรวจสอบการชน (Collision Checking) กับ Environment และ Self-Collision
    *   สร้าง Motion Plan (Trajectory) ผ่าน Algorithm ที่กำหนด

2.  **Mid-Level Control (ROS 2 Control Manager):**
    *   รับ Trajectory Point ($\theta, \dot{\theta}, \ddot{\theta}$) ผ่าน Action Interface
    *   ใช้ **`ur_arm_controller`** (Type: `joint_trajectory_controller/JointTrajectoryController`) ในการ Interpolate ค่าระหว่างจุด
    *   ส่งคำสั่งไปยัง Hardware Interface

3.  **Low-Level Actuation (Simulation/Hardware):**
    *   **Simulation (Ignition Gazebo):** `gz_ros2_control` plugin จำลอง dynamics ของมอเตอร์และส่ง Joint States กลับมา
    *   **Hardware (Real Robot):** ส่งผ่าน EtherCAT/Modbus หรือ TCP/IP Driver ของ UR

---

### 3. Motion Planning & Analysis (การวิเคราะห์การเคลื่อนที่)

#### 3.1 Planning Algorithm: RRT-Connect
ระบบเลือกใช้ **RRT-Connect (Rapidly-exploring Random Tree Connect)** ซึ่งเป็น Sampling-based algorithm ที่มีประสิทธิภาพสูงสำหรับหุ่นยนต์หลายแกน (High-dimensional Configuration Space)
*   **Bi-directional Search:** สร้าง Tree จากจุด Start และ Goal พร้อมกัน แล้วพยายามเชื่อมต่อกันตรงกลาง
*   **Probabilistic Completeness:** ยิ่งมีเวลาคำนวณมาก ยิ่งมีโอกาสหาคำตอบเจอ (ถ้ามีคำตอบ)
*   **Optimization Objective:** `PathLength` (พยายามหาเส้นทางที่สั้นที่สุดใน Joint Space)

#### 3.2 Trajectory Generation
เมื่อได้ Path (Set of Waypoints) ระบบจะใช้ **Time-Optimal Trajectory Generation (TOTG)** เพื่อใส่ Profile ความเร็ว (Velocity) และความเร่ง (Acceleration) เข้าไปในเส้นทาง โดยคำนึงถึง:
*   **Velocity Limits:** $|\dot{\theta}_i| \le \dot{\theta}_{max}$
*   **Acceleration Limits:** $|\ddot{\theta}_i| \le \ddot{\theta}_{max}$
เพื่อให้การเคลื่อนที่เเป็นไปอย่างต่อเนื่อง (Smoothness $C^2$ continuity) และไม่ทำให้มอเตอร์รับภาระเกินพิกัด

#### 3.3 Collision Matrix (SRDF)
การจัดการพื้นที่การทำงาน (Workspace Analysis) ทำผ่านไฟล์ `.srdf` โดยมีการ Disable Collision ในส่วนที่:
*   **Adjacent Links:** ชิ้นส่วนที่ต่อกันอยู่แล้ว (เช่น `ur5_shoulder_link` กับ `ur5_upper_arm_link`)
*   **Fixed Mounts:** จุดยึดฐานหุ่นยนต์กับตัวรถ (`ur5_base_link_inertia` vs `chassis_link`)

---

### 4. Operational Instructions (คู่มือการใช้งาน)

#### 4.1 Launching the System
คำสั่งเดียวในการเริ่มระบบทั้งหมด (Simulation + Planning + Visualization):
```bash
ros2 launch amr_mtt_moveit_config moveit.launch.py use_sim_time:=true
```

#### 4.2 Using RViz for Motion Planning
1.  **Startup:** รอจนกว่า Robot Model ปรากฏ และ Status ขึ้น "Available"
2.  **Set Goal:** ลากลูกบอลปลายแขน (End-Effector) ไปยังตำแหน่งเป้าหมาย
3.  **Plan:** กดปุ่ม **Plan** เพื่อคำนวณเส้นทาง (เส้นสีเหลืองจะปรากฏถ้าหาเส้นทางได้)
4.  **Execute:** กดปุ่ม **Execute** เพื่อส่ง Trajectory ไปยัง Controller
5.  **Monitor:** สังเกตการเคลื่อนที่จริงใน Gazebo

#### 4.3 Troubleshooting
*   **"Motion planning start tree could not be initialized":** เกิดจาก Start State ชนกับวัตถุ (Self-Collision) -> ให้เช็คท่าเริ่มต้นใน `amr_mtt_ign_spawn.launch.py`
*   **"Controller failed":** ระบบหา Controller ไม่เจอ -> ให้เช็ค `moveit_controllers.yaml` ว่าชื่อ Controller ตรงกับ `ros2_control` หรือไม่

---
**Author:** Antigravity AI (Implementation Partner)
**Project:** AMR-MTT (Advanced Mobile Robot - Mechatronics Tech Team)
