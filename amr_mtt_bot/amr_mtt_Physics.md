# AMR-MTT Robot: Mechatronics & Physics Analysis

This document details the engineering analysis, parameter derivation, and physics calculations used to design and validate the **AMR-MTT** (Autonomous Mobile Robot) system.

---

## 1. Drive System Analysis (ระบบขับเคลื่อน)

### 1.1 Physical Parameters
*   **Wheel Configuration:** Differential Drive (Mid-traction) + 4 Caster Wheels (Trolley)
*   **Wheel Radius ($r$):** 0.08 m (80 mm)
*   **Track Width ($L$):** 0.436 m (436 mm)
*   **Friction Coefficients:**
    *   Traction Wheels: $\mu = 5.0$ (High grip, optimized for odometry)
    *   Caster Wheels: $\mu = 0.0$ (Zero friction, optimized for frictionless swiveling in simulation)

### 1.2 Velocity & Motor Requirements
To achieve a target linear velocity of **1.0 m/s**:

$$ \omega_{req} = \frac{v}{r} = \frac{1.0 \text{ m/s}}{0.08 \text{ m}} = 12.5 \text{ rad/s} \approx 119 \text{ RPM} $$

*   **Controller Limit:** `max_velocity` set to 1.0 m/s
*   **Acceleration Limit:** 0.4 m/s² (Smooth startup profile)

---

## 2. Motor Sizing Verification (การเลือกขนาดมอเตอร์)

Comparison between **Simulation Load** and **Oriental Motor BLMR6400SKM-GFV-F (400W)** specifications.

### 2.1 Motor Specifications
*   **Rated Output:** 400 W
*   **Gear Ratio:** 10:1
*   **Rated Torque:** 11.4 N·m
*   **Max Speed:** ~400 RPM (at gear output)

### 2.2 Sizing Calculation (Worst Case)
*   **Total Robot Mass ($M$):** 172.2 kg (Chassis 135kg + Battery 20kg + Arm 18.4kg)
*   **Required Traction Force ($F_{req}$):** For 0.4 m/s² acceleration
    $$ F_{req} = M \times a = 172.2 \text{ kg} \times 0.4 \text{ m/s}^2 \approx 68.9 \text{ N} $$
*   **Available Traction Force ($F_{avail}$):** From 2 motors
    $$ F_{avail} = \frac{2 \times \tau_{motor}}{r} = \frac{2 \times 11.4 \text{ N}\cdot\text{m}}{0.08 \text{ m}} = 285 \text{ N} $$

### 2.3 Result: **PASSED** ✅
*   **Safety Factor:** $285 / 68.9 \approx 4.13$
*   The selected motor is sufficient to drive the robot with a 4.1x safety margin at the desired acceleration.

---

## 3. Stability & Tipping Analysis (เสถียรภาพและการพลิกคว่ำ)

Analysis of the Center of Mass (CoM) stability when the UR5 arm is fully extended.

### 3.1 Worst-Case Scenarios
Tipping occurs when **Overturning Moment > Restoring Moment**.

#### Case A: Sideways Reach (Max Extension)
*   **Reach:** 0.85 m (Arm)
*   **Support Base (Half-width):** 0.25 m
*   **Overturning Limit:**
    $$ \tau_{tip} = (M_{arm} + M_{load}) \times g \times Reach \approx (18.4+5) \times 9.8 \times 0.85 \approx 195 \text{ N}\cdot\text{m} $$
*   **Restoring Limit:**
    $$ \tau_{restore} = M_{base} \times g \times Support \approx 135 \times 9.8 \times 0.25 \approx 330 \text{ N}\cdot\text{m} $$
*   **Result:** **SAFE (SF ~1.7)**

#### Case B: Forward Reach
*   **Reach:** 1.05 m (Arm 0.85 + Mount Offset 0.2)
*   **Support Base (Front Axle Distance):** 0.35 m
*   **Overturning Limit:** ~241 N·m
*   **Restoring Limit:** ~463 N·m
*   **Result:** **SAFE (SF ~1.9)**

---
**Summary:** The AMR-MTT design is mechanically sound, with adequate drive power and stability margins for standard operations.