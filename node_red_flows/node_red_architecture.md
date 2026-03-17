# Node-RED Architecture — AMR MTT

## 1. ภาพรวมระบบ (System Overview)

```mermaid
graph TB
    subgraph UI["🖥️  Node-RED Dashboard  (localhost:1880/ui)"]
        subgraph TAB1["Tab 1 · 🦾 UR5 Arm Control"]
            JS[("🎚️ Joint Sliders\nJ1–J6  ±π rad")]
            PP[("📌 Preset Poses\nHOME / PRE_APPROACH\nLIFT_UP / DROP_ROTATE")]
            GC[("✊ Gripper\nOPEN / CLOSE")]
            LF[("📊 Live Feedback\n+ SYNC Sliders")]
        end
        subgraph TAB2["Tab 2 · 🗺️ AMR Map View"]
            WM[("🌍 Worldmap\nOpenStreetMap\nReal-time Position")]
        end
    end

    subgraph BRIDGE["🔗  rosbridge_suite  (ws://localhost:9090)"]
        WS["WebSocket Bridge\nROS 2 ↔ JSON"]
    end

    subgraph ROS2["⚙️  ROS 2 Humble"]
        subgraph TOPICS_SUB["📥 Subscribed Topics"]
            JS_T["/joint_states\nsensor_msgs/JointState"]
            AMCL_T["/amcl_pose\ngeometry_msgs/\nPoseWithCovarianceStamped"]
        end
        subgraph TOPICS_PUB["📤 Published Topics"]
            ARM_T["/ur_arm_controller\n/joint_trajectory\ntrajectory_msgs/\nJointTrajectory"]
            GRIP_T["/robotiq_gripper_controller\n/gripper_cmd\ncontrol_msgs/GripperCommand"]
        end
        subgraph CTRL["🤖 ros2_control"]
            ARM_C["ur_arm_controller\n(JointTrajectoryController)"]
            GRIP_C["robotiq_gripper_controller\n(GripperActionController)"]
            DIFF_C["diff_drive_controller\n(DiffDriveController)"]
            JSB["joint_state_broadcaster"]
        end
        AMCL["amcl_node\n(Localization)"]
    end

    subgraph SIM["🌐  Ignition Gazebo Fortress"]
        UR5["UR5 Arm\n(6 DOF)"]
        GRIPPER["Robotiq 2F-85\nGripper"]
        BASE["Differential Drive\nAMR Base"]
    end

    %% UI → Bridge
    JS -->|"build JointTrajectory"| WS
    PP -->|"preset positions"| WS
    GC -->|"GripperCommand"| WS
    WS -->|"live joint positions"| LF
    WS -->|"amcl_pose → lat/lon/heading"| WM

    %% Bridge ↔ ROS2 Topics
    WS <-->|"WebSocket JSON"| JS_T
    WS <-->|"WebSocket JSON"| AMCL_T
    WS -->|"WebSocket JSON"| ARM_T
    WS -->|"WebSocket JSON"| GRIP_T

    %% Topics → Controllers
    ARM_T --> ARM_C
    GRIP_T --> GRIP_C
    JSB --> JS_T
    AMCL --> AMCL_T

    %% Controllers → Simulation
    ARM_C --> UR5
    GRIP_C --> GRIPPER
    DIFF_C --> BASE
    UR5 --> JSB
    GRIPPER --> JSB
    BASE --> JSB

    %% Styling
    classDef dashNode fill:#1e3a5f,stroke:#4a9eff,color:#fff
    classDef bridgeNode fill:#2d4a2d,stroke:#4aff4a,color:#fff
    classDef topicNode fill:#4a2d00,stroke:#ffaa00,color:#fff
    classDef ctrlNode fill:#3a1a3a,stroke:#cc44cc,color:#fff
    classDef simNode fill:#1a1a3a,stroke:#6688ff,color:#fff

    class JS,PP,GC,LF,WM dashNode
    class WS bridgeNode
    class JS_T,AMCL_T,ARM_T,GRIP_T topicNode
    class ARM_C,GRIP_C,DIFF_C,JSB,AMCL ctrlNode
    class UR5,GRIPPER,BASE simNode
```

---

## 2. Flow A: arm_control_flow.json (Data Flow Detail)

```mermaid
flowchart LR
    subgraph INPUT["📥 User Input"]
        SL["🎚️ Slider J1–J6"]
        BTN_SEND["▶️ SEND TO ARM"]
        BTN_STOP["⏹️ STOP"]
        BTN_PRESET["📌 Preset\nHOME/PRE/LIFT/DROP"]
        BTN_GRIP_O["👐 OPEN"]
        BTN_GRIP_C["✊ CLOSE"]
        BTN_SYNC["🔄 SYNC"]
        DUR["⏱️ Duration Slider\n0.5 – 5 sec"]
    end

    subgraph FUNC["⚙️ Function Nodes"]
        F_COLLECT["func-collect-joints\n(store in flow context)"]
        F_TRAJ["func-build-trajectory\n(JointTrajectory msg)"]
        F_STOP["func-stop-arm\n(hold current position)"]
        F_PRESET_X["func-preset-X\n(fixed positions)"]
        F_GRIP_O["func-gripper-open\n(position: 0.0)"]
        F_GRIP_C["func-gripper-close\n(position: 0.8)"]
        F_PARSE["func-parse-joint-states\n(extract UR5 joints)"]
        F_SYNC["func-trigger-sync"]
        F_DUR["func-set-duration"]
    end

    subgraph DISPLAY["🖥️ Display"]
        TXT_TARGET["📝 Target Joints\n(text display)"]
        TXT_FB["📝 Current Joints\n(text display)"]
        DEBUG["🐛 Debug sidebar"]
        SL_FB["🎚️ Sliders\n(auto-update)"]
    end

    subgraph WS_LAYER["🔗 WebSocket (rosbridge)"]
        WS_OUT["ws-arm-out\n→ ROS 2"]
        WS_IN["ws-arm-in\n← ROS 2"]
    end

    subgraph ROS["⚙️ ROS 2 Topics"]
        T_ARM["/ur_arm_controller\n/joint_trajectory"]
        T_GRIP["/robotiq_gripper_controller\n/gripper_cmd"]
        T_JS["/joint_states"]
    end

    SL --> F_COLLECT
    DUR --> F_DUR
    F_COLLECT --> TXT_TARGET
    F_COLLECT --> DEBUG
    BTN_SEND --> F_TRAJ
    BTN_STOP --> F_STOP
    BTN_PRESET --> F_PRESET_X
    BTN_SYNC --> F_SYNC
    BTN_GRIP_O --> F_GRIP_O
    BTN_GRIP_C --> F_GRIP_C

    F_TRAJ --> WS_OUT
    F_STOP --> WS_OUT
    F_PRESET_X --> WS_OUT
    F_GRIP_O --> WS_OUT
    F_GRIP_C --> WS_OUT

    WS_OUT --> T_ARM
    WS_OUT --> T_GRIP
    T_JS --> WS_IN
    WS_IN --> F_PARSE
    F_PARSE --> TXT_FB
    F_PARSE --> SL_FB
    F_SYNC --> F_PARSE
```

---

## 3. Flow B: amr_map_flow.json (Data Flow Detail)

```mermaid
flowchart LR
    subgraph ROS["⚙️ ROS 2"]
        AMCL["/amcl_pose\ngeometry_msgs/\nPoseWithCovarianceStamped"]
    end

    subgraph WS_LAYER["🔗 WebSocket (rosbridge)"]
        WS_OUT_MAP["ws-out-rosbridge\n(subscribe request)"]
        WS_IN_MAP["ws-in-rosbridge\n(receive pose)"]
    end

    subgraph FUNC_MAP["⚙️ Function Nodes"]
        F_SUB["func-subscribe-amcl\n(subscribe on startup)"]
        F_PARSE_MAP["func-parse-pose\n───────────────\n1. Extract x, y\n2. Quaternion → Yaw\n3. Map coords → lat/lon\n   scale = 0.000009 °/m\n   base = 13.7563, 100.5018\n4. Build worldmap payload"]
    end

    subgraph INIT["🚀 Startup"]
        INJ["inject\n(on deploy / start)"]
    end

    subgraph OUTPUT["🖥️ Output"]
        WM_NODE["🌍 Worldmap Node\n───────────────\nicon: 🤖 robot\ncolor: #00ff88\npopup: X, Y, Heading"]
        DBG["🐛 Debug\n(pose values)"]
    end

    INJ --> F_SUB --> WS_OUT_MAP
    WS_OUT_MAP -->|"subscribe /amcl_pose"| AMCL
    AMCL -->|"PoseWithCovarianceStamped"| WS_IN_MAP
    WS_IN_MAP --> F_PARSE_MAP
    F_PARSE_MAP --> WM_NODE
    F_PARSE_MAP --> DBG
```

---

## 4. Preset Poses Reference

```mermaid
graph LR
    subgraph POSES["📌 UR5 Preset Positions (radians)"]
        HOME["🏠 HOME\n[0.0, -1.571, -1.571,\n -3.159,  0.0,   0.0]\nduration: 2s"]
        PRE["🎯 PRE_APPROACH\n[-1.396, -2.269, -1.152,\n -1.222,  1.571,  0.0]\nduration: 3s"]
        LIFT["⬆️ LIFT_UP\n[0.0,  -2.758, -1.571,\n-3.334, -1.571,  0.0]\nduration: 2s"]
        DROP["🔄 DROP_ROTATE\n[1.571, -1.571, -1.571,\n-3.159,  0.0,   0.0]\nduration: 2s"]

        HOME -->|"move to pick zone"| PRE
        PRE -->|"after grip"| LIFT
        LIFT -->|"rotate to drop"| DROP
        DROP -->|"reset"| HOME
    end
```
