#!/bin/bash
# AMR MTT — Start Full Simulation Stack

WORKSPACE=~/amr_mtt
FLOWS_FILE="$WORKSPACE/src/amr_mtt/node_red_flows/amr_mtt_flows.json"
LOG_DIR="$WORKSPACE/logs"
mkdir -p "$LOG_DIR"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

log()  { echo -e "${GREEN}[AMR MTT]${NC} $1"; }
warn() { echo -e "${YELLOW}[WAIT]${NC}    $1"; }
info() { echo -e "${CYAN}[INFO]${NC}    $1"; }
err()  { echo -e "${RED}[ERROR]${NC}   $1"; }

# ── Cleanup on Ctrl+C ────────────────────────────
cleanup() {
    echo ""
    echo "================================================"
    echo -e "   ${YELLOW} stopping process...${NC}"
    echo "================================================"

    for NAME_PID in "Gazebo+MoveIt:$PID_MOVEIT" "Nav2:$PID_NAV2" "Rosbridge:$PID_ROSBRIDGE" "Node-RED:$PID_NODERED" "ArmJog:$PID_ARMJOG" "NavWP:$PID_NAVWP" "MapMgr:$PID_MAPMGR"; do
        NAME="${NAME_PID%%:*}"
        PID="${NAME_PID##*:}"
        if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; then
            echo -e "  ${YELLOW}stopping${NC} $NAME (PID $PID)..."
            kill -SIGTERM "$PID" 2>/dev/null
            # รอให้ process หยุดก่อน force kill
            for i in $(seq 1 5); do
                kill -0 "$PID" 2>/dev/null || break
                sleep 1
            done
            kill -SIGKILL "$PID" 2>/dev/null
            echo -e "  ${GREEN}stopped${NC}  $NAME"
        fi
    done

    # kill ลูกของ process ที่เหลือ
    kill -- -$$ 2>/dev/null

    rm -f "$LOG_DIR/pids.txt"
    echo ""
    echo -e "   ${GREEN} stop process now!${NC}"
    echo "================================================"
    echo ""
    clear
    exit 0
}

trap cleanup SIGINT SIGTERM

# ── Source workspace ─────────────────────────────
source /opt/ros/humble/setup.bash
if ! source "$WORKSPACE/install/setup.bash" 2>/dev/null; then
    err "ยังไม่ได้ build — รัน: cd $WORKSPACE && colcon build --symlink-install"
    exit 1
fi

echo ""
echo "================================================"
echo "   AMR MTT — Full Simulation Startup"
echo "================================================"
echo ""

# ── Dependency check ─────────────────────────────
if ! python3 -c "import flask_socketio" 2>/dev/null; then
    err "flask-socketio ไม่ได้ติดตั้ง — รัน: pip install flask-socketio"
    exit 1
fi

# helper: poll topic พร้อมแสดง log ล่าสุด
wait_for_topic() {
    local TOPIC=$1 LOGFILE=$2 TIMEOUT=${3:-90} ELAPSED=0
    while ! ros2 topic list 2>/dev/null | grep -q "^${TOPIC}$"; do
        sleep 2; ELAPSED=$((ELAPSED+2))
        LAST=$(tail -1 "$LOGFILE" 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g')
        printf "\r  %-80s" "$LAST"
        if [ $ELAPSED -ge $TIMEOUT ]; then
            echo ""; err "Timeout! ดู log: tail -f $LOGFILE"; cleanup; fi
    done
    printf "\r%-80s\n" "  ✓ $TOPIC พร้อมแล้ว (${ELAPSED}s)"
}

# ── Step 1: Gazebo Simulation ────────────────────
log "Step 1/6 — Gazebo Simulation"
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
    ros2 launch amr_mtt_bot launch_sim_amr.launch.py \
    > "$LOG_DIR/gazebo.log" 2>&1 &
PID_MOVEIT=$!
wait_for_topic "/clock" "$LOG_DIR/gazebo.log" 90
sleep 3

# ── Step 2: Nav2 ─────────────────────────────────
log "Step 2/6 — Nav2"
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
    ros2 launch amr_mtt_bot nav2.launch.py \
    > "$LOG_DIR/nav2.log" 2>&1 &
PID_NAV2=$!
wait_for_topic "/map" "$LOG_DIR/nav2.log" 60
sleep 2

# ── Step 2.5: Set Initial Pose (AMCL) ────────────
log "Step 2.5/6 — Set Initial Pose (AMCL)"
ros2 run amr_mtt_task_planner set_initial_pose \
    > "$LOG_DIR/set_initial_pose.log" 2>&1
echo "  ✓ set_initial_pose เสร็จแล้ว"

# ── Step 3: Rosbridge ────────────────────────────
log "Step 3/7 — Rosbridge WebSocket"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    > "$LOG_DIR/rosbridge.log" 2>&1 &
PID_ROSBRIDGE=$!
ELAPSED=0
until ss -tlnp 2>/dev/null | grep -q ":9090"; do
    sleep 1; ELAPSED=$((ELAPSED+1))
    if [ $ELAPSED -ge 20 ]; then
        err "Timeout! ดู log: tail -f $LOG_DIR/rosbridge.log"; cleanup; fi
done
echo "  ✓ Rosbridge พร้อมแล้ว (${ELAPSED}s)"

# ── Step 4: Arm Jog Node ─────────────────────────
# รอ ur_arm_controller action server พร้อมก่อน (controller โหลดหลัง Gazebo เสร็จ)
log "Step 4/7 — รอ ur_arm_controller พร้อม..."
ELAPSED=0
until ros2 action list 2>/dev/null | grep -q "follow_joint_trajectory"; do
    sleep 2; ELAPSED=$((ELAPSED+2))
    if [ $ELAPSED -ge 60 ]; then
        err "ur_arm_controller timeout — ดู log: tail -f $LOG_DIR/gazebo.log"; cleanup
    fi
done
echo "  ✓ ur_arm_controller พร้อมแล้ว (${ELAPSED}s)"

log "Step 4/7 — Arm Jog Node (HTTP :5005)"
ros2 run amr_mtt_task_planner arm_jog_node \
    > "$LOG_DIR/armjog.log" 2>&1 &
PID_ARMJOG=$!
ELAPSED=0
until ss -tlnp 2>/dev/null | grep -q ":5005"; do
    sleep 1; ELAPSED=$((ELAPSED+1))
    if [ $ELAPSED -ge 30 ]; then
        warn "Arm jog node ยังไม่พร้อม — ดู log: tail -f $LOG_DIR/armjog.log"
        break
    fi
done
[ $ELAPSED -lt 30 ] && echo "  ✓ Arm Jog Node พร้อมแล้ว (${ELAPSED}s)"

# ── Step 5: Nav Waypoint Server ──────────────────
log "Step 5/7 — Nav Waypoint Server (HTTP :5006)"
ros2 run amr_mtt_task_planner nav_waypoint_server \
    > "$LOG_DIR/navwp.log" 2>&1 &
PID_NAVWP=$!
ELAPSED=0
until ss -tlnp 2>/dev/null | grep -q ":5006"; do
    sleep 1; ELAPSED=$((ELAPSED+1))
    if [ $ELAPSED -ge 30 ]; then
        warn "Nav Waypoint Server ยังไม่พร้อม — ดู log: tail -f $LOG_DIR/navwp.log"
        break
    fi
done
[ $ELAPSED -lt 30 ] && echo "  ✓ Nav Waypoint Server พร้อมแล้ว (${ELAPSED}s)"

# ── Step 6: Map Manager Node ─────────────────────
log "Step 6/7 — Map Manager Node (HTTP :5007)"
ros2 run amr_mtt_task_planner map_manager_node \
    > "$LOG_DIR/mapmgr.log" 2>&1 &
PID_MAPMGR=$!
ELAPSED=0
until ss -tlnp 2>/dev/null | grep -q ":5007"; do
    sleep 1; ELAPSED=$((ELAPSED+1))
    if [ $ELAPSED -ge 30 ]; then
        warn "Map Manager ยังไม่พร้อม — ดู log: tail -f $LOG_DIR/mapmgr.log"
        break
    fi
done
[ $ELAPSED -lt 30 ] && echo "  ✓ Map Manager Node พร้อมแล้ว (${ELAPSED}s)"

# ── Step 7: Node-RED ─────────────────────────────
log "Step 7/7 — Node-RED flows"
PID_NODERED=""
if ! pgrep -x "node-red" > /dev/null; then
    node-red > "$LOG_DIR/nodered.log" 2>&1 &
    PID_NODERED=$!
fi

# รอ Node-RED พร้อม (poll port 1880 แทน sleep คงที่)
ELAPSED=0
until ss -tlnp 2>/dev/null | grep -q ":1880"; do
    sleep 1; ELAPSED=$((ELAPSED+1))
    if [ $ELAPSED -ge 30 ]; then
        err "Node-RED timeout — ดู log: tail -f $LOG_DIR/nodered.log"
        break
    fi
done

HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" -X POST http://localhost:1880/flows \
    -H "Content-Type: application/json" \
    -d @"$FLOWS_FILE" 2>/dev/null)

if [ "$HTTP_CODE" = "200" ] || [ "$HTTP_CODE" = "204" ]; then
    echo "  ✓ Node-RED flows imported (${ELAPSED}s)"
else
    err "Import flows ล้มเหลว (HTTP $HTTP_CODE) — ดู log: tail -f $LOG_DIR/nodered.log"
fi

echo "$PID_MOVEIT $PID_NAV2 $PID_ROSBRIDGE $PID_ARMJOG $PID_NAVWP $PID_MAPMGR $PID_NODERED" > "$LOG_DIR/pids.txt"

echo ""
echo "================================================"
echo -e "   ${GREEN} ✅ Startup Complete!${NC}"
echo "================================================"
info "Dashboard      : http://localhost:1880/ui"
info "Arm Jog REST   : http://localhost:5005/status"
info "Arm Jog WS     : ws://localhost:5005  (SocketIO event: 'status')"
info "Nav WP API     : http://localhost:5006/status"
info "Map Mgr API    : http://localhost:5007/status"
echo ""
echo -e "  ${YELLOW}⚠️  ขั้นตอนก่อนใช้งาน:${NC}"
echo "  1. กด ENABLE ARM บน dashboard ก่อนส่งคำสั่งแขน"
echo ""
echo -e "  ${YELLOW}On-demand (รัน manual เมื่อต้องการ):${NC}"
echo -e "     ${CYAN}ros2 run amr_mtt_task_planner coordinator_node${NC}   # full pick-and-place"
echo -e "     ${CYAN}ros2 run amr_mtt_task_planner task_sequence${NC}       # arm sequence only"
echo ""
info "กด Ctrl+C เพื่อหยุดทุก process"
echo ""

wait
