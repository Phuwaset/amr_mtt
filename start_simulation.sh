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

    for NAME_PID in "Gazebo+MoveIt:$PID_MOVEIT" "Nav2:$PID_NAV2" "Rosbridge:$PID_ROSBRIDGE" "Node-RED:$PID_NODERED"; do
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

# ── Step 1: Gazebo + MoveIt ──────────────────────
log "Step 1/4 — Gazebo + MoveIt"
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
    ros2 launch amr_mtt_moveit_config moveit.launch.py \
    > "$LOG_DIR/moveit.log" 2>&1 &
PID_MOVEIT=$!
wait_for_topic "/clock" "$LOG_DIR/moveit.log" 90
sleep 3

# ── Step 2: Nav2 ─────────────────────────────────
log "Step 2/4 — Nav2"
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
    ros2 launch amr_mtt_bot nav2.launch.py \
    > "$LOG_DIR/nav2.log" 2>&1 &
PID_NAV2=$!
wait_for_topic "/map" "$LOG_DIR/nav2.log" 60
sleep 2

# ── Step 3: Rosbridge ────────────────────────────
log "Step 3/4 — Rosbridge WebSocket"
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

# ── Step 4: Node-RED + Import flows ──────────────
log "Step 4/4 — Node-RED flows"
PID_NODERED=""
if ! pgrep -x "node-red" > /dev/null; then
    node-red > "$LOG_DIR/nodered.log" 2>&1 &
    PID_NODERED=$!
    sleep 4
fi

HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" -X POST http://localhost:1880/flows \
    -H "Content-Type: application/json" \
    -d @"$FLOWS_FILE" 2>/dev/null)

if [ "$HTTP_CODE" = "200" ] || [ "$HTTP_CODE" = "204" ]; then
    log "Node-RED flows imported ✓"
else
    err "Import flows ล้มเหลว (HTTP $HTTP_CODE)"
fi

echo "$PID_MOVEIT $PID_NAV2 $PID_ROSBRIDGE $PID_NODERED" > "$LOG_DIR/pids.txt"

echo ""
echo "================================================"
echo -e "   ${GREEN} ✅ Startup Complet!${NC}"
echo "================================================"
info "Dashboard : http://localhost:1880/ui"
info "⚠️  กด ENABLE ARM ก่อนส่งคำสั่งแขน"
info "กด Ctrl+C เพื่อหยุดทุก process"
echo ""

wait
