#!/bin/bash
# =============================================================
# Amiga Robot Başlatma Scripti
# Kullanım:
#   bash start_robot.sh          → SLAM modu (harita oluştur)
#   bash start_robot.sh nav      → Navigasyon modu (kayıtlı harita)
# =============================================================

set -e

WS_DIR="$HOME/ros2_amiga_ws"
BRIDGE_DIR="$WS_DIR/src/ros2_bridge"
VENV="$HOME/farm-ng-amiga/.venv/bin/python"
LOG_DIR="$BRIDGE_DIR/logs"
MODE="${1:-slam}"   # slam | nav

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"

# ── Renkler ────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'
YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'

info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ── Ön kontroller ──────────────────────────────────────────
[ ! -f "$ROS_SETUP" ]  && error "ROS Humble bulunamadı: $ROS_SETUP" && exit 1
[ ! -f "$WS_SETUP" ]   && error "Workspace build edilmemiş. Önce: colcon build --symlink-install" && exit 1
[ ! -f "$VENV" ]       && error "farm-ng venv bulunamadı: $VENV" && exit 1

source "$ROS_SETUP"
source "$WS_SETUP"

mkdir -p "$LOG_DIR"

# ── PID listesi (CTRL+C ile hepsini öldürür) ───────────────
PIDS=()

cleanup() {
    echo ""
    warn "Durduruluyor..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    # tmux oturumu varsa kapat
    tmux kill-session -t amiga 2>/dev/null || true
    ok "Tüm servisler durduruldu."
    exit 0
}
trap cleanup SIGINT SIGTERM

# ── Yardımcı: gecikme ile başlat ──────────────────────────
wait_for_topic() {
    local topic="$1"
    local timeout="${2:-15}"
    info "Bekleniyor: $topic (max ${timeout}s)..."
    for i in $(seq 1 "$timeout"); do
        if ros2 topic info "$topic" --no-daemon &>/dev/null; then
            ok "$topic hazır."
            return 0
        fi
        sleep 1
    done
    warn "$topic ${timeout}s içinde gelmedi, devam ediliyor."
}

# =============================================================
# ADIM 1 — Farm-ng Donanım Köprüsü
# =============================================================
info "ADIM 1: Farm-ng donanım köprüsü başlatılıyor..."

SRC="$BRIDGE_DIR/src"
CFG="$BRIDGE_DIR/config"

MOTOR_PY="$HOME/farm-ng-amiga/py/examples/motor_states_stream/motor_battery.py"
MOTOR_CFG="$HOME/farm-ng-amiga/py/examples/motor_states_stream/service_config.json"

$VENV "$SRC/ros2_to_twist.py"  --service-config "$CFG/ros2_to_twist.json" \
    >> "$LOG_DIR/control_ros2.log" 2>&1 &
PIDS+=($!)

$VENV "$SRC/gps_filter.py"     --config        "$CFG/gps_filter.json"    \
    >> "$LOG_DIR/gps_filter.log" 2>&1 &
PIDS+=($!)

$VENV "$SRC/odometry.py"       --service-config "$CFG/odometry.json"      \
    >> "$LOG_DIR/odometry.log" 2>&1 &
PIDS+=($!)

$VENV "$SRC/imu_to_ros.py"     --service-config "$CFG/imu_to_ros.json"    \
    >> "$LOG_DIR/imu_to_ros.log" 2>&1 &
PIDS+=($!)

$VENV "$SRC/cam_to_ros.py"     --service-config "$CFG/cam_to_ros.json"    \
    >> "$LOG_DIR/camera.log" 2>&1 &
PIDS+=($!)

$VENV "$MOTOR_PY"              --service-config "$MOTOR_CFG"               \
    >> "$LOG_DIR/motor.log" 2>&1 &
PIDS+=($!)

ok "Farm-ng köprüsü başlatıldı. Loglar: $LOG_DIR"
sleep 3

# =============================================================
# ADIM 2 — Robot Description (TF ağacı)
# =============================================================
info "ADIM 2: Robot description (TF) başlatılıyor..."
ros2 launch amiga_description description.launch.py \
    >> "$LOG_DIR/description.log" 2>&1 &
PIDS+=($!)
sleep 2

# =============================================================
# ADIM 3 — EKF + NavSat (Lokalizasyon + /fromLL servisi)
# =============================================================
info "ADIM 3: EKF lokalizasyon başlatılıyor..."
ros2 launch amiga_navsat_ekf navsat_ekf.launch.py \
    >> "$LOG_DIR/navsat_ekf.log" 2>&1 &
PIDS+=($!)

wait_for_topic "/odometry/filtered_local" 20
sleep 2

# =============================================================
# ADIM 4 — LiDAR + Laser Filter
# =============================================================
info "ADIM 4: RPLidar S2 + laser filter başlatılıyor..."
ros2 launch amiga_lidar lidar.launch.py \
    >> "$LOG_DIR/lidar.log" 2>&1 &
PIDS+=($!)

wait_for_topic "/scan_filtered" 15

# =============================================================
# ADIM 5 — SLAM veya Navigasyon
# =============================================================
if [ "$MODE" = "nav" ]; then
    info "ADIM 5: Navigasyon modu başlatılıyor..."
    ros2 launch amiga_navigation navigation.launch.py \
        >> "$LOG_DIR/navigation.log" 2>&1 &
    PIDS+=($!)
else
    info "ADIM 5: SLAM modu başlatılıyor..."
    ros2 launch amiga_slam slam.launch.py \
        >> "$LOG_DIR/slam.log" 2>&1 &
    PIDS+=($!)
fi
sleep 4

# =============================================================
# ADIM 6 — WebSocket Bridge
# =============================================================
info "ADIM 6: WebSocket bridge başlatılıyor..."
ros2 run ros2_bridge websocket_bridge \
    >> "$LOG_DIR/websocket_bridge.log" 2>&1 &
PIDS+=($!)
sleep 2

# =============================================================
# ADIM 7 — Task Manager
# =============================================================
info "ADIM 7: Task manager başlatılıyor..."
ros2 run task_manager task_manager_node \
    >> "$LOG_DIR/task_manager.log" 2>&1 &
PIDS+=($!)

# =============================================================
# Hazır
# =============================================================
echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}  Amiga robot hazır!  Mod: ${MODE^^}${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo -e "  Log dizini : $LOG_DIR"
echo -e "  Durdurmak  : CTRL+C"
echo ""
echo -e "  Canlı logları izlemek için:"
echo -e "  ${CYAN}tail -f $LOG_DIR/*.log${NC}"
echo ""

wait
