#!/bin/bash
# ==========================================
# Farm-ng Amiga ROS Bridge Launcher
# ==========================================

source /opt/ros/humble/setup.bash
source ~/ros2_amiga_ws/install/setup.bash

# Paket dizinleri
PACKAGE_DIR="/home/cuma_karaaslan/ros2_amiga_ws/src/ros2_bridge"
SRC="$PACKAGE_DIR/src"
CONFIG="$PACKAGE_DIR/config"

# farm-ng sanal ortamı (farm-ng kütüphaneleri burada kurulu)
VENV="/home/cuma_karaaslan/farm-ng-amiga/.venv/bin/python"

# Pakette olmayan servisler (eski konumları)
MOTOR="/home/cuma_karaaslan/farm-ng-amiga/py/examples/motor_states_stream/motor_battery.py"
MOTOR_CONFIG="/home/cuma_karaaslan/farm-ng-amiga/py/examples/motor_states_stream/service_config.json"

WEBSOCKET_SERVER="/home/cuma_karaaslan/ros2_amiga_ws/src/robot_web_server/robot_web_server/websocket_server.py"
GOAL_SENDER="/home/cuma_karaaslan/ros2_amiga_ws/src/robot_web_server/robot_web_server/goal_sender2.py"

# ==========================================
# LOG dizini
# ==========================================
LOG_DIR="$PACKAGE_DIR/logs"
mkdir -p "$LOG_DIR"

# ==========================================
# Servisleri paralel başlat
# ==========================================

echo "Starting Farm-ng Amiga ROS bridge services..."
echo "Logs are saved under: $LOG_DIR"

$VENV $SRC/ros2_to_twistpy --service-config $CONFIG/ros2_to_twist.json > "$LOG_DIR/control_ros2.log" 2>&1 &
PID_CONTROL=$!

$VENV $SRC/gps_filter.py --config $CONFIG/gps_filter.json > "$LOG_DIR/gps_filter.log" 2>&1 &
PID_GPS=$!

$VENV $SRC/odometry.py --service-config $CONFIG/odometry.json > "$LOG_DIR/odometry.log" 2>&1 &
PID_ODO=$!

$VENV $SRC/imu_to_ros.py --service-config $CONFIG/imu_to_ros.json > "$LOG_DIR/imu_to_ros.log" 2>&1 &
PID_IMU=$!

$VENV $SRC/cam_to_ros.py --service-config $CONFIG/cam_to_ros.json > "$LOG_DIR/camera.log" 2>&1 &
PID_CAMERA=$!

$VENV $MOTOR --service-config $MOTOR_CONFIG > "$LOG_DIR/motor.log" 2>&1 &
PID_MOTOR=$!

$VENV $WEBSOCKET_SERVER > "$LOG_DIR/websocket_server.log" 2>&1 &
PID_WS=$!

$VENV $GOAL_SENDER > "$LOG_DIR/goal_sender.log" 2>&1 &
PID_GOAL=$!

echo "All services started."
echo "Control PID:          $PID_CONTROL"
echo "GPS Filter PID:       $PID_GPS"
echo "Odometry PID:         $PID_ODO"
echo "IMU PID:              $PID_IMU"
echo "Camera PID:           $PID_CAMERA"
echo "Motor PID:            $PID_MOTOR"
echo "WebSocket Server PID: $PID_WS"
echo "Goal Sender PID:      $PID_GOAL"
echo
echo "Press [CTRL+C] to stop all."

# ==========================================
# CTRL+C ile hepsini durdur
# ==========================================
trap "echo 'Stopping all services...'; \
kill $PID_CONTROL $PID_GPS $PID_ODO $PID_IMU $PID_CAMERA $PID_MOTOR $PID_WS $PID_GOAL 2>/dev/null; exit 0" SIGINT

wait
