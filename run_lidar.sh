#!/bin/bash

# ===============================================
#  Lidar Data Viewer Launcher (AUTO-RESTART GUI)
# ===============================================

BASE_DIR="/home/yants/pertamina-gui"

echo "[🔧] Aktifkan ROS2..."
source /opt/ros/humble/setup.bash

echo "[🔧] Aktifkan Livox workspace..."
source ~/livox_ws/install/setup.bash

echo "[🚀] Menjalankan Livox MID360 ROS2 driver..."
ros2 launch livox_ros_driver2 lidar_only_MID360.launch.py &
ROS2_PID=$!

sleep 0.5
echo "[🟢] ROS2 berjalan (PID=$ROS2_PID)"

cd "$BASE_DIR" || exit 1

cleanup() {
    echo "[🧹] Cleanup: matikan ROS2..."
    if kill -0 "$ROS2_PID" 2>/dev/null; then
        kill "$ROS2_PID" 2>/dev/null
        sleep 2
        kill -9 "$ROS2_PID" 2>/dev/null
    fi
}
trap cleanup EXIT

while true; do
    echo "[🚀] Menjalankan GUI..."
    python3 main.py
    EXIT_CODE=$?

    echo "[ℹ️] GUI exit (code=$EXIT_CODE)"

    # ===========================================================
    #  JIKa GUI DITUTUP MANUAL (EXIT CODE = 0) → STOP LAUNCHER
    # ===========================================================
    if [ $EXIT_CODE -eq 0 ]; then
        echo "[🛑] GUI ditutup manual → launcher berhenti TOTAL."
        exit 0
    fi

    # Jika exit karena auto-close, restart GUI
    echo "[⏱] Tunggu 1 detik sebelum start GUI lagi..."
    sleep 0.5
done

