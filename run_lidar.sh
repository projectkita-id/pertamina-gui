#!/bin/bash
# ===============================================
#  Lidar Data Viewer Launcher (Ubuntu 22.04)
#  Jalankan GUI Lidar secara otomatis
#  Folder proyek: /home/username/dimension
# ===============================================

# ---- Cek Python dan ROS2 ----
if ! command -v python3 &> /dev/null; then
    echo "[❌] Python3 tidak ditemukan! Silakan install dulu: sudo apt install python3"
    exit 1
fi

if [ ! -d "/opt/ros/humble" ]; then
    echo "[❌] ROS2 Humble tidak ditemukan! Pastikan sudah terinstal di /opt/ros/humble"
    exit 1
fi

# ---- Source environment ROS2 & Livox ----
echo "[🔧] Mengaktifkan ROS2 environment..."
source /opt/ros/humble/setup.bash

if [ -d "$HOME/livox_ws/install" ]; then
    echo "[🔧] Mengaktifkan Livox workspace..."
    source ~/livox_ws/install/setup.bash
else
    echo "[⚠️] Livox workspace tidak ditemukan di ~/livox_ws"
fi

# ---- Masuk ke direktori proyek ----
cd /home/uppkb/dimension || {
    echo "[❌] Folder proyek tidak ditemukan di /home/username/dimension"
    exit 1
}

# ---- Jalankan aplikasi utama ----
echo "[🚀] Menjalankan Lidar Data Viewer..."
python3 main.py

# ---- Jika program selesai ----
echo ""
echo "[✅] Program selesai dijalankan."
