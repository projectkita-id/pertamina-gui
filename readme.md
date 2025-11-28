# README - Prosedur Instalasi Sistem Pengukuran Dimensi Kendaraan

---

## 1. Instalasi ROS 2 Humble

### 1.1 Set up UTF-8 System Locale

Periksa locale:
```
locale -a
```

Jika belum ada UTF-8:
```
sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8
```

Cek kembali:
```
locale -a
```

Set locale sistem:
```
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Buka terminal baru lalu cek:
```
locale
```

---

### 1.2 Set up Source Repos

Aktifkan repository universe:
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Install tools repositori:
```
sudo apt update && sudo apt install curl gnupg lsb-release
```

Tambahkan key GPG ROS:
```
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key   -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Tambahkan repository ROS 2:
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

---

### 1.3 Install ROS 2 packages

```
sudo apt update
sudo apt install ros-humble-desktop
```

---

## 2. Instalasi Livox SDK2

### Dependencies:
- CMake 3.0.0+
- gcc 4.8.1+

Install CMake:
```
sudo apt install cmake
```

Clone dan build Livox-SDK2:
```
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build
cd build
cmake .. && make -j
sudo make install
```

---

## 3. Install Livox ROS Driver 2

Clone source code:
```
git clone https://github.com/Livox-SDK/livox_ros_driver2.git livox_ws/src/livox_ros_driver2
```

Build driver:
```
source /opt/ros/humble/setup.sh
./build.sh humble
```

Test run:
```
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

---

## 4. Instalasi Aplikasi Pengukuran

Download aplikasi:
```
https://github.com/projectkita-id/pertamina-gui.git dimension
```

Langkah lainnya:
1. Ekstrak file `livox_ws.zip`
2. Replace folder `livox_ws` lama
3. Copy file launch:
```
cp lidar_only_MID360.launch.py   livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/
```
4. Konfigurasi IP pada `MID360_config.json`
5. Sesuaikan IP Host dan IP Lidar
6. Edit `run_lidar.sh`:
```
nano run_lidar.sh
```
7. Set executable:
```
chmod +x main.py
chmod +x run_lidar.sh
```
8. Buat launcher `.desktop`
9. Ubah auto restart main.py menjadi interval 10 detik
10. Jalankan file `.desktop`

---

## SELESAI
