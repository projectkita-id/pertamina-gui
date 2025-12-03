# Sistem Deteksi Livox MID360 + ROS2 Humble + GUI
**Platform:** Ubuntu  22.04 (GUI)   
**Device:** Livox MID360

## 📘 Overview
Dokumen ini berisi panduan lengkap instalasi:

- ROS2 Humble  
- Livox SDK2  
- Livox ROS Driver 2  
- Workspace `livox_ws`  
- GUI Dimensi  
- Autologin LightDM  
- Autorun GUI  

Semua langkah telah diuji dan dipastikan berfungsi untuk sistem produksi.

## 📦 1. Persiapan Sistem
Pastikan sistem menggunakan:

- Ubuntu 22.04 Server GUI  
- Display manager: **LightDM**

Update sistem:

```
sudo apt update && sudo apt upgrade -y
```

Install paket dasar:

```
sudo apt install git curl wget build-essential cmake python3-pip python3-tk libgl1-mesa-glx -y
```

## 🟦 2. Instalasi ROS2 Humble

### 2.1 Set Locale
```
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2.2 Tambahkan Repository ROS2
```
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
```

Tambahkan ros2-apt-source:

```
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list
```

### 2.3 Instalasi ROS2 Humble Desktop
```
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop ros-dev-tools -y
```

### 2.4 Tambahkan ke `.bashrc`
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 🟧 3. Instalasi Colcon
```
sudo apt install python3-colcon-common-extensions -y
```

## 🟩 4. Instalasi Livox SDK2
```
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install
```

Tambahkan library path:

```
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc
source ~/.bashrc
```

## 🟪 5. Membuat Workspace Livox
```
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src
```

Clone Livox ROS Driver 2:

```
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

## 🟥 6. Build Livox ROS Driver 2
Source ROS2:

```
source /opt/ros/humble/setup.bash
```

Build driver:

```
cd ~/livox_ws
./src/livox_ros_driver2/build.sh humble
```

Aktifkan workspace:

```
echo "source ~/livox_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 🟦 7. Konfigurasi IP LiDAR MID360
Edit file konfigurasi:

```
cd ~/livox_ws/src/livox_ros_driver2/config
```

Sesuaikan parameter wajib dalam file `MID360_config.json`.

## 🟨 8. Instalasi Python Dependencies
```
pip install open3d==0.19.0
pip install customtkinter pillow matplotlib numpy==1.24.4
pip install scipy==1.10.1
pip install scikit-learn==1.2.2
pip install sensor-msgs-py
pip install PyMySQL mysql-connector-python
pip install watchdog
```

Instalasi rclpy:

```
source /opt/ros/humble/setup.bash
pip install rclpy
```

Dependencies GUI:

```
sudo apt install xdotool wmctrl -y
```

## 🟧 9. Clone GUI  & Integrasikan livox_ws
```
cd ~
git clone https://github.com/projectkita-id/pertamina-gui dimension
rm -rf ~/dimension/livox_ws
cp -r ~/livox_ws ~/dimension/livox_ws
```

## 🟩 10. Testing Driver Livox
```
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

## 🟦 11. Autologin LightDM
Edit konfigurasi:

```
sudo nano /etc/lightdm/lightdm.conf
```

Isi:

```
[Seat:*]
autologin-user=YOUR_USERNAME
autologin-user-timeout=0
user-session=ubuntu
```

## 🟪 12. Autorun GUI Setelah reboot
```
nano ~/Desktop/run_lidar.desktop
```

Isi:

```
[Desktop Entry]
Version=1.0
Type=Application
Name=Lidar Dimension
Comment=Pengukuran dimensi kendaraan
Exec=gnome-terminal -- bash -c "/home/uppkb/dimension/run_lidar.sh"
Icon=/home/uppkb/dimension/assets/icon/logo.ico
Terminal=false
Categories=Utility;
X-GNOME-Autostart-enabled=true
```
Beri izin eksekusi:
```
chmod +x ~/Desktop/run_lidar.desktop
```
Autostart:
```
mkdir -p ~/.config/autostart
cp ~/Desktop/run_lidar.desktop ~/.config/autostart/
```
Aktifkan AUTOLOGIN:
```
sudo nano /etc/lightdm/lightdm.conf
```
isi dengan:
```
[Seat:*]
autologin-user=uppkb
autologin-user-timeout=0
```
## 🟩 13. Testing Final
Reboot:
Reboot LightDM:
```
sudo systemctl restart lightdm
```
Reboot Total:
```
sudo reboot
```

Sistem siap digunakan.

## 🔧 Troubleshooting
- RViz tidak menampilkan point cloud → set Fixed Frame: `livox_frame`
- rclpy error → reinstall setelah source ROS2
- liblivox_sdk_shared.so hilang → tambahkan LD_LIBRARY_PATH

## 📄 Lisensi
Dokumen ini digunakan untuk implementasi sistem deteksi LiDAR MID360 di lingkungan operasional Pertamina & Project Kita.
