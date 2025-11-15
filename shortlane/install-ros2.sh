#!/bin/bash
set -e

echo "=== 🚀 Starting ROS2 Humble installation ==="

# Step 1: Setup locale
echo ">>> Setting up locale..."
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Step 2: Add the ROS 2 apt repository
echo ">>> Adding ROS 2 repository..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
source /etc/os-release
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Step 3: Install ROS 2 Humble
echo ">>> Installing ROS 2 Humble Desktop..."
sudo apt update
sudo apt install -y ros-humble-desktop

# Step 4: Environment setup
echo ">>> Setting up environment..."
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# # Step 5: Install dev tools
# echo ">>> Installing development tools..."
# sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete build-essential

# # Step 6: Initialize rosdep
# echo ">>> Initializing rosdep..."
# sudo rosdep init || true
# rosdep update

echo "=== ✅ ROS2 Humble installation complete! ==="
echo "Run 'source /opt/ros/humble/setup.bash' to start using ROS2."