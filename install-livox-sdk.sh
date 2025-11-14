#!/bin/bash
set -e

echo "=== 🚀 Starting Livox SDK installation ==="

echo ">>> Installing compiler tools..."
sudo apt update && sudo apt install -y build-essential cmake git

echo ">>> Preparing workspace..."
mkdir -p ~/lidar_dependencies
cd ~/lidar_dependencies

echo ">>> Cloning Livox SDK repository..."
if [ -d "Livox-SDK" ]; then
  echo "⚠️  Livox-SDK folder already exists. Skipping clone."
else
  echo ">>> Cloning Livox SDK repository..."
  git clone https://github.com/Livox-SDK/Livox-SDK.git
fi

echo ">>> Compiling Livox SDK..."
cd Livox-SDK
cd build && cmake -DCMAKE_CXX_STANDARD=11 ..
make
sudo make install

echo "=== ✅ Livox SDK installation complete! ==="