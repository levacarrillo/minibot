#!/bin/bash

# VERIFY IF USER IS ROOT
if [ "$EUID" -ne 0 ]; then
  echo "❌ PLEASE RUN THIS SCRIPT AS A ROOT (sudo ./installer.sh)"
  exit 1
fi

echo "🔄 UPDATING PACKAGES..."
apt update && apt upgrade -y

# PACKAGE LIST TO INSTALL BY APT
APT_PACKAGES=(
  vim
  tmux
  htop
  xterm
  python3
  python3-tk
  screenfetch
  ros-jazzy-plotjuggler-ros
  ros-jazzy-rviz2
  ros-jazzy-xacro
  ros-jazzy-random-numbers
  ros-jazzy-nav2-map-server
  ros-jazzy-nav2-lifecycle-manager
)

echo "🔧 INSTALLING PACKAGES WITH APT..."
for pkg in "${APT_PACKAGES[@]}"; do
  echo "📦 INSTALLING $pkg..."
  apt install -y "$pkg"
done

echo "✅ INSTALLATION COMPLETED."
