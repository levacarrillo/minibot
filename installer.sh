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
  screenfetch
  ros-jazzy-xacro
  ros-jazzy-random-numbers
  ros-jazzy-nav2-map-server
)

echo "🔧 INSTALLING PACKAGES WITH APT..."
for pkg in "${APT_PACKAGES[@]}"; do
  echo "📦 INSTALLING $pkg..."
  apt install -y "$pkg"
done

# VERIFY IF STAP IS INSTALLED
if ! command -v snap &> /dev/null; then
  echo "⚠️ SNAP IS NOT INSTALLED. INSTALLING SNAP..."
  apt install -y snapd
fi

# SNAP LIST TO INSTALL
SNAP_PACKAGES=(
  code
)

echo "🔧 INSTALLING PACKAGES WITH SNAP..."
for snap_pkg in "${SNAP_PACKAGES[@]}"; do
  echo "📦 INSTALLING $snap_pkg..."
  snap install $snap_pkg
done

echo "✅ INSTALLATION COMPLETED."
