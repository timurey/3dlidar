#!/usr/bin/env bash
# Full bootstrap for a fresh Orange Pi 5 Plus.
# Run on the Pi: bash bootstrap.sh
# Or from Mac:   ssh openclaw@192.168.1.108 "bash -s" < scripts/bootstrap.sh
set -euo pipefail

REPO_URL="https://github.com/taipov/3dlidar"   # TODO: update with actual remote
REPO_DIR="$HOME/3dlidar"
ROS2_WS="$HOME/ros2_ws"
IMU_WS="$HOME/ros_imu_ws"

info()    { echo ""; echo "▶▶ $*"; }
ok()      { echo "   ✓ $*"; }
warn()    { echo "   ⚠ $*"; }
step()    { echo "   · $*"; }

# ─────────────────────────────────────────────────────────────────────────────
info "[1/9] System packages"

# ROS2 Jazzy (skip if already installed)
if ! command -v ros2 &>/dev/null; then
  step "Adding ROS2 apt repo..."
  sudo apt-get install -y curl gnupg lsb-release
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list
  sudo apt-get update
  sudo apt-get install -y ros-jazzy-ros-base
fi

sudo apt-get install -y \
  ros-jazzy-velodyne \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-tf2-ros \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  git rsync dnsmasq

ok "apt packages installed"

# ─────────────────────────────────────────────────────────────────────────────
info "[2/9] Python packages (Pi)"

pip3 install --break-system-packages \
  flask \
  mcap \
  mcap-ros2-support \
  numpy \
  zstandard

ok "pip packages installed"

# ─────────────────────────────────────────────────────────────────────────────
info "[3/9] Clone this repo"

if [[ ! -d "$REPO_DIR/.git" ]]; then
  git clone "$REPO_URL" "$REPO_DIR"
  ok "repo cloned → $REPO_DIR"
else
  git -C "$REPO_DIR" pull --ff-only
  ok "repo updated"
fi

# ─────────────────────────────────────────────────────────────────────────────
info "[4/9] ROS2 workspace: external dependencies"

mkdir -p "$ROS2_WS/src"

if [[ ! -d "$ROS2_WS/src/HesaiLidar_ROS_2.0" ]]; then
  step "Cloning HesaiLidar_ROS_2.0 (pinned e7e112f)..."
  git clone --recurse-submodules \
    https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git \
    "$ROS2_WS/src/HesaiLidar_ROS_2.0"
  git -C "$ROS2_WS/src/HesaiLidar_ROS_2.0" checkout e7e112f
  ok "HesaiLidar cloned"
else
  ok "HesaiLidar already present"
fi

# IMU driver (optional — needed only for SLAM with wheeltec N100)
if [[ ! -d "$IMU_WS/src/ros2_wheeltec_n100_imu" ]]; then
  step "Cloning ros2_wheeltec_n100_imu..."
  mkdir -p "$IMU_WS/src"
  git clone https://github.com/tthom289/ros2_wheeltec_n100_imu.git \
    "$IMU_WS/src/ros2_wheeltec_n100_imu"
  ok "wheeltec IMU cloned"
else
  ok "wheeltec IMU already present"
fi

# ─────────────────────────────────────────────────────────────────────────────
info "[5/9] Link ROS2 packages from repo into workspace"

for pkg in slam_bringup spin_controller hmi_manager hmi_bridge encoder_bridge; do
  src="$REPO_DIR/orangepi/ros2/$pkg"
  dst="$ROS2_WS/src/$pkg"
  if [[ -d "$src" && ! -e "$dst" ]]; then
    ln -s "$src" "$dst"
    step "linked $pkg"
  fi
done

ok "packages linked"

# ─────────────────────────────────────────────────────────────────────────────
info "[6/9] System configs (network, udev, sysctl)"

SYSCONF="$REPO_DIR/orangepi/system"

# netplan (eth0 static IP for Hesai 40P — skip if no Pandar 40P)
if ls "$SYSCONF/network/"*.yaml &>/dev/null; then
  sudo cp "$SYSCONF/network/"*.yaml /etc/netplan/
  sudo chmod 600 /etc/netplan/*.yaml
  sudo netplan apply 2>/dev/null || warn "netplan apply failed (check config)"
fi

# captive portal DNS (field AP)
if [[ -f "$SYSCONF/network/dnsmasq-shared.d/captive.conf" ]]; then
  sudo mkdir -p /etc/NetworkManager/dnsmasq-shared.d
  sudo cp "$SYSCONF/network/dnsmasq-shared.d/captive.conf" \
    /etc/NetworkManager/dnsmasq-shared.d/
fi

# sysctl
sudo cp "$SYSCONF/sysctl.d/"*.conf /etc/sysctl.d/ 2>/dev/null || true
sudo sysctl --system

# udev
sudo cp "$SYSCONF/udev/"*.rules /etc/udev/rules.d/ 2>/dev/null || true
sudo udevadm control --reload-rules && sudo udevadm trigger

ok "system configs applied"

# ─────────────────────────────────────────────────────────────────────────────
info "[7/9] HMI files"

mkdir -p "$HOME/hmi/templates"
rsync -a "$REPO_DIR/orangepi/hmi/" "$HOME/hmi/"
rsync -a "$REPO_DIR/shared/offline_deskew.py" "$HOME/hmi/"
chmod +x "$HOME/hmi/start.sh"

mkdir -p "$HOME/bags" "$HOME/bags_trash" "$HOME/scripts"
cp "$REPO_DIR/orangepi/scripts/update.sh" "$HOME/scripts/"
chmod +x "$HOME/scripts/update.sh"

ok "HMI files deployed"

# ─────────────────────────────────────────────────────────────────────────────
info "[8/9] colcon build"

source /opt/ros/jazzy/setup.bash

# IMU workspace first (slam_bringup may depend on it)
if [[ -d "$IMU_WS/src" ]]; then
  cd "$IMU_WS"
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  ok "ros_imu_ws built"
fi

cd "$ROS2_WS"
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
ok "ros2_ws built"

# ─────────────────────────────────────────────────────────────────────────────
info "[9/9] systemd services"

sudo cp "$REPO_DIR/orangepi/system/systemd/"*.service /etc/systemd/system/
sudo systemctl daemon-reload

for svc in hmi hmi_bridge; do
  sudo systemctl enable "$svc"
  sudo systemctl start  "$svc" || warn "$svc failed to start (check logs)"
done

# wifi boot scripts
for svc in wifi-mode-selector wifi-regdom; do
  [[ -f "/etc/systemd/system/${svc}.service" ]] && \
    sudo systemctl enable "$svc" || true
done

# captive portal (runs as openclaw, not cave)
[[ -f "/etc/systemd/system/captive-portal.service" ]] && \
  sudo systemctl enable captive-portal || true

ok "services enabled"

# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════"
echo " Bootstrap complete!"
echo " HMI:  http://$(hostname -I | awk '{print $1}'):3000"
echo "════════════════════════════════════════"
echo ""

# Quick health check
sleep 3
if curl -sf http://localhost:3000/api/status >/dev/null; then
  echo "✓ HMI is up"
else
  warn "HMI not responding yet — check: sudo journalctl -u hmi -f"
fi
