#!/usr/bin/env bash
# OTA update — runs on the Pi.
# Triggered by: HMI button, SSH, or cron.
# Usage: ~/scripts/update.sh [--dry-run]
set -euo pipefail

REPO_DIR="$HOME/3dlidar"
LOG="$HOME/update.log"
DRY="${1:-}"

log() { echo "[$(date '+%H:%M:%S')] $*" | tee -a "$LOG"; }

exec > >(tee -a "$LOG") 2>&1
echo "" >> "$LOG"
log "=== UPDATE STARTED ==="

if [[ ! -d "$REPO_DIR/.git" ]]; then
  log "ERROR: repo not found at $REPO_DIR — run bootstrap.sh first"
  exit 1
fi

cd "$REPO_DIR"

log "git pull..."
git fetch origin
BEFORE=$(git rev-parse HEAD)
git pull --ff-only
AFTER=$(git rev-parse HEAD)

if [[ "$BEFORE" == "$AFTER" ]]; then
  log "Already up to date ($(git rev-parse --short HEAD))"
  log "UPDATE_DONE $(date -Iseconds)"
  exit 0
fi

log "Updated $(git rev-parse --short "$BEFORE")..$(git rev-parse --short "$AFTER")"

CHANGED=$(git diff --name-only "$BEFORE" "$AFTER")
log "Changed files:"
echo "$CHANGED" | sed 's/^/  /' | tee -a "$LOG"

REBUILD_ROS=false
REDEPLOY_HMI=false

echo "$CHANGED" | grep -q "^orangepi/ros2/" && REBUILD_ROS=true
echo "$CHANGED" | grep -qE "^orangepi/hmi/|^shared/" && REDEPLOY_HMI=true

if [[ "$DRY" == "--dry-run" ]]; then
  log "DRY RUN — rebuild_ros=$REBUILD_ROS  redeploy_hmi=$REDEPLOY_HMI"
  log "UPDATE_DONE $(date -Iseconds)"
  exit 0
fi

if $REBUILD_ROS; then
  log "Rebuilding ROS2 packages..."
  source /opt/ros/jazzy/setup.bash
  cd "$HOME/ros2_ws"
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -20
  log "ROS2 build done"
  sudo systemctl restart hmi_bridge 2>/dev/null || true
  sudo systemctl restart slam-scanner 2>/dev/null || true
  log "ROS2 services restarted"
  cd "$REPO_DIR"
fi

if $REDEPLOY_HMI; then
  log "Redeploying HMI..."
  rsync -a --delete --exclude='__pycache__' \
    "$REPO_DIR/orangepi/hmi/" "$HOME/hmi/"
  rsync -a "$REPO_DIR/shared/offline_deskew.py" "$HOME/hmi/"
  sudo systemctl restart hmi
  log "HMI redeployed and restarted"
fi

log "=== UPDATE_DONE $(date -Iseconds) ==="
