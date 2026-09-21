#!/usr/bin/env bash
# Deploy code from Mac → Orange Pi.
# Usage: ./scripts/deploy.sh [hmi|ros2|all]   (default: all)
set -euo pipefail

PI=openclaw@192.168.1.108
TARGET="${1:-all}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

info()  { echo "▶ $*"; }
ok()    { echo "✓ $*"; }

if [[ "$TARGET" == "hmi" || "$TARGET" == "all" ]]; then
  info "rsync orangepi/hmi/"
  rsync -av --delete --exclude='__pycache__' \
    "$ROOT/orangepi/hmi/" "$PI:~/hmi/"
  rsync -av "$ROOT/shared/offline_deskew.py" "$PI:~/hmi/offline_deskew.py"
  ok "hmi synced"
fi

if [[ "$TARGET" == "ros2" || "$TARGET" == "all" ]]; then
  info "rsync orangepi/ros2/"
  rsync -av --delete --exclude='__pycache__' --exclude='*.pyc' \
    "$ROOT/orangepi/ros2/" "$PI:~/ros2_ws/src/"
  ok "ros2 packages synced"

  info "colcon build on Pi"
  ssh "$PI" "bash -lc '
    source /opt/ros/jazzy/setup.bash
    cd ~/ros2_ws
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -10
  '"
  ok "build done"
fi

if [[ "$TARGET" == "system" ]]; then
  info "rsync orangepi/system/ (requires sudo on Pi)"
  rsync -av "$ROOT/orangepi/system/systemd/"  "$PI:/tmp/systemd-new/"
  ssh "$PI" "sudo cp /tmp/systemd-new/*.service /etc/systemd/system/ && sudo systemctl daemon-reload"
  ok "systemd units updated"
fi

# Restart services
info "restarting services ($TARGET)"
if [[ "$TARGET" == "hmi" ]]; then
  ssh "$PI" "sudo systemctl restart hmi" && ok "hmi restarted"
elif [[ "$TARGET" == "ros2" ]]; then
  ssh "$PI" "sudo systemctl restart hmi_bridge slam-scanner 2>/dev/null || true" && ok "ros2 services restarted"
elif [[ "$TARGET" == "all" ]]; then
  ssh "$PI" "sudo systemctl restart hmi; sudo systemctl restart hmi_bridge slam-scanner 2>/dev/null || true"
  ok "all services restarted"
fi

echo ""
echo "✓ Deploy complete → $PI ($TARGET)"
