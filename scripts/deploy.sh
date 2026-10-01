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
  rsync -avL "$ROOT/shared/offline_deskew.py" "$PI:~/hmi/offline_deskew.py"
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
# Flask HMI (hmi.service): NOPASSWD sudo granted for this unit.
# hmi_bridge: restarted via HMI API (POST /api/sensors/restart_all).
#   Wait up to 8 s for Flask to come up after restart before calling API.
info "restarting services ($TARGET)"

_wait_hmi_up() {
  local host; host="$(echo "$PI" | cut -d@ -f2)"
  for i in 1 2 3 4; do
    sleep 2
    if curl -sf "http://${host}:3000/api/status" -o /dev/null 2>/dev/null; then
      return 0
    fi
  done
  return 1
}

_restart_hmi_bridge() {
  local host; host="$(echo "$PI" | cut -d@ -f2)"
  local body
  body=$(curl -sf -X POST "http://${host}:3000/api/sensors/restart_all" 2>/dev/null) || {
    echo "  warning: HMI API unreachable — hmi_bridge not restarted"
    return 1
  }
  local msg; msg=$(echo "$body" | python3 -c "import sys,json; print(json.load(sys.stdin).get('msg','?'))" 2>/dev/null || echo "$body")
  echo "  hmi_bridge: $msg"
}

if [[ "$TARGET" == "hmi" || "$TARGET" == "all" ]]; then
  ssh "$PI" "sudo systemctl restart hmi" && ok "hmi restarted"
  info "waiting for HMI to come up..."
  if _wait_hmi_up; then
    ok "HMI is up"
  else
    echo "  warning: HMI did not respond within 8 s"
  fi
fi
if [[ "$TARGET" == "ros2" || "$TARGET" == "all" ]]; then
  # When 'all': HMI was just (re)started, may need a moment even after _wait_hmi_up
  [[ "$TARGET" == "all" ]] && sleep 1
  if _restart_hmi_bridge; then
    ok "hmi_bridge restart requested via API"
  fi
fi

echo ""
echo "✓ Deploy complete → $PI ($TARGET)"
