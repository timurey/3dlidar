#!/bin/bash
# Boot-time wifi mode selector (single radio, boot-time-only policy):
#   - If the home wifi SSID is in range and NetworkManager connected to it -> stay a client.
#   - Otherwise -> bring up the field AP ("hotspot"), which also starts the captive portal DNS.
# eth0 (lidar, 192.168.3.x, NM-unmanaged) is never touched here.
set -u
HOME_SSID="internet"
AP_CON="hotspot"
log(){ echo "wifi-mode-selector: $*"; }

# Regulatory domain — required for legal AP channel/power selection.
iw reg set US 2>/dev/null || log "WARN: 'iw reg set US' failed (continuing)"

# Let NetworkManager finish startup + autoconnect attempts (home wifi if present).
nm-online -s -t 30 >/dev/null 2>&1

# What SSID, if any, is wlan0 actually connected to right now?
active_ssid="$(nmcli -t -f ACTIVE,SSID dev wifi 2>/dev/null | awk -F: '$1=="yes"{print $2; exit}')"

if [ "$active_ssid" = "$HOME_SSID" ]; then
    log "home wifi '$HOME_SSID' connected — client mode, AP not started"
    exit 0
fi

log "home wifi '$HOME_SSID' not connected (active='${active_ssid:-none}') — starting field AP '$AP_CON'"
nmcli connection up "$AP_CON" || { log "ERROR: failed to bring up AP '$AP_CON'"; exit 1; }
