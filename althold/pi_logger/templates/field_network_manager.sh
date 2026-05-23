#!/usr/bin/env bash
set -euo pipefail

# Template only. Do not run from the repo without adapting SSIDs and credentials.
# Desired field behavior:
# 1. Try smartphone hotspot.
# 2. If unavailable, try home Wi-Fi.
# 3. If both fail, open the Pi access point.
# 4. Bring Tailscale up only when connected to the smartphone hotspot.

HOTSPOT_SSID="${HOTSPOT_SSID:-phone-hotspot}"
HOME_SSID="${HOME_SSID:-home-wifi}"
AP_PROFILE="${AP_PROFILE:-althold-ap}"

if nmcli -t -f active,ssid dev wifi | grep -q "^yes:${HOTSPOT_SSID}$"; then
  tailscale up --accept-routes=false
  exit 0
fi

if nmcli dev wifi connect "$HOTSPOT_SSID"; then
  tailscale up --accept-routes=false
  exit 0
fi

if nmcli dev wifi connect "$HOME_SSID"; then
  tailscale down || true
  exit 0
fi

tailscale down || true
nmcli connection up "$AP_PROFILE"
