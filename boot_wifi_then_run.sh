#!/bin/bash
#
# boot_wifi_then_run.sh
#
# Boot-time launcher for SonicSole:
#   1. Wait for wlan0.
#   2. Loop until connected to Wi-Fi SSID "SonicSole" (retries forever).
#   3. Play beep.wav to signal success.
#   4. Exec ./run_RPi_combined.sh so it becomes the main process.
#
# Designed to be started by systemd (see sonicsole.service).
# Safe to run manually for testing: sudo ./boot_wifi_then_run.sh
#

set -u

TARGET_SSID="SonicSole"
RETRY_SECS=5
IFACE="wlan0"

# Resolve script directory so this works regardless of cwd.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

BEEP="$SCRIPT_DIR/beep.wav"
MAIN_SCRIPT="$SCRIPT_DIR/run_RPi_combined.sh"

log() { echo "[boot_wifi] $(date '+%F %T') $*"; }

# --- Detect which Wi-Fi stack is in use ---------------------------------------
have_nmcli=0
if command -v nmcli >/dev/null 2>&1 && systemctl is-active --quiet NetworkManager; then
    have_nmcli=1
fi

current_ssid() {
    # Prefer iwgetid (works with both NM and wpa_supplicant), fall back to nmcli.
    if command -v iwgetid >/dev/null 2>&1; then
        iwgetid -r 2>/dev/null
    elif [ "$have_nmcli" -eq 1 ]; then
        nmcli -t -f active,ssid dev wifi 2>/dev/null | awk -F: '$1=="yes"{print $2; exit}'
    fi
}

try_connect() {
    if [ "$have_nmcli" -eq 1 ]; then
        # Assumes a connection profile named "SonicSole" already exists.
        # Create it once with:
        #   sudo nmcli connection add type wifi con-name SonicSole ifname wlan0 ssid SonicSole
        #   sudo nmcli connection modify SonicSole wifi-sec.key-mgmt wpa-psk wifi-sec.psk 'PASSWORD'
        #   sudo nmcli connection modify SonicSole connection.autoconnect yes
        nmcli -w 20 connection up SonicSole >/dev/null 2>&1
    else
        # wpa_supplicant path: ask it to re-scan/reconnect.
        wpa_cli -i "$IFACE" reconfigure >/dev/null 2>&1
        wpa_cli -i "$IFACE" reconnect   >/dev/null 2>&1
    fi
}

# --- 1. Wait for the Wi-Fi interface to appear --------------------------------
log "Waiting for interface $IFACE ..."
for _ in $(seq 1 60); do
    ip link show "$IFACE" >/dev/null 2>&1 && break
    sleep 1
done

# --- 2. Retry until we are on the target SSID ---------------------------------
log "Target SSID: $TARGET_SSID"
while : ; do
    ssid="$(current_ssid)"
    if [ "$ssid" = "$TARGET_SSID" ]; then
        log "Connected to $TARGET_SSID."
        break
    fi
    log "Not connected (current: '${ssid:-none}'). Retrying in ${RETRY_SECS}s."
    try_connect
    sleep "$RETRY_SECS"
done

# --- 3. Play success beep (non-fatal if audio device isn't ready) -------------
if [ -f "$BEEP" ] && command -v aplay >/dev/null 2>&1; then
    aplay -q "$BEEP" </dev/null >/dev/null 2>&1 || log "Beep failed (continuing)."
else
    log "No beep.wav or aplay missing; skipping sound."
fi

# --- 4. Hand off to the main script -------------------------------------------
if [ ! -x "$MAIN_SCRIPT" ]; then
    log "ERROR: $MAIN_SCRIPT not executable. chmod +x it."
    exit 1
fi

log "Launching $MAIN_SCRIPT"
exec "$MAIN_SCRIPT"
