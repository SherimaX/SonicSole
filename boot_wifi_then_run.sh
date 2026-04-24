#!/bin/bash
#
# boot_wifi_then_run.sh
#
# Boot-time launcher + persistent Wi-Fi watchdog for SonicSole.
#   1. Wait for wlan0 to appear.
#   2. Loop until associated with SSID "SonicSole" (open network).
#   3. Play beep.wav on (re)connect.
#   4. Launch ./run_RPi_combined.sh.
#   5. Fork a background watchdog that keeps monitoring for the lifetime of
#      the service: if the Pi drops off SonicSole, it re-selects that network
#      and keeps retrying until it's back, then beeps again.
#
# Works on Raspbian Buster (wpa_supplicant, no NetworkManager) and on newer
# releases that have nmcli. Designed to run via systemd (sonicsole.service).
# The SonicSole network is expected to already exist in
# /etc/wpa_supplicant/wpa_supplicant.conf (the user set it up once; we never
# touch that file or need a password).
#

set -u

TARGET_SSID="SonicSole"
IFACE="wlan0"
RETRY_SECS=5          # delay between reconnect attempts
MONITOR_SECS=10       # watchdog poll interval

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

BEEP="$SCRIPT_DIR/beep.wav"
MAIN_SCRIPT="$SCRIPT_DIR/run_RPi_combined.sh"

log() { echo "[boot_wifi] $(date '+%F %T') $*" >&2; }

# --- Pick the right Wi-Fi CLI -------------------------------------------------
# Prefer nmcli when NetworkManager is active; otherwise fall back to wpa_cli.
# wpa_cli lives in /sbin on Buster which isn't in pi's default PATH, so we
# resolve an absolute path.
have_nmcli=0
if command -v nmcli >/dev/null 2>&1 && systemctl is-active --quiet NetworkManager 2>/dev/null; then
    have_nmcli=1
fi

WPA_CLI=""
if [ "$have_nmcli" -eq 0 ]; then
    for p in /sbin/wpa_cli /usr/sbin/wpa_cli /usr/bin/wpa_cli wpa_cli; do
        if command -v "$p" >/dev/null 2>&1 || [ -x "$p" ]; then
            WPA_CLI="$p"
            break
        fi
    done
    # wpa_cli control socket requires being in group 'netdev' or running as
    # root. systemd should launch us as a user that qualifies.
fi

current_ssid() {
    if [ "$have_nmcli" -eq 1 ]; then
        nmcli -t -f active,ssid dev wifi 2>/dev/null \
            | awk -F: '$1=="yes"{print $2; exit}'
    elif [ -n "$WPA_CLI" ]; then
        "$WPA_CLI" -i "$IFACE" status 2>/dev/null \
            | awk -F= '$1=="ssid"{print $2; exit}'
    fi
}

sonicsole_net_id() {
    [ -n "$WPA_CLI" ] || return 0
    # list_networks output:
    #   network id / ssid / bssid / flags
    #   0\tSomeSSID\tany\t[DISABLED]
    "$WPA_CLI" -i "$IFACE" list_networks 2>/dev/null \
        | awk -F'\t' -v s="$TARGET_SSID" 'NR>1 && $2==s {print $1; exit}'
}

try_connect() {
    if [ "$have_nmcli" -eq 1 ]; then
        nmcli -w 20 connection up "$TARGET_SSID" >/dev/null 2>&1
        return
    fi
    [ -n "$WPA_CLI" ] || return 0
    local id
    id="$(sonicsole_net_id)"
    if [ -n "$id" ]; then
        # select_network enables only this network in the current session,
        # so wpa_supplicant keeps retrying SonicSole specifically.
        "$WPA_CLI" -i "$IFACE" select_network "$id" >/dev/null 2>&1
    else
        "$WPA_CLI" -i "$IFACE" reassociate >/dev/null 2>&1
    fi
}

play_beep() {
    if [ -f "$BEEP" ] && command -v aplay >/dev/null 2>&1; then
        aplay -q "$BEEP" </dev/null >/dev/null 2>&1 || \
            log "beep failed (continuing)"
    fi
}

# Turn off Wi-Fi power save. The BCM radio on the Pi parks itself between
# beacons when traffic is quiet; that batches outgoing UDP and makes the
# sender look intermittent. Interactive SSH sessions mask this because they
# keep the radio busy. Safe to re-run; silent on older kernels without iw.
disable_wifi_powersave() {
    local iw_bin=""
    for p in /sbin/iw /usr/sbin/iw /usr/bin/iw iw; do
        if [ -x "$p" ] || command -v "$p" >/dev/null 2>&1; then
            iw_bin="$p"; break
        fi
    done
    # `iw set` needs CAP_NET_ADMIN. When the service is started by systemd with
    # AmbientCapabilities=CAP_NET_ADMIN this works as pi user directly; when
    # run manually, fall through to sudo -n.
    _try_iw() {
        local bin="$1"
        "$bin" dev "$IFACE" set power_save off >/dev/null 2>&1 && return 0
        sudo -n "$bin" dev "$IFACE" set power_save off >/dev/null 2>&1
    }
    if [ -n "$iw_bin" ] && _try_iw "$iw_bin"; then
        log "Wi-Fi power save off."
        return 0
    fi
    # Fallback: iwconfig is available on some images even when iw isn't.
    if command -v iwconfig >/dev/null 2>&1; then
        if iwconfig "$IFACE" power off >/dev/null 2>&1 \
           || sudo -n iwconfig "$IFACE" power off >/dev/null 2>&1; then
            log "Wi-Fi power save off (via iwconfig)."
            return 0
        fi
    fi
    log "Wi-Fi power save toggle failed (continuing)."
}

wait_until_connected() {
    while : ; do
        local ssid
        ssid="$(current_ssid)"
        if [ "$ssid" = "$TARGET_SSID" ]; then
            return 0
        fi
        log "Not connected (current: '${ssid:-none}'). Retrying in ${RETRY_SECS}s."
        try_connect
        sleep "$RETRY_SECS"
    done
}

# --- 1. Wait for interface ----------------------------------------------------
log "Waiting for interface $IFACE ..."
for _ in $(seq 1 60); do
    ip link show "$IFACE" >/dev/null 2>&1 && break
    sleep 1
done

# --- 2. Initial connect -------------------------------------------------------
log "Target SSID: $TARGET_SSID (nmcli=$have_nmcli, wpa_cli=${WPA_CLI:-none})"
wait_until_connected
log "Connected to $TARGET_SSID."
disable_wifi_powersave
play_beep

# --- 3. Background watchdog: reconnect forever --------------------------------
(
    while : ; do
        sleep "$MONITOR_SECS"
        ssid="$(current_ssid)"
        if [ "$ssid" != "$TARGET_SSID" ]; then
            log "[watchdog] lost connection (current: '${ssid:-none}'); reconnecting"
            wait_until_connected
            log "[watchdog] reconnected to $TARGET_SSID"
            disable_wifi_powersave
            play_beep
        fi
    done
) &
WATCHDOG_PID=$!
log "Watchdog running (pid $WATCHDOG_PID)."

cleanup() {
    kill "$WATCHDOG_PID" 2>/dev/null || true
    wait "$WATCHDOG_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# --- 4. Run main program ------------------------------------------------------
if [ ! -x "$MAIN_SCRIPT" ]; then
    log "ERROR: $MAIN_SCRIPT not executable."
    exit 1
fi

log "Launching $MAIN_SCRIPT"
"$MAIN_SCRIPT"
rc=$?
log "Main exited (rc=$rc)."
exit "$rc"
