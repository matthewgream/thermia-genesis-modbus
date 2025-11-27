#!/bin/bash
#
# thermia-publish - Monitors for a device by MAC address and publishes its IP via Avahi
#
# This script periodically scans the local network for a specific MAC address,
# and when found, publishes the IP as a .local hostname via avahi-publish-address.
#

set -euo pipefail

# Configuration (can be overridden via environment or config file)
CONFIG_FILE="${CONFIG_FILE:-/etc/default/thermia-publish}"
MAC_ADDRESS="${MAC_ADDRESS:-}"
HOSTNAME="${HOSTNAME:-thermia.local}"
INTERFACE="${INTERFACE:-}"
SCAN_INTERVAL="${SCAN_INTERVAL:-60}"
PROBE_INTERVAL="${PROBE_INTERVAL:-300}"
LOG_FACILITY="${LOG_FACILITY:-local0}"

# Load config file if exists
if [[ -f "$CONFIG_FILE" ]]; then
    source "$CONFIG_FILE"
fi

# Validate required config
if [[ -z "$MAC_ADDRESS" ]]; then
    echo "ERROR: MAC_ADDRESS not set. Set via environment or $CONFIG_FILE" >&2
    exit 1
fi

# Normalize MAC address to lowercase with colons
MAC_ADDRESS=$(echo "$MAC_ADDRESS" | tr '[:upper:]' '[:lower:]' | sed 's/-/:/g')

# State
CURRENT_IP=""
AVAHI_PID=""

log() {
    local level="$1"
    shift
    logger -t thermia-publish -p "${LOG_FACILITY}.${level}" "$*"
    echo "$(date '+%Y-%m-%d %H:%M:%S') [$level] $*"
}

cleanup() {
    log info "Shutting down..."
    if [[ -n "$AVAHI_PID" ]] && kill -0 "$AVAHI_PID" 2>/dev/null; then
        kill "$AVAHI_PID" 2>/dev/null || true
        wait "$AVAHI_PID" 2>/dev/null || true
    fi
    exit 0
}

trap cleanup SIGTERM SIGINT SIGHUP

# Find IP address for a MAC address using ARP cache and optional scan
find_ip_for_mac() {
    local mac="$1"
    local ip=""

    # First check ARP cache
    ip=$(ip neigh show | grep -i "$mac" | awk '{print $1}' | head -1)

    if [[ -n "$ip" ]]; then
        echo "$ip"
        return 0
    fi

    # If not in cache, do a network scan
    # Determine network range from interface
    local network=""
    if [[ -n "$INTERFACE" ]]; then
        network=$(ip -4 addr show "$INTERFACE" | grep -oP 'inet \K[\d.]+/\d+' | head -1)
    else
        network=$(ip -4 route | grep -v default | grep -oP '[\d.]+\.0/\d+' | head -1)
    fi

    if [[ -n "$network" ]]; then
        # Quick ping sweep to populate ARP cache
        local base_ip="${network%/*}"
        local prefix="${base_ip%.*}"
        
        # Parallel ping sweep (background, don't wait for all)
        for i in {1..254}; do
            ping -c 1 -W 1 "${prefix}.${i}" &>/dev/null &
        done
        
        # Wait a bit for responses
        sleep 2
        
        # Kill any remaining pings
        jobs -p | xargs -r kill 2>/dev/null || true
        wait 2>/dev/null || true

        # Check ARP cache again
        ip=$(ip neigh show | grep -i "$mac" | awk '{print $1}' | head -1)
    fi

    if [[ -n "$ip" ]]; then
        echo "$ip"
        return 0
    fi

    return 1
}

# Verify IP still responds and has correct MAC
verify_ip() {
    local ip="$1"
    local mac="$2"

    # Ping to refresh ARP
    if ! ping -c 1 -W 2 "$ip" &>/dev/null; then
        return 1
    fi

    # Verify MAC in ARP cache
    local current_mac
    current_mac=$(ip neigh show "$ip" 2>/dev/null | grep -oP 'lladdr \K[^ ]+' | tr '[:upper:]' '[:lower:]')
    
    if [[ "$current_mac" == "$mac" ]]; then
        return 0
    fi

    return 1
}

# Start or restart avahi-publish-address
start_avahi_publish() {
    local ip="$1"
    local hostname="$2"

    # Kill existing if running
    if [[ -n "$AVAHI_PID" ]] && kill -0 "$AVAHI_PID" 2>/dev/null; then
        kill "$AVAHI_PID" 2>/dev/null || true
        wait "$AVAHI_PID" 2>/dev/null || true
        AVAHI_PID=""
    fi

    # Start new publish
    avahi-publish-address -R -a "$hostname" "$ip" &
    AVAHI_PID=$!

    log info "Publishing $hostname -> $ip (pid $AVAHI_PID)"
}

stop_avahi_publish() {
    if [[ -n "$AVAHI_PID" ]] && kill -0 "$AVAHI_PID" 2>/dev/null; then
        kill "$AVAHI_PID" 2>/dev/null || true
        wait "$AVAHI_PID" 2>/dev/null || true
        log info "Stopped publishing $HOSTNAME"
    fi
    AVAHI_PID=""
    CURRENT_IP=""
}

main() {
    log info "Starting thermia-publish"
    log info "Looking for MAC=$MAC_ADDRESS, publishing as $HOSTNAME"
    log info "Scan interval=${SCAN_INTERVAL}s, probe interval=${PROBE_INTERVAL}s"

    local last_scan=0
    local last_probe=0

    while true; do
        local now
        now=$(date +%s)

        # If we have a current IP, periodically verify it
        if [[ -n "$CURRENT_IP" ]]; then
            if (( now - last_probe >= PROBE_INTERVAL )); then
                last_probe=$now
                if verify_ip "$CURRENT_IP" "$MAC_ADDRESS"; then
                    log debug "Verified $HOSTNAME -> $CURRENT_IP"
                else
                    log warning "Lost contact with $CURRENT_IP, rescanning..."
                    stop_avahi_publish
                fi
            fi
        fi

        # If no current IP or time for a scan, look for the device
        if [[ -z "$CURRENT_IP" ]] && (( now - last_scan >= SCAN_INTERVAL )); then
            last_scan=$now
            log debug "Scanning for MAC $MAC_ADDRESS..."

            local new_ip
            if new_ip=$(find_ip_for_mac "$MAC_ADDRESS"); then
                if [[ "$new_ip" != "$CURRENT_IP" ]]; then
                    log info "Found device at $new_ip"
                    CURRENT_IP="$new_ip"
                    start_avahi_publish "$CURRENT_IP" "$HOSTNAME"
                    last_probe=$now
                fi
            else
                log debug "Device not found"
            fi
        fi

        # Check if avahi-publish is still running
        if [[ -n "$AVAHI_PID" ]] && ! kill -0 "$AVAHI_PID" 2>/dev/null; then
            log warning "avahi-publish-address died, restarting..."
            if [[ -n "$CURRENT_IP" ]]; then
                start_avahi_publish "$CURRENT_IP" "$HOSTNAME"
            else
                AVAHI_PID=""
            fi
        fi

        sleep 5
    done
}

main "$@"
