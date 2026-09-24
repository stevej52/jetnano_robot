#!/usr/bin/env bash
# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
#
# Wi-Fi link watchdog for the robot's Jetson.
#
# On 2026-09-23 the Orin sat "associated" to the home access point for a
# quarter of an hour with a dead data path: the supplicant saw beacons and
# logged nothing, no driver error, and nobody on the LAN could reach it
# (the mesh had just been reconfigured). Earlier the same day it spent four
# hours on the wrong network after a run of failed handshakes. Neither state
# fixes itself. This pings the gateway through the wireless interface every
# INTERVAL seconds and, after FAILURES consecutive misses, bounces the
# connection with NetworkManager. The static address survives a bounce, so
# running ROS 2 nodes keep talking; only newly started processes would care.
#
#     wifi_watchdog.sh [interface] [gateway]
#
# Run by wifi-watchdog.service (jetnano_bringup/systemd). It does nothing
# while the interface has no address (the connection is already down and
# NetworkManager is retrying on its own).

IFACE=${1:-wlP1p1s0}
GATEWAY=${2:-192.168.1.1}
INTERVAL=${WIFI_WATCHDOG_INTERVAL:-30}
FAILURES=${WIFI_WATCHDOG_FAILURES:-4}     # 4 x 30 s = 2 minutes of silence

misses=0
while true; do
    if ip -4 -o addr show dev "${IFACE}" 2>/dev/null | grep -q inet; then
        if ping -I "${IFACE}" -c 2 -i 0.5 -W 2 "${GATEWAY}" >/dev/null 2>&1; then
            [ "${misses}" -gt 0 ] && echo "wifi-watchdog: ${IFACE} reaches ${GATEWAY} again after ${misses} misses"
            misses=0
        else
            misses=$((misses + 1))
            echo "wifi-watchdog: ${IFACE} cannot reach ${GATEWAY} (${misses}/${FAILURES})"
            if [ "${misses}" -ge "${FAILURES}" ]; then
                CON=$(nmcli -t -f NAME,DEVICE con show --active 2>/dev/null | awk -F: -v i="${IFACE}" '$2==i{print $1; exit}')
                echo "wifi-watchdog: bouncing '${CON:-?}' on ${IFACE}"
                if [ -n "${CON}" ]; then
                    nmcli con down "${CON}" >/dev/null 2>&1
                    sleep 3
                    nmcli con up "${CON}" >/dev/null 2>&1 || echo "wifi-watchdog: 'nmcli con up ${CON}' failed; NetworkManager will keep retrying"
                fi
                misses=0
                sleep 30
            fi
        fi
    else
        misses=0
    fi
    sleep "${INTERVAL}"
done
