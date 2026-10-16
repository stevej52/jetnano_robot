#!/usr/bin/env bash
# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
#
# What the computer is doing while a goal starts (drive.sh runs it as the route begins).
#
#     goal_start_profile.sh OUT_DIR [SECONDS]      (default 15 s)
#
# Why (2026-10-15): within ~3 s of a goal's start Nav2's own TF listener has fallen 0.16-0.45 s
# behind (drives 57, 59, 62; 5 starts since 10-07) while the EKF's odom->base_footprint reached
# the drive bag every 10 ms - so something inside or around the Nav2 container stalls. The
# transform tolerances (0.5 s) hide it; this is to find it. Into OUT_DIR, side by side:
#   top.txt           every core's load and the busiest processes, 4 times a second
#   nav2_threads.txt  the Nav2 container's threads, 4 times a second
#   nav2.perf.data    perf samples with call stacks - only where perf works (not on her
#                     6.8.12-tegra kernel: no linux-tools for it; the thread names name DDS's)
# Line them up with the drive's journal ("extrapolation", "invalid source") by time.
OUT=${1:?OUT_DIR}
SECS=${2:-15}
mkdir -p "$OUT"
N=$((SECS * 4))
PID=$(pgrep -f '^/opt/ros/jazzy/lib/rclcpp_components/component_container_isolated .*__node:=nav2_container' | head -1)
date +%T.%N > "$OUT/started"
# top: the header, one line per core (-1) and the 15 busiest each sample (-w: whole names)
top -1 -b -d 0.25 -n "$N" -w 200 -o %CPU | awk '/^top -/{c=0} {if (c++ < 28) print}' > "$OUT/top.txt" 2>&1 &
if [ -n "$PID" ]; then
    echo "nav2 container pid $PID" >> "$OUT/started"
    top -H -b -d 0.25 -n "$N" -w 200 -o %CPU -p "$PID" | awk '/^top -/{c=0} {if (c++ < 20) print}' > "$OUT/nav2_threads.txt" 2>&1 &
    if sudo -n perf --version > /dev/null 2>&1; then
        sudo -n perf record -q -F 199 -g -p "$PID" -o "$OUT/nav2.perf.data" -- sleep "$SECS" > "$OUT/perf.log" 2>&1 &
    fi
else
    echo "no Nav2 container running" >> "$OUT/started"
fi
wait
sudo -n chown "$(id -u):$(id -g)" "$OUT/nav2.perf.data" 2>/dev/null
date +%T.%N > "$OUT/finished"
