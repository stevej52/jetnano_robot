#!/usr/bin/env bash
# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
#
# Record a drive, and get the report when it stops.
#
#     drive_record.sh start          -> ~/bags/drive-<date>/  (a few MB a minute)
#     drive_record.sh stop           -> closes the bag and prints drive_report
#     drive_record.sh status
#
# Records the commands, the odometry (VO and EKF), the IMU, the lidar, the
# collision guard's decisions and TF - everything drive_report reads - and no
# images. The recorder is started detached and told to stop with SIGTERM
# (SIGINT is ignored by a background job from a non-interactive shell, which
# is how the first recorder had to be power-cycled away).

BAGS=${DRIVE_BAGS:-$HOME/bags}
TOPICS="/cmd_vel /cmd_vel_mux /cmd_vel_web /vo /odometry/filtered /odom_hold /imu/data /collision_guard/state /scan /tf /tf_static"

mkdir -p "$BAGS"
# The recorder is found by its command line, not a saved PID: a background
# job's PID is the shell that started it, and signalling that orphans the
# recorder with the bag half written.
recorder_pid() { pgrep -f "ros2 bag record -o $BAGS/drive-" | head -1; }

case "${1:-status}" in
    start)
        if [ -n "$(recorder_pid)" ]; then echo "already recording ($(cat "$BAGS/current" 2>/dev/null))"; exit 0; fi
        NAME=drive-$(date +%Y%m%d-%H%M%S)
        echo "$BAGS/$NAME" > "$BAGS/current"
        setsid nohup ros2 bag record -o "$BAGS/$NAME" $TOPICS > "$BAGS/$NAME.log" 2>&1 < /dev/null &
        sleep 3
        [ -n "$(recorder_pid)" ] && echo "recording $BAGS/$NAME" || { echo "recorder did not start:"; tail -3 "$BAGS/$NAME.log"; exit 1; }
        ;;
    stop)
        PID=$(recorder_pid)
        if [ -z "$PID" ]; then echo "not recording"; exit 0; fi
        kill -TERM "$PID"
        for _ in $(seq 1 30); do kill -0 "$PID" 2>/dev/null || break; sleep 0.5; done
        BAG=$(cat "$BAGS/current")
        # metadata.yaml is the last thing the recorder writes
        for _ in $(seq 1 10); do [ -f "$BAG/metadata.yaml" ] && break; sleep 0.5; done
        echo "stopped; report for $BAG:"
        echo
        ros2 run jetnano_bringup drive_report "$BAG"
        ;;
    status)
        if [ -n "$(recorder_pid)" ]; then echo "recording $(cat "$BAGS/current" 2>/dev/null)"; else echo "not recording"; fi
        ;;
    *)
        echo "usage: drive_record.sh start|stop|status" >&2; exit 2 ;;
esac
