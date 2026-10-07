#!/usr/bin/env bash
# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
#
# Record a drive, and get the report when it stops.
#
#     drive_record.sh start          -> ~/bags/drive-<date>/  (a few MB a minute)
#     drive_record.sh start full     -> the same plus everything else worth looking
#                                       back at (~40 MB a minute and ~15 MB of pictures),
#                                       see FULL below; for a test drive, not every drive
#     drive_record.sh stop           -> stops ALL of it; H2-Host analyses the recording
#                                       within minutes (http://192.168.1.238:8087/)
#     drive_record.sh stop report    -> the same, plus drive_report run HERE (when H2-Host
#                                       is off). Not the default since 2026-09-30: on
#                                       drive 5 (64 min, 1.8 GB) drive_report grew to
#                                       2.9 GB and the OOM killer took it, with 35 MB
#                                       left for everything else (ears, IMU and odometry
#                                       all restarted, the microphone wedged).
#     drive_record.sh status
#
# The plain recording holds the commands, the odometry (VO and EKF), the IMU, the
# lidar, the collision guard's decisions and TF - everything drive_report reads - and
# no images. The recorder is started detached and told to stop with SIGTERM (SIGINT
# is ignored by a background job from a non-interactive shell, which is how the first
# recorder had to be power-cycled away).
#
# FULL (2026-09-28, Steve: "test everything, log everything while we're driving so
# we can look back at it better, but turn it off when we're done") adds:
#   - the whole command chain from Nav2's controller to the servo board, every stop
#     and guard, MOLA's own pose and quality, where_am_i's verdicts, SLAM's pose, the
#     map, Nav2's goal, plans, behaviour tree, costmaps and feedback, the camera's
#     obstacle points, the watchdog, diagnostics and every node's log (/rosout)
#   - drive-<date>-extra/:
#       tegrastats.log           every second: CPU and GPU load and clocks, the
#                                temperatures, the power rails
#       system.csv, processes.csv, frames/, drive_log.txt
#                                drive_log: CPU per core, memory, the Wi-Fi (access
#                                point, signal, bit rates), every busy process, and a
#                                picture a second from each camera
#       rates-start.txt, rates-end.txt   the watchdog's rate of every stream
#       journal.txt              the system journal (kernel included) for the drive
# `stop` stops the recorder, drive_log and tegrastats and says so; drive_log stops
# the lot by itself after two hours in case nobody did.
# Trimmed after the first full drive (2026-09-28, 22 min, 985 MB, the recorder 30-37 %
# of a core): nvblox's obstacle points (35 a second, half the bag) are replaced by the
# same grid laid on the map twice a second (/nvblox_node/map_grid, what the planner
# sees); Nav2's display-only topics (the footprint 19 a second, the followed path and
# collision arc 20 a second while driving) are out - /plan and the lookahead point stay.

BAGS=${DRIVE_BAGS:-$HOME/bags}
# + what the planner saw (drive 41, 2026-10-06: a gap closed at turn three and nothing recorded why;
# ~150 KB/s, mostly the camera grid)
TOPICS="/cmd_vel /cmd_vel_mux /cmd_vel_web /vo /lidar_odom /odometry/filtered /odom_hold /imu/data /collision_guard/state /scan /tf /tf_static
    /global_costmap/costmap /global_costmap/costmap_updates /nvblox_node/map_grid /plan"
FULL_TOPICS="$TOPICS
    /cmd_vel_teleop /cmd_vel_settle /cmd_vel_tilt /cmd_vel_nav /cmd_vel_nav_raw /cmd_vel_nav_smoothed
    /cmd_vel_nav_mps /joint_states /motion/mode
    /e_stop /e_stop_joy /e_stop_web /e_stop_motion /cliff/drop /safety_monitor/tilt_status
    /motion_check/state /motion_check/pause /collision_monitor_state
    /lidar_odometry/pose /lidar_odometry/pose_quality /mola_diagnostics/lidar_odom/status
    /scan_raw /battery /visual_slam/status
    /map /map_metadata /pose /initialpose /where_am_i/state
    /goal_pose /plan_smoothed /local_plan /lookahead_point /speed_limit /behavior_tree_log
    /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback
    /local_costmap/costmap /local_costmap/costmap_updates
    /watchdog/status /watchdog/events /watchdog/topics /diagnostics /rosout /parameter_events
    /say /speak /robot_description"
STOP_AFTER=7200     # s; drive_log's backstop

mkdir -p "$BAGS"
# Everything is found by its command line, not a saved PID: a background job's PID
# is the shell that started it, and signalling that orphans the recorder with the
# bag half written. The brackets keep pgrep from matching a shell that mentions them.
recorder_pid() { pgrep -f "ros2 bag record -o $BAGS/drive-" | head -1; }
helper_pids() { pgrep -f "[t]egrastats --interval 1000 --logfile $BAGS/"; pgrep -f "[d]rive_log $BAGS/"; }
left_running() { { recorder_pid; helper_pids; } | while read -r p; do ps -o pid=,args= -p "$p" | cut -c1-110; done; }

stop_pids() {       # SIGTERM, then wait up to 15 s for each
    for p in "$@"; do kill -TERM "$p" 2>/dev/null; done
    for p in "$@"; do
        for _ in $(seq 1 30); do kill -0 "$p" 2>/dev/null || break; sleep 0.5; done
    done
}

case "${1:-status}" in
    start)
        if [ -n "$(recorder_pid)" ]; then echo "already recording ($(cat "$BAGS/current" 2>/dev/null))"; exit 0; fi
        NAME=drive-$(date +%Y%m%d-%H%M%S)
        echo "$BAGS/$NAME" > "$BAGS/current"
        if [ "${2:-}" = full ]; then
            EXTRA="$BAGS/$NAME-extra"
            mkdir -p "$EXTRA"
            date +%s > "$EXTRA/started"
            setsid nohup ros2 bag record -o "$BAGS/$NAME" --include-hidden-topics $FULL_TOPICS \
                > "$BAGS/$NAME.log" 2>&1 < /dev/null &
            setsid nohup tegrastats --interval 1000 --logfile "$EXTRA/tegrastats.log" > /dev/null 2>&1 < /dev/null &
            setsid nohup "$(ros2 pkg prefix jetnano_bringup)/lib/jetnano_bringup/drive_log" "$EXTRA" \
                --stop-after $STOP_AFTER --stop-cmd "bash $(readlink -f "$0") stop" \
                > "$EXTRA/drive_log.err" 2>&1 < /dev/null &
            timeout 20 ros2 run jetnano_bringup rates > "$EXTRA/rates-start.txt" 2>&1
            sleep 3
            if [ -z "$(recorder_pid)" ]; then
                echo "recorder did not start:"; tail -3 "$BAGS/$NAME.log"
                # shellcheck disable=SC2046
                stop_pids $(helper_pids); exit 1
            fi
            echo "recording $BAGS/$NAME   (+ $EXTRA)"
            [ -n "$(pgrep -f "[t]egrastats --interval 1000 --logfile $EXTRA/")" ] && echo "  tegrastats: on" || echo "  tegrastats: DID NOT START"
            [ -n "$(pgrep -f "[d]rive_log $EXTRA")" ] && echo "  drive_log:  on" || { echo "  drive_log:  DID NOT START"; tail -3 "$EXTRA/drive_log.err"; }
            # the check: which topics the recorder has found (Nav2's appear when Nav2 starts)
            sleep 4
            found=$(grep -o "Subscribed to topic '[^']*'" "$BAGS/$NAME.log" | sed "s/.*'\(.*\)'/\1/" | sort -u)
            missing=""
            for t in $FULL_TOPICS; do echo "$found" | grep -qx "$t" || missing="$missing $t"; done
            echo "  recording $(echo "$found" | grep -c .) topics; not published (yet):$missing"
            echo "  watchdog at the start:"
            sed 's/^/    /' "$EXTRA/rates-start.txt"
        else
            setsid nohup ros2 bag record -o "$BAGS/$NAME" $TOPICS > "$BAGS/$NAME.log" 2>&1 < /dev/null &
            sleep 3
            [ -n "$(recorder_pid)" ] && echo "recording $BAGS/$NAME" || { echo "recorder did not start:"; tail -3 "$BAGS/$NAME.log"; exit 1; }
        fi
        ;;
    stop)
        PID=$(recorder_pid)
        HELPERS=$(helper_pids)
        if [ -z "$PID" ] && [ -z "$HELPERS" ]; then echo "not recording"; exit 0; fi
        BAG=$(cat "$BAGS/current")
        EXTRA="$BAG-extra"
        [ -d "$EXTRA" ] && [ -n "$PID" ] && timeout 20 ros2 run jetnano_bringup rates > "$EXTRA/rates-end.txt" 2>&1
        [ -n "$PID" ] && stop_pids "$PID"
        # metadata.yaml is the last thing the recorder writes
        for _ in $(seq 1 10); do [ -f "$BAG/metadata.yaml" ] && break; sleep 0.5; done
        # shellcheck disable=SC2086
        [ -n "$HELPERS" ] && stop_pids $HELPERS
        LEFT=$(left_running)
        if [ -d "$EXTRA" ]; then
            journalctl --since "@$(cat "$EXTRA/started")" --no-pager -o short-iso-precise > "$EXTRA/journal.txt" 2>&1
        fi
        if [ "${2:-}" = report ]; then
            echo "stopped; report for $BAG:"
            echo
            nice -n 10 timeout 900 ros2 run jetnano_bringup drive_report "$BAG"
            echo
        else
            echo "stopped: $BAG ($(du -sh "$BAG" | cut -f1))"
            echo "H2-Host pulls and analyses it within minutes: http://192.168.1.238:8087/"
            echo "(drive_record.sh stop report would run drive_report here, 2-3 GB of memory)"
        fi
        if [ -d "$EXTRA" ]; then
            echo "kept: $(du -sh "$BAG" | cut -f1) bag, $(du -sh "$EXTRA" | cut -f1) extra" \
                 "($(find "$EXTRA/frames" -name '*.jpg' 2>/dev/null | wc -l) pictures)"
        fi
        if [ -z "$LEFT" ]; then echo "all drive logging is off"; else echo "STILL RUNNING:"; echo "$LEFT"; exit 1; fi
        ;;
    status)
        PID=$(recorder_pid)
        if [ -n "$PID" ]; then
            BAG=$(cat "$BAGS/current" 2>/dev/null)
            echo "recording $BAG for $(ps -o etime= -p "$PID" | tr -d ' '), $(du -sh "$BAG" 2>/dev/null | cut -f1)"
            [ -d "$BAG-extra" ] && echo "  full: $(helper_pids | wc -l) of 2 helpers running," \
                "$(find "$BAG-extra/frames" -name '*.jpg' 2>/dev/null | wc -l) pictures"
        else
            echo "not recording"
        fi
        if [ -z "$PID" ] && [ -n "$(helper_pids)" ]; then echo "but still running:"; left_running; fi
        ;;
    *)
        echo "usage: drive_record.sh start [full]|stop|status" >&2; exit 2 ;;
esac
