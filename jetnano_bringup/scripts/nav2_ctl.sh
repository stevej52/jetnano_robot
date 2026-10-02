#!/usr/bin/env bash
# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
#
# Nav2 on the robot, one instance only.
#
#     nav2_ctl.sh start [launch args]   -> refuses if Nav2 is already running, else starts it
#                                          detached (mode:=external nvblox:=true unless given),
#                                          log ~/audit/nav2_<date>.log, waits until active
#     nav2_ctl.sh stop                  -> SIGTERM to every Nav2 launch, then SIGKILL to any
#                                          container left after 10 s; checks with ps, not
#                                          `ros2 node list` (the daemon's cache lies)
#     nav2_ctl.sh status
#
# 2026-09-30: two Nav2 stacks ran side by side for 25 minutes (a SIGINT that was ignored,
# then a second start) - duplicate planners and controllers under the same names, and
# slam_toolbox's map->odom stopped re-publishing behind a Fast-DDS shared-memory writer
# blocked on the dead reader. Nothing drove until a reboot. Hence: check first, kill properly.
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
# no Fast-DDS shared memory (robot-environment/system/fastdds_udp_only.xml, 2026-10-02)
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/etc/jetnano/fastdds_udp_only.xml}
LAUNCH_RE='^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch jetnano_navigation navigation.launch.py'
CONTAINER_RE='^/opt/ros/jazzy/lib/rclcpp_components/component_container_isolated'
# navigation.launch.py's own Python nodes: when the launch is SIGKILLed (stop, 2026-10-01 21:36)
# they live on as orphans of init, and the next start doubles them - two nav_translators, two
# battery_homes, two nav_helpers (the duplicates Steve found again on 2026-10-01 22:10).
HELPER_RE='^/usr/bin/python3 /home/jeston/ros2_ws/install/jetnano_navigation/lib/jetnano_navigation/(nav_helper|battery_home|nav_translator|mission)( |$)'
helpers() { pgrep -f "$HELPER_RE"; }
orphan_helpers() { for p in $(helpers); do [ "$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')" = "1" ] && echo "$p"; done; }

launches()   { pgrep -f "$LAUNCH_RE"; }
containers() { pgrep -f "$CONTAINER_RE"; }

case "${1:-status}" in
    start)
        O=$(orphan_helpers)
        # shellcheck disable=SC2086
        [ -n "$O" ] && { kill -INT $O 2>/dev/null; sleep 1; echo "orphaned nav helpers stopped: $(echo $O | wc -w)"; }
        shift
        if [ -n "$(launches)$(containers)" ]; then
            echo "Nav2 is already running (launch $(launches | tr '\n' ' ')container $(containers | tr '\n' ' ')); nav2_ctl.sh stop first"
            exit 1
        fi
        ARGS="${*:-mode:=external nvblox:=true}"
        LOG="$HOME/audit/nav2_$(date +%m%d-%H%M).log"
        mkdir -p "$HOME/audit"
        # shellcheck disable=SC2086
        setsid nohup ros2 launch jetnano_navigation navigation.launch.py $ARGS > "$LOG" 2>&1 < /dev/null &
        echo "Nav2 starting ($ARGS), log $LOG"
        for _ in $(seq 1 30); do
            sleep 3
            if timeout 5 ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>/dev/null | grep -q "success=True"; then
                echo "Nav2 active after $((_ * 3)) s, $(containers | wc -l) container"
                exit 0
            fi
        done
        echo "Nav2 not active after 90 s: see $LOG"
        exit 1
        ;;
    stop)
        L=$(launches); C=$(containers)
        if [ -z "$L$C" ]; then
            H=$(helpers)
            # shellcheck disable=SC2086
            [ -n "$H" ] && { kill -INT $H 2>/dev/null; sleep 1; echo "Nav2 not running; $(echo $H | wc -w) orphaned helper(s) stopped"; } || echo "Nav2 not running"
            exit 0
        fi
        # shellcheck disable=SC2086
        [ -n "$L" ] && kill -TERM $L 2>/dev/null
        for _ in $(seq 1 10); do sleep 1; [ -z "$(launches)$(containers)" ] && break; done
        C=$(containers); L=$(launches)
        # shellcheck disable=SC2086
        [ -n "$L$C" ] && { kill -KILL $L $C 2>/dev/null; sleep 1; echo "Nav2 needed SIGKILL"; }
        H=$(helpers)
        # shellcheck disable=SC2086
        [ -n "$H" ] && { kill -INT $H 2>/dev/null; sleep 1; H=$(helpers); [ -n "$H" ] && kill -KILL $H 2>/dev/null; }
        [ -z "$(launches)$(containers)$(helpers)" ] && echo "Nav2 stopped" || { echo "Nav2 STILL running:"; ps -o pid,args -p $(launches) $(containers) $(helpers); exit 1; }
        ;;
    status)
        L=$(launches | wc -l); C=$(containers | wc -l)
        if [ "$L" -eq 0 ] && [ "$C" -eq 0 ]; then echo "Nav2 not running"; exit 0; fi
        echo "Nav2: $L launch, $C container$([ "$L" -gt 1 ] || [ "$C" -gt 1 ] && echo '  <- MORE THAN ONE: nav2_ctl.sh stop')"
        timeout 5 ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>/dev/null | grep -q "success=True" && echo "active" || echo "not active"
        ;;
    *)
        echo "usage: nav2_ctl.sh start [launch args] | stop | status"; exit 2 ;;
esac
