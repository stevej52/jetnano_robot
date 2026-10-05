#!/usr/bin/env bash
# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
#
# Nav2 on Rosie: one supervised unit, jetnano-nav2.service (jetnano_bringup/systemd), since
# 2026-10-03. This script is the wrapper everything else calls:
#
#     nav2_ctl.sh start    -> start the unit and wait until Nav2's lifecycle manager says active
#     nav2_ctl.sh stop     -> stop the unit (the launch takes its helpers with it), then make
#                             sure nothing of Nav2's is left running
#     nav2_ctl.sh status   -> the unit, the processes, and whether Nav2 is active
#
# History: until 2026-10-03 this started `ros2 launch` by hand with nohup. Two starts gave
# two planners under the same names (drive 8, 2026-09-30); a SIGKILLed launch left its Python
# helpers as orphans of init and the next start doubled them (2026-10-01); a respawned
# container came back empty (review 2026-10-03). systemd owns the process tree now.
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
# no Fast-DDS shared memory (robot-environment/system/fastdds_udp_only.xml, 2026-10-02)
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/etc/jetnano/fastdds_udp_only.xml}
UNIT=jetnano-nav2.service
LAUNCH_RE='^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch jetnano_navigation navigation.launch.py'
CONTAINER_RE='^/opt/ros/jazzy/lib/rclcpp_components/component_container_isolated'
HELPER_RE='^/usr/bin/python3 /home/jeston/ros2_ws/install/jetnano_navigation/lib/jetnano_navigation/(nav_helper|battery_home|nav_translator|mission|nav_park)( |$)'
launches()   { pgrep -f "$LAUNCH_RE"; }
containers() { pgrep -f "$CONTAINER_RE"; }
helpers()    { pgrep -f "$HELPER_RE"; }
nav2_active() { timeout 5 ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>/dev/null | grep -q "success=True"; }

case "$1" in
    start)
        if systemctl is-active --quiet "$UNIT"; then
            if nav2_active; then echo "Nav2 is already running and active"; exit 0; fi
            echo "Nav2 unit is up, waiting for it to become active"
        else
            # anything of Nav2's outside the unit (an old hand start): away first
            S=$(launches; containers; helpers)
            # shellcheck disable=SC2086
            [ -n "$S" ] && { kill -INT $S 2>/dev/null; sleep 2; echo "stopped Nav2 processes outside the unit: $(echo $S | wc -w)"; }
            sudo -n systemctl reset-failed "$UNIT" 2>/dev/null
            sudo -n systemctl start "$UNIT" || { echo "could not start $UNIT (sudo? unit installed?)"; exit 1; }
            echo "Nav2 starting ($UNIT); log: journalctl -u $UNIT"
        fi
        for i in $(seq 1 40); do
            sleep 3
            if nav2_active; then echo "Nav2 active after $((i * 3)) s, $(containers | wc -l) container"; exit 0; fi
            systemctl is-active --quiet "$UNIT" || { echo "Nav2 unit stopped while starting: journalctl -u $UNIT"; exit 1; }
        done
        echo "Nav2 not active after 120 s (no map frame? journalctl -u $UNIT)"
        exit 1
        ;;
    stop)
        if systemctl is-active --quiet "$UNIT"; then
            sudo -n systemctl stop "$UNIT"
        fi
        S=$(launches; containers; helpers)
        # shellcheck disable=SC2086
        [ -n "$S" ] && { kill -INT $S 2>/dev/null; sleep 2; S=$(launches; containers; helpers); [ -n "$S" ] && kill -KILL $S 2>/dev/null; }
        [ -z "$(launches)$(containers)$(helpers)" ] && echo "Nav2 stopped" || { echo "Nav2 STILL running:"; ps -o pid,args -p $(launches) $(containers) $(helpers); exit 1; }
        ;;
    status)
        echo "unit: $(systemctl is-active "$UNIT") (enabled: $(systemctl is-enabled "$UNIT" 2>/dev/null))"
        echo "launch: $(launches | wc -l)  container: $(containers | wc -l)  helpers: $(helpers | wc -l)"
        nav2_active && echo "Nav2: active" || echo "Nav2: not active"
        ;;
    *)
        echo "usage: nav2_ctl.sh start|stop|status" >&2
        exit 2
        ;;
esac
