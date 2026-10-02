#!/usr/bin/env bash
# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
#
# Pre-drive check: everything that stopped a drive on 2026-09-30, in ten seconds.
#
#     ros2 run jetnano_bringup predrive.sh        -> GO or NO-GO, with every reason
#
# Checks, in the order the command travels: the robot software is up, she knows where she
# is (not on the bench), the map frame is fresh, no twist_mux lock is engaged, the collision
# guard is active, exactly one Nav2 and it is active, visual odometry flows, the motion check
# is on, the motor driver armed the ESC and enabled its outputs this boot, the pack is up to
# a drive, and whether a recording is running. Exit 0 = GO.
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
# no Fast-DDS shared memory (robot-environment/system/fastdds_udp_only.xml, 2026-10-02)
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/etc/jetnano/fastdds_udp_only.xml}
NOGO=(); WARN=()
ok()   { printf '  ok    %s\n' "$1"; }
bad()  { printf '  NO-GO %s\n' "$1"; NOGO+=("$1"); }
warn() { printf '  warn  %s\n' "$1"; WARN+=("$1"); }

systemctl is-active -q jetnano-robot.service && ok "robot software up" || bad "jetnano-robot.service is not active"

W=$(timeout 6 ros2 topic echo --once /where_am_i/state 2>/dev/null | head -1)
case "$W" in
    *'"placed"'*)  ok "placed on the map ($(echo "$W" | grep -oE 'fit [0-9.]+'))" ;;
    *'"bench"'*)   bad "she is on the bench" ;;
    *)             bad "not localised: ${W:-no verdict yet}" ;;
esac

AGE=$(timeout 12 python3 - <<'EOF' 2>/dev/null
import time, rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node('predrive_tf'); b = Buffer(); TransformListener(b, n)
t0 = time.time()
while time.time() - t0 < 4: rclpy.spin_once(n, timeout_sec=0.1)
try:
    tr = b.lookup_transform('map', 'odom', Time())
    print(round(n.get_clock().now().nanoseconds / 1e9 - Time.from_msg(tr.header.stamp).nanoseconds / 1e9, 1))
except Exception as e:
    print('none')
try:
    b.lookup_transform('map', 'base_footprint', Time())
    print('chain-ok')
except Exception:
    print('chain-broken')
rclpy.shutdown()
EOF
)
A=$(echo "$AGE" | sed -n 1p); C=$(echo "$AGE" | sed -n 2p)
if [ "$A" = "none" ] || [ -z "$A" ]; then bad "no map->odom transform"
elif awk "BEGIN{exit !($A > 5)}"; then bad "map->odom is $A s old (slam_toolbox stopped re-publishing)"
else ok "map->odom fresh ($A s)"; fi
[ "$C" = "chain-ok" ] && ok "map->base_footprint resolves" || bad "map->base_footprint does not resolve"

LOCKS=$(timeout 6 ros2 topic echo /diagnostics 2>/dev/null | grep -A1 "key: lock locks\." | paste - - | grep -oE "locks\.[a-z_]+.*(locked|free)" | sed -E 's/\s+value: .//' | sort -u)
if [ -z "$LOCKS" ]; then warn "twist_mux diagnostics not seen"
elif echo "$LOCKS" | grep -q locked; then bad "twist_mux lock engaged: $(echo "$LOCKS" | grep locked | grep -oE 'locks\.[a-z_]+' | tr '\n' ' ')"
else ok "no twist_mux lock engaged"; fi

G=$(timeout 5 ros2 lifecycle get /collision_guard 2>/dev/null | tail -1)
[[ "$G" == active* ]] && ok "collision guard active" || bad "collision guard: ${G:-unknown}"

NL=$(pgrep -fc '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch jetnano_navigation navigation.launch.py')
NC=$(pgrep -fc '^/opt/ros/jazzy/lib/rclcpp_components/component_container_isolated')
if [ "$NL" -eq 0 ] && [ "$NC" -eq 0 ]; then bad "Nav2 not running (nav2_ctl.sh start)"
elif [ "$NL" -gt 1 ] || [ "$NC" -gt 1 ]; then bad "more than one Nav2 ($NL launch, $NC container): nav2_ctl.sh stop, then start"
elif timeout 6 ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>/dev/null | grep -q "success=True"; then ok "one Nav2, active"
else bad "Nav2 running but not active"; fi

VO=0; for try in 1 2 3; do timeout 5 ros2 topic echo --once /vo >/dev/null 2>&1 && { VO=1; break; }; done   # discovery can take over 4 s (drive 15, 2026-10-01: a false NO-GO)
[ "$VO" = 1 ] && ok "visual odometry flowing" || bad "no /vo"

M=$(timeout 5 ros2 param get /safety_monitor motion.enabled 2>/dev/null | grep -oE "True|False")
[ "$M" = "True" ] && ok "motion check on" || bad "motion check is ${M:-unknown} (ros2 param set /safety_monitor motion.enabled true)"

D=$(journalctl -b -o cat 2>/dev/null | grep -E "pca9685\]" | grep -E "ESC armed|outputs (ENABLED|DISABLED)")
echo "$D" | grep -q "ESC armed" && ok "ESC armed this boot" || bad "the motor driver never armed the ESC this boot"
[ "$(echo "$D" | grep -E "outputs" | tail -1 | grep -c ENABLED)" = 1 ] && ok "servo outputs enabled" || bad "servo outputs are DISABLED"

B=$(timeout 6 ros2 topic echo --once battery/level 2>/dev/null | head -1)
V=$(echo "$B" | grep -oE '"volts": [0-9.]+' | grep -oE '[0-9.]+'); L=$(echo "$B" | grep -oE '"level": "[a-z]+"' | grep -oE '[a-z]+$')
if [ -z "$V" ]; then warn "no battery reading"
elif [ "$L" = low ] || [ "$L" = flat ]; then bad "battery $L ($V V)"
elif awk "BEGIN{exit !($V < 10.8)}"; then warn "pack at $V V: a short drive at most (low line 10.5)"
else ok "pack $V V ($L)"; fi

R=$(ros2 run jetnano_bringup drive_record.sh status 2>/dev/null | head -1)
case "$R" in recording*) ok "$R" ;; *) warn "not recording (drive_record.sh start full)" ;; esac

echo
if [ ${#NOGO[@]} -eq 0 ]; then echo "GO${WARN:+ (with ${#WARN[@]} warning(s))}"; exit 0
else echo "NO-GO: ${#NOGO[@]} reason(s)"; printf '  - %s\n' "${NOGO[@]}"; exit 1; fi
