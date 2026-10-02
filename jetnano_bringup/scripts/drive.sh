#!/usr/bin/env bash
# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
#
# One autonomous drive, the same way every time (Steve, 2026-10-01: "make it smoother and
# learn more from each one").
#
#     drive.sh N                       -> the house lap (kitchen island + dining table), then park
#     drive.sh N X Y H  X Y H ...      -> that route instead (map metres and degrees), then park
#     drive.sh N --no-park ...         -> route only
#     drive.sh N --force ...           -> drive on a NO-GO from predrive (you were warned)
#
# Steps: predrive.sh (GO/NO-GO), rail_check (the head), record on, nav_route, nav_park,
# record off. Everything lands in ~/audit/<date>/driveN/ (console.log, route.log, park.log,
# drive.json) and drive.json is copied into the bag, so H2-Host's analysis can pair the
# bag with the run and the scorecard knows which drive it is scoring.
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
S="$HOME/ros2_ws/src/jetnano_robot/jetnano_bringup/scripts"

N=$1; shift
[ -n "$N" ] || { echo "usage: drive.sh N [--no-park] [--force] [X Y H ...]" >&2; exit 2; }
PARK=1; FORCE=0; RAIL=0
while [ "${1#--}" != "$1" ]; do
    case "$1" in
        --no-park) PARK=0 ;;
        --force)   FORCE=1 ;;
        --rail)    RAIL=1 ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
# the lap as driven 2026-09-30 (drives 11-14): down the hall, round the island, round the
# table, back up the hall to (2.0, -2.0) pointing at the parking spot
# ... and ends at nav_park's arc start for a lap coming down the hall (0.85, -0.81, 134), so the
# parking carries straight on: no 15 s stand at (2, -2) while nav_park starts and plans (drives 17/19)
LAP="2.8 -3.5 -90  1.2 -4.4 180  -0.45 -5.6 -90  1.5 -7.3 0  4.5 -7.5 0  6.5 -7.7 20  7.05 -6.5 90  5.5 -5.5 180  3.2 -4.3 110  0.85 -0.81 134"
ROUTE="${*:-$LAP}"

D="$HOME/audit/$(date +%F)/drive$N"
mkdir -p "$D"
exec > >(tee -a "$D/console.log") 2>&1
echo "== drive $N  $(date '+%F %T')"
echo "   route: $ROUTE"

echo "== preflight  $(date +%T)"
# tests are driven with her voice and hearing unloaded (Steve, 2026-10-01: the microphone's USB
# audio stream panicked the kernel mid-lap). `voice on` brings them back.
ros2 run jetnano_bringup voice off 2>/dev/null | sed 's/^/   /'
# The ros2 CLI's daemon must know the graph before predrive asks it anything: cold, it
# reported "no /vo" and "not localised" on streams that were fine (drive 15, 2026-10-01).
ros2 daemon start > /dev/null 2>&1
for try in 1 2 3 4 5 6; do
    NODES=$(ros2 node list 2>/dev/null | wc -l)     # not N: that is the drive number (drive 17 was filed as "drive 42")
    [ "$NODES" -ge 20 ] && break
    sleep 3
done
echo "   ros2 graph: $NODES nodes known to the daemon  $(date +%T)"
# Nav2 is not part of the robot service: start it (one instance, nav2_ctl.sh refuses a second)
if "$S/nav2_ctl.sh" status 2>/dev/null | grep -qi "not running"; then
    echo "   Nav2 not running: starting it"
    "$S/nav2_ctl.sh" start 2>&1 | tail -2 | sed 's/^/   /'
fi
echo "   predrive  $(date +%T)"
# jetnano_bringup/predrive.py: the same checks as predrive.sh in one process, seconds not a minute
if ! ros2 run jetnano_bringup predrive; then
    if [ "$FORCE" = 1 ]; then echo "   NO-GO overridden (--force)"; else echo "   NO-GO: not driving"; exit 1; fi
fi
# the rail check (a head-camera pan, 2 s) is OFF by default since 2026-10-01 21:00: it starts and
# stops the head camera's pipeline, and that teardown is the known nvgpu kernel-panic trigger
# (2026-09-28); drive 20's reset came 2 s after it. --rail turns it on.
if [ "$RAIL" = 1 ]; then
    echo "   rail check  $(date +%T)"
    ros2 run jetnano_bringup rail_check 2>/dev/null | tail -1 | sed 's/^/   /'
fi
# she must start ON the spot: drive 16 (2026-10-01) began 1 m off it, nose against the couch,
# and Nav2 could not move her at all
START=$(python3 - <<'EOF'
import math, rclpy, tf2_ros
rclpy.init(); n = rclpy.create_node('start_check'); buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
end = n.get_clock().now().nanoseconds + int(4e9)
while n.get_clock().now().nanoseconds < end and not buf.can_transform('map', 'base_footprint', rclpy.time.Time()):
    rclpy.spin_once(n, timeout_sec=0.1)
try:
    t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
    q = t.rotation; yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    print(f'{t.translation.x:+.2f} {t.translation.y:+.2f} {yaw:+.0f} {math.hypot(t.translation.x, t.translation.y):.2f}')
except Exception as exc:
    print(f'? ? ? ? {exc}')
rclpy.shutdown()
EOF
)
set -- $START
echo "   start: x $1 y $2 heading $3 deg, $4 m from the spot  $(date +%T)"
if [ "$4" = "?" ] || awk "BEGIN{exit !($4 > 0.6)}"; then
    if [ "$FORCE" = 1 ]; then echo "   not on the spot: driving anyway (--force)"; else echo "   NO-GO: not on the parking spot (drive her there first, facing the hall)"; exit 1; fi
fi

T0=$(date +%s)
"$S/drive_record.sh" start
BAG=$(cat "$HOME/bags/current" 2>/dev/null)
python3 - "$D/drive.json" "$N" "$ROUTE" "$BAG" "$PARK" <<'EOF'
import json, sys, time
out, n, route, bag, park = sys.argv[1:6]
json.dump({'drive': int(n), 'date': time.strftime('%Y-%m-%d'), 'started': time.strftime('%Y-%m-%dT%H:%M:%S'),
           'route': [float(v) for v in route.split()], 'bag': bag, 'park': park == '1'}, open(out, 'w'), indent=1)
EOF

# nav_route's pictures (a stop, a detour) into this run's folder, so H2-Host's pull gets them
ros2 param set /nav_helper out_dir "$D/pictures" > /dev/null 2>&1 || echo "   (nav_helper not running: no pictures on a stop)"
echo "== route  $(date +%T)"
# shellcheck disable=SC2086
timeout 700 ros2 run jetnano_navigation nav_route $ROUTE --timeout 300 > "$D/route.log" 2>&1
RC=$?
grep -v "^\[WARN\]" "$D/route.log" | grep -E "^result|passed waypoint|stretches|moved|refus|clear|battery|timeout|nobody" | sed 's/^/   /'
T1=$(date +%s)
echo "   route took $((T1 - T0)) s (exit $RC)"

if [ "$PARK" = 1 ]; then
    sleep 1
    echo "== park  $(date +%T)"
    timeout 600 ros2 run jetnano_navigation nav_park > "$D/park.log" 2>&1
    grep -v "^\[WARN\]" "$D/park.log" | sed 's/^/   /'
    echo "   park took $(( $(date +%s) - T1 )) s"
fi
echo "== done  $(date +%T), $(( $(date +%s) - T0 )) s in all"
sleep 3
"$S/drive_record.sh" stop
# the run's own files into the bag, for the analysis on H2-Host
if [ -d "$BAG" ]; then
    python3 - "$D/drive.json" "$D" <<'EOF'
import json, sys, time
p, d = sys.argv[1:3]
j = json.load(open(p))
j['ended'] = time.strftime('%Y-%m-%dT%H:%M:%S')
j['logs'] = d
json.dump(j, open(p, 'w'), indent=1)
EOF
    cp "$D/drive.json" "$D/route.log" "$BAG/" 2>/dev/null
    [ -f "$D/park.log" ] && cp "$D/park.log" "$BAG/"
    echo "   bag: $BAG (+ drive.json, route.log, park.log)"
fi
