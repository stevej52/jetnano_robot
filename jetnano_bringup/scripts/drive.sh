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
# Steps: predrive (GO/NO-GO), rail_check (the head), record on, the route through the mission
# controller (mission_cmd route), nav_park,
# record off. Everything lands in ~/audit/<date>/driveN/ (console.log, route.log, park.log,
# drive.json) and drive.json is copied into the bag, so H2-Host's analysis can pair the
# bag with the run and the scorecard knows which drive it is scoring.
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
# no Fast-DDS shared memory (robot-environment/system/fastdds_udp_only.xml, 2026-10-02)
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/etc/jetnano/fastdds_udp_only.xml}
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
# waypoints 1, 8 and 9 where the planner's costmap moved them on every run (drives 21-25: 15, 5 and 7-10 cm off the furniture margins)
LAP="2.94 -3.48 -90  1.2 -4.4 180  -0.45 -5.6 -90  1.5 -7.3 0  4.5 -7.5 0  6.5 -7.7 20  7.05 -6.5 90  5.54 -5.49 180  3.14 -4.33 110  0.85 -0.81 134"
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
if ! pgrep -f '^/opt/ros/jazzy/lib/rclcpp_components/component_container_isolated' > /dev/null; then
    echo "   Nav2 not running: starting it"
    # not through a pipe: something Nav2's start spawns kept the pipe open and "| tail" waited
    # 72 s after "Nav2 active" (drive 21, 2026-10-02: the preflight was 99 s, 81 of them here)
    "$S/nav2_ctl.sh" start > "$D/nav2_start.log" 2>&1
    tail -2 "$D/nav2_start.log" | sed 's/^/   /'
    echo "   Nav2 up  $(date +%T)"
fi
echo "   predrive  $(date +%T)"
# the recorder's ~6 s of start-up overlap the checks; a NO-GO stops it again
"$S/drive_record.sh" start > "$D/record_start.log" 2>&1 &
REC=$!
# jetnano_bringup/predrive.py: the same checks as predrive.sh in one process, seconds not a minute
if ! ros2 run jetnano_bringup predrive; then
    if [ "$FORCE" = 1 ]; then echo "   NO-GO overridden (--force)"; else
        echo "   NO-GO: not driving"; wait $REC; "$S/drive_record.sh" stop > /dev/null 2>&1; exit 1; fi
fi
# the rail check (a head-camera pan, 2 s) is OFF by default since 2026-10-01 21:00: it starts and
# stops the head camera's pipeline, and that teardown is the known nvgpu kernel-panic trigger
# (2026-09-28); drive 20's reset came 2 s after it. --rail turns it on.
if [ "$RAIL" = 1 ]; then
    echo "   rail check  $(date +%T)"
    ros2 run jetnano_bringup rail_check 2>/dev/null | tail -1 | sed 's/^/   /'
fi
T0=$(date +%s)
wait $REC; cat "$D/record_start.log" | head -1
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
# the lap runs inside the mission controller (mission.py + route_run.py, 2026-10-02): the safety
# gate can stop and resume it there; this client only prints its story into route.log
# with parking, the lap's end is handed to the parking server while she still rolls (2026-10-04:
# she stood ~6 s at the arc's start while a separate nav_park started); its lines come back
# in route.log as "park: ..."
PARKARG=""
[ "$PARK" = 1 ] && PARKARG="--park 0 0 0"
timeout 700 ros2 run jetnano_navigation mission_cmd route $ROUTE --timeout 300 $PARKARG > "$D/route.log" 2>&1
RC=$?
grep -v "^\[WARN\]" "$D/route.log" | grep -E "^result|passed waypoint|stretches|moved|refus|clear|battery|timeout|nobody|handing over" | sed 's/^/   /'
T1=$(date +%s)
echo "   route took $((T1 - T0)) s (exit $RC)"

if [ "$PARK" = 1 ]; then
    if grep -q "^park: \(parked\|the last leg\)" "$D/route.log"; then
        echo "== park  (handed over rolling, inside the route)"
        grep "^park: " "$D/route.log" | grep -v "^park: took" | sed 's/^park: //' > "$D/park.log"
        sed 's/^/   /' "$D/park.log"
        echo "   park took $(grep -o "^park: took [0-9.]*" "$D/route.log" | grep -o "[0-9.]*$" | cut -d. -f1) s"
    else
        # no handover (an older controller, no parking server, a route that ended elsewhere): as before
        sleep 1
        echo "== park  $(date +%T)"
        timeout 600 ros2 run jetnano_navigation nav_park > "$D/park.log" 2>&1
        grep -v "^\[WARN\]" "$D/park.log" | sed 's/^/   /'
        echo "   park took $(( $(date +%s) - T1 )) s"
    fi
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
