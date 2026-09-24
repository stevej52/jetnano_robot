#!/usr/bin/env bash
# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
#
# Persistent mapping at boot: slam_toolbox picks up the house map where it left
# off, keeps extending it while the robot drives, and saves it on the way out.
#
#     slam_boot.sh [map]            default map: ~/maps/home
#
# Run by jetnano-slam.service after the robot stack is up. If <map>.posegraph
# exists the launch is mode:=continue (the known rooms are remembered, new ones
# get added, loop closure keeps correcting); otherwise mode:=mapping starts the
# map from nothing. Either way the map is serialised every AUTOSAVE_S seconds
# and once more when the service stops, BEFORE slam_toolbox is told to quit -
# the other order loses the session.
#
# One assumption that matters: mode:=continue starts the robot at the map's
# origin (map_start_at_dock), which is wherever the very first mapping session
# began. Power the robot up in the same spot each time. Started somewhere else,
# it will map that place on top of the old map until a loop closure sorts it
# out, or does not. To start the map over, stop the service and delete
# ~/maps/home.* first.
#
# Saving writes home.posegraph + home.data (the graph slam_toolbox reloads) and
# home.pgm + home.yaml (a picture, and what map_server/AMCL would want); see
# jetnano_navigation/save_map.py.

set +u

MAP=${1:-$HOME/maps/home}
AUTOSAVE_S=${SLAM_AUTOSAVE_S:-300}

say() { echo "slam_boot: $*"; }

mkdir -p "$(dirname "$MAP")"
if [ -f "$MAP.posegraph" ]; then
    MODE=continue
    ARGS="mode:=continue map:=$MAP"
    say "continuing the map at $MAP ($(stat -c %y "$MAP.posegraph" | cut -d. -f1))"
else
    MODE=mapping
    ARGS="mode:=mapping"
    say "no map at $MAP.posegraph: starting a new one"
fi

save() {
    # Quiet unless something fails; save_map's own output is the detail.
    if out=$(ros2 run jetnano_navigation save_map "$MAP" 2>&1); then
        say "map saved ($MODE) to $MAP.*"
    else
        say "map save FAILED:"; echo "$out" | tail -5
    fi
}

stopping=0
stop() {
    [ "$stopping" -eq 1 ] && return
    stopping=1
    trap '' INT TERM
    say "stopping: saving the map first"
    save
    kill -INT "$LAUNCH" 2>/dev/null
    wait "$LAUNCH" 2>/dev/null
}
trap 'stop; exit 0' INT TERM

# Background jobs from a non-interactive shell inherit SIGINT ignored; job
# control makes the launch a proper child that can be stopped.
set -m
ros2 launch jetnano_navigation slam.launch.py $ARGS &
LAUNCH=$!
set +m

# Give slam_toolbox time to come up (and, when continuing, to load the graph)
# before the first autosave; then save on the interval while the launch lives.
elapsed=0
while kill -0 "$LAUNCH" 2>/dev/null; do
    sleep 5
    elapsed=$((elapsed + 5))
    if [ "$elapsed" -ge "$AUTOSAVE_S" ]; then
        elapsed=0
        save
    fi
done
wait "$LAUNCH"
status=$?
say "slam launch exited with status $status"
exit "$status"
