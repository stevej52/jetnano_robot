#!/usr/bin/env bash
# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
#
# Several laps in a row, each one a complete drive.sh run (Steve, 2026-10-03: "a lap, park,
# save data, then another lap, save data, then another lap, save data"):
#
#     laps.sh N COUNT [drive.sh options and route]      e.g.  laps.sh 28 3
#
# Lap k is drive N+k-1: preflight, route, park, recorder stopped, the run's files into the
# bag (drive.sh does all of that), then a short pause for the recorder to finish writing,
# then the next one from the spot she parked on. A lap that does not park on the spot is
# the end of the series: the next preflight would refuse the start anyway (0.6 m), and a
# series that keeps going from the wrong place measures nothing. --keep-going overrides.
# Everything lands in ~/audit/<date>/driveN.. and the series summary in
# ~/audit/<date>/laps-N-<count>.txt.
set +u
S="$HOME/ros2_ws/src/jetnano_robot/jetnano_bringup/scripts"
N=$1; COUNT=$2; shift 2
[ -n "$N" ] && [ -n "$COUNT" ] || { echo "usage: laps.sh N COUNT [drive.sh options and route]" >&2; exit 2; }
KEEP=0
ARGS=()
for a in "$@"; do
    [ "$a" = "--keep-going" ] && KEEP=1 || ARGS+=("$a")
done
OUT="$HOME/audit/$(date +%F)/laps-$N-$COUNT.txt"
mkdir -p "$(dirname "$OUT")"
{
echo "== $COUNT laps from drive $N, $(date '+%F %T')"
printf '%-6s %-9s %-8s %-30s %-8s %s\n' drive route park "route result" total "note"
} | tee "$OUT"
T0=$(date +%s)
for k in $(seq 1 "$COUNT"); do
    n=$((N + k - 1))
    D="$HOME/audit/$(date +%F)/drive$n"
    "$S/drive.sh" "$n" "${ARGS[@]}" > /dev/null 2>&1
    rc=$?
    C="$D/console.log"
    route=$(grep -oE "route took [0-9]+ s" "$C" 2>/dev/null | tail -1 | grep -oE "[0-9]+" | head -1)
    park=$(grep -oE "park took [0-9]+ s" "$C" 2>/dev/null | tail -1 | grep -oE "[0-9]+" | head -1)
    total=$(grep -oE "[0-9]+ s in all" "$C" 2>/dev/null | tail -1 | grep -oE "[0-9]+" | head -1)
    result=$(grep -oE "result [A-Z]+ after [0-9.]+ s" "$C" 2>/dev/null | head -1)
    parked=$(grep -oE "ended [0-9]+ cm from the spot, heading off by [-+0-9]+ deg" "$C" 2>/dev/null | tail -1)
    nogo=$(grep -c "NO-GO: not driving" "$C" 2>/dev/null)
    note=""
    [ "$nogo" != 0 ] && note="NO-GO: $(grep -E "^  NO-GO" "$C" | head -1 | cut -c9-60)"
    echo "$result" | grep -q SUCCEEDED || note="${note:-route $result}"
    grep -q "^   parked" "$C" 2>/dev/null || note="${note:-did not park}"
    printf '%-6s %-9s %-8s %-30s %-8s %s\n' "$n" "${route:-?} s" "${park:-?} s" "${result:-none}" "${total:-?} s" "${note:-ok; $parked}" | tee -a "$OUT"
    if [ -n "$note" ] && [ "$KEEP" = 0 ]; then
        echo "   stopping the series: $note" | tee -a "$OUT"
        break
    fi
    [ "$k" -lt "$COUNT" ] && sleep 8        # the recorder finishes writing; the costmaps settle
done
echo "== done, $(( $(date +%s) - T0 )) s for the series, $(date +%T)" | tee -a "$OUT"
