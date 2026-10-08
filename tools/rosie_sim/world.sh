#!/usr/bin/env bash
# SIM_SPEED 1.0 since 2026-10-07: Nav2 paces its controller (20 Hz) and the BT replan (every 3 s) by the WALL clock, so at 3x she got 7 corrections and a plan every 9 s per sim second -> 35-46 reversals a lap
# sim realism 2026-10-07: drive 51 lidar world + vacuum under the lidar + 1 cm range noise
cd "$(dirname "$0")"
export PLAN_MAP=$PWD/map/home_plan.yaml PHYS_MAP=$PWD/map/world_d51.yaml LOW_MAP=$PWD/map/low_vacuum.yaml RANGE_NOISE=0.01 SIM_SPEED=${SIM_SPEED:-1.0} START="0.85 -0.81 -46"
export LAP="2.95 -2.40 -90  3.00 -3.40 -90  1.2 -4.4 180  -0.45 -5.6 -90  1.5 -7.3 0  4.5 -7.5 0  6.5 -7.7 20  7.05 -6.5 90  5.54 -5.49 180  3.14 -4.33 110  3.00 -1.95 90  0.85 -0.81 134"
F="p[\"controller_server\"][\"ros__parameters\"][\"FollowPath\"]"
TAG=${1:-w30}; R=${2:-0.30}; N=${3:-3}
./variant.sh $TAG "$F[\"min_lookahead_dist\"]=0.65" "$F[\"regulated_linear_scaling_min_radius\"]=$R" | tail -1
(source env.sh; timeout $((N*700)) python3 lap.py $TAG --laps $N 2>&1 | grep "^{")
python3 lapscore.py out/lap-$TAG-*.jsonl | sed -E "s/\"weave_p90.*\"plans\"/.../"
echo ALLDONE
