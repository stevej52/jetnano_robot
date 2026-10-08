#!/usr/bin/env bash
# world.sh + near_cap (2026-10-07) between the smoother and the monitor; MIN_RADIUS 0.90
cd "$(dirname "$0")"
TAG=${1:-c90}; N=${2:-3}
export PLAN_MAP=$PWD/map/home_plan.yaml PHYS_MAP=$PWD/map/world_d51.yaml LOW_MAP=$PWD/map/low_vacuum.yaml RANGE_NOISE=0.01 SIM_SPEED=1.0 START="0.85 -0.81 -46"
export LAP=${LAP_OVERRIDE:-"2.95 -2.40 -90  3.00 -3.40 -90  1.2 -4.4 180  -0.45 -5.6 -90  1.5 -7.3 0  4.5 -7.5 0  6.5 -7.7 20  7.05 -6.5 90  5.54 -5.49 180  3.14 -4.33 110  3.00 -1.95 90  0.85 -0.81 134"}
F="p[\"controller_server\"][\"ros__parameters\"][\"FollowPath\"]"
C="p[\"collision_monitor\"][\"ros__parameters\"]"
./variant.sh $TAG "$F[\"min_lookahead_dist\"]=0.65" "$F[\"regulated_linear_scaling_min_radius\"]=0.90" "$C[\"cmd_vel_in_topic\"]=\"cmd_vel_nav_capped\"" | tail -1
source env.sh
python3 ../../jetnano_navigation/jetnano_navigation/near_cap.py --ros-args -p use_sim_time:=true -r cmd_vel_nav_smoothed:=cmd_vel_smoothed > out/near_cap-$TAG.log 2>&1 &
CAP=$!
timeout $((N*700)) python3 lap.py $TAG --laps $N 2>&1 | grep "^{"
kill $CAP
python3 lapscore.py out/lap-$TAG-*.jsonl | sed -E "s/\"weave_p90.*\"plans\"/.../"
grep -c "held to" out/near_cap-$TAG.log
echo ALLDONE
