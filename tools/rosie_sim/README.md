# Rosie in a box: house tours in simulation (H2-Host)

A kinematic stand-in for Rosie (0.40 m turning circle, four-wheel steering, no turning on the
spot, speed and steering lag, body 0.444 x 0.296 m, walls stop her) under her real Nav2:
nav2.yaml and the behaviour trees copied from Rosie, same Nav2 version (1.3.13). A lidar
scan is ray-cast in the house map. Runs on ROS domain 42, localhost only: it cannot reach Rosie.

Setup on H2-Host (~/rosie-sim): map/home.{pgm,yaml} and nav2_rosie.yaml + bt/*.xml from Rosie,
then `sed "s#__BT_DIR__#/home/steve/rosie-sim/bt#g" nav2_rosie.yaml > nav2_sim.yaml`.

    source env.sh; setsid ros2 launch ./sim.launch.py > out/launch.log 2>&1 &
    python3 tour.py [--limit N] [--seed S]     # one goal per 0.6 m of floor, random heading
    python3 render.py                          # out/tour.png + out/summary.json

Limits: only what the lidar mapped exists (no table tops or chair seats from the camera);
no people, cats or moved furniture; localization is perfect. A goal that fails resets her
by teleport to that goal, which can drop her in an awkward pose (counted separately).

First tour 2026-10-04: 199 goals, 110 clean, 58 struggled, 31 failed; failures = starting
off near walls/furniture (FollowPath patience exceeded: "collision ahead", the BackUp
recovery blocked too) and final approaches with a fixed heading in tight spots (invalid path).

Fix rounds 2026-10-05 (replay.py replays a tour's failures from their start poses; variant.sh
rebuilds nav2_sim.yaml with edits and restarts the sim):
| tour | clean | struggled | failed | bumps |
| her settings | 110 | 58 | 31 | 0 |
| RPP use_collision_detection false | 132 | 45 | 22 | 0 |
| + collision monitor time_before_collision 0.8 | 137 | 44 | 18 | 0 |
NOT applied to nav2.yaml: both loosen collision protection and need Steve's explicit go-ahead.
The 18 left are time-outs after little progress, mostly among the dining chairs and hallway walls.
| + PoseProgressChecker 0.25 m / 0.5 rad / 20 s, planner costmap_update_timeout 3 s | 130 | 46 | 23 | 0 |
The last row is within run-to-run noise of the one before (a plain replay of the same failures
already recovers about half by chance), so it is not shown to help. It did remove the planner's
"Costmap timed out" failures, which look like a 3x sim-speed artefact (none on Rosie since 10-01).
