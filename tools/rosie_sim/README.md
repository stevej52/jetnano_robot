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

Round 5 (2026-10-05): getout.py, a multi-point "wiggle out" recovery (serves nav2 BackUp as
server "getout"; trees call it with server_name="getout" server_timeout="3000"; GETOUT=1 starts
it in sim.launch.py). Replays of round 3's 18 failures: 13/18 (in the round-robin), 11/18 (after
every failure). A plain replay already recovers ~half, so NO measurable gain - not adopted.
What is left: long trips that time out mid-recovery (3 of 5 finish with 3x the time) and a few
truly stuck corners (kitchen corner toward the island facing south; the bottom-right room), where
every new path starts toward an obstacle 5-7 cm away and the collision monitor holds her.

Round 6 (2026-10-05, after Steve's OK the round-3 settings went to nav2.yaml, bc9e37a). Several
instances now run side by side: copy the folder, give its env.sh its own ROS_DOMAIN_ID; variant.sh
stops only its own launch (process group in out/launch.pgid). Failed trips of 199, three goal sets:
| settings (all with planner costmap_update_timeout 3 s, a sim-speed fix) | seed 7 | seed 8 | seed 9 | total of 597 |
| her settings now | 13 | 22 | 14 | 49 |
| + planner reverse_penalty 2.0 -> 1.3 | 10 | 11 | 17 | 38 |
| + planner cost_penalty 2.0 -> 4.0 | 17 | - | - | dropped |
reverse_penalty 1.3: a small gain, not consistent (worse on seed 9, more direction changes there);
not adopted on Rosie without a floor check of how often she then reverses on a normal lap.
