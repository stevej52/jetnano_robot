# Driving a lap, and learning from it

## drive.sh

```
drive.sh N                       the house lap (kitchen island + dining table), then park
drive.sh N X Y H  X Y H ...      that route instead (map metres and degrees), then park
drive.sh N --no-park ...         route only
drive.sh N --force ...           drive on a NO-GO from predrive (you were warned)
drive.sh N --rail ...            the head-camera pan test of the servo rail first (2 s)
```

`N` is the drive number. Everything lands in `~/audit/<date>/driveN/`: `console.log`,
`route.log`, `park.log`, `drive.json`, `pictures/`. `drive.json` is copied into the bag so
H2-Host's analysis can pair the bag with the run.

What it does, in order:

1. `voice off` (tests run with her voice and hearing unloaded).
2. Warms the ros2 daemon (cold, it reported healthy streams as missing).
3. Starts Nav2 if it is not running (`nav2_ctl.sh start`). Since 2026-10-03 Nav2 is one
   systemd unit, `jetnano-nav2.service`, started with the robot service: the launch owns the
   container, its components, the lifecycle manager and the helpers, and a dead container
   brings the whole subsystem back in about 30 s (proven on the bench with a SIGKILL).
4. Starts the recorder in the background and runs `predrive`.
5. Sends the route through the mission controller (`mission_cmd route`), then `nav_park`.
6. Stops the recorder and writes the run's files into the bag.

Preflight is about 12 s with Nav2 already up, about 25 s when Nav2 has to start.

## Several laps in a row

```
laps.sh N COUNT [drive.sh options and route]      e.g. laps.sh 28 3
```

Each lap is a complete `drive.sh` run (preflight, route, park, recorder stopped, files
into the bag), then the next from the spot she parked on. A lap that does not park on the
spot ends the series (`--keep-going` overrides). The summary table lands in
`~/audit/<date>/laps-N-COUNT.txt`.

## predrive: GO or NO-GO

`ros2 run jetnano_bringup predrive`, one process, a few seconds. Every line is `ok` or the
reason:

robot software up, placed on the map (and the fit), map->odom fresh, map->base_footprint
resolves, no twist_mux lock, collision guard active, one Nav2 and active, visual odometry
flowing, motion check on, ESC armed this boot, servo outputs enabled, start on the spot
(within 0.6 m), pack voltage, safety gate `ok`, recording.

Any NO-GO stops the drive unless `--force`.

## The scorecard

`tools/drive_analysis/scorecard.py <bag>` writes `analysis/scorecard.json` beside the bag:
lap and route time, distance, mean and 95th-percentile speed, time to first motion, stops
(count, seconds, where), reversals (count and per metre), forward/reverse switches, guard
stops, the closest 1 % of lidar clearance, lateral acceleration, whether she parked, and the
waypoints. H2-Host runs it for every new bag (`tools/h2host/rosie-drives.sh`, every 15
minutes, finished drives only) and shows the table and the hotspots on the health page,
http://192.168.1.238:8087/.

## When it goes wrong

- `route.log` has the story a line a second: where she was, what the driver got, every
  waypoint passed, every detour, every picture.
- The pictures of a stop are in `pictures/<time>/`: the house map round her, a close-up with
  the lidar's points, the camera sweep, and `facts.txt`.
- The Nav2 log of the run is `~/audit/nav2_<date>-<time>.log`.
