# The mission controller

`jetnano_navigation/jetnano_navigation/mission.py` and `route_run.py`, started with Nav2 by
`navigation.launch.py`. Since 2026-10-02.

## Why

Until then every script (`nav_goal`, `nav_route`, `nav_park`, `meet`, `battery_home`) opened
its own connection to Nav2. Nobody held the list of accepted goals, a safety stop cancelled only
the one action it knew about, and the parker could start on top of a lap. Now one node owns the
Nav2 action clients and the goal ids, and it is the one the safety gate talks to.

## Commands

JSON on `/mission/command`:

```
{"do": "goal",  "x": 2.0, "y": -2.0, "heading_deg": 135, "timeout_s": 45, "planner": "GridBased"}
{"do": "route", "waypoints": [[x, y, heading_deg], ...], "timeout_s": 300}
{"do": "cancel"}   {"do": "resume"}   {"do": "status"}
```

The client that waits and prints:

```
ros2 run jetnano_navigation mission_cmd goal X Y HEADING_DEG [TIMEOUT_S]
ros2 run jetnano_navigation mission_cmd route X Y H  X Y H ...  [--timeout S]
ros2 run jetnano_navigation mission_cmd cancel | resume | status
```

It exits 0 on SUCCEEDED. `drive.sh` runs the lap through it.

## What it publishes

- `/mission/status` (latched JSON, twice a second): `mission`, `phase`, `id`, `goal`,
  `since_s`, `feedback` (`x`, `y`, `h_deg`, `dist_m`, `left`), `held`, `gate`
- `/mission/log`: the story of a route, one line at a time, as `nav_route` used to print it:
  the costmap check, the stretches, the line a second, waypoints passed, detours, pictures,
  the result line
- `/mission/result`: one line per finished mission, with its `outcome`

## The gate

A verdict counts only while it is fresh: a gate that has not spoken for 2 s permits nothing
(`PERMIT_MAX_AGE_S`), so a dead or stalled gate refuses missions and holds a running one. On
start the controller cancels any goal Nav2 still holds from a controller that died.

- Not permitted when a command arrives: `REFUSED`, with the gate's reasons.
- Gate stops her mid-mission: the controller cancels the Nav2 goal, waits for Nav2 to confirm,
  and holds the mission with the reason.
- Gate permits again: if the hold lasted under `resume_within_s` (30 s) she resumes on her own
  with the waypoints still to go; longer, she stays held and the status says
  "send resume to continue" (Steve's choice).

## A route, step by step (`route_run.py`)

The lap that `nav_route` used to run in a blocking loop, as a state machine on the
controller's tick. Nothing spins inside a callback.

1. `checking`: every waypoint against the planner's live costmap; moved to the nearest clear
   spot within 0.8 m, or the route is refused.
2. `starting`: her pose from TF; the route cut into stretches that do not end on themselves
   (so Nav2's "within 10 cm of the path's end" test cannot finish a lap early).
3. `sending` / `driving`: the next stretch is sent as soon as only the current one's last
   waypoint is left, so Nav2 swaps goals on the move. The `Through` planner (any heading at a
   waypoint) is selected for the route and `GridBased` handed back at the end.
4. The detour guard: Nav2 planning far more than the way (1.4x plus 2 m) means a blocked
   path. Cancel, wait 5 s, send the rest again; costmaps cleared on the third try; six tries,
   then the route fails.
5. Pictures on a 3 s stop or the first detour (`nav_helper ~/snapshot`), at most six.
6. The result line: outcome, time, waypoints passed, distance and heading from the last,
   driver commands, forward/reverse switches.

`test/test_route_run.py` drives the state machine with a fake Nav2.

## Still outside the controller

`nav_park` (the arcs and the reverse-in), `nav_goal`'s rescue, `battery_home`'s way home and
`meet`. They move in next; until then they run beside it as before.
