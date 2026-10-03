# The safety gate

`jetnano_bringup/jetnano_bringup/safety_gate.py`, running inside the housekeeping process
(with the motion watch and the battery monitor). Since 2026-10-02.

## What it is

One verdict about whether she may drive, from everything that used to stop her separately:

| State | Meaning | What she may do |
|---|---|---|
| `inhibited` | not fit to drive at all: no pack (bench power), on the bench, lidar or odometry missing | nothing |
| `stopped` | a STOP is held: a mux lock (page, joystick, battery, motion check), a flat pack, a tilt, the guard holding | nothing, until the reason clears |
| `degraded` | something is weak: camera obstacles stale, visual odometry old, low pack, a watchdog problem | drive at 50 % |
| `ok` | all good | drive |

The gate publishes:

- `/safety/state` (latched JSON): `state`, `reasons`, `speed_pct`, `since_s`, `permit`
- `/safety/permit` (Bool): what the mission controller asks before it sends a goal
- `/e_stop_gate` (Bool, repeated every second while held): its own lock in twist_mux
- `/speed_limit` (nav2_msgs/SpeedLimit): the 50 % while degraded

It starts `inhibited` and only opens when the required streams are fresh (lidar scan under
1.5 s old, odometry under 1 s) and the battery verdict allows.

## Bench driving

```
ros2 param set /safety_gate allow_bench true
```

turns "on the bench / no pack" from `inhibited` into `degraded` with the reason "bench driving
allowed (wheels up?)", for wheels-up tests. It is off again at the next restart.

## Who reads it

- The driving page: the banner shows `STOPPED: …`, `NOT PERMITTED: …` or `slowed to N %`,
  and the header shows the state.
- `predrive` (the GO/NO-GO before a drive): a NO-GO on anything but `ok`.
- The mission controller: refuses a goal when `permit` is false, cancels a running one when the
  gate stops her, resumes it when the gate opens again (see `mission-controller.md`).

## Decisions are a pure function

`decide(now, inputs)` has no ROS in it, so `test/test_safety_gate.py` runs every rule on the
laptop. A logging quirk worth knowing: rclpy pins a log severity per source line, so the gate
has two call sites for its state line (warning for a stop, info for the rest).
