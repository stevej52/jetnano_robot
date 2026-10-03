# guard_flow: watching the collision guard's plumbing

`jetnano_bringup/jetnano_bringup/guard_flow.py`, inside the housekeeping process. Since
2026-10-01 (drive 16).

## Why

Nav2's collision monitor sits last in the chain, between the velocity smoother and twist_mux.
It refuses to pass any command while one of its sources is stale ("stop due to invalid
source"). On drive 16 the camera's obstacle cloud kept its last stamp when nvblox stopped
integrating, the guard held every command, the lidar's included, and she stood for twelve
minutes with nothing in the log that said why.

## What it watches

- Commands going in (`/cmd_vel_mux`) against commands coming out (`/cmd_vel`): told to drive
  but nothing comes out is "holding".
- The age of the camera obstacle cloud's stamp, and the depth camera's own stamps
  (`/camera/depth/camera_info`), so the log says which of the three it is: no depth from the
  camera, depth but no nvblox grid, or a grid with a frozen stamp.
- The guard's own state (`CollisionMonitorState`): a STOP for an obstacle is not a fault.

## What it does

- Publishes `/guard_flow/status` (the page's banner reads it; the safety gate turns a hold
  into `stopped`).
- Holding on a stale cloud: restarts `grid_to_points` first, then the camera container's
  launch, and checks each came back.

It no longer caps the speed itself; the safety gate owns `/speed_limit`.

## The C++ side

`jetnano_watchdog/src/grid_to_points.cpp` publishes an empty cloud stamped now when nvblox's
grid is older than `stale_s` (3 s), so the guard keeps passing the lidar's commands, and says
so on `/camera_obstacles/health` ("ok" or "stale N s"). The camera's obstacles are gone until
nvblox moves again; the lidar still guards. Degraded, and said, rather than frozen.
