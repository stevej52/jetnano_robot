# jetnano_robot

ROS 2 Jazzy software for a **DANCHEE RidgeRock 1/10 four-wheel-steering rock
crawler** running on an NVIDIA Jetson Orin Nano Super — the robot written up on
[Hackaday](https://hackaday.com/2020/10/16/jetson-nano-robot/), rebuilt from a
Jetson Nano on ROS 2 Eloquent to an Orin Nano on Jazzy.

| | |
|---|---|
| Motors | Adafruit PCA9685: ESC ch 0, front steering ch 1, rear steering ch 2, headlight relay ch 5, camera pan ch 8, tilt ch 9 |
| Lidar | RPLidar A1M8 |
| Cameras | Intel RealSense D435 (odometry, depth, the video feed); two Raspberry-Pi-style IMX219 CSI cameras, one on the pan-tilt (streaming, not yet in ROS) |
| IMU | BNO055 |
| Audio | reSpeaker Flex XVF3800 linear 4-mic array (USB, ALSA card `L16K6Ch`, firmware 1.0.3): 16 kHz, 6 channels, echo cancelling; her speaker on its JST output |
| Odometry | visual (NVIDIA cuVSLAM on the GPU at 89 Hz, or rtabmap on the CPU) + IMU, optionally + lidar (MOLA), fused by robot_localization — **no wheel encoders** |
| Driving | the phone page (below); a Thrustmaster HOTAS or Xbox pad is supported by `jetnano_teleop` but not started at boot |

## Packages

| Package | What it is |
|---|---|
| `jetnano_description` | URDF/xacro and RViz layouts |
| `jetnano_bringup` | Launch files and config that tie everything together, plus a fake-sensor rig for bench testing |
| `jetnano_teleop` | One joystick node that detects which controller is plugged in |
| `jetnano_navigation` | slam_toolbox and Nav2, set up for a car-like chassis |
| `jetnano_watchdog` | `topic_watch`, a small C++ node that counts the important streams for the watchdog |
| `jetnano_gazebo` | the Gazebo simulator (run on the host PC, `ROS_DOMAIN_ID=77`) |

Setup for the machines themselves (Ubuntu 24.04 + ROS 2 Jazzy) lives in
[robot-environment](https://github.com/stevej52/robot-environment). The
PCA9685 driver is [ros2_pca9685](https://github.com/stevej52/ros2_pca9685).

## Installing

### 1. Ubuntu 24.04 with ROS 2 Jazzy

Nothing here works on any other combination — Jazzy binaries only exist for
24.04. Use [robot-environment](https://github.com/stevej52/robot-environment),
which does the whole thing from apt:

```bash
sudo apt install -y git
git clone https://github.com/stevej52/robot-environment.git ~/robot-environment
~/robot-environment/scripts/install_ros2_jazzy.sh --domain-id 7 --workspace
```

Use the **same `--domain-id` on every machine** — it is what separates this
robot from other ROS 2 traffic on the network.

### 2. Clone this and the driver into the same workspace

These packages do **not** contain the PCA9685 driver; they depend on it. Both
have to be in the workspace or the build will not resolve:

```bash
cd ~/ros2_ws/src
git clone https://github.com/stevej52/ros2_pca9685.git
git clone https://github.com/stevej52/jetnano_robot.git
```

(`--workspace` in step 1 already cloned `ros2_pca9685`; skip it if it is
there.)

### 3. Let rosdep install the rest

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
```

That pulls `twist_mux`, `robot_localization`, `slam_toolbox`, `navigation2`,
`nav2_lifecycle_manager`, `rplidar_ros`, `realsense2_camera`, `bno055`,
`rtabmap_odom`, `python3-evdev` and everything else — all from apt. **No
source builds.**

### 4. Build

```bash
colcon build
source install/setup.bash
```

The robot's workspace is a plain (copy) install. Do not switch an existing
workspace to `--symlink-install`: on 2026-09-26 a half-converted install
made every respawning node die with `PackageNotFoundError`. Rebuild with
`--packages-select <package>` after pulling.

### 5. Permissions

```bash
sudo usermod -aG i2c,dialout,input $USER    # then log out and back in
```

| Group | Needed for |
|---|---|
| `i2c` | the PCA9685 and the BNO055 |
| `dialout` | the RPLidar's USB serial port |
| `input` | joysticks — `jetnano_teleop` reads `/dev/input/event*` directly |

Logging out and back in is not optional; group membership is read at login.

### 6. Device rules

```bash
sudo cp jetnano_bringup/udev/99-robot-sensors.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

This opens the D435 for libusb without root and names the lidar's serial
port `/dev/rplidar`, which is what `sensors.launch.py` expects. The
RealSense apt packages for Jazzy do not ship udev rules of their own.

## Running it

On the robot:

```bash
ros2 launch jetnano_bringup robot.launch.py
```

On a machine with a screen (`ROS_DOMAIN_ID=7` on both):

```bash
ros2 launch jetnano_bringup rviz.launch.py view:=drive     # or view:=nav, with the Nav2 panel
ros2 launch jetnano_bringup teleop.launch.py
```

### Watching it

The camera's colour stream is on whenever the GPU odometry runs (it comes from the
same container; `ros2_gpu_robot/cuvslam_d435/README.md`, "Watching the camera"):

- **Any browser, phone included, no ROS needed**: the dashboard
  `http://192.168.1.7:8081/dash`, the driving page `http://192.168.1.7:8081/`, or the
  raw streams from `csi_cameras`: `http://192.168.1.7:8082/d435.mjpg` (also `front.mjpg`,
  `rear.mjpg`; `.jpg` for one frame). Port 8080 (`web_video_server`) was retired on
  2026-09-27.
- **RViz**: the `drive` view shows the camera large, with the laser scan, nvblox's
  obstacle grid and the EKF's trail in the `odom` frame, following the robot; the `nav`
  view is the same in the `map` frame with the Nav2 panel. Both read the JPEG topic
  (`/camera/color/image_raw/compressed`, 2.3 MB/s at 30 Hz over Wi-Fi); the raw image
  topics are ~10x that and are not for another machine.

### Driving it from a phone

`http://192.168.1.7:8081/` - the camera feed with a virtual joystick and STOP
under it, served by the robot itself (`jetnano_teleop web_teleop`, started by
`robot.launch.py`). Drag the knob: up and down is throttle, left and right is
steering, further from the centre is more, and anywhere in between is the mix
(upper right = forward and turning right). It springs back to nothing when let
go. The slider is the throttle limit, a percentage of the page's maximum
(`max_linear` 0.5, half of full throttle); it starts at 40 %. Measured on
the hard floor on 2026-09-27: slider 25 % = 0.28 m/s forward / 0.21 m/s
back, 50 % = 0.51 / 0.33, 75 % = 0.65 / 0.45 (100 % not yet measured), so
the 40 % default is about 0.45 m/s forward. The page posts a command ten
times a second while the knob is held and the node publishes `cmd_vel_web` only
while those keep coming, so a closed page, a sleeping phone or a lost Wi-Fi
link stops the robot within half a second and hands control back to Nav2. The
EMERGENCY STOP bar across the bottom (it stays on screen when the page scrolls)
raises the page's own `e_stop_web` lock (the joystick has `e_stop_joy`, so
neither can cancel the other's stop), which blocks everything including Nav2
until it is tapped again for GO. On a laptop the arrow keys /
WASD (full deflection) and space do the same. No login: it is for the robot's
own network.

Mapping, in three modes:

```bash
ros2 launch jetnano_navigation navigation.launch.py mode:=mapping
ros2 launch jetnano_navigation navigation.launch.py mode:=continue     map:=~/maps/home
ros2 launch jetnano_navigation navigation.launch.py mode:=localization map:=~/maps/home
ros2 run jetnano_navigation save_map ~/maps/home
```

`continue` is the interesting one: it loads a saved map **and keeps adding to
it**. An occupancy-grid image cannot do that, because it has thrown away the
pose-graph; slam_toolbox's serialised graph has not.

### No robot? Test on a desk

```bash
ros2 launch jetnano_bringup sim.launch.py                    # fake room, fake lidar
ros2 launch jetnano_navigation slam.launch.py mode:=mapping
ros2 launch jetnano_bringup rviz.launch.py
```

This is a wiring harness, not a simulator. It proves the graph is connected; it
proves nothing about physics. Gazebo is the right tool for that, on a desktop.

## Starting at boot

Two systemd units in `jetnano_bringup/systemd/` bring the robot up on power:
`isaac-vo.service` starts the Isaac ROS container that runs the GPU visual
odometry (waiting for the GPU driver first), and `jetnano-robot.service` runs
`robot.launch.py` as the robot user once the container is up.

```bash
sudo cp jetnano_bringup/systemd/*.service /etc/systemd/system/
sudo cp jetnano_bringup/systemd/logind-removeipc.conf /etc/systemd/logind.conf.d/
sudo systemctl daemon-reload
sudo systemctl restart systemd-logind      # only with nobody logged in on a desktop
sudo systemctl enable --now jetson-clocks.service isaac-vo.service jetnano-robot.service wifi-watchdog.service
```

`jetnano-slam.service` (the persistent lidar map) is installed but not
enabled: it starts on request - "Rosie, start mapping" / "stop mapping", or
`sudo systemctl start|stop jetnano-slam` - because the map's origin is the
parking spot and a map started anywhere else (the bench) spoils it. Its
`KillMode=mixed` matters: with the default, the stop signal reached
slam_toolbox at the same moment as the wrapper that was asking it to save,
the save hung, and 90 s later everything was killed unsaved.

`jetson-clocks.service` pins the CPU clocks at boot. With the default governor
the cores idle down between bursts and ramp late, and every late ramp costs
the camera pipeline a frame: measured 2026-09-24 with everything running, VO
fell to 11-27 Hz with fifty "frame gap" warnings a minute; pinned, 36 Hz
steady and two. It costs nothing measurable in power (9.1 W for the whole
robot on the bench).

`wifi-watchdog.service` pings the gateway through the wireless interface every
30 s and bounces the connection after two minutes of silence: on 2026-09-23 the
Orin twice became unreachable while its Wi-Fi believed it was connected.

`logind-removeipc.conf` is not optional. The service runs as a normal user
outside any login session, and logind's default `RemoveIPC=yes` deletes that
user's shared memory in `/dev/shm` the moment their last SSH session ends -
which is Fast DDS's transport between nodes on one machine. The symptom is
silent and total: every node on the robot stops hearing every other node (the
tilt guard reports `imu/data has stopped` and holds the robot still) while
`ros2 topic hz` on the host PC still shows everything arriving over the
network, and `ls -l /proc/<pid>/fd` of any node shows its `fastrtps_*` files
as `(deleted)`. It cost most of an evening on 2026-09-23.

The service starts `robot.launch.py nvblox:=true`: the GPU visual odometry
(cuVSLAM, ~40 Hz) plus nvblox's 3D map of what the camera sees, published as
an occupancy grid that `navigation.launch.py nvblox:=true` puts into the local
costmap - steps, low rocks and table edges the lidar's single plane cannot
see. Without `nvblox:=true`, `robot.launch.py` runs the odometry alone at
89 Hz. Both are in `ros2_gpu_robot/cuvslam_d435/README.md`.

`jetnano-slam.service` keeps **one persistent map of the house**. It is not
started at boot (see above); when it is started, `slam_boot.sh` runs
slam_toolbox in `continue` mode on `~/maps/home` if that map exists, or
`mapping` mode if it does not, autosaves it every five minutes and once more
when the service stops. So the map grows every time she maps, and loop
closure keeps correcting it. The one rule: **start mapping at the parking
spot** - a continued map starts the robot at the map's origin, which is
wherever the first session began. To start over, stop the service and delete
`~/maps/home.*`. RViz's `nav` view shows the map; the
`drive` view draws it too, under the lidar.

Stopping or restarting the service takes about 5 s: the odometry wrapper
(`scripts/cuvslam_vo.sh`) gives the launch inside the container 10 s to stop on
SIGINT and then kills whatever is left, and a launch it finds already running
in the container at start (one that lost its wrapper) is killed after 20 s
rather than deferred to. Before this, a restart could leave an orphaned launch
in the container that blocked every respawn with "already running".

For bench work, `sudo systemctl stop jetnano-robot` and launch things by hand.
Every bringup launch (`robot`, `drive`, `sensors`, `odometry`) refuses to start a
second copy of itself on the same machine (`jetnano_bringup/launch_lock.py`),
so a manual `robot.launch.py` while the service is running just prints who
holds the lock and exits; two lidar drivers on one serial port both die, and
two of everything else is worse because nothing complains.

When starting a launch from a script or over SSH, keep it in the foreground or
enable job control first (`set -m`): a background job from a non-interactive
shell inherits SIGINT-ignored, and so does every node it starts, which turns a
clean shutdown into SIGKILLs - and a SIGKILLed lidar driver leaves the sensor
needing a USB reset.

## How commands reach the wheels

```
tilt_guard ──/cmd_vel_tilt  (priority 150)──┐
teleop     ──/cmd_vel_teleop (priority 100)─┤
web page   ──/cmd_vel_web    (priority 90)──┼─ twist_mux ──/cmd_vel_mux──▶ collision_guard ──/cmd_vel──▶ ros2_pca9685 ──I²C──▶ ESC + servos
Nav2       ──/cmd_vel_nav    (priority 10)──┤        ▲                          ▲
settle     ──/cmd_vel_settle (priority 5)───┘        │                    /scan, nvblox points
  /e_stop_web, /e_stop_joy, /e_stop, /e_stop_motion ┘  (locks, priority 255: any one stops her)
```

`tilt_guard` uses its input only while backing the robot off a tilt; `settle`
only for a small steering wiggle after a stop, when her microphone hears a
steering servo buzzing (below). `/e_stop` is the battery monitor's;
`/e_stop_motion` is the motion check's (in `safety_monitor`, below): if she has been told to drive for 1.5 s
(a command of at least 0.08 reaching the driver) and the visual odometry has
moved less than 4 cm - or not published at all - it stops her and says "nope",
until every driver lets go. That is the check that was missing on 2026-09-26,
when a blind camera reported "all fine" and she drove into the curtains. With
lidar odometry running (below), either odometry seeing her move is enough, so
she can keep driving while the camera's odometry restarts; but the lidar
odometry seeing her move 15 cm or more while the camera's hardly moves is the
curtains again, and stops her the same way. On the bench with the wheels in
the air: `ros2 param set /safety_monitor motion.enabled false`.

The tilt guard, this motion check and the visual-odometry watchdog run as one
C++ node, `jetnano_watchdog safety_monitor` (2026-09-27: as three Python nodes
they cost ~19 % of a core, as one C++ node 3 %). Same topics, thresholds and
behaviour; `cpp_safety:=false` on `robot.launch.py` or `drive.launch.py` brings
back the Python originals (`tilt_guard`, `motion_check`, `vo_watchdog` in
jetnano_bringup; of those only `vo_watchdog` knows about lidar odometry).

### Lidar odometry (off by default)

`robot.launch.py lidar_odom:=true` adds a second odometry that does not need
the camera: MOLA (`sudo apt install ros-jazzy-mola-lidar-odometry`) matches
each `/scan` against a local map of her recent scans
(`launch/lidar_odometry.launch.py`). `lidar_odom_relay` turns MOLA's pose into
speeds on `/lidar_odom`, and the EKF fuses them next to the camera's
(`config/ekf.yaml`, `odom2`). MOLA publishes no transform. With it running:

- when `/vo` goes quiet, the safety monitor does not hold the EKF still: the
  lidar odometry keeps her position moving while the camera pipeline restarts;
- the motion check takes either odometry as proof she is moving (above);
- the watchdog restarts MOLA if `/lidar_odom` goes silent while `/scan` is fine;
- `drive_record.sh` records `/lidar_odom`, and `drive_report` says how far the
  camera's and the lidar's odometry disagree over each 2 s of the drive.

MOLA runs pinned to one CPU core (`cpus:=` on the launch file, default the last
core). Its scan matcher otherwise starts a thread per core: on the host PC's 32
threads that cost 177 % of a core while driving, pinned to one core 17 % at the
same 7.7 Hz (synthetic scans, 2026-09-27).

In the simulator (2026-09-27) MOLA alone tracked a 12 m loop within 5 cm of
ground truth, its speeds were within 3-4 cm/s, and with the camera odometry
frozen for 6 s mid-drive the EKF stayed within 9 cm and never held still. The
fusion is only as good as its worst source, though: when the simulator's CPU
visual odometry published wrong speeds instead of going quiet, the EKF followed
them to 1.9 m off, lidar or not. Rejecting outliers (robot_localization's
`odomN_twist_rejection_threshold`) needs the real cuVSLAM and lidar noise to
set, so it is not on yet. Before making lidar odometry the default: one
recorded test drive on Rosie with it on, and `drive_report` on the bag.

### Recording a drive

```bash
ros2 run jetnano_bringup drive_record.sh start     # actually: bash $(ros2 pkg prefix jetnano_bringup)/lib/jetnano_bringup/drive_record.sh start
...drive...
bash $(ros2 pkg prefix jetnano_bringup)/lib/jetnano_bringup/drive_record.sh stop
```

`drive_record.sh start|stop|status` writes `~/bags/drive-<date>/` (commands,
odometry, IMU, lidar, guard decisions, TF; no images; a few MB a minute) and
on `stop` prints `drive_report` - VO and EKF health and whether they agree,
phone command dropouts, every guard stop attributed to the lidar or the
camera, and throttle-to-ground-speed from steady stretches - and saves it as
`report.txt` in the bag. `ros2 run jetnano_bringup drive_report <bag>` runs
it again later. The first mapping drive (2026-09-24) was diagnosed from
exactly this data: the EKF had left its sensor and run 6.5 km.

`drive_record.sh start full` is for a test drive to look back at in detail:
the bag also gets the whole command chain from Nav2's controller to the servo
board, every stop and guard, MOLA's raw pose, where_am_i's verdicts, Nav2's
goal, plans, behaviour tree and costmaps, the camera's obstacle points, the
watchdog, diagnostics and every node's log; beside it `drive-<date>-extra/`
gets tegrastats, `drive_log`'s CPU, memory, Wi-Fi and busy processes, a
picture a second from each camera, the watchdog's rates at both ends and the
journal. About 55 MB a minute and 0.9 of a core (bench, 2026-09-28); `stop`
turns every part of it off and says so, and it stops itself after two hours.

### The collision guard

Whoever is driving - phone, joystick, Nav2, the tilt guard's recovery - the
last thing before the wheels is `collision_guard`: Nav2's collision monitor,
run from `drive.launch.py` with `config/collision_guard.yaml`, after twist_mux
instead of only inside Nav2's own chain. It looks at the lidar and, with
`nvblox:=true`, at the camera's 3D map (`grid_to_points` turns nvblox's grid
into points), in the direction of the commanded throttle:

- anything within **15 cm** of the bumper ahead (or behind, when reversing):
  the command becomes zero - the robot will not drive into it;
- anything within about **80 cm** ahead (60 cm behind): the command is
  scaled to 90 %.

Steve set these on 2026-09-25 after the first drives: the original 30 cm
stop kept him away from everything, and the original slow-down to 30 % was
as good as a stop, because back then 40 % throttle did not move her at all.
Since the throttle calibration of 2026-09-27 (`pca9685.yaml`, the ESC's
start points) every command above the dead band moves her, so a slow-down
to 30-50 % would now really mean slow, and 90 % barely slows her. The ratio
is `slowdown_ratio` in `config/collision_guard.yaml`; tune it on the floor.

The web page says "blocked: obstacle" / "slowed: obstacle near" while this is
happening, and has an ON/OFF switch for it: OFF sets the zones' `enabled`
parameters false (`ros2 param set /collision_guard stop_zone.enabled false`
does the same) and the guard passes commands through untouched; it is ON again
whenever the guard restarts. The zones are drawn in RViz's `drive` view. If the lidar goes quiet
for a second the guard stops the robot, like the tilt guard does without its
IMU; it also needs the EKF's `odom → base_footprint` transform to place the
scans, so without odometry nothing drives (the guard says "invalid source").
`guard:=false` on `drive.launch.py` wires twist_mux straight to the driver for
bench work without it. The zones are measured from the robot's centre, and
the body box they assume is still an estimate; tune them on the floor, not
the bench.

For this to work the lidar must not see the robot itself (it did: a Wi-Fi
antenna 25 cm away, every turn), so `sensors.launch.py` runs the raw scan
through a `laser_filters` box filter around the body (`config/scan_filter.yaml`)
and publishes the result as `/scan`; the driver's own output is `/scan_raw`.

Three rules hold this together, and each was a bug before it was a rule:

1. **Only `ros2_pca9685` touches the I²C bus.** Teleop publishes a Twist; it
   does not drive servos.
2. **Nav2 never publishes to `/cmd_vel`.** Its outputs are remapped to
   `cmd_vel_nav_raw` and `cmd_vel_nav`, so it cannot bypass twist_mux — which
   would mean bypassing the joystick override and the e-stop.
3. **Only the EKF publishes `odom → base_footprint`.** The visual odometry
   (cuVSLAM in the container, or `rgbd_odometry` on the CPU) publishes no
   transform and feeds the EKF as a measurement on `/vo` - velocities only,
   so a restart of the odometry cannot yank the robot's position. The lidar
   odometry (MOLA) is the same: no transform, speeds on `/lidar_odom`.

Tested in simulate mode: teleop overrides Nav2 mid-run and Nav2 resumes when
teleop lets go; the e-stop lock blocks a full-throttle command; every channel
returns to neutral 0.5 s after commands stop.

## Rosie: voice, brain and watchdog

The robot is called Rosie. Everything below starts with `robot.launch.py`;
what she understands is in [docs/talking-to-rosie.md](docs/talking-to-rosie.md),
and the full rebuild in robot-environment `REBUILD.md`.

| Node | What it does |
|---|---|
| `ears` | owns the mic (the reSpeaker, card `L16K6Ch`, its echo-cancelled speech channel): the room's level on `sound/level`, the audio on `sound/audio`, "huh?" at a clap |
| `listen` | speech detection and speech-to-text on the CPU (sherpa-onnx, Moonshine); decides what was meant; her name wakes her, a conversation then runs without it; knows voices apart (`speaker.py`, WeSpeaker via sherpa-onnx) |
| `motion_watch` | the lidar sees someone move while she is parked: by default she reacts only to someone coming right at her; "Rosie, keep watch" turns on watchdog mode (a "huh?" and a look at every mover), "Rosie, stand down" turns it off. Aims the pan-tilt only with `aim:=true` |
| `settle` | listens for the front steering servo buzzing after a stop (a tone near 1.6 kHz) and wiggles the steering a few degrees until it is quiet |
| `brain` | answers: small talk on the local model upstairs (llama.cpp, Qwen 2.5 14B), everything real handed to Claude Opus 5.5; "think hard" gets full thinking; a daily budget |
| `speak` | text to her English voice (Piper), sentence by sentence, cached |
| `sounds` | plays everything on the speaker (on the reSpeaker, card `L16K6Ch`, so the mic's echo canceller hears it), one sound at a time; `mute` |
| `health` (in listen) | her real status for "how are you" and the spoken report |
| `topic_watch` (jetnano_watchdog, C++) | a cheap live count of the important streams |
| `watchdog` | restarts whatever goes silent, step by step and within limits; shows problems on the driving page; `~/watchdog/events.jsonl`; checks the GPU every 30 s and, if it failed to start at boot, reboots her once (never while driving or mapping) |

Her voice nodes run in `~/venv-voice` (robot-environment
`scripts/install_voice.sh`); the Claude key sits in
`/etc/default/jetnano-robot` (`scripts/add-rosie-key.sh`); the local model
lives on the PC upstairs (robot-environment `scripts/jedipc_brain.md`).
Without the PC she answers with Claude; without the key, with the PC;
without either, in her own beeps.

Measured on the Orin, 2026-09-25: listen about 4 % of one core idle,
`topic_watch` plus `watchdog` about 2 %. A Claude answer costs 0.3 to 0.7
cents.

## Frames

```
map ──▶ odom ──▶ base_footprint ──▶ base_link ──▶ wheels, lidar_link, camera_link, imu_link
 │        │             │
 │        │             └── robot_state_publisher, from the URDF
 │        └── the EKF (robot_localization)
 └── slam_toolbox
```

**The URDF contains no `map`, `odom` or `world` link, deliberately.** The
previous version welded `map → odom → base_link` in as fixed joints, which
bolts the robot to the map origin and quietly breaks Nav2, SLAM and RViz at
once. Those two transforms have to move, and they are published by the two
nodes above.

Likewise `odom` parents `base_footprint`, not `base_link` — `base_link`
already has a parent in the URDF, and a frame with two parents is not a tree.
TF stops resolving and nothing tells you why.

## The small boards on the I²C header

All on bus 7, the same header as the PCA9685 (0x40) and the BNO055 (0x28).

**INA219 battery monitor - address 0x41** (bridge the A0 jumper: the
default 0x40 is the servo board). Wire it for *voltage only*: battery + to
Vin−, GND to GND, nothing through the shunt - the breakout's 0.1 Ω shunt is
rated ~3 A and the motor rail stalls at ten times that. `battery_monitor`
publishes `battery` (`sensor_msgs/BatteryState`), the page shows it, and at
3.3 V/cell the node raises the `e_stop` lock every second until the pack is
charged. A pack below 6 V (the robot on its wall supply) is "absent" and
never trips anything. For current on the motor rail, later: an external
10 mΩ shunt and `battery_voltage_only:=false shunt_ohms:=0.01`.

**Four VL53L0X cliff sensors behind a TCA9548A - mux at 0x70, sensors on
channels 0-3 = front_left, front_right, rear_left, rear_right** (they all
answer at 0x29, hence the mux). Mount them at the very front and rear edges,
looking down and tilted ~15° outward so each sees the floor 5-10 cm beyond
the bumper: the sensor is what stops the robot, and a sensor over the edge
means the wheels are 3 cm from it. Their positions are the node's `xs`/`ys`
parameters. `cliff_guard` learns the floor distance from the first readings
at start-up - **the robot must be on flat floor when it boots** - and calls
60 mm more, or no return, a drop. Drops go to the collision guard as points
just outside that corner (`drive.launch.py cliff:=true` adds the source), so
a drop ahead is refused like a wall, reversing away is allowed, the page
says "blocked", and a dead sensor stops the robot. Its driver is Adafruit's,
in `~/venv-sensors` (system site-packages, so rclpy still imports):

```bash
python3 -m venv --system-site-packages ~/venv-sensors
~/venv-sensors/bin/pip install adafruit-circuitpython-vl53l0x adafruit-circuitpython-tca9548a adafruit-extended-bus
```

Turn it on with `use_cliff:=true`; to make it the boot default, write
`ROBOT_ARGS="nvblox:=true use_cliff:=true"` to `/etc/default/jetnano-robot`
(the service reads it; no unit edit). Do not turn it on before the sensors
are wired: the guard would time out on the missing source and refuse to
drive, which is the fail-safe doing its job. `use_cliff:=true
cliff_simulate:=true "cliff_simulated_drops:=[front_left]"` exercises the
whole chain without hardware.

**CR2032 on the carrier's 2-pin RTC connector**: no software. The boot
sequence checks whether the clock is already later than the workspace's
last build and, if so, skips the 90 s wait for NTP.

## What is measured and what is still a guess

The drive chain was calibrated on the real robot on 2026-09-21 and the
sensors were brought up on the Orin on 2026-09-22 - see
[docs/bench-calibration-2026-09-21.md](docs/bench-calibration-2026-09-21.md)
for the methods, the numbers and the lessons. **Measured**: the ESC's
neutral (1375 us - it runs the opposite of RC convention, shorter is
forward), both throttle endpoints, the steering limits (40-125), the rear
mirror and the twist signs; on the floor on 2026-09-27, the throttle start
points (she first rolls at 0.337 output both ways; `throttle_calibration`),
the speeds above and the steering trim (front 81 / rear 87, straight within
the linkage's play, about ±4°/m) - all in `pca9685.yaml`; the pan-tilt's
centre and reach (pan 1425 µs dead ahead, both servos run backwards); the
I2C bus and both addresses; the lidar's zero (the nose) and direction
(counter-clockwise); the camera's orientation (upright); and the IMU's
mount - upside down, turned 90 degrees, 10 degrees of bracket tilt -
which is `imu_rpy` in the URDF and which the tilt guard reads back
through TF.

Still guesses:

- **IMU position** (`jetnano.urdf.xacro`). The overall size is measured
  (2026-09-27, tyre to tyre): 0.444 m long, 0.295 m wide, 0.27 m tall - no
  bumpers, the tyres are the edges; Nav2's footprint uses it. The wheelbase
  (0.330 m), track (0.230 m), wheel radius (0.065 m), the lidar's and
  camera's positions and the lidar's height were tape-measured on
  2026-09-22, and every sensor orientation is measured; the body box and
  where the IMU sits are still estimates.
- **`minimum_turning_radius: 0.30`** (`nav2.yaml`): geometry gives 0.29 m for
  both axles at 30 deg, but tyre scrub on a crawler makes the real figure
  larger. Drive a full-lock circle and measure it.
- **Throttle to ground speed**: measured at 25/50/75 % of the page (above),
  not at 100 %, and `cmd_vel` is still a throttle fraction, not m/s.
  **Servo to wheel angle**: the `-18.33` gain is a guess until a protractor
  meets a tyre.
- **Joystick axis and button numbers** (`joysticks.yaml`): run
  `ros2 run jetnano_teleop list_devices --watch` and replace them with what
  you actually see.

## Licence

Apache-2.0.
