# jetnano_robot

ROS 2 Jazzy software for a **DANCHEE RidgeRock 1/10 four-wheel-steering rock
crawler** running on an NVIDIA Jetson Orin Nano Super — the robot written up on
[Hackaday](https://hackaday.com/2020/10/16/jetson-nano-robot/), rebuilt from a
Jetson Nano on ROS 2 Eloquent to an Orin Nano on Jazzy.

| | |
|---|---|
| Motors | Adafruit PCA9685, ESC on ch 0, front steering ch 1, rear steering ch 2 |
| Lidar | RPLidar A1M8 |
| Camera | Intel RealSense D435 |
| IMU | BNO055 |
| Odometry | visual (rtabmap) + IMU, fused by robot_localization — **no wheel encoders** |
| Teleop | Thrustmaster HOTAS or Xbox pad, auto-detected |

## Packages

| Package | What it is |
|---|---|
| `jetnano_description` | URDF/xacro and RViz layouts |
| `jetnano_bringup` | Launch files and config that tie everything together, plus a fake-sensor rig for bench testing |
| `jetnano_teleop` | One joystick node that detects which controller is plugged in |
| `jetnano_navigation` | slam_toolbox and Nav2, set up for a car-like chassis |

Setup for the machines themselves (Ubuntu 24.04 + ROS 2 Jazzy) lives in
[robot-environment](https://github.com/stevej52/robot-environment). The
PCA9685 driver is [ros2_pca9685](https://github.com/stevej52/ros2_pca9685).

## Running it

On the robot:

```bash
ros2 launch jetnano_bringup robot.launch.py
```

On a machine with a screen (`ROS_DOMAIN_ID=7` on both):

```bash
ros2 launch jetnano_bringup rviz.launch.py
ros2 launch jetnano_bringup teleop.launch.py
```

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

## How commands reach the wheels

```
teleop  ──/cmd_vel_teleop (priority 100)──┐
                                          ├─ twist_mux ──/cmd_vel──▶ ros2_pca9685 ──I²C──▶ ESC + servos
Nav2    ──/cmd_vel_nav    (priority 10)───┘        ▲
                                                   │
                                        /e_stop ───┘  (lock, priority 255)
```

Three rules hold this together, and each was a bug before it was a rule:

1. **Only `ros2_pca9685` touches the I²C bus.** Teleop publishes a Twist; it
   does not drive servos.
2. **Nav2 never publishes to `/cmd_vel`.** Its outputs are remapped to
   `cmd_vel_nav_raw` and `cmd_vel_nav`, so it cannot bypass twist_mux — which
   would mean bypassing the joystick override and the e-stop.
3. **Only the EKF publishes `odom → base_footprint`.** `rgbd_odometry` runs
   with `publish_tf:=false` and feeds it as a measurement on `/vo`.

Tested in simulate mode: teleop overrides Nav2 mid-run and Nav2 resumes when
teleop lets go; the e-stop lock blocks a full-throttle command; every channel
returns to neutral 0.5 s after commands stop.

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

## Things that are guesses, not measurements

Marked `MEASURE ME` in the files:

- **Chassis geometry** (`jetnano.urdf.xacro`): wheelbase 0.313 m, track
  0.220 m, wheel radius 0.060 m, and the sensor mount positions. Estimates for
  a 1/10 crawler, written while the robot was in storage.
- **`i2c_bus: 7`** (`pca9685.yaml`): it was 1 on the Jetson Nano. JetPack 7 on
  the Orin numbers the 40-pin header differently. Confirm with `i2cdetect -l`
  and `i2cdetect -y -r 7`, looking for address `40`.
- **`minimum_turning_radius: 0.30`** (`nav2.yaml`): geometry gives 0.27 m for
  both axles at 30°, but tyre scrub on a crawler makes the real figure larger.
- **Joystick axis and button numbers** (`joysticks.yaml`): run
  `ros2 run jetnano_teleop list_devices --watch` and replace them with what
  you actually see.

## Licence

Apache-2.0.
