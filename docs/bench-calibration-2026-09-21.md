# Bench calibration, 2026-09-21

What was measured on the real robot the night it came out of storage, how,
and what it changed. Every number here replaced a guess in the config; the
guesses that are still guesses are listed at the end.

## Setup

- The robot on a bin, **wheels off the ground**, on its own battery (a 3S
  that had sat five years at storage charge and still read 11.5 V).
- Driven from the **old Jetson Nano** (`jetnano`, JetPack 4.5.1), because
  it still had the 2021 software and the harness on it. Its Ethernet jack is
  dead, so it was reached over USB device mode (`192.168.55.1`) through
  H2-Host, with root via its existing `NOPASSWD` sudoers line.
- Pulses generated with `adafruit_servokit` on **ServoKit defaults**
  (750-2250 us over 180 degrees) at **100 Hz** - the same library and
  settings the 2021 code used, so every number below is what that code
  would have sent.
- Separate regulators for the computer and the servo rail. Switching the
  servo rail on while the Nano was running still reset the Nano once - the
  inrush sagging the shared 12 V feed, not a shared regulator. For the Orin:
  fat 12 V wiring and some bulk capacitance, not a new regulator.

## What the I2C bus said

`i2cdetect -y -r 1` (header pins 3/5): `28` BNO055, `40` PCA9685, `70` the
PCA9685's all-call. Bus 0 (pins 27/28) empty. BNO055 chip ID `0xa0`,
self-test `0x0f` (all four pass). On the Orin the same pins are `/dev/i2c-7`,
which the configs already assume.

## Steering

| | Result |
|---|---|
| Centre | **84** is dead straight (config had 85) |
| Travel | 45 and 120 reach cleanly, no buzz, no binding; limits set **40-125** |
| Rear | mirrored about the centre, `168 - front` - opposite phase |
| Sign | a **lower** front angle steers the robot **left**, so the existing `-18.33` / `+18.33` twist gains were right |

## ESC

The ESC runs the **opposite of RC convention**: neutral is **1375 us**
(1500 drives the robot backward at a medium pace), and **shorter pulses go
forward**. It arms silently at neutral; the first test looked dead only
because +/-25 us nudges sit inside its dead zone.

Found by stepping the pulse 20 us at a time, holding **at least 4 s** per
step, with the owner rating wheel speed out of 10:

```
forward  1292:2  1272:4  1252:6  1232:8  1212:9  1192:10  1172:11
reverse  1514:3  1530:4  1550:5  1570:7  1595:8  1625:8   1665:8
```

Forward keeps gaining to 1172; reverse plateaus around 1600. So the robot's
forward is the ESC's **full-power** side and its reverse the **limited**
side - the right way round for a crawler.

Two lessons that cost an hour:

- **Short holds lie.** With 2.5 s holds, the extreme of each range read as
  *slower* than the step before it, on both sides, in two separate runs.
  That was the ESC's soft-start ramp not finishing, not an edge of its
  calibration. Hold 4 s or more.
- **Ranking free-spinning wheels by eye is unreliable** past a certain speed
  - two runs disagreed outright. Rating each step against the previous one
  (2, 4, 6...) worked where "which was faster" did not.

The 2021 code stopped at 1250 / 1583 (ServoKit 60 / 100 degrees) and left
speed on the table on both sides.

## What went into `pca9685.yaml`

```
pwm_frequency        100
throttle  min / neutral / max   1172 / 1375 / 1665 us
          limits               -1.0 / 1.0
          twist linear_x       -1.0        (forward = shorter pulse)
          reverse_start 0.12   (robot forward; 25 us dead zone / 203 us range)
          forward_start 0.09   (robot backward; 25 / 290)
steering  home 84, limits 40-125, angular_z -18.33
rear      home 84, limits 43-128, angular_z +18.33
```

The driver names throttle sides by pulse direction, so its "reverse" is the
robot's forward once `linear_x` is negative. The comments in the file say so
at every place it matters.

## Recovered from the old card, not measured tonight

- BNO055: NDOF mode `0x0C`, 100 Hz, and a saved calibration profile -
  acc `[0xFFEC, 0x00A5, 0xFFE8]`, mag `[0xFFB4, 0xFE9E, 0x027D]`,
  gyr `[0x0002, 0xFFFF, 0xFFFF]` (`/home/main/bno055_params.yaml`).
- D435 serial `936723023901`; RPLidar on a CP2102 (`10c4:ea60`); the HC-12
  radio was on the header UART `/dev/ttyTHS1` at 9600, so nothing competes
  with the lidar for `ttyUSB0`.
- Robot power: 12 V rail -> 5 V buck -> Nano jack. The Orin takes the 12 V
  rail directly (9-20 V input); only the barrel plug changes, 5.5x2.1 to
  5.5x2.5 mm.

A full copy of the old card (rootfs plus the 13 bootloader partitions) is on
the Orin at `/home/jeston/nano-backup/`.

## Sensors on the Orin, 2026-09-22

The harness moved to the Orin - pins 1/3/5/9 (3.3 V, SDA, SCL, GND), all on
the inner row, nothing on 5 V - and the sensors were brought up with the
Jetson on its 56 W wall adapter and the drive rail dead. `i2cdetect -y -r 7`
shows `28`, `40`, `70` exactly as the Nano's bus did; BNO055 chip id `0xa0`,
self-test `0x0f`.

What `sensors.launch.py` needed before any driver ran:

- rplidar_ros 2.1.0 ships `rplidar_composition`, not `rplidar_node`.
- The A1M8 (firmware 1.27) rejects the `Sensitivity` scan mode (an A2/A3
  mode). `Boost` gives **720 points per turn at 7.6 Hz** - the count the
  simulator was built around.
- The bno055 driver publishes `imu/imu`; everything downstream wants
  `imu/data`, so the launch remaps it.
- The D435's IR streams use a Y8 format the stock Jetson kernel's UVC driver
  does not know, and leaving them enabled stalled depth ("Frames didn't
  arrive within 5 seconds"). With `enable_infra1/2: false`: colour, depth and
  aligned depth all at 30 Hz over USB 3.2.
- `jeston` needs `dialout`; `jetnano_bringup/udev/99-robot-sensors.rules`
  opens the D435 for libusb and names the lidar `/dev/rplidar`.

Rates: `/scan` 7.7 Hz, `/imu/data` 49.8 Hz, colour / depth / aligned 30 Hz.

Orientations, checked by hand:

- **Camera**: the colour frame is upright; depth matches it pixel for pixel,
  91 % valid, the desk at 0.28 m along the bottom edge. After Steve lowered
  it out of the lidar plane it points **27.1 deg down** with **+0.9 deg
  roll**, lens **0.178 m above the floor** - a plane fit to the floor in the
  aligned depth with the robot on the ground (19,600 px, 0.1 mm residual,
  three identical readings), cross-checked by the depth at the image centre
  (0.178 / sin 27.1 deg = 0.39 m, as measured). `camera_rpy = "0.016 0.473
  0"`, `camera_xyz` z = 0.118.
- **Lidar**: the owner stood at the robot's left and appeared at **+90 deg**,
  so angles run counter-clockwise (REP-103) and **0 deg is the nose** - the
  proof being that the thing blocking 0 deg was the camera. The camera
  mount sat in the scan plane and blinded **-19..+13 deg** at 0.17 m; it is
  being lowered out of the plane.
- **IMU**: raw accelerometer at rest x -0.6, y +1.7, **z -9.9** - the chip's
  z points at the floor. Tipping the nose up moved the sensor's **+y** (+1.7
  to +6.2); lifting the left side moved its **+x** (-0.5 to +4.4). So the
  sensor's +y is the robot's +x, its +x the robot's +y, its +z the robot's
  -z, with a 10 deg residual from the bracket. Solving those three vectors
  gives `imu_rpy = "2.9698 0.0552 1.5680"` (roll 170.2, pitch 3.2, yaw 89.8
  deg); rotated through it the rest reading is x 0.00, y 0.00, z +9.96. The
  tilt guard reads the mount from TF and, with the robot level, stays SAFE
  where the raw reading would have said "rolled 170 degrees".
- The "saved calibration" on the old card is the driver's example defaults
  (`DEFAULT_OFFSET_ACC` and friends), not a calibration. `imu/calib_status`
  at power-up: sys 0, gyro 3, accel 1, mag 0. **Calibrated for real that
  evening**: six accelerometer placements and some figure-eights by hand
  took it to 3/3/3/3 in 47 s. The offsets were read in CONFIG mode (a
  fusion-mode read returns zeros for the magnetometer) and live in
  `config/bno055.yaml`, which the launch loads with `set_offsets`.

### Visual odometry on the real camera

`odometry.launch.py` (rtabmap `rgbd_odometry` + the EKF) against the live
D435, robot parked on the tile floor, MAXN_SUPER:

| | |
|---|---|
| `/vo` | ~10 Hz (9.9 over a 30 s window; 8.5-14.9 in short ones) |
| tracking | 810 features, 508 matches, 357 inliers, 67 ms per estimate, never lost |
| `rgbd_odometry` | **84-88 % of one core** (single-threaded, as with the stand-in) |
| `realsense2_camera` | 39-54 % of a core (depth alignment plus stamp sync) |
| bno055 / ekf / rplidar | 12-15 % / 5-6 % / 1 % |
| whole system | 27-39 % of six cores, 6.4-7.0 W, GPU 0 %, 1.17 GB used |

Two things had to be fixed before `/vo` appeared at all: realsense2_camera
4.58 prefixes `base_frame_id` with the camera name, so `'camera_link'`
produced `camera_camera_link` and the TF chain to the images was cut
(`'link'` is right); and without `enable_sync` the aligned depth trailed the
colour by a frame, so rtabmap logged "time difference ... is high" and
dropped pairs about a hundred times a minute - synced, that falls to ~20
(colour settles at 25 Hz).

**Slid by hand, 2026-09-22 evening.** Forward about a metre and back on the
tile floor, robot pushed by hand. VO tracked the whole way: 0 to **0.83 m**
out, back to **x 0.09, y -0.08 m** - about 12 cm off the true start after a
1.7 m round trip (~7 %), heading **3.8 deg** off at the end. Inliers fell
from ~340 at rest to 90-200 while moving (motion blur on a plain floor 0.4 m
from the lens) and tracking was lost **once**, for one frame, which is where
most of the heading error came from (a 6 deg jump on recovery). The EKF
followed VO within a centimetre.

The EKF's heading had been flapping between -15 and -105 deg with the robot
parked: `ekf.yaml` fused two absolute yaws (VO's, and the BNO055's
magnetometer heading through the mount) and flipped between them. The IMU
now contributes yaw *rate* only, and `imu0_remove_gravitational_acceleration`
is off because this driver's `imu/data` is already gravity-free. Heading
steady at 0.0 deg afterwards.

### Tilt guard, live

Real robot, level, mount read from TF: SAFE. Steve lifted the left side past
30 deg: `safe -> armed (roll 25.2) -> recovering` (reverse published for the
full 5 s) `-> locked_out -> safe` once level. In Gazebo the same guard first
fired non-stop on flat ground, because Gazebo's IMU reports orientation
against its own starting pose (identity at rest, whatever the mount) while
the BNO055 reports it against the world; the sim IMU now uses an ENU
reference and behaves like the chip - at rest it reports the mount, and the
robot dropped from 0.5 m rolled 40 deg gives `safe -> armed (40.0) ->
recovering -> safe`. Headless Gazebo on the host's iGPU sometimes starves
the simulated IMU for over a second, so the guard's "IMU stopped, holding
still" trips occasionally there; it never does on the robot.

Lessons: hold each pose ten seconds and let the plateaus speak; start the
recorder *before* asking for the motion; scan for Wi-Fi only while
disconnected (the Realtek driver lists just its current AP otherwise); a
`SIGKILL`ed rplidar driver leaves the next one spinning at 100 % CPU with no
scans, so stop it gently; and never `pkill -x ros2` on a robot with more
than one launch running.

## Still guesses

- Chassis geometry in `jetnano.urdf.xacro`: wheelbase, track, loaded wheel
  radius, body box, and every sensor's position (orientations are done).
- Real minimum turning radius on the floor (`nav2.yaml` has 0.30 m; geometry
  says 0.27; tyre scrub makes it larger).
- Throttle to ground speed - `cmd_vel` is not in metres per second until a
  taped-out run is timed.
- Servo degrees to wheel degrees (a protractor at the tyre), which is what
  turns the `-18.33` gain from a guess into a number.
- Static tip angles, roll and pitch, on a board with a phone inclinometer -
  they set `tilt_guard`'s thresholds.
- Joystick axis and button numbers (`ros2 run jetnano_teleop list_devices`).
- BNO055 calibration offsets - drive it around until `imu/calib_status` is
  3/3, then read them back and save them.

## To repeat the sweep

Wheels up, ESC on, then hold each pulse at least 4 s and rate it against
the previous one. The script that drove tonight's runs is
`/home/main/servo_check.py` on the old card's backup; anything that can set
a PCA9685 channel to a pulse width will do.
