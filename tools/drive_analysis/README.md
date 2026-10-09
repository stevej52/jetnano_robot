# Drive analysis on the laptop

Tools for digging into a drive recording (`drive_record.sh start full`) away from the
robot: they read the MCAP file directly (`pip install mcap mcap-ros2-support numpy`),
so no ROS is needed. Written for the 2026-09-29 floor drives.

| Tool | What it does |
|---|---|
| `extract_heading.py BAG.mcap OUT.npz` | every heading source (BNO055 gyro and fused orientation, camera odometry, MOLA, the EKF, SLAM's map -> odom, the drive commands) into one small file |
| `heading_drift.py OUT.npz` | which source drifted: per 20 s window against SLAM, a scale (turns over/under-counted) and a bias (drift standing still) for each |
| `imu_mount_check.py BAG.mcap OUT.npz` | the BNO055's orientation turned into base_link with the URDF mount (as `imu_mount.py` does): level? heading slope +1? |
| `run_report_laptop.py BAG_DIR` | `drive_report` on the laptop (`fake_ros/` stands in for rosbag2_py and rclpy), with its peak memory |

What 2026-09-29 showed: over an hour and 3700 deg of turning the EKF's heading lost
63 deg (camera yaw rate 5.7 % short on every turn, gyro 3.6 %), while the BNO055's own
fused heading was off 12.4 deg with its scale right - so the EKF now takes its heading
from that (`jetnano_bringup/config/ekf.yaml`, imu0).

## Collision-monitor stops and near_cap (2026-10-08)

`stop_why.py BAG T DUR` prints, around a stop, what Nav2 asked, the monitor's state and the
lidar points inside the DirectionalStop zones. `near_cap_replay.py BAG...` replays drives
through near_cap (`SMOOTH=1` = with its hold and ramp): stops, time capped, time lost.

Drives 48-51 (10-07): every DirectionalStop stop had the slow zone EMPTY. All were fast-zone
stops, the kind near_cap turns into a crawl. Cost: 0-14 s capped per lap (most of it at the
dining-north line-up), 0-10 s lost. Hold + ramp: +0.3 s and no sub-0.3 s caps.
