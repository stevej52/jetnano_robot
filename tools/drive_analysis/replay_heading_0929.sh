#!/usr/bin/env bash
# Replay the 2026-09-29 drive's sensors through the NEW EKF settings (imu/base heading),
# isolated on ROS domain 88 so the live robot (domain 7) never sees any of it.
# Plays /imu/data, /vo, /lidar_odom, /odom_hold and /tf_static only (not /tf: the old EKF's
# odom->base and SLAM's map->odom would fight the new filter), turns /imu/data into /imu/base
# with imu_base_relay, runs ekf_node with the installed ekf.yaml on sim time (publish_tf off),
# and logs the new filter's heading to ~/audit/2026-09-29/replay_ekf_yaw.csv.
set -u
set +u; source /opt/ros/jazzy/setup.bash; source /home/jeston/ros2_ws/install/setup.bash; set -u
RATE=${1:-4}
export ROS_DOMAIN_ID=${2:-88}
TAG=${3:-new}
shift 3 2>/dev/null || true
EXTRA=("$@")
BAG=/home/jeston/bags/drive-20260929-160656
OUT=/home/jeston/audit/2026-09-29/replay_ekf_yaw_$TAG.csv
CFG=${CFG:-/home/jeston/ros2_ws/install/jetnano_bringup/share/jetnano_bringup/config/ekf.yaml}
LOG=/tmp/replay-heading-$TAG
rm -rf $LOG; mkdir -p $LOG

nice -n 10 ros2 run jetnano_bringup imu_base_relay --ros-args -p use_sim_time:=true > $LOG/relay.log 2>&1 &
RELAY=$!
nice -n 10 /opt/ros/jazzy/lib/robot_localization/ekf_node --ros-args -r __node:=ekf_filter_node \
    --params-file $CFG -p use_sim_time:=true -p publish_tf:=false "${EXTRA[@]}" > $LOG/ekf.log 2>&1 &
EKF=$!
nice -n 10 python3 - "$OUT" > $LOG/logger.log 2>&1 <<'PY' &
import math, sys, rclpy
from nav_msgs.msg import Odometry
rclpy.init()
n = rclpy.create_node('replay_logger', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
f = open(sys.argv[1], 'w')
f.write('stamp,x,y,yaw\n')
def cb(m):
    q = m.pose.pose.orientation
    y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    f.write(f'{m.header.stamp.sec + m.header.stamp.nanosec * 1e-9:.3f},{m.pose.pose.position.x:.4f},'
            f'{m.pose.pose.position.y:.4f},{y:.6f}\n')
n.create_subscription(Odometry, 'odometry/filtered', cb, 50)
try:
    rclpy.spin(n)
except Exception:
    pass
finally:
    f.close()
PY
LOGGER=$!
sleep 6
T0=$(date +%s)
nice -n 10 ros2 bag play $BAG --clock 100 --rate $RATE \
    --topics /imu/data /vo /lidar_odom /odom_hold /tf_static > $LOG/play.log 2>&1
echo "played in $(( $(date +%s) - T0 )) s"
sleep 3
kill -INT $LOGGER $EKF $RELAY 2>/dev/null
sleep 3
kill -KILL $LOGGER $EKF $RELAY 2>/dev/null
wc -l $OUT
grep -c "Failed to meet" $LOG/ekf.log
grep -iE "error|warn" $LOG/ekf.log | grep -v "Failed to meet" | head -5
