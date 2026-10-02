#!/usr/bin/env bash
# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
#
# A bench soak (Steve, 2026-10-02: "find out why everything crashes all the time"):
# leave her running - robot service up, Nav2 up, voice off - and write one line every
# SAMPLE_S seconds to ~/audit/soak-<start>.log with what a crash would show:
#   respawns  processes the launch started again since the soak began (a real death)
#   wd_down   watchdog "down:" verdicts since the soak began
#   shm       Fast-DDS shared-memory port files in /dev/shm (0 once UDP-only is in)
#   cpu       % busy over the last sample (from /proc/stat), mem free MB, SoC temp
#   odom/vo/scan/imu  topic rates measured by a node (not the ros2 CLI), Hz
# Pass mark for 8 hours: respawns 0, wd_down 0.
#
#     soak.sh [HOURS] [SAMPLE_S]        (default 8 h, every 300 s; run it with nohup)
#     tail -f ~/audit/soak-*.log
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/etc/jetnano/fastdds_udp_only.xml}
HOURS=${1:-8}; SAMPLE_S=${2:-300}
START=$(date +%Y-%m-%dT%H:%M:%S)
LOG="$HOME/audit/soak-$(date +%Y%m%d-%H%M).log"
mkdir -p "$HOME/audit"
N=$(( HOURS * 3600 / SAMPLE_S ))
echo "# soak from $START, $HOURS h, every $SAMPLE_S s; uptime $(uptime -p)" | tee "$LOG"
echo "# time  respawns wd_down shm  cpu%  memfree_MB temp_C  odom vo scan imu (Hz)" | tee -a "$LOG"

cpu_line() { awk '/^cpu /{print $2+$3+$4+$6+$7+$8, $5}' /proc/stat; }
read -r BUSY0 IDLE0 <<< "$(cpu_line)"

rates() {
    python3 - <<'EOF' 2>/dev/null
import rclpy, time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy
rclpy.init(); n = rclpy.create_node('soak_rates'); c = {'odom': 0, 'vo': 0, 'scan': 0, 'imu': 0}
be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(Odometry, '/odometry/filtered', lambda m: c.__setitem__('odom', c['odom'] + 1), 10)
n.create_subscription(Odometry, '/vo', lambda m: c.__setitem__('vo', c['vo'] + 1), be)
n.create_subscription(LaserScan, '/scan', lambda m: c.__setitem__('scan', c['scan'] + 1), be)
n.create_subscription(Imu, '/imu/data', lambda m: c.__setitem__('imu', c['imu'] + 1), be)
t0 = time.time()
while time.time() - t0 < 5.0: rclpy.spin_once(n, timeout_sec=0.1)
print(' '.join(f"{c[k] / 5.0:.0f}" for k in ('odom', 'vo', 'scan', 'imu')))
n.destroy_node(); rclpy.shutdown()
EOF
}

for i in $(seq 1 "$N"); do
    sleep "$SAMPLE_S"
    read -r BUSY1 IDLE1 <<< "$(cpu_line)"
    CPU=$(( (BUSY1 - BUSY0) * 100 / (BUSY1 - BUSY0 + IDLE1 - IDLE0 + 1) ))
    BUSY0=$BUSY1; IDLE0=$IDLE1
    RESP=$(journalctl -u jetnano-robot -u jetnano-voice -u jetnano-localize --since "$START" --no-pager 2>/dev/null | grep -c "process started with pid")
    FIRST=$(journalctl -u jetnano-robot -u jetnano-voice -u jetnano-localize --since "$START" --no-pager 2>/dev/null | grep -c "Starting jetnano-\|Started jetnano-")
    WD=$(journalctl -u jetnano-robot --since "$START" --no-pager 2>/dev/null | grep -c "watchdog\]: down:")
    SHM=$(ls /dev/shm 2>/dev/null | grep -c fastrtps)
    MEM=$(awk '/MemAvailable/{printf "%d", $2/1024}' /proc/meminfo)
    TEMP=$(awk '{printf "%.0f", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null)
    echo "$(date +%H:%M:%S)  $RESP $WD $SHM  $CPU  $MEM $TEMP  $(rates)  (service starts since: $FIRST)" | tee -a "$LOG"
done
echo "# soak done $(date +%Y-%m-%dT%H:%M:%S)" | tee -a "$LOG"
