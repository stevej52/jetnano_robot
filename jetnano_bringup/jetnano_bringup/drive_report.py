# Copyright 2026 stevej52
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""What happened on a drive, from the bag drive_record.sh wrote.

    ros2 run jetnano_bringup drive_report ~/bags/drive-20260924-145523

Prints, and writes to <bag>/report.txt:

- odometry health: VO rate and gaps, EKF-vs-VO separation (the two must
  agree; on 2026-09-24 the EKF ran 6.5 km while VO moved 4.5 m), jumps;
- the phone link: how long the page was driving, command dropouts > 0.4 s
  (each one stops the robot);
- the collision guard: stop events, and for each whether the lidar had
  points in the stop zone at that moment - if not, it was the camera's map
  (nvblox), which is either a low obstacle the lidar cannot see or a phantom;
- throttle -> ground speed from steady stretches, measured by VO, which is
  what the throttle start offsets in pca9685.yaml and the guard's zone
  lengths want.

The stop-zone geometry and the lidar's pose are copied from
config/collision_guard.yaml and the URDF; if those change, change ZONES.
"""

import glob
import math
import os
import sys
from collections import defaultdict

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

LIDAR_X, LIDAR_YAW = -0.005, math.pi          # jetnano.urdf.xacro
ZONES = {'forward': (0.20, 0.52, 0.16), 'backward': (-0.52, -0.20, 0.16)}   # collision_guard.yaml stop_zone


def load(bag):
    reader = SequentialReader()
    try:
        reader.open(StorageOptions(uri=bag, storage_id=''), ConverterOptions('', ''))
    except RuntimeError:
        # No metadata.yaml (the recorder was killed): open the file directly.
        reader = SequentialReader()
        reader.open(StorageOptions(uri=glob.glob(os.path.join(bag, '*.mcap'))[0], storage_id='mcap'),
                    ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    msgs = defaultdict(list)
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in types:
            msgs[topic].append((t / 1e9, deserialize_message(data, types[topic])))
    for v in msgs.values():
        v.sort(key=lambda p: p[0])
    return msgs


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def nearest(lst, t):
    lo, hi = 0, len(lst) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if lst[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    return lst[lo]


def report(bag):
    msgs = load(bag)
    lines = []
    say = lines.append
    if not msgs:
        say('empty bag')
        return lines
    t0 = min(v[0][0] for v in msgs.values() if v)
    t1 = max(v[-1][0] for v in msgs.values() if v)
    say(f'{os.path.basename(bag.rstrip("/"))}: {t1 - t0:.0f} s, '
        + ', '.join(f'{k.split("/")[-1]}={len(v)}' for k, v in sorted(msgs.items())))

    # --- odometry health ---------------------------------------------------
    def poses(topic):
        return [(t, m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
                for t, m in msgs.get(topic, [])]

    vo, ekf = poses('/vo'), poses('/odometry/filtered')
    for name, P in (('VO', vo), ('EKF', ekf)):
        if len(P) < 2:
            say(f'{name}: no data')
            continue
        length = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(P, P[1:]))
        jumps = sum(1 for a, b in zip(P, P[1:]) if math.hypot(b[1] - a[1], b[2] - a[2]) > 0.10)
        gaps = [b[0] - a[0] for a, b in zip(P, P[1:]) if b[0] - a[0] > 0.5]
        say(f'{name}: {len(P) / (P[-1][0] - P[0][0]):.1f} Hz, path {length:.1f} m, jumps >10 cm: {jumps}, '
            f'gaps >0.5 s: {len(gaps)}' + (f' (worst {max(gaps):.1f} s)' if gaps else ''))
    if vo and ekf:
        sep = max(math.hypot(e[1] - v[1], e[2] - v[2]) for e in ekf[::10] for v in [nearest(vo, e[0])])
        say(f'EKF vs VO: worst separation {sep:.2f} m' + ('  <-- the filter left its sensor' if sep > 0.5 else ''))
    hold = msgs.get('/odom_hold', [])
    if hold:
        say(f'vo_watchdog held the EKF {len(hold) / 10:.0f} s in total (VO was silent that long)')

    # --- the phone link ------------------------------------------------------
    W = msgs.get('/cmd_vel_web', [])
    driving_t, drops = 0.0, []
    for (ta, ma), (tb, mb) in zip(W, W[1:]):
        if ma.linear.x != 0 or ma.angular.z != 0:
            driving_t += min(tb - ta, 0.4)
            if tb - ta > 0.4 and (mb.linear.x != 0 or mb.angular.z != 0):
                drops.append(tb - ta)
    if W:
        say(f'phone: {driving_t:.0f} s of driving, dropouts >0.4 s mid-drive: {len(drops)}'
            + (f' (worst {max(drops):.1f} s)' if drops else ''))

    # --- the collision guard -------------------------------------------------
    S = msgs.get('/collision_guard/state', [])
    scans, mux = msgs.get('/scan', []), msgs.get('/cmd_vel_mux', [])
    events, prev = [], 0
    for t, m in S:
        if m.action_type == 1 and prev != 1:
            events.append(t)
        prev = m.action_type
    slowdowns = sum(1 for (ta, a), (tb, b) in zip(S, S[1:]) if b.action_type == 2 and a.action_type != 2)
    lidar_caused = camera_caused = 0
    for t in events:
        if not scans or not mux:
            break
        ts, scan = nearest(scans, t)
        cmd = nearest(mux, t)[1]
        if abs(ts - t) > 0.3:
            continue
        x0, x1, hw = ZONES['forward' if cmd.linear.x >= 0 else 'backward']
        n = 0
        for i, r in enumerate(scan.ranges):
            if scan.range_min <= r <= scan.range_max:
                a = scan.angle_min + i * scan.angle_increment + LIDAR_YAW
                x, y = r * math.cos(a) + LIDAR_X, r * math.sin(a)
                if x0 <= x <= x1 and abs(y) <= hw:
                    n += 1
        if n >= 3:
            lidar_caused += 1
        else:
            camera_caused += 1
    if S:
        say(f'guard: {len(events)} stops ({lidar_caused} with lidar points in the zone, {camera_caused} camera-only), '
            f'{slowdowns} slowdowns')

    # --- throttle -> ground speed, measured by VO ----------------------------
    C = msgs.get('/cmd_vel', [])
    by_throttle = defaultdict(list)
    i = 0
    while i < len(C) and vo:
        th = round(C[i][1].linear.x, 2)
        j = i
        while j + 1 < len(C) and round(C[j + 1][1].linear.x, 2) == th:
            j += 1
        ta, tb = C[i][0], C[j][0]
        if abs(th) >= 0.05 and tb - ta >= 1.0:
            pa, pb = nearest(vo, ta + 0.5), nearest(vo, tb)
            if pb[0] - pa[0] >= 0.4:
                by_throttle[th].append(math.hypot(pb[1] - pa[1], pb[2] - pa[2]) / (pb[0] - pa[0]))
        i = j + 1
    if by_throttle:
        say('throttle -> ground speed (steady >= 1 s, by VO):')
        for th in sorted(by_throttle):
            v = by_throttle[th]
            say(f'  {th:+.2f}: {sum(v) / len(v):.2f} m/s (n={len(v)}, max {max(v):.2f})')
    elif C:
        say('throttle -> speed: no steady stretch of a second or more (the guard kept interrupting, or too little driving)')
    return lines


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    bag = os.path.expanduser(sys.argv[1])
    lines = report(bag)
    text = '\n'.join(lines)
    print(text)
    try:
        with open(os.path.join(bag, 'report.txt'), 'w') as handle:
            handle.write(text + '\n')
    except OSError:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
