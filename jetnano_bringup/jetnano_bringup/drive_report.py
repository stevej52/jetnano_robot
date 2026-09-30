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

- odometry health: VO rate, gaps and restarts, and how far the EKF and VO
  disagree about how far she moved and turned over each 2 s (the two must
  agree; on 2026-09-24 the EKF ran 6.5 km while VO moved 4.5 m); with lidar
  odometry running (lidar_odom:=true) the same for it against VO. Only
  motion over windows is compared, never positions: VO starts over from its
  own origin whenever the camera pipeline restarts, and the lidar odometry
  is in a frame of its own - absolute positions read metres apart for no
  fault (2026-09-27: "11.51 m" on a drive where the EKF was fine). Windows
  across a VO restart or gap are skipped;
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

import bisect
import glob
import math
import os
import sys
from collections import defaultdict

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.utilities import get_message

LIDAR_X, LIDAR_YAW = -0.005, math.pi          # jetnano.urdf.xacro
ZONES = {'forward': (0.20, 0.52, 0.16), 'backward': (-0.52, -0.20, 0.16)}   # collision_guard.yaml stop_zone

# Only these are read, and of each only the numbers the report uses. Until 2026-09-29 every
# message of every topic was decoded and kept: an hour's full recording (1.8 GB, 1.9 million
# messages) grew the report to 2.9 GB, the Jetson ran out of memory, the camera, mic and
# lidar odometry stalled for a minute and the kernel killed the report. Scans (720 ranges
# each, 8 a second) are read in a second pass, only those next to a guard stop.
POSES = ('/vo', '/odometry/filtered', '/lidar_odom')
TWISTS = ('/cmd_vel_web', '/cmd_vel_mux', '/cmd_vel')
GUARD = '/collision_guard/state'
HOLD = '/odom_hold'
# MOLA's own output, before lidar_odom_relay's outlier gate: only its times are kept, to tell
# a gap the gate made (MOLA talking, its samples refused) from MOLA itself going quiet. Drive 7,
# 2026-09-29: "gaps: 14, worst 6.2 s" read as lost lidar odometry; it was the gate refusing
# MOLA through the parking shuffles, where it got the direction wrong.
MOLA = '/lidar_odometry/pose'


def open_reader(bag):
    reader = SequentialReader()
    try:
        reader.open(StorageOptions(uri=bag, storage_id=''), ConverterOptions('', ''))
    except RuntimeError:
        # No metadata.yaml (the recorder was killed): open the file directly.
        reader = SequentialReader()
        reader.open(StorageOptions(uri=glob.glob(os.path.join(bag, '*.mcap'))[0], storage_id='mcap'),
                    ConverterOptions('', ''))
    return reader


def load(bag):
    """{topic: [(t, ...numbers)]} sorted by time, and {topic: message count} for every topic.
    Poses are (t, x, y, yaw), commands (t, linear.x, angular.z), the guard (t, action_type)."""
    reader = open_reader(bag)
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    try:
        counts = {i.topic_metadata.name: i.message_count
                  for i in reader.get_metadata().topics_with_message_count}
    except (AttributeError, RuntimeError):
        counts = {}
    wanted = [t for t in POSES + TWISTS + (GUARD, HOLD, MOLA) if t in types]
    reader.set_filter(StorageFilter(topics=wanted))
    msgs = defaultdict(list)
    while reader.has_next():
        topic, data, t = reader.read_next()
        t /= 1e9
        if topic == MOLA:
            msgs[topic].append((t,))                 # the time is all it is read for
            continue
        m = deserialize_message(data, types[topic])
        if topic in POSES:
            p = m.pose.pose
            msgs[topic].append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
        elif topic in TWISTS:
            msgs[topic].append((t, m.linear.x, m.angular.z))
        elif topic == GUARD:
            msgs[topic].append((t, m.action_type))
        else:
            msgs[topic].append((t,))
    for v in msgs.values():
        v.sort(key=lambda p: p[0])
    if not counts:
        counts = {k: len(v) for k, v in msgs.items()}
    return msgs, counts


def scans_near(bag, times, window=0.3):
    """The /scan messages within `window` s of any of `times`, decoded: [(t, LaserScan)]."""
    if not times:
        return []
    reader = open_reader(bag)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if '/scan' not in types:
        return []
    scan_type = get_message(types['/scan'])
    reader.set_filter(StorageFilter(topics=['/scan']))
    times = sorted(times)
    out = []
    while reader.has_next():
        _, data, t = reader.read_next()
        t /= 1e9
        i = bisect.bisect_left(times, t - window)
        if i < len(times) and times[i] <= t + window:
            out.append((t, deserialize_message(data, scan_type)))
    out.sort(key=lambda p: p[0])
    return out


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


def index_at(lst, t):
    """Index of the first sample at or after t."""
    lo, hi = 0, len(lst)
    while lo < hi:
        mid = (lo + hi) // 2
        if lst[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    return lo


def _turned(a, b):
    return abs(math.degrees(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))))


def clean(P, ta, tb, max_step=0.10, max_turn=10.0, max_gap=0.3):
    """True if P runs through [ta, tb] without a gap or a jump (a restart): a restart
    near its origin barely moves the position but can reset the heading."""
    i, j = index_at(P, ta), index_at(P, tb)
    if i == 0 or j >= len(P):
        return False
    seg = P[i - 1:j + 1]
    return all(b[0] - a[0] <= max_gap and math.hypot(b[1] - a[1], b[2] - a[2]) <= max_step
               and _turned(a, b) <= max_turn for a, b in zip(seg, seg[1:]))


def restarts(P, jump=0.3, turn=20.0):
    """Times at which P started over: more than `jump` metres or `turn` degrees
    between two samples (VO: 22 ms apart, far beyond anything she can do)."""
    return [b[0] for a, b in zip(P, P[1:])
            if math.hypot(b[1] - a[1], b[2] - a[2]) > jump or _turned(a, b) > turn]


def window_diffs(A, B, span=2.0, step=1.0, a_must_be_clean=False):
    """How far apart A and B are about how far she moved and how much she turned over
    each `span` s, every `step` s: (sorted distance diffs m, sorted turn diffs deg). A
    window is skipped where B (and A if asked) has a gap or a restart in it."""
    dist, turn = [], []
    t = max(A[0][0], B[0][0])
    end = min(A[-1][0], B[-1][0]) - span
    while t <= end:
        a, b = nearest(A, t), nearest(A, t + span)
        va, vb = nearest(B, t), nearest(B, t + span)
        t += step
        if (b[0] - a[0] < 0.75 * span or abs(va[0] - a[0]) > 0.2 or abs(vb[0] - b[0]) > 0.2
                or not clean(B, a[0], b[0])
                or (a_must_be_clean and not clean(A, a[0], b[0], max_step=0.25, max_turn=30.0))):
            continue
        dist.append(abs(math.hypot(b[1] - a[1], b[2] - a[2]) - math.hypot(vb[1] - va[1], vb[2] - va[2])))
        d = (b[3] - a[3]) - (vb[3] - va[3])
        turn.append(abs(math.degrees(math.atan2(math.sin(d), math.cos(d)))))
    return sorted(dist), sorted(turn)


def gated(mola_times, a, b, min_hz=4.0):
    """Did MOLA keep publishing through the gap (a, b) - at least min_hz, and twice?"""
    n = sum(1 for (t,) in mola_times if a < t < b)
    return n >= max(2, min_hz * (b - a) * 0.5)


def report(bag):
    msgs, counts = load(bag)
    lines = []
    say = lines.append
    if not any(msgs.values()):
        say('empty bag' if not counts else 'none of the topics this report reads are in the bag')
        return lines
    t0 = min(v[0][0] for v in msgs.values() if v)
    t1 = max(v[-1][0] for v in msgs.values() if v)
    say(f'{os.path.basename(bag.rstrip("/"))}: {t1 - t0:.0f} s, '
        + ', '.join(f'{k.split("/")[-1]}={n}' for k, n in sorted(counts.items())))

    # --- odometry health ---------------------------------------------------
    vo, ekf, lo = msgs.get('/vo', []), msgs.get('/odometry/filtered', []), msgs.get('/lidar_odom', [])
    for name, P in [('VO', vo), ('EKF', ekf)] + ([('lidar odometry', lo)] if lo else []):
        if len(P) < 2:
            say(f'{name}: no data')
            continue
        length = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(P, P[1:]))
        jumps = sum(1 for a, b in zip(P, P[1:]) if math.hypot(b[1] - a[1], b[2] - a[2]) > 0.10)
        spans = [(a[0], b[0]) for a, b in zip(P, P[1:]) if b[0] - a[0] > 0.5]
        gaps = [b - a for a, b in spans]
        why = ''
        if name == 'lidar odometry' and spans and msgs.get(MOLA):
            held = sum(1 for a, b in spans if gated(msgs[MOLA], a, b))
            why = f': {held} held back by the outlier gate (MOLA talking), {len(spans) - held} MOLA itself quiet'
        say(f'{name}: {len(P) / (P[-1][0] - P[0][0]):.1f} Hz, path {length:.1f} m, jumps >10 cm: {jumps}, '
            f'gaps >0.5 s: {len(gaps)}' + (f' (worst {max(gaps):.1f} s)' if gaps else '') + why)
    if len(vo) > 1:
        again = restarts(vo)
        if again:
            say(f'VO started over {len(again)} time(s), at ' + ', '.join(f'{t - t0:.0f} s' for t in again)
                + ' (a camera pipeline restart: its jumps above include these)')

    def agreement(name, dist, turn, flag):
        if not dist:
            return
        say(f'{name} over 2 s ({len(dist)} windows): distance differs by {dist[len(dist) // 2] * 100:.0f} cm '
            f'typically, {dist[int(len(dist) * 0.99)] * 100:.0f} cm at the 99th percentile, '
            f'{dist[-1] * 100:.0f} cm at worst; turn by {turn[len(turn) // 2]:.1f} deg typically, '
            f'{turn[-1]:.0f} deg at worst' + (f'  <-- {flag}' if dist[-1] > 0.3 or turn[-1] > 20 else ''))
    if len(vo) > 1 and len(ekf) > 1:
        # the EKF may jump (that is what this is looking for); only VO must run clean
        agreement('EKF vs VO', *window_diffs(ekf, vo), 'the filter left its sensor')
    if len(vo) > 1 and len(lo) > 1:
        # the relay drops the stretches the lidar odometry got wrong: compare where it ran
        agreement('VO vs lidar odometry', *window_diffs(lo, vo, a_must_be_clean=True), 'one of them lost her')
    hold = msgs.get(HOLD, [])
    if hold:
        say(f'vo_watchdog held the EKF {len(hold) / 10:.0f} s in total (VO was silent that long)')

    # --- the phone link ------------------------------------------------------
    W = msgs.get('/cmd_vel_web', [])
    driving_t, drops = 0.0, []
    for (ta, ax, az), (tb, bx, bz) in zip(W, W[1:]):
        if ax != 0 or az != 0:
            driving_t += min(tb - ta, 0.4)
            if tb - ta > 0.4 and (bx != 0 or bz != 0):
                drops.append(tb - ta)
    if W:
        say(f'phone: {driving_t:.0f} s of driving, dropouts >0.4 s mid-drive: {len(drops)}'
            + (f' (worst {max(drops):.1f} s)' if drops else ''))

    # --- the collision guard -------------------------------------------------
    S = msgs.get(GUARD, [])
    mux = msgs.get('/cmd_vel_mux', [])
    events, prev = [], 0
    for t, action in S:
        if action == 1 and prev != 1:
            events.append(t)
        prev = action
    slowdowns = sum(1 for (ta, a), (tb, b) in zip(S, S[1:]) if b == 2 and a != 2)
    scans = scans_near(bag, events) if mux else []
    lidar_caused = camera_caused = 0
    for t in events:
        if not scans or not mux:
            break
        ts, scan = nearest(scans, t)
        cmd_x = nearest(mux, t)[1]
        if abs(ts - t) > 0.3:
            continue
        x0, x1, hw = ZONES['forward' if cmd_x >= 0 else 'backward']
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
        th = round(C[i][1], 2)
        j = i
        while j + 1 < len(C) and round(C[j + 1][1], 2) == th:
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
