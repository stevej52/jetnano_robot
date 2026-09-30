"""Every gap in the fused lidar odometry (/lidar_odom, what the EKF gets) and why: was MOLA
itself quiet (/lidar_odometry/pose), or talking and held back by the outlier gate? With her
motion, MOLA's quality and the scan rate across each gap.

    python lidar_odom_gaps.py BAG.mcap [MIN_GAP_S]
"""
import math
import sys
from collections import defaultdict

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

TOPICS = ['/lidar_odom', '/lidar_odometry/pose', '/lidar_odometry/pose_quality', '/scan', '/odometry/filtered',
          '/mola_diagnostics/lidar_odom/status', '/rosout']


def main(bag, min_gap=0.5):
    dec, ds = DecoderFactory(), {}
    t = defaultdict(list)
    quality, odom, rosout = [], [], []
    t0 = None
    with open(bag, 'rb') as f:
        for schema, ch, msg in make_reader(f).iter_messages(topics=TOPICS, log_time_order=True):
            tt = msg.log_time * 1e-9
            t0 = t0 or tt
            t[ch.topic].append(tt - t0)
            if ch.topic in ('/lidar_odometry/pose_quality', '/odometry/filtered', '/rosout'):
                d = ds.get(ch.id) or ds.setdefault(ch.id, dec.decoder_for('cdr', schema))
                m = d(msg.data)
                if ch.topic == '/lidar_odometry/pose_quality':
                    quality.append((tt - t0, float(m.data)))
                elif ch.topic == '/odometry/filtered':
                    tw = m.twist.twist
                    odom.append((tt - t0, tw.linear.x, tw.angular.z))
                elif any(k in m.name for k in ('mola', 'lidar', 'gate', 'lidar_odom')):
                    rosout.append((tt - t0, m.name, m.msg[:140]))
    arr = {k: np.array(v) for k, v in t.items()}
    q, od = np.array(quality), np.array(odom)
    lo = arr['/lidar_odom']
    print(f'bag {t0:.0f}: {len(lo)} /lidar_odom, {len(arr.get("/lidar_odometry/pose", []))} MOLA poses, '
          f'{len(arr.get("/scan", []))} scans over {max(v[-1] for v in arr.values()):.0f} s')
    gaps = [(a, b) for a, b in zip(lo[:-1], lo[1:]) if b - a > min_gap]
    for a, b in gaps:
        def n(topic):
            v = arr.get(topic, np.array([]))
            return int(((v > a) & (v < b)).sum())
        qs = q[(q[:, 0] > a) & (q[:, 0] < b), 1] if len(q) else np.array([])
        mv = od[(od[:, 0] > a) & (od[:, 0] < b)]
        speed = np.abs(mv[:, 1]).max() if len(mv) else float('nan')
        turn = math.degrees(np.abs(mv[:, 2]).max()) if len(mv) else float('nan')
        print(f'  {a:6.1f}-{b:6.1f} s ({b - a:4.1f} s): MOLA poses {n("/lidar_odometry/pose"):3d}, '
              f'scans {n("/scan"):3d} ({n("/scan") / (b - a):.1f}/s), quality '
              f'{("%.2f..%.2f" % (qs.min(), qs.max())) if len(qs) else "-"}, '
              f'max speed {speed:.2f} m/s, max turn {turn:.0f} deg/s')
    if len(q):
        print(f'quality overall: median {np.median(q[:, 1]):.2f}, p10 {np.percentile(q[:, 1], 10):.2f}')
    for r in rosout[:30]:
        print(f'  rosout {r[0]:6.1f} {r[1]}: {r[2]}')


if __name__ == '__main__':
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 0.5)
