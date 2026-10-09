"""Replay a drive bag through near_cap's logic.  [SMOOTH=1] python3 near_cap_replay.py BAG [BAG...]
SMOOTH=1 uses near_cap's Ceiling (hold + ramp), else the bare cap.
Per drive: monitor stops (fast-zone-only = near_cap would have prevented), time capped,
cap on/off flips, estimated time lost to the cap."""
import sys, math, json
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
import os.path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../jetnano_navigation/jetnano_navigation'))
from near_cap import Ceiling, cap_twist, zone_counts
import os
SMOOTH = os.environ.get('SMOOTH') == '1'

CAP, XIN, XOUT, HWC, MINP = 0.24, 0.12, 0.55, 0.19, 4
SLOW, FAST, HW = 0.372, 0.502, 0.17
def q2y(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

def run(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
    names = {t.name for t in r.get_all_topics_and_types()}
    want = [t for t in ('/tf_static', '/scan', '/cmd_vel_nav_smoothed', '/collision_monitor_state') if t in names]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    ceil = Ceiling(CAP)
    laser = None; near = (False, False); zone = None; scan_t = None
    t0 = None; last_cmd = None; capped = False
    st = dict(cmds=0, capped_s=0.0, lost_s=0.0, flips=0, short_caps=0, stops=[], moving_s=0.0)
    cap_on_t = None; stop_t = None
    while r.has_next():
        t, d, ts = r.read_next(); ts *= 1e-9
        if t0 is None: t0 = ts
        if t == '/tf_static':
            for tr in deserialize_message(d, TFMessage).transforms:
                if tr.child_frame_id in ('lidar_link', 'laser', 'laser_frame'):
                    laser = (tr.transform.translation.x, tr.transform.translation.y, q2y(tr.transform.rotation))
        elif t == '/scan':
            if laser is None: continue
            m = deserialize_message(d, LaserScan)
            rr = np.asarray(m.ranges, float); a = m.angle_min + np.arange(len(rr)) * m.angle_increment + laser[2]
            ok = np.isfinite(rr) & (rr > m.range_min) & (rr < 1.2)
            px, py = laser[0] + rr[ok] * np.cos(a[ok]), laser[1] + rr[ok] * np.sin(a[ok])
            f, b = zone_counts(px, py, XIN, XOUT, HWC)
            near = (f >= MINP, b >= MINP)
            side = np.abs(py) <= HW
            zone = dict(slow_f=int(np.count_nonzero(side & (px >= 0.12) & (px <= SLOW))),
                        fast_f=int(np.count_nonzero(side & (px >= 0.12) & (px <= FAST))),
                        slow_r=int(np.count_nonzero(side & (px <= -0.12) & (px >= -SLOW))),
                        fast_r=int(np.count_nonzero(side & (px <= -0.12) & (px >= -FAST))))
            scan_t = ts
        elif t == '/cmd_vel_nav_smoothed':
            m = deserialize_message(d, Twist); st['cmds'] += 1
            fresh = scan_t is not None and ts - scan_t < 0.5
            nf, nr = near if fresh else (False, False)
            if SMOOTH:
                v, w = ceil.step(m.linear.x, m.angular.z, nf, nr, ts); c = v != m.linear.x
            else:
                v, w = cap_twist(m.linear.x, m.angular.z, nf, nr, CAP); c = v != m.linear.x
            if last_cmd is not None:
                dt = min(ts - last_cmd[0], 0.2)
                if abs(last_cmd[1]) > 0.02: st['moving_s'] += dt
                if last_cmd[2]:
                    st['capped_s'] += dt
                    st['lost_s'] += dt * (abs(last_cmd[1]) / last_cmd[3] - 1)
            if c != capped:
                st['flips'] += 1
                if c: cap_on_t = ts
                elif cap_on_t is not None and ts - cap_on_t < 0.3: st['short_caps'] += 1
                capped = c
            last_cmd = (ts, m.linear.x, c, max(abs(v), 0.01))
        elif t == '/collision_monitor_state':
            m = deserialize_message(d, CollisionMonitorState)
            if m.action_type == 1 and stop_t is None:
                stop_t = ts
                ask = last_cmd[1] if last_cmd else 0
                z = zone or {}
                fwd = ask >= 0
                slow_pts = z.get('slow_f' if fwd else 'slow_r', -1)
                st['stops'].append(dict(t=round(ts - t0, 1), poly=m.polygon_name, ask=round(ask, 2),
                                        slow_pts=slow_pts, fast_pts=z.get('fast_f' if fwd else 'fast_r', -1),
                                        nc_capped=bool(last_cmd and last_cmd[2])))
            elif m.action_type != 1 and stop_t is not None:
                st['stops'][-1]['dur'] = round(ts - stop_t, 1); stop_t = None
    for s in st['stops']:
        s['avoidable'] = s['slow_pts'] < MINP and abs(s['ask']) >= 0.25
    st['capped_s'] = round(st['capped_s'], 1); st['lost_s'] = round(st['lost_s'], 1); st['moving_s'] = round(st['moving_s'], 1)
    return st

for b in sys.argv[1:]:
    try:
        s = run(b)
    except Exception as e:
        print(b.split('/')[-1], 'ERR', e); continue
    stops = s.pop('stops')
    print(b.split('/')[-1], json.dumps(s))
    for x in stops: print('   stop', json.dumps(x))
