#!/usr/bin/env python3
"""lap_report.py BAG [DIR] : one lap's scorecard, against the ground-zero baseline.

drive.sh runs it after every lap (2026-10-07, "ground zero"): the same numbers every time, so a
change is judged by what it did to them, not by one look at one lap. Written to DIR/report.json
and DIR/report.txt (and copied into the bag); printed too. Light on memory: it reads only the
topics it needs, one message at a time (drive_report OOM-killed at 2.9 GB on 2026-09-30).

  time        the route (route.log) and the park (park.log); moving time; direction switches
  stops       every time she was told to drive and did not (commanded >= 0.08 m/s, moving < 0.03
              for 0.5 s+), with where, how long, and who held her: Nav2's collision monitor
              (collision_monitor_state), the lidar collision guard, the gate, or nothing known
  weave       spread of the driven curvature on near-straight 2 s windows (1/m; lower = smoother)
  plans       new plans, and how far each one's next metre sits from the one before (cm)
  localise    map->odom corrections over 3 cm while driving: count and sizes (cm)
  clearance   nearest lidar return to her body outline, overall and at each of Steve's turns (cm)
"""
import json, math, os, re, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

HL, HW = 0.222, 0.148                      # her body outline (base_link)
TURNS = {'t1 chair': (2.9, -3.2, 0.9), 't2 island N': (1.0, -4.6, 0.9), 't3 island S': (0.2, -6.6, 1.0),
         't4 vacuum': (6.8, -7.3, 0.9), 't5 dining N': (6.0, -5.6, 0.9), 't6 chair': (2.6, -2.6, 0.9)}
BASELINE = os.path.expanduser('~/audit/ground_zero.json')


def q2y(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
    have = {t.name for t in r.get_all_topics_and_types()}
    want = ['/tf', '/tf_static', '/odometry/filtered', '/cmd_vel_mux', '/cmd_vel_nav_smoothed', '/plan', '/scan',
            '/collision_monitor_state', '/collision_guard/state', '/safety/state']
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in have]))
    from nav2_msgs.msg import CollisionMonitorState
    from std_msgs.msg import String
    D = {'mo': [], 'ob': [], 'odo': [], 'cmd': [], 'ask': [], 'plans': [], 'scan_min': [], 'mon': [], 'guard': [], 'gate': []}
    laser = (0.0, 0.0, 0.0)
    mo = ob = None
    while r.has_next():
        t, d, ts = r.read_next()
        ts *= 1e-9
        if t in ('/tf', '/tf_static'):
            for tr in deserialize_message(d, TFMessage).transforms:
                v = (tr.transform.translation.x, tr.transform.translation.y, q2y(tr.transform.rotation))
                if tr.child_frame_id == 'odom':
                    mo = v
                    D['mo'].append((ts,) + v)
                elif tr.child_frame_id == 'base_footprint':
                    ob = v
                    D['ob'].append((ts,) + v)
                elif 'laser' in tr.child_frame_id:
                    laser = v
        elif t == '/odometry/filtered':
            m = deserialize_message(d, Odometry)
            D['odo'].append((ts, m.twist.twist.linear.x, m.twist.twist.angular.z))
        elif t == '/cmd_vel_mux':
            m = deserialize_message(d, Twist)
            D['cmd'].append((ts, m.linear.x, m.angular.z))
        elif t == '/cmd_vel_nav_smoothed':          # what Nav2 asked for, BEFORE its collision monitor
            m = deserialize_message(d, Twist)
            D['ask'].append((ts, m.linear.x, m.angular.z))
        elif t == '/plan':
            m = deserialize_message(d, Path)
            D['plans'].append((ts, np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses[::2]])))
        elif t == '/scan' and mo and ob:
            m = deserialize_message(d, LaserScan)
            rr = np.array(m.ranges)
            a = m.angle_min + np.arange(len(rr)) * m.angle_increment
            ok = np.isfinite(rr) & (rr > m.range_min) & (rr < 3.0)
            px = laser[0] + rr[ok] * np.cos(a[ok] + laser[2])
            py = laser[1] + rr[ok] * np.sin(a[ok] + laser[2])
            ox, oy = np.maximum(np.abs(px) - HL, 0), np.maximum(np.abs(py) - HW, 0)
            inside = (np.abs(px) <= HL) & (np.abs(py) <= HW)          # her own parts: not obstacles
            gap = np.hypot(ox, oy)[~inside]
            pose = compose(mo, ob)
            D['scan_min'].append((ts, float(gap.min()) if len(gap) else 9.0, pose[0], pose[1]))
        elif t == '/collision_monitor_state':
            m = deserialize_message(d, CollisionMonitorState)
            D['mon'].append((ts, m.action_type, m.polygon_name))
        elif t == '/collision_guard/state':
            m = deserialize_message(d, CollisionMonitorState)
            D['guard'].append((ts, m.action_type, m.polygon_name))
        elif t == '/safety/state':
            D['gate'].append((ts, deserialize_message(d, String).data[:80]))
    return D


def state_at(events, t):
    """The last (action, name) at time t; action 0 = none."""
    last = (0, '')
    for ts, a, n in events:
        if ts > t:
            break
        last = (a, n)
    return last


def report(bag, out_dir):
    D = read(bag)
    R = {'bag': os.path.basename(bag.rstrip('/'))}
    # ---- route / park from the logs
    for name in ('route.log', 'park.log'):
        p = os.path.join(out_dir, name)
        if os.path.exists(p):
            txt = open(p, errors='replace').read()
            m = re.findall(r'^result (\w+).*$', txt, re.M)
            if name == 'route.log':
                R['route_result'] = m[-1] if m else None
                R['bumps'] = len(re.findall(r'BUMP \d', txt))
                R['person_took_wheel'] = 'took the wheel' in txt
            else:
                m2 = re.findall(r'(\d+(?:\.\d+)?) cm.*?(-?\d+(?:\.\d+)?) deg', txt)
                R['park_last'] = (txt.strip().splitlines() or [''])[-1][:120]
    odo = np.array(D['odo']) if D['odo'] else np.zeros((0, 3))
    cmd = np.array(D['cmd']) if D['cmd'] else np.zeros((0, 3))
    if len(odo) < 10:
        R['error'] = 'no odometry'
        return R
    moving = np.abs(odo[:, 1]) > 0.03
    t_move = odo[moving, 0]
    R['moving_s'] = round(float(np.sum(np.diff(odo[:, 0])[moving[1:]])), 1)
    R['span_s'] = round(float(t_move[-1] - t_move[0]), 1) if len(t_move) else 0.0
    # direction switches: sign changes of the commanded speed (|v| > 0.05)
    sv = np.sign(cmd[np.abs(cmd[:, 1]) > 0.05, 1]) if len(cmd) else np.zeros(0)
    R['switches'] = int(np.sum(np.diff(sv) != 0)) if len(sv) else 0
    # ---- stops: Nav2 asked to drive (before its monitor, recorded since 2026-10-07) and she did not
    # move, 0.5 s or more. Older bags (no request recorded): standing still 1 s or more mid-route.
    stops = []
    ask = np.array(D['ask']) if D['ask'] else None
    if ask is not None and len(ask):
        tt = ask[:, 0]
        held = (np.abs(ask[:, 1]) >= 0.08) & (np.interp(tt, odo[:, 0], np.abs(odo[:, 1])) < 0.03)
        min_s, known = 0.5, True
    else:
        tt = odo[:, 0]
        inside = (tt > t_move[0]) & (tt < t_move[-1]) if len(t_move) else np.zeros(len(tt), bool)
        held = inside & (np.abs(odo[:, 1]) < 0.03)
        min_s, known = 1.0, False
    i = 0
    while i < len(tt):
        if held[i]:
            j = i
            while j + 1 < len(tt) and held[j + 1] and tt[j + 1] - tt[j] < 0.5:
                j += 1
            dur = tt[j] - tt[i]
            if dur >= min_s:
                t = tt[i] + 0.2
                mon, guard = state_at(D['mon'], t), state_at(D['guard'], t)
                who = (f'nav2 monitor ({mon[1]})' if mon[0] else f'lidar guard ({guard[1]})' if guard[0]
                       else 'gate/other' if known else 'not recorded (older bag)')
                pose = None
                if D['scan_min']:
                    k = min(range(len(D['scan_min'])), key=lambda k: abs(D['scan_min'][k][0] - t))
                    pose = D['scan_min'][k][2:4]
                stops.append({'t': round(t - odo[0, 0], 1), 's': round(dur, 1), 'who': who,
                              'where': [round(pose[0], 2), round(pose[1], 2)] if pose else None})
            i = j + 1
        else:
            i += 1
    R['stops'] = stops
    R['stop_s'] = round(sum(s['s'] for s in stops), 1)
    # ---- weave
    mv = odo[np.abs(odo[:, 1]) > 0.12]
    sd = []
    if len(mv):
        kap = mv[:, 2] / mv[:, 1]
        i = 0
        while i < len(mv):
            j = np.searchsorted(mv[:, 0], mv[i, 0] + 2.0)
            if j - i > 10 and abs(kap[i:j].mean()) < 0.4:
                sd.append(float(kap[i:j].std()))
            i = max(j, i + 1)
    R['weave_med'] = round(float(np.median(sd)), 3) if sd else None
    R['weave_p90'] = round(float(np.percentile(sd, 90)), 3) if sd else None
    # ---- plans
    shift = []
    for (_, a), (_, b) in zip(D['plans'], D['plans'][1:]):
        if len(a) < 5 or len(b) < 5:
            continue
        sb = np.r_[0, np.cumsum(np.hypot(*np.diff(b, axis=0).T))]
        seg = b[(sb > 0.3) & (sb < 1.0)]
        if len(seg):
            shift.append(max(np.hypot(a[:, 0] - x, a[:, 1] - y).min() for x, y in seg))
    R['plans'] = len(D['plans'])
    R['plan_shift_med_cm'] = round(100 * float(np.median(shift)), 1) if shift else None
    # ---- localisation corrections
    mo = np.array(D['mo']) if D['mo'] else np.zeros((0, 4))
    if len(mo) > 2:
        dj = np.hypot(np.diff(mo[:, 1]), np.diff(mo[:, 2]))
        tj = mo[1:, 0]
        drv = np.interp(tj, odo[:, 0], np.abs(odo[:, 1])) > 0.1
        big = dj[(dj > 0.03) & drv]
        R['loc_jumps'] = int(len(big))
        R['loc_jump_med_cm'] = round(100 * float(np.median(big)), 1) if len(big) else 0.0
        R['loc_jump_max_cm'] = round(100 * float(big.max()), 1) if len(big) else 0.0
    # ---- clearance (only while moving: parked against the wall does not count)
    sm = np.array(D['scan_min']) if D['scan_min'] else np.zeros((0, 4))
    if len(sm):
        mvs = np.interp(sm[:, 0], odo[:, 0], np.abs(odo[:, 1])) > 0.03
        sm = sm[mvs]
    if len(sm):
        R['clear_min_cm'] = round(100 * float(sm[:, 1].min()), 1)
        R['turns_cm'] = {}
        for name, (tx, ty, rad) in TURNS.items():
            m = np.hypot(sm[:, 2] - tx, sm[:, 3] - ty) < rad
            R['turns_cm'][name] = round(100 * float(sm[m, 1].min()), 1) if m.any() else None
    return R


def text(R, B):
    def cmp(key, fmt='{}', lower_better=True):
        v = R.get(key)
        b = B.get(key) if B else None
        s = fmt.format(v) if v is not None else '-'
        if b is not None and v is not None and isinstance(v, (int, float)) and isinstance(b, (int, float)) and v != b:
            better = (v < b) == lower_better
            s += f'  (ground zero {fmt.format(b)}, {"better" if better else "worse"})'
        return s
    L = [f"LAP REPORT {R.get('bag')}",
         f"route {R.get('route_result')}  bumps {R.get('bumps')}  person took wheel {R.get('person_took_wheel')}",
         f"moving {cmp('moving_s', '{} s')}   first-to-last motion {cmp('span_s', '{} s')}",
         f"direction switches {cmp('switches')}",
         f"stops {len(R.get('stops', []))}, {cmp('stop_s', '{} s')} in all"]
    for s in R.get('stops', []):
        L.append(f"   at {s['t']:6.1f} s for {s['s']:4.1f} s at {s['where']}: {s['who']}")
    L += [f"weave median {cmp('weave_med')}  p90 {cmp('weave_p90')}",
          f"plans {R.get('plans')}  shift median {cmp('plan_shift_med_cm', '{} cm')}",
          f"localisation jumps >3 cm {cmp('loc_jumps')}  median {cmp('loc_jump_med_cm', '{} cm')}  max {cmp('loc_jump_max_cm', '{} cm')}",
          f"closest to anything while moving {cmp('clear_min_cm', '{} cm', lower_better=False)}"]
    for k, v in (R.get('turns_cm') or {}).items():
        b = ((B or {}).get('turns_cm') or {}).get(k)
        L.append(f"   {k:<12} {v if v is not None else '-':>6} cm" + (f"   (ground zero {b} cm)" if b is not None else ''))
    if R.get('park_last'):
        L.append('park: ' + R['park_last'])
    return '\n'.join(L)


def main():
    bag = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else bag
    R = report(bag, out_dir)
    B = json.load(open(BASELINE)) if os.path.exists(BASELINE) else None
    T = text(R, B)
    print(T)
    for d in {out_dir, bag}:
        if os.path.isdir(d):
            json.dump(R, open(os.path.join(d, 'report.json'), 'w'), indent=1)
            open(os.path.join(d, 'report.txt'), 'w').write(T + '\n')
    if '--baseline' in sys.argv:
        json.dump(R, open(BASELINE, 'w'), indent=1)
        print('saved as the ground-zero baseline:', BASELINE)


if __name__ == '__main__':
    main()
