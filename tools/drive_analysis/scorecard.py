"""The scorecard of a drive: the numbers that say whether a change made her smoother.

    python scorecard.py BAG_DIR [--audit MIRROR/audit] [--drive N]

Reads the bag's MCAP directly (mcap + mcap-ros2-support, no ROS) and the run's own logs
(route.log, park.log, drive.json - in the bag since drive.sh, 2026-10-01, or found in the
audit mirror by the bag's name for the drives before it). Writes BAG_DIR/analysis/scorecard.json
and prints one line. rosie-health.py puts every scorecard in a table, drive by drive, and
clusters the stops across drives into hotspots.

What is scored, and from where:
  lap_s, dist_m, v_mean, v_p95      /odometry/filtered: first motion to last motion, path length,
                                    speed while moving (> 0.03 m/s)
  stops                             standing still >= 1 s between the first and last motion: how
                                    many, how long in all, and each one's place on the map
                                    (map -> odom from /tf at that moment) and whether the collision
                                    guard was holding her then
  reversals_per_m                   /cmd_vel: steering sign changes bigger than 0.5 (the servo
                                    swung from one side to the other), per metre driven
  fr_switches                       /cmd_vel: forward <-> reverse changes
  lat_acc_p95                       |v * yaw rate| while moving, m/s^2 (corner harshness)
  guard_stops                       /collision_guard/state: how often the guard went to STOP
  clear_p1_m                        /scan: the 1st percentile of the nearest return (> 0.12 m,
                                    her own body) - how close she runs to things
  route                             route.log: SUCCEEDED/ABORTED, waypoints passed, seconds
  park_cm, park_deg, park_s         park.log: the reverse-in's "ended N cm ... off by D deg",
                                    and how long the parking took
"""

import argparse
import glob
import json
import math
import os
import re
import sys
import time

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

STILL_V = 0.03          # m/s: below this she is standing
STOP_MIN_S = 1.0
REVERSAL = 0.5          # steering units (angular.z is the servo position, +-2 full lock)
BODY_M = 0.12           # lidar returns closer than this are her own parts


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def read_bag(mcap_path):
    rows = {'ekf': [], 'cmd': [], 'mapodom': [], 'guard': [], 'scan': []}
    nav = web = 0
    dec, decoders = DecoderFactory(), {}
    with open(mcap_path, 'rb') as f:
        r = make_reader(f)
        topics = ['/odometry/filtered', '/cmd_vel', '/tf', '/collision_guard/state', '/scan', '/cmd_vel_nav', '/cmd_vel_web']
        for schema, channel, msg in r.iter_messages(topics=topics):
            if channel.topic == '/tf' and b'map\x00' not in msg.data:
                continue
            if channel.topic == '/cmd_vel_nav':
                nav += 1
                continue
            d = decoders.get(channel.id)
            if d is None:
                d = decoders[channel.id] = dec.decoder_for(channel.message_encoding, schema)
            m = d(msg.data)
            tp = channel.topic
            if tp == '/odometry/filtered':
                p = m.pose.pose
                rows['ekf'].append((stamp(m.header), p.position.x, p.position.y, yaw_of(p.orientation),
                                    m.twist.twist.linear.x, m.twist.twist.angular.z))
            elif tp == '/cmd_vel':
                rows['cmd'].append((msg.log_time * 1e-9, m.linear.x, m.angular.z))
            elif tp == '/cmd_vel_web':
                web += abs(m.linear.x) > 0.02            # the phone's page, actually driving
            elif tp == '/collision_guard/state':
                rows['guard'].append((msg.log_time * 1e-9, int(m.action_type)))
            elif tp == '/scan':
                rng = np.asarray(m.ranges, dtype=np.float32)
                rng = rng[(rng > BODY_M) & np.isfinite(rng)]
                rows['scan'].append((stamp(m.header), float(rng.min()) if rng.size else float('nan')))
            elif tp == '/tf':
                for tr in m.transforms:
                    if tr.header.frame_id.lstrip('/') == 'map' and tr.child_frame_id.lstrip('/') == 'odom':
                        t = tr.transform
                        rows['mapodom'].append((stamp(tr.header), t.translation.x, t.translation.y, yaw_of(t.rotation)))
    out = {k: np.array(v, dtype=np.float64) for k, v in rows.items()}
    out['nav_msgs'], out['web_msgs'] = nav, web
    return out


def on_map(mapodom, t, x, y):
    """An odom-frame point on the map, with the map -> odom in force at time t."""
    if not len(mapodom):
        return None
    i = int(np.searchsorted(mapodom[:, 0], t)) - 1
    _, mx, my, myaw = mapodom[max(i, 0)]
    c, s = math.cos(myaw), math.sin(myaw)
    return [round(mx + c * x - s * y, 2), round(my + s * x + c * y, 2)]


def motion_scores(ekf, cmd, mapodom, guard):
    out = {}
    if len(ekf) < 10:
        return {'lap_s': None, 'dist_m': None, 'note': 'no /odometry/filtered in the bag'}
    t, x, y, v, w = ekf[:, 0], ekf[:, 1], ekf[:, 2], ekf[:, 4], ekf[:, 5]
    moving = np.abs(v) > STILL_V
    if not moving.any():
        return {'lap_s': 0.0, 'dist_m': 0.0, 'note': 'she never moved'}
    first, last = int(np.argmax(moving)), len(moving) - 1 - int(np.argmax(moving[::-1]))
    out['t_first_motion'] = float(t[first])
    out['lap_s'] = round(float(t[last] - t[first]), 1)
    steps = np.hypot(np.diff(x[first:last + 1]), np.diff(y[first:last + 1]))
    out['dist_m'] = round(float(steps.sum()), 2)
    vm = np.abs(v[first:last + 1][moving[first:last + 1]])
    out['v_mean'] = round(float(vm.mean()), 3)
    out['v_p95'] = round(float(np.percentile(vm, 95)), 3)
    out['v_std'] = round(float(vm.std()), 3)
    lat = np.abs(v[first:last + 1] * w[first:last + 1])[moving[first:last + 1]]
    out['lat_acc_p95'] = round(float(np.percentile(lat, 95)), 3) if lat.size else None
    # stops: still for >= STOP_MIN_S between the first and last motion
    stops = []
    i = first
    while i <= last:
        if not moving[i]:
            j = i
            while j + 1 <= last and not moving[j + 1]:
                j += 1
            dur = float(t[j] - t[i])
            if dur >= STOP_MIN_S:
                held = bool(len(guard)) and int(guard[max(int(np.searchsorted(guard[:, 0], t[i])) - 1, 0), 1]) != 0
                stops.append({'t': round(float(t[i] - t[first]), 1), 's': round(dur, 1),
                              'map': on_map(mapodom, t[i], x[i], y[i]), 'guard': held})
            i = j + 1
        else:
            i += 1
    out['stops'] = stops
    out['stops_n'] = len(stops)
    out['stopped_s'] = round(sum(s['s'] for s in stops), 1)
    # the driver's commands
    if len(cmd):
        ct, lin, ang = cmd[:, 0], cmd[:, 1], cmd[:, 2]
        sel = (ct >= t[first]) & (ct <= t[last])
        lin, ang = lin[sel], ang[sel]
        drive = lin[np.abs(lin) > 0.05]
        out['fr_switches'] = int(np.sum(np.sign(drive[1:]) != np.sign(drive[:-1]))) if drive.size > 1 else 0
        rev = 0
        last_side = 0.0
        for a in ang:
            if abs(a) >= REVERSAL / 2:
                side = math.copysign(1.0, a)
                if last_side and side != last_side:
                    rev += 1
                last_side = side
        out['reversals'] = rev
        out['reversals_per_m'] = round(rev / out['dist_m'], 2) if out['dist_m'] > 1 else None
    else:
        out['fr_switches'] = out['reversals'] = out['reversals_per_m'] = None
    # the guard
    if len(guard):
        a = guard[:, 1]
        out['guard_stops'] = int(np.sum((a[1:] == 1) & (a[:-1] != 1)))
    else:
        out['guard_stops'] = None
    return out


def clearance(scan, t0, t1):
    if not len(scan):
        return None
    sel = (scan[:, 0] >= t0) & (scan[:, 0] <= t1) & np.isfinite(scan[:, 1])
    return round(float(np.percentile(scan[sel, 1], 1)), 2) if sel.sum() > 10 else None


def console_text(logs_dir, n):
    """The run's console: console.log in its folder (drive.sh) or driveN_console.log beside it."""
    if not logs_dir:
        return ''
    txt = read(os.path.join(logs_dir, 'console.log'))
    if not txt and n is not None:
        txt = read(os.path.join(os.path.dirname(logs_dir), f'drive{n}_console.log'))
    return txt


def parse_logs(logs_dir, n=None, day=None):
    """route.log / park.log / the console -> the result lines' numbers, and park_t: the epoch
    second the parking began (the console's '== park HH:MM:SS' on the bag's day, local time)."""
    out = {}
    route = read(os.path.join(logs_dir, 'route.log')) if logs_dir else ''
    res = [ln for ln in route.splitlines() if ln.startswith('result')]
    if res:
        ln = res[-1]
        m = re.search(r'result (\w+) after ([\d.]+) s', ln)
        out['route'] = m.group(1) if m else ln[:40]
        out['route_s'] = float(m.group(2)) if m else None
        m = re.search(r'passed (\d+) of (\d+) waypoints', ln)
        if m:
            out['waypoints'] = f'{m.group(1)}/{m.group(2)}'
        m = re.search(r'forward/reverse switches (\d+)', ln)
        if m:
            out['route_switches'] = int(m.group(1))
    park = read(os.path.join(logs_dir, 'park.log')) if logs_dir else ''
    if park:
        m = re.findall(r'ended (\d+) cm from the spot, heading off by ([+-]?\d+) deg', park)
        if m:
            out['park_cm'], out['park_deg'] = int(m[-1][0]), int(m[-1][1])
        m = re.findall(r'heading ([+-]?\d+) deg, ([+-]?\d+) to go', park)
        if m:
            out['park_deg_final'] = int(m[-1][1])
        out['parked'] = 'parked' in park
    console = console_text(logs_dir, n)
    tp = re.search(r'== park\s+(\d+):(\d+):(\d+)', console)
    if tp and day:
        y, mo, d = int(day[:4]), int(day[4:6]), int(day[6:8])
        out['park_t'] = time.mktime((y, mo, d, int(tp.group(1)), int(tp.group(2)), int(tp.group(3)), 0, 0, -1))
    return out


def read(path):
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            return f.read()
    except OSError:
        return ''


def find_logs(bag_dir, audit_dir):
    """Where the run's logs are: in the bag (drive.sh) or in the audit mirror, found by the
    'recording <bag>' line of a driveN_console.log -> (logs_dir, drive_number)."""
    j = read(os.path.join(bag_dir, 'drive.json'))
    if j:
        try:
            d = json.loads(j)
            return bag_dir, d.get('drive')
        except ValueError:
            pass
    name = os.path.basename(bag_dir.rstrip('/'))
    if audit_dir:
        for con in glob.glob(os.path.join(audit_dir, '*', 'drive*_console.log')):
            if name in read(con):
                m = re.search(r'drive(\d+)', os.path.basename(con))
                n = int(m.group(1)) if m else None
                d = os.path.join(os.path.dirname(con), f'drive{n}')
                return (d if os.path.isdir(d) else os.path.dirname(con)), n
    return None, None


def score(bag_dir, audit_dir=None, drive=None):
    mcap = sorted(glob.glob(os.path.join(bag_dir, '*.mcap')))
    if not mcap:
        raise SystemExit(f'no .mcap in {bag_dir}')
    t0 = time.time()
    rows = read_bag(mcap[0])
    logs_dir, n = find_logs(bag_dir, audit_dir)
    card = {'bag': os.path.basename(bag_dir.rstrip('/')), 'drive': drive if drive is not None else n, 'logs': logs_dir}
    m = re.search(r'drive-(\d{8})-(\d{6})', card['bag'])
    if m:
        card['when'] = f'{m.group(1)[:4]}-{m.group(1)[4:6]}-{m.group(1)[6:]}T{m.group(2)[:2]}:{m.group(2)[2:4]}:{m.group(2)[4:]}'
    logs = parse_logs(logs_dir, card['drive'], m.group(1) if m else None)
    # hand-driven: the phone drove and nothing says a route or a park ran (the bags since
    # 2026-09-30 do not carry /cmd_vel_nav, so Nav2's commands cannot be counted there)
    scripted = bool(logs.get('route') or logs.get('park_cm') is not None or logs.get('parked'))
    card['mode'] = 'auto' if scripted or rows['nav_msgs'] > 50 or rows['web_msgs'] < 50 else 'hand'
    ekf = rows['ekf']
    park_t = logs.pop('park_t', None)
    if park_t is not None and len(ekf) and ekf[0, 0] < park_t < ekf[-1, 0]:
        lap = ekf[ekf[:, 0] < park_t]             # the lap's numbers end where the parking begins
        after = ekf[ekf[:, 0] >= park_t]
        moving = np.abs(after[:, 4]) > STILL_V
        if moving.any():
            logs['park_s'] = round(float(after[np.where(moving)[0][-1], 0] - park_t), 0)
    else:
        lap = ekf
    card.update(motion_scores(lap, rows['cmd'], rows['mapodom'], rows['guard']))
    if card.get('t_first_motion') is not None:
        card['clear_p1_m'] = clearance(rows['scan'], card['t_first_motion'], card['t_first_motion'] + card['lap_s'])
    card.update(logs)
    card['scored_in_s'] = round(time.time() - t0, 1)
    os.makedirs(os.path.join(bag_dir, 'analysis'), exist_ok=True)
    with open(os.path.join(bag_dir, 'analysis', 'scorecard.json'), 'w') as f:
        json.dump(card, f, indent=1)
    return card


def one_line(c):
    d = f'drive {c["drive"]}' if c.get('drive') is not None else c['bag']
    if c.get('lap_s') is None or c.get('v_mean') is None:
        return f'{d}: {c.get("note", "nothing to score")}'
    if c.get('mode') == 'hand':
        d += ' (hand-driven)'
    bits = [f'{c["lap_s"]:.0f} s, {c["dist_m"]:.1f} m at {c["v_mean"]:.2f} m/s (p95 {c["v_p95"]:.2f})',
            f'{c["stops_n"]} stops ({c["stopped_s"]:.0f} s)']
    if c.get('reversals_per_m') is not None:
        bits.append(f'{c["reversals_per_m"]:.2f} reversals/m')
    if c.get('lat_acc_p95') is not None:
        bits.append(f'lat {c["lat_acc_p95"]:.2f} m/s2')
    if c.get('guard_stops') is not None:
        bits.append(f'guard {c["guard_stops"]}')
    if c.get('clear_p1_m') is not None:
        bits.append(f'clearance {c["clear_p1_m"]:.2f} m')
    if c.get('route'):
        bits.append(f'route {c["route"]}' + (f' {c["waypoints"]}' if c.get('waypoints') else ''))
    if c.get('park_cm') is not None:
        bits.append(f'park {c["park_cm"]} cm / {c.get("park_deg_final", c["park_deg"])} deg' + (f' in {c["park_s"]} s' if c.get('park_s') else ''))
    return f'{d}: ' + '; '.join(bits)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--audit', default=None, help='the audit mirror, to find the logs of drives before drive.sh')
    ap.add_argument('--drive', type=int, default=None)
    a = ap.parse_args()
    c = score(a.bag.rstrip('/'), a.audit, a.drive)
    print(one_line(c))


if __name__ == '__main__':
    sys.exit(main())
