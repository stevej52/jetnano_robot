"""Replay lidar_odom_relay's outlier gate (LoGate) on a drive's MOLA poses and say, for every
dropped sample, which test failed (sideways / speed change / an episode's recovery samples) and
how far MOLA's speed was from the camera's odometry (/vo) at that moment - was the gate right?

    python lidar_gate_replay.py BAG.mcap [FROM_S TO_S]    (seconds into the bag)
"""
import ast
import math
import os
import sys

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

RELAY = os.path.join(os.path.dirname(__file__), '..', '..', 'jetnano_bringup', 'jetnano_bringup',
                     'lidar_odom_relay.py')


def load_gate():
    tree = ast.parse(open(RELAY, encoding='utf-8').read())
    keep = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LoGate']
    ns = {'math': math}
    exec(compile(ast.Module(body=keep, type_ignores=[]), RELAY, 'exec'), ns)
    return ns['LoGate']


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def body_velocities(samples):
    """[(t, x, y, yaw)] -> [(t_mid, vx, vy, wz, dt)] in the robot's own frame."""
    out = []
    for (t0, x0, y0, h0), (t1, x1, y1, h1) in zip(samples[:-1], samples[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        dx, dy = x1 - x0, y1 - y0
        c, s = math.cos(h0), math.sin(h0)
        out.append((0.5 * (t0 + t1), (c * dx + s * dy) / dt, (-s * dx + c * dy) / dt,
                    math.atan2(math.sin(h1 - h0), math.cos(h1 - h0)) / dt, dt))
    return out


def main(bag, a=None, b=None):
    dec, ds = DecoderFactory(), {}
    mola, vo = [], []
    t0 = None
    with open(bag, 'rb') as f:
        for schema, ch, msg in make_reader(f).iter_messages(topics=['/lidar_odometry/pose', '/vo'],
                                                             log_time_order=True):
            tt = msg.log_time * 1e-9
            t0 = t0 or tt
            d = ds.get(ch.id) or ds.setdefault(ch.id, dec.decoder_for('cdr', schema))
            m = d(msg.data)
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p = m.pose.pose if hasattr(m.pose, 'pose') else m.pose
            if ch.topic == '/vo':
                vo.append((tt - t0, m.twist.twist.linear.x, m.twist.twist.linear.y))
            else:
                mola.append((st, tt - t0, p.position.x, p.position.y, yaw(p.orientation)))
    vo = np.array(vo)
    stamps = [(s, x, y, h) for s, _, x, y, h in mola]
    rel = {s: r for s, r, *_ in mola}
    LoGate = load_gate()
    gate = LoGate()
    rows = []
    prev_v = None
    for tm, vx, vy, wz, dt in body_velocities(stamps):
        trel = rel.get(min(rel, key=lambda s: abs(s - tm))) if False else None
        rows.append((tm, vx, vy, wz, dt))
    # bag-relative times for the MOLA stamps
    s0 = stamps[0][0] - mola[0][1]
    dropped = {'sideways': 0, 'speed change': 0, 'recovering': 0}
    agree_bad, n_drop = [], 0
    for tm, vx, vy, wz, dt in rows:
        t = tm - s0
        accel = 0.0 if prev_v is None else math.hypot(vx - prev_v[0], vy - prev_v[1]) / max(dt, 1e-3)
        side = abs(vy) > gate.side_base + gate.side_per_forward * abs(vx)
        ok, _ = gate.check(tm, vx, vy, dt)
        prev_v = (vx, vy)
        if a is not None and not (a <= t <= b):
            continue
        if not ok:
            n_drop += 1
            why = 'sideways' if side else 'speed change' if accel > gate.max_accel else 'recovering'
            dropped[why] += 1
            near = vo[np.argmin(np.abs(vo[:, 0] - t))]
            agree_bad.append((t, why, vx, vy, near[1], near[2], accel))
    print(f'{len(rows)} MOLA samples; in the window {n_drop} dropped: {dropped}')
    for t, why, vx, vy, vvx, vvy, acc in agree_bad[:60]:
        print(f'  {t:6.1f} s  {why:12s} MOLA vx {vx:+.2f} vy {vy:+.2f} | camera vx {vvx:+.2f} vy {vvy:+.2f}'
              f'  | speed change {acc:.1f} m/s2')


if __name__ == '__main__':
    main(sys.argv[1], *(float(v) for v in sys.argv[2:4]))
