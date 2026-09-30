"""One leg of a drive, second by second: where she was (map), her heading, what Nav2 asked
(cmd_vel_nav_mps: forward/back, turn), where each forward/reverse switch happened, and the
planned path's end - to see why a leg shuffled or ended pointing the wrong way.

    python leg_replay.py BAG.mcap FROM_S TO_S      (seconds into the bag)
"""
import math
import sys

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def main(bag, a, b):
    dec, ds = DecoderFactory(), {}
    mo = ob = None
    t0 = None
    poses, cmds, plans = [], [], []
    with open(bag, 'rb') as f:
        for schema, ch, msg in make_reader(f).iter_messages(
                topics=['/tf', '/cmd_vel_nav_mps', '/plan'], log_time_order=True):
            tt = msg.log_time * 1e-9
            t0 = t0 or tt
            t = tt - t0
            if t > b + 1:
                break
            d = ds.get(ch.id) or ds.setdefault(ch.id, dec.decoder_for('cdr', schema))
            m = d(msg.data)
            if ch.topic == '/tf':
                for tr in m.transforms:
                    p = (tr.transform.translation.x, tr.transform.translation.y, yaw(tr.transform.rotation))
                    if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                        mo = p
                    elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_footprint':
                        ob = p
                        if mo and a <= t <= b:
                            poses.append((t,) + compose(mo, ob))
            elif t >= a - 1 and t <= b:
                if ch.topic == '/cmd_vel_nav_mps':
                    cmds.append((t, m.linear.x, m.angular.z))
                else:
                    pts = [(q.pose.position.x, q.pose.position.y, yaw(q.pose.orientation)) for q in m.poses]
                    if pts:
                        plans.append((t, pts))
    P, C = np.array(poses), np.array(cmds)
    print(f'{len(P)} poses, {len(C)} commands, {len(plans)} plans')
    for t in np.arange(a, b, 1.0):
        i = np.argmin(np.abs(P[:, 0] - t))
        cm = C[(C[:, 0] >= t) & (C[:, 0] < t + 1.0)]
        v = cm[:, 1].mean() if len(cm) else float('nan')
        w = cm[:, 2].mean() if len(cm) else float('nan')
        print(f'  {t:6.1f} s  at ({P[i, 1]:+.2f}, {P[i, 2]:+.2f}) heading {math.degrees(P[i, 3]):+5.0f}  '
              f'asked v {v:+.2f} m/s  w {math.degrees(w):+5.0f} deg/s')
    sign = np.sign(np.where(np.abs(C[:, 1]) < 0.02, 0, C[:, 1]))
    nz = [(C[k, 0], sign[k]) for k in range(len(C)) if sign[k] != 0]
    switches = [nz[k][0] for k in range(1, len(nz)) if nz[k][1] != nz[k - 1][1]]
    for s in switches:
        i = np.argmin(np.abs(P[:, 0] - s))
        print(f'  SWITCH at {s:6.1f} s, ({P[i, 1]:+.2f}, {P[i, 2]:+.2f}) heading {math.degrees(P[i, 3]):+.0f}')
    for t, pts in plans:
        e = pts[-1]
        tail = pts[-8:]
        print(f'  plan at {t:6.1f} s: {len(pts)} poses, ends ({e[0]:+.2f}, {e[1]:+.2f}) heading '
              f'{math.degrees(e[2]):+.0f}; last 8 headings ' + ' '.join(f'{math.degrees(p[2]):+.0f}' for p in tail))


if __name__ == '__main__':
    main(sys.argv[1], float(sys.argv[2]), float(sys.argv[3]))
