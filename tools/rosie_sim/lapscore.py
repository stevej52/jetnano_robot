#!/usr/bin/env python3
"""Score lap.py recordings: steering weave, track vs plan, body clearance per turn.

    python3 lapscore.py out/lap-A-0.jsonl [...]

weave   = on near-straight 2 s windows (|mean curvature| < 0.4 1/m, moving), the spread of the
          driven curvature (yaw rate / speed) in 1/m; at her 0.40 m turning circle 2.5 = full lock
xte     = distance from her centre to the newest plan, cm
shift   = how far each new plan's next metre (0.3-1.0 m) sits from the previous plan, cm
clear   = nearest map obstacle to her body outline (0.444 x 0.296), cm, overall and per turn
"""
import json, math, sys
import numpy as np
import yaml
from PIL import Image

MAP = 'map/home.yaml'
TURNS = {  # Steve's numbering (memory lap-turn-names): name -> (x, y, radius m)
    't1 chair': (2.9, -3.2, 0.9), 't2 island N': (1.0, -4.6, 0.9), 't3 island S': (0.2, -6.6, 1.0),
    't4 vacuum': (6.8, -7.3, 0.9), 't5 dining N': (6.0, -5.6, 0.9), 't6 chair': (2.6, -2.6, 0.9)}
HL, HW = 0.222, 0.148


def load_map():
    m = yaml.safe_load(open(MAP))
    img = np.array(Image.open(MAP.rsplit('/', 1)[0] + '/' + m['image']))
    occ = img < 100 if not m.get('negate') else img > 155
    occ = occ[::-1]                     # row 0 = lowest y
    ys, xs = np.nonzero(occ)
    pts = np.c_[m['origin'][0] + (xs + 0.5) * m['resolution'], m['origin'][1] + (ys + 0.5) * m['resolution']]
    return pts


def body_clear(pts, x, y, th):
    near = pts[(np.abs(pts[:, 0] - x) < 0.8) & (np.abs(pts[:, 1] - y) < 0.8)]
    if not len(near):
        return 0.8
    c, s = math.cos(th), math.sin(th)
    dx, dy = near[:, 0] - x, near[:, 1] - y
    lx, ly = c * dx + s * dy, -s * dx + c * dy             # obstacle in her frame
    ox, oy = np.maximum(np.abs(lx) - HL, 0), np.maximum(np.abs(ly) - HW, 0)
    return float(np.hypot(ox, oy).min())


def score(path, pts):
    recs = [json.loads(l) for l in open(path)]
    o = np.array([(r['t'], r['x'], r['y'], r['th'], r['v'], r['wz']) for r in recs if r['k'] == 'o'])
    plans = [(r['t'], np.array(r['xy'])) for r in recs if r['k'] == 'p' and r['xy']]
    o = o[np.argsort(o[:, 0])]
    mv = o[np.abs(o[:, 4]) > 0.12]
    kap = mv[:, 5] / mv[:, 4]
    # 2 s windows
    w, sd = [], []
    i = 0
    while i < len(mv):
        j = np.searchsorted(mv[:, 0], mv[i, 0] + 2.0)
        if j - i > 10:
            k = kap[i:j]
            if abs(k.mean()) < 0.4:
                sd.append(k.std())
        i = max(j, i + 1)
    # cross-track vs newest plan
    xte = []
    pi = 0
    for t, x, y, *_ in o[::5]:
        while pi + 1 < len(plans) and plans[pi + 1][0] <= t:
            pi += 1
        if plans and plans[pi][0] <= t:
            P = plans[pi][1]
            xte.append(np.hypot(P[:, 0] - x, P[:, 1] - y).min())
    cl = np.array([body_clear(pts, x, y, th) for _, x, y, th, *_ in o[::4]])
    oo = o[::4]
    per = {}
    for name, (tx, ty, r) in TURNS.items():
        m = np.hypot(oo[:, 1] - tx, oo[:, 2] - ty) < r
        per[name] = round(100 * cl[m].min(), 1) if m.any() else None
    sd = np.array(sd)
    # collision-monitor stops (action 1), counted and timed
    mon = sorted((r['t'], r['a']) for r in recs if r['k'] == 'm')
    n_stop, s_stop, since = 0, 0.0, None
    for t, a in mon:
        if a == 1 and since is None:
            since, n_stop = t, n_stop + 1
        elif a != 1 and since is not None:
            s_stop, since = s_stop + t - since, None
    # how far each new plan's next metre sits from the plan before it
    shift = []
    for (_, a), (_, b) in zip(plans, plans[1:]):
        if len(a) < 5 or len(b) < 5:
            continue
        sb = np.r_[0, np.cumsum(np.hypot(*np.diff(b, axis=0).T))]
        seg = b[(sb > 0.3) & (sb < 1.0)]
        if len(seg):
            shift.append(max(np.hypot(a[:, 0] - x, a[:, 1] - y).min() for x, y in seg))
    return {'weave_med': round(float(np.median(sd)), 3), 'weave_p90': round(float(np.percentile(sd, 90)), 3),
            'xte_med_cm': round(100 * float(np.median(xte)), 1), 'xte_p95_cm': round(100 * float(np.percentile(xte, 95)), 1),
            'clear_min_cm': round(100 * float(cl.min()), 1), 'plans': len(plans),
            'shift_med_cm': round(100 * float(np.median(shift)), 1) if shift else None, 'mon_stops': n_stop, 'mon_stop_s': round(s_stop, 1), 'turns': per}


if __name__ == '__main__':
    pts = load_map()
    for f in sys.argv[1:]:
        print(f.split('/')[-1], json.dumps(score(f, pts)))
