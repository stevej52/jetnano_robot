"""The global costmap at a moment of a drive, two ways: the last FULL /global_costmap/costmap
before then (all a late subscriber such as nav_goal gets - it is latched, and Nav2 sends only
updates while the size stays the same) and that grid with every /global_costmap/costmap_updates
patch applied up to then (what the planner really had). Runs nav_goal's clear_goal on both.

    python costmap_at.py BAG.mcap 2026-09-30T04:55:45Z OUTDIR "x y" ["x y" ...]
"""
import os
import sys

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from goal_check_replay import load_clear_goal
from maps_at_times import when


class Grid:
    def __init__(self, info, data):
        self.info, self.data = info, data


def at(bag, t_want):
    full, live, t_full, n_upd = None, None, None, 0
    dec, decoders = DecoderFactory(), {}
    with open(bag, 'rb') as f:
        for schema, ch, msg in make_reader(f).iter_messages(
                topics=['/global_costmap/costmap', '/global_costmap/costmap_updates'], log_time_order=True):
            if msg.log_time * 1e-9 > t_want:
                break
            d = decoders.get(ch.id) or decoders.setdefault(ch.id, dec.decoder_for('cdr', schema))
            m = d(msg.data)
            if ch.topic == '/global_costmap/costmap':
                full, t_full = m, msg.log_time * 1e-9
                live = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
                n_upd = 0
            elif live is not None:
                live[m.y:m.y + m.height, m.x:m.x + m.width] = np.array(m.data, np.int16).reshape(m.height, m.width)
                n_upd += 1
    stale = np.array(full.data, np.int16).reshape(full.info.height, full.info.width)
    return full.info, stale, live, t_want - t_full, n_upd


def render(a, info, path, goals, box):
    """box = (x0, x1, y0, y1) in metres; 0 white, 1-98 grey, 99 blue, 100 black, -1 light."""
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    u0, u1 = int((box[0] - ox) / res), int((box[1] - ox) / res)
    v0, v1 = int((box[2] - oy) / res), int((box[3] - oy) / res)
    sub = a[v0:v1, u0:u1]
    img = np.full(sub.shape + (3,), 255, np.uint8)
    img[sub < 0] = (200, 200, 200)
    band = (sub > 0) & (sub < 99)
    g = (230 - sub[band] * 1.5).clip(80, 230).astype(np.uint8)
    img[band] = np.stack([g, g, g], -1)
    img[sub == 99] = (230, 120, 60)
    img[sub == 100] = (0, 0, 0)
    img = cv2.resize(img[::-1], None, fx=10, fy=10, interpolation=cv2.INTER_NEAREST)
    for name, x, y in goals:
        p = (int((x - box[0]) / res * 10), int((box[3] - y) / res * 10))
        cv2.circle(img, p, 7, (0, 160, 0), 2)
        cv2.circle(img, p, int(0.32 / res * 10), (0, 160, 0), 1)       # nav_goal's REACH_M
        cv2.putText(img, name, (p[0] + 9, p[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 120, 0), 1)
    cv2.imwrite(path, img)


def main(bag, t, out, goals):
    os.makedirs(out, exist_ok=True)
    info, stale, live, age, n = at(bag, when(t))
    print(f'last full costmap {age:.0f} s old, {n} updates since; {info.width}x{info.height}')
    clear_goal = load_clear_goal()
    named = []
    for i, s in enumerate(goals):
        x, y = (float(v) for v in s.split())
        named.append((f'G{i + 1}', x, y))
        for label, a in (('latched (nav_goal)', stale), ('live (planner)', live)):
            f = clear_goal(Grid(info, a.reshape(-1).tolist()), x, y)
            print(f'  ({x:+.2f}, {y:+.2f}) {label}:', 'REFUSED' if f is None else
                  f'ok at ({f[0]:+.2f}, {f[1]:+.2f}), moved {f[2] * 100:.0f} cm')
    xs, ys = [g[1] for g in named], [g[2] for g in named]
    box = (min(xs) - 1.2, max(xs) + 1.2, min(ys) - 1.2, max(ys) + 1.2)
    render(stale, info, os.path.join(out, 'latched.png'), named, box)
    render(live, info, os.path.join(out, 'live.png'), named, box)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:])
