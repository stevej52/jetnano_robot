"""When did the camera grid's cells in a region first appear, and where was she then? For
telling far-range depth noise (marked from 3-5 m away) from things seen up close. Region is
a polygon in map metres; cells are /nvblox_node/map_grid's (already on /map's cells).

    python camera_cells_origin.py BAG.mcap UNTIL_UTC "x,y x,y x,y ..."
"""
import math
import sys

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from layers_at import compose, yaw_of
from maps_at_times import when


def inside(poly, x, y):
    n, ok = len(poly), np.zeros(np.shape(x), bool)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        cross = ((y1 > y) != (y2 > y)) & (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1)
        ok ^= cross
    return ok


def main(bag, until, poly_s):
    poly = [tuple(float(v) for v in p.split(',')) for p in poly_s.split()]
    t_end = when(until)
    dec, ds = DecoderFactory(), {}
    mo, ob = None, None
    first = {}                                   # cell -> (t, robot pose, distance)
    region = None
    t0 = None
    with open(bag, 'rb') as f:
        for schema, ch, msg in make_reader(f).iter_messages(topics=['/tf', '/nvblox_node/map_grid'],
                                                             log_time_order=True):
            t = msg.log_time * 1e-9
            t0 = t0 or t
            if t > t_end:
                break
            d = ds.get(ch.id) or ds.setdefault(ch.id, dec.decoder_for('cdr', schema))
            m = d(msg.data)
            if ch.topic == '/tf':
                for tr in m.transforms:
                    p = (tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation))
                    if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                        mo = p
                    elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_footprint':
                        ob = p
                continue
            i = m.info
            if region is None or region[0] != (i.width, i.height, i.origin.position.x, i.origin.position.y):
                u, v = np.meshgrid(np.arange(i.width), np.arange(i.height))
                x = i.origin.position.x + (u + 0.5) * i.resolution
                y = i.origin.position.y + (v + 0.5) * i.resolution
                region = ((i.width, i.height, i.origin.position.x, i.origin.position.y), inside(poly, x, y), x, y)
                first.clear()
            _, mask, xs, ys = region
            occ = (np.array(m.data, np.int16).reshape(i.height, i.width) >= 65) & mask
            if mo is None or ob is None:
                continue
            pose = compose(mo, ob)
            for vv, uu in zip(*np.nonzero(occ)):
                if (vv, uu) not in first:
                    first[(vv, uu)] = (t - t0, pose, math.hypot(xs[vv, uu] - pose[0], ys[vv, uu] - pose[1]),
                                       xs[vv, uu], ys[vv, uu])
            last_occ = occ
    still = {k: v for k, v in first.items() if last_occ[k]}
    print(f'{len(first)} cells were marked in the region; {len(still)} still marked at {until}')
    dist = np.array([v[2] for v in still.values()])
    if len(dist):
        for lo, hi in ((0, 1), (1, 2), (2, 3), (3, 4), (4, 9)):
            print(f'  first marked from {lo}-{hi} m away: {((dist >= lo) & (dist < hi)).sum()}')
        by_t = sorted(still.values(), key=lambda v: v[0])
        print('  earliest / latest first-marks (s into the bag, her pose, range, cell):')
        for v in by_t[:4] + by_t[-4:]:
            print(f'    {v[0]:6.1f} s  at ({v[1][0]:+.2f}, {v[1][1]:+.2f}) facing {math.degrees(v[1][2]):+4.0f}  '
                  f'{v[2]:.2f} m  cell ({v[3]:+.2f}, {v[4]:+.2f})')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])
