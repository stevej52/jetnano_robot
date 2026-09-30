"""Are the camera grid's extra obstacles depth noise behind thin things? For every camera cell
in a region that the lidar never hit (nothing within 10 cm), look along the line from where
the camera was when the cell first appeared: does that line pass a lidar-seen object (a leg)
nearer the camera? Depth cameras put "flying pixels" behind edges, along the line of sight.

    python camera_streaks.py BAG.mcap UNTIL_UTC "x0 x1 y0 y1"
"""
import math
import sys

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from layers_at import LIDAR_X, LIDAR_YAW, compose, yaw_of
from maps_at_times import when

CAMERA_X = 0.185                                  # base_link -> camera_link (URDF camera_xyz)


def main(bag, until, box_s):
    x0, x1, y0, y1 = (float(v) for v in box_s.split())
    t_end = when(until)
    dec, ds = DecoderFactory(), {}
    mo = ob = None
    first = {}                                   # (x, y) cell centre -> camera (x, y) when first marked
    lidar = set()
    info = None
    with open(bag, 'rb') as f:
        for schema, ch, msg in make_reader(f).iter_messages(
                topics=['/tf', '/nvblox_node/map_grid', '/scan'], log_time_order=True):
            if msg.log_time * 1e-9 > t_end:
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
            if mo is None or ob is None:
                continue
            base = compose(mo, ob)
            if ch.topic == '/scan':
                if info is None:
                    continue
                lx, ly, la = compose(base, (LIDAR_X, 0.0, LIDAR_YAW))
                r = np.asarray(m.ranges)
                a = m.angle_min + np.arange(len(r)) * m.angle_increment + la
                ok = np.isfinite(r) & (r > m.range_min) & (r < 8.0)
                for px, py in zip(lx + r[ok] * np.cos(a[ok]), ly + r[ok] * np.sin(a[ok])):
                    if x0 - 0.5 < px < x1 + 0.5 and y0 - 0.5 < py < y1 + 0.5:
                        lidar.add((round(px / info[0]), round(py / info[0])))
                continue
            info = (m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, m.info.width)
            res, ox, oy, w = info
            g = np.array(m.data, np.int16).reshape(m.info.height, m.info.width) >= 65
            cam = compose(base, (CAMERA_X, 0.0, 0.0))
            for v, u in zip(*np.nonzero(g)):
                cx, cy = ox + (u + 0.5) * res, oy + (v + 0.5) * res
                if x0 <= cx <= x1 and y0 <= cy <= y1 and (u, v) not in first:
                    first[(u, v)] = (cx, cy, cam[0], cam[1])
            last = g
    res = info[0]
    lidar_xy = np.array(sorted(lidar), float) * res
    still = [c for (u, v), c in first.items() if last[v, u]]
    near, behind, open_, far = 0, 0, 0, []
    for cx, cy, sx, sy in still:
        dist = np.hypot(lidar_xy[:, 0] - cx, lidar_xy[:, 1] - cy)
        if dist.min() <= 0.10:
            near += 1                                        # a real thing the lidar saw too
            continue
        # lidar points between the camera and the cell, within 6 cm of the line of sight
        vx, vy = cx - sx, cy - sy
        L = math.hypot(vx, vy)
        t = ((lidar_xy[:, 0] - sx) * vx + (lidar_xy[:, 1] - sy) * vy) / (L * L)
        off = np.abs((lidar_xy[:, 0] - sx) * vy - (lidar_xy[:, 1] - sy) * vx) / L
        gap = (1 - t) * L                                    # how far behind that point the cell is
        if np.any((t > 0.05) & (gap > 0.12) & (off < 0.06)):
            behind += 1
        else:
            open_ += 1
        far.append(L)
    print(f'{len(still)} camera cells in the box at {until}:')
    print(f'  {near:4d} within 10 cm of something the lidar hit (real, at the lidar height too)')
    print(f'  {behind:4d} lidar never hit, and BEHIND a lidar-seen object along the camera\'s line of sight')
    print(f'  {open_:4d} lidar never hit, nothing the lidar saw in front of them')
    if far:
        print(f'  range when first marked (not-lidar cells): median {np.median(far):.2f} m, '
              f'p90 {np.percentile(far, 90):.2f} m')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])
