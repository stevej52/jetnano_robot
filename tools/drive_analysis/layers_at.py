"""Which layer closed a passage? The planner's costmap at a moment of a drive, taken apart:
SLAM's /map, the camera grid (/nvblox_node/map_grid), the global costmap as the planner had it
(costmap_at.py), and the lidar's own hits from a stretch of the drive projected on the map.
For each combination of layers, inflated as Nav2 does, says whether goal B can be reached
from goal A, and renders the box round them.

    python layers_at.py BAG.mcap 2026-09-30T04:55:45Z OUTDIR "ax ay" "bx by" [SCAN_FROM SCAN_TO]
"""
import math
import os
import sys

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from costmap_at import at as costmap_at
from maps_at_times import when

INSCRIBED = 0.148 + 0.03                          # padded half-width: the centre may not come closer
LIDAR_X, LIDAR_YAW = -0.005, math.pi              # base_link -> lidar_link (tf_static)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def read(bag, t_want, scan_from, scan_to):
    """-> /map msg, camera grid msg (latest before t_want), [(t, map->lidar pose, scan msg)]."""
    latest = {}
    mo, ob = [], []
    scans = []
    dec, ds = DecoderFactory(), {}
    topics = ['/map', '/nvblox_node/map_grid', '/tf', '/scan']
    with open(bag, 'rb') as f:
        for schema, ch, msg in make_reader(f).iter_messages(topics=topics, log_time_order=True):
            t = msg.log_time * 1e-9
            if t > max(t_want, scan_to):
                break
            if ch.topic in ('/map', '/nvblox_node/map_grid') and t > t_want:
                continue
            if ch.topic == '/scan' and not (scan_from <= t <= scan_to):
                continue
            d = ds.get(ch.id) or ds.setdefault(ch.id, dec.decoder_for('cdr', schema))
            m = d(msg.data)
            if ch.topic == '/tf':
                for tr in m.transforms:
                    p = (tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation))
                    if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                        mo.append((t, p))
                    elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_footprint':
                        ob.append((t, p))
            elif ch.topic == '/scan':
                if mo and ob:
                    base = compose(mo[-1][1], ob[-1][1])
                    scans.append((t, compose(base, (LIDAR_X, 0.0, LIDAR_YAW)), m))
            else:
                latest[ch.topic] = m
    return latest['/map'], latest.get('/nvblox_node/map_grid'), scans


def hits(scans, info, shape):
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    count = np.zeros(shape, np.int32)
    for _, (lx, ly, lyaw), s in scans:
        r = np.asarray(s.ranges, np.float64)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment + lyaw
        ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, 8.0))
        x, y = lx + r[ok] * np.cos(a[ok]), ly + r[ok] * np.sin(a[ok])
        u, v = ((x - ox) / res).astype(int), ((y - oy) / res).astype(int)
        inside = (u >= 0) & (u < shape[1]) & (v >= 0) & (v < shape[0])
        np.add.at(count, (v[inside], u[inside]), 1)
    return count


def reachable(lethal, res, a_cell, b_cell):
    """Is b reachable from a with her centre kept INSCRIBED from every lethal cell?"""
    d = cv2.distanceTransform((~lethal).astype(np.uint8), cv2.DIST_L2, 5) * res
    free = (d > INSCRIBED).astype(np.uint8)
    _, lab = cv2.connectedComponents(free, connectivity=8)
    la, lb = lab[a_cell], lab[b_cell]
    return la != 0 and la == lb, d


def main(bag, t, out, a, b, scan_from=None, scan_to=None):
    os.makedirs(out, exist_ok=True)
    tw = when(t)
    sf = when(scan_from) if scan_from else tw - 60
    st = when(scan_to) if scan_to else tw + 60
    info, _, live, _, _ = costmap_at(bag, tw)
    m, cam, scans = read(bag, tw, sf, st)
    assert (m.info.width, m.info.height) == (info.width, info.height), 'map and costmap differ in size'
    shape = (info.height, info.width)
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    smap = np.array(m.data, np.int16).reshape(shape) >= 65
    scam = (np.array(cam.data, np.int16).reshape(shape) >= 65) if cam is not None else np.zeros(shape, bool)
    costly = live == 100
    lidar_marks = costly & ~smap & ~scam            # what only the global obstacle layer has
    n = hits(scans, info, shape)
    seen = n >= 2
    cell = lambda xy: (int((xy[1] - oy) / res), int((xy[0] - ox) / res))
    ac, bc = cell(a), cell(b)
    print(f'{len(scans)} scans; cells lethal: /map {smap.sum()}, camera-only {(scam & ~smap).sum()}, '
          f'obstacle-layer-only {lidar_marks.sum()}')
    for name, lethal in (('planner (all layers, live)', costly),
                         ('/map + camera (the static layer alone)', smap | scam),
                         ('/map alone', smap),
                         ('/map + this stretch of lidar hits', smap | seen),
                         ('camera alone', scam),
                         ('this stretch of lidar hits alone', seen)):
        ok, _ = reachable(lethal, res, ac, bc)
        print(f'  {name:42s}: {"B reachable from A" if ok else "CLOSED"}')

    # the picture: box round A and B
    xs, ys = (a[0], b[0]), (a[1], b[1])
    x0, x1, y0, y1 = min(xs) - 1.3, max(xs) + 1.3, min(ys) - 1.3, max(ys) + 1.3
    u0, u1 = max(0, int((x0 - ox) / res)), min(shape[1], int((x1 - ox) / res))
    v0, v1 = max(0, int((y0 - oy) / res)), min(shape[0], int((y1 - oy) / res))
    x0, x1, y0, y1 = ox + u0 * res, ox + u1 * res, oy + v0 * res, oy + v1 * res
    _, d_all = reachable(costly, res, ac, bc)
    img = np.full((v1 - v0, u1 - u0, 3), 255, np.uint8)
    sub = lambda z: z[v0:v1, u0:u1]
    img[sub(d_all) <= INSCRIBED] = (225, 215, 255)                    # pink: her centre may not go
    img[sub(lidar_marks)] = (255, 120, 40)                            # blue: obstacle layer only
    img[sub(scam & ~smap)] = (40, 40, 230)                            # red: camera only
    img[sub(smap)] = (0, 0, 0)                                        # black: SLAM's /map
    img = cv2.resize(img[::-1], None, fx=10, fy=10, interpolation=cv2.INTER_NEAREST)
    for v, u in zip(*np.nonzero(sub(seen))):
        cv2.circle(img, (int(u * 10 + 5), int((v1 - v0 - 1 - v) * 10 + 5)), 3, (0, 170, 0), -1)
    for name, p in (('A', a), ('B', b)):
        q = (int((p[0] - x0) / res * 10), int((y1 - p[1]) / res * 10))
        cv2.circle(img, q, 8, (0, 120, 255), 3)
        cv2.putText(img, name, (q[0] + 10, q[1] + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 120, 255), 2)
    for gx in np.arange(math.ceil(x0), x1, 1.0):
        X = int((gx - x0) / res * 10)
        cv2.line(img, (X, 0), (X, img.shape[0]), (170, 170, 170), 1)
        cv2.putText(img, f'x{gx:.0f}', (X + 3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1)
    for gy in np.arange(math.ceil(y0), y1, 1.0):
        Y = int((y1 - gy) / res * 10)
        cv2.line(img, (0, Y), (img.shape[1], Y), (170, 170, 170), 1)
        cv2.putText(img, f'y{gy:.0f}', (3, Y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1)
    cv2.imwrite(os.path.join(out, 'layers.png'), img)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3], tuple(float(v) for v in sys.argv[4].split()),
         tuple(float(v) for v in sys.argv[5].split()), *sys.argv[6:8])
