"""The house as her lidar saw it on a drive: hit and pass-through counts per map cell.

    python3 scan_world.py BAG REF.yaml OUT.npz

Every /scan is placed with the recorded map->odom->base_footprint->lidar_link transforms (lidar_link
is mounted turned 180 deg) and ray-cast on REF's grid: the end cell counts a hit, the cells it
crossed count a pass. The sim then trusts this over the old map wherever she actually looked.
"""
import math, sys
import numpy as np
import yaml
import rosbag2_py
from PIL import Image
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan

bag, ref, out = sys.argv[1:4]
m = yaml.safe_load(open(ref))
img = np.array(Image.open(ref.rsplit('/', 1)[0] + '/' + m['image']))
H, W = img.shape
res, ox, oy = float(m['resolution']), float(m['origin'][0]), float(m['origin'][1])
hit = np.zeros((H, W), np.int32)
pas = np.zeros((H, W), np.int32)
MAXR = 5.0


def q2y(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def comp(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def rc(x, y):
    c = np.floor((x - ox) / res).astype(int)
    r = H - 1 - np.floor((y - oy) / res).astype(int)
    ok = (c >= 0) & (c < W) & (r >= 0) & (r < H)
    return r[ok], c[ok]


r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/tf_static', '/scan']))
mo = ob = None
laser = (0.0, 0.0, math.pi)
last_pose, n = None, 0
while r.has_next():
    t, d, ts = r.read_next()
    if t != '/scan':
        for tr in deserialize_message(d, TFMessage).transforms:
            v = (tr.transform.translation.x, tr.transform.translation.y, q2y(tr.transform.rotation))
            if tr.child_frame_id == 'odom':
                mo = v
            elif tr.child_frame_id == 'base_footprint':
                ob = v
            elif tr.child_frame_id in ('lidar_link', 'laser', 'laser_frame'):
                laser = v
        continue
    if mo is None or ob is None:
        continue
    L = comp(comp(mo, ob), laser)
    # one scan per 3 cm or 3 deg of motion is plenty (and standing still adds nothing new)
    if last_pose and math.hypot(L[0] - last_pose[0], L[1] - last_pose[1]) < 0.03 and abs(L[2] - last_pose[2]) < 0.05:
        continue
    last_pose = L
    s = deserialize_message(d, LaserScan)
    rr = np.array(s.ranges, float)
    a = s.angle_min + np.arange(len(rr)) * s.angle_increment + L[2]
    good = np.isfinite(rr) & (rr > max(s.range_min, 0.12))
    end = good & (rr < MAXR)
    # passes: sample each ray every half cell up to 1 cell short of the end (or MAXR for no return)
    span = np.where(end, rr - res, np.where(good, MAXR, 0.0))
    for k in np.nonzero(span > 0)[0]:
        ds = np.arange(0.12, span[k], res / 2)
        R_, C_ = rc(L[0] + ds * math.cos(a[k]), L[1] + ds * math.sin(a[k]))
        pas[R_, C_] += 1
    R_, C_ = rc(L[0] + rr[end] * np.cos(a[end]), L[1] + rr[end] * np.sin(a[end]))
    np.add.at(hit, (R_, C_), 1)
    n += 1
np.savez_compressed(out, hit=hit, pas=pas)
print(f'{n} scans used; cells hit {int((hit > 0).sum())}, crossed {int((pas > 0).sum())}')
