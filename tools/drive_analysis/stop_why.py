"""What was in the DirectionalStop zone during a stop.  python3 stop_why.py BAG T_REL DUR"""
import sys, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState

bag, trel, dur = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
ZONES = {'fwd_slow': (0.12, 0.372), 'fwd_fast': (0.12, 0.502)}
HW = 0.17

def q2y(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
def comp(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/tf_static', '/scan', '/odometry/filtered',
                                              '/cmd_vel_nav_smoothed', '/collision_monitor_state']))
t0 = None; laser = (0, 0, 0); mo = ob = None
while r.has_next():
    t, d, ts = r.read_next(); ts *= 1e-9
    if t in ('/tf', '/tf_static'):
        for tr in deserialize_message(d, TFMessage).transforms:
            v = (tr.transform.translation.x, tr.transform.translation.y, q2y(tr.transform.rotation))
            if tr.child_frame_id == 'odom': mo = v
            elif tr.child_frame_id == 'base_footprint': ob = v
            elif tr.child_frame_id in ('lidar_link', 'laser', 'laser_frame'): laser = v
        continue
    if t == '/odometry/filtered':
        m = deserialize_message(d, Odometry)
        if t0 is None: t0 = ts
        rt = ts - t0
        if trel - 1.5 <= rt <= trel + dur + 0.5 and int(rt * 10) % 5 == 0:
            print(f'{rt:6.1f} odo v {m.twist.twist.linear.x:+.2f} w {m.twist.twist.angular.z:+.2f}')
        continue
    if t0 is None: continue
    rt = ts - t0
    if not (trel - 1.5 <= rt <= trel + dur + 0.5): continue
    if t == '/cmd_vel_nav_smoothed':
        m = deserialize_message(d, Twist)
        print(f'{rt:6.1f} ask v {m.linear.x:+.2f} w {m.angular.z:+.2f}')
    elif t == '/collision_monitor_state':
        m = deserialize_message(d, CollisionMonitorState)
        print(f'{rt:6.1f} MONITOR action {m.action_type} {m.polygon_name}')
    elif t == '/scan' and mo and ob:
        m = deserialize_message(d, LaserScan)
        rr = np.array(m.ranges); a = m.angle_min + np.arange(len(rr)) * m.angle_increment
        ok = np.isfinite(rr) & (rr > m.range_min)
        px = laser[0] + rr[ok] * np.cos(a[ok] + laser[2]); py = laser[1] + rr[ok] * np.sin(a[ok] + laser[2])
        pose = comp(mo, ob)
        line = f'{rt:6.1f} pose ({pose[0]:.2f},{pose[1]:.2f}) hdg {math.degrees(pose[2]):+.0f}'
        for z, (x0, x1) in ZONES.items():
            k = (px >= x0) & (px <= x1) & (np.abs(py) <= HW)
            if k.sum():
                c, s = math.cos(pose[2]), math.sin(pose[2])
                mx = pose[0] + c * px[k] - s * py[k]; my = pose[1] + s * px[k] + c * py[k]
                pts = ' '.join(f'[{x:.2f},{y:+.2f}->{X:.2f},{Y:.2f}]' for x, y, X, Y in zip(px[k], py[k], mx, my))
                line += f' | {z} {k.sum()} pts {pts}'
        # nearest ahead outside zone, for context
        fk = (px > 0.12) & (px < 1.0) & (np.abs(py) < 0.35)
        if fk.sum():
            j = np.argmin(px[fk]); line += f' | nearest ahead x {px[fk][j]:.2f} y {py[fk][j]:+.2f}'
        print(line)
