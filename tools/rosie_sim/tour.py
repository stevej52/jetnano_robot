#!/usr/bin/env python3
"""Send the simulated Rosie to spots all over the house map and record how each went.

Goals: one per 0.6 m square of floor where her centre has >= 0.30 m to the nearest wall or
unknown cell, reachable from the parking spot, each with a random arrival heading (N/E/S/W)
so turning round is tested too, visited in random order (seed fixed). A goal that fails or
runs out of time is recorded with where she stood, and she is moved to the goal to go on.

    python3 tour.py [--limit N] [--seed S]      -> out/results.jsonl, out/track.jsonl
"""
import json
import math
import os
import random
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from PIL import Image
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy import ndimage
from std_msgs.msg import String

D = os.path.dirname(os.path.realpath(__file__))
STEP = 0.6
CLEAR = 0.30


def candidates(seed):
    m = yaml.safe_load(open(f'{D}/map/home.yaml'))
    img = np.array(Image.open(f'{D}/map/{m["image"]}'))
    res, ox, oy = m['resolution'], m['origin'][0], m['origin'][1]
    h, w = img.shape
    clear = ndimage.distance_transform_edt(img == 254) * res
    lab, _ = ndimage.label(clear >= 0.20)
    r0, c0 = h - 1 - int((0 - oy) / res), int((0 - ox) / res)
    home = lab[r0, c0]
    n = int(STEP / res)
    out = []
    for r in range(0, h - n + 1, n):
        for c in range(0, w - n + 1, n):
            blk = np.where(lab[r:r + n, c:c + n] == home, clear[r:r + n, c:c + n], 0)
            i = np.unravel_index(np.argmax(blk), blk.shape)
            if blk[i] >= CLEAR:
                rr, cc = r + i[0], c + i[1]
                out.append((round(ox + (cc + 0.5) * res, 2), round(oy + (h - 1 - rr + 0.5) * res, 2),
                            round(float(blk[i]), 2)))
    rnd = random.Random(seed)
    rnd.shuffle(out)
    return [(x, y, rnd.choice([0, 90, 180, -90]), cl) for x, y, cl in out]


class Tour(Node):
    def __init__(self):
        super().__init__('tour', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tele = self.create_publisher(PoseStamped, '/sim/teleport', 10)
        self.clears = [self.create_client(ClearEntireCostmap, s) for s in
                       ('/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap')]
        self.stats = None
        self.create_subscription(String, '/sim/stats', self.on_stats, 10)
        self.track = open(f'{D}/out/track.jsonl', 'w')

    def on_stats(self, m):
        self.stats = json.loads(m.data)
        self.track.write(m.data + '\n')

    def pose(self, x, y, hdeg):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(math.radians(hdeg) / 2), math.cos(math.radians(hdeg) / 2)
        return p

    def spin_for(self, wall_s):
        end = time.monotonic() + wall_s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def go(self, x, y, hdeg):
        s0 = dict(self.stats)
        dist = math.hypot(x - s0['x'], y - s0['y'])
        limit = (45.0 + 6.0 * dist) * float(os.environ.get('LIMIT_SCALE', '1'))
        g = NavigateToPose.Goal()
        g.pose = self.pose(x, y, hdeg)
        g.pose.header.stamp = self.get_clock().now().to_msg()
        rec = {'goal': [x, y, hdeg], 'from': [s0['x'], s0['y']], 'straight_m': round(dist, 2), 'recoveries': 0}

        def fb(f):
            rec['recoveries'] = max(rec['recoveries'], f.feedback.number_of_recoveries)
        fut = self.nav.send_goal_async(g, feedback_callback=fb)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        h = fut.result()
        if not h.accepted:
            rec['outcome'] = 'REJECTED'
            return rec, s0
        rf = h.get_result_async()
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.stats['t'] - s0['t'] > limit:
                cf = h.cancel_goal_async()
                while not cf.done():
                    rclpy.spin_once(self, timeout_sec=0.05)
                self.spin_for(1.0)
                rec['outcome'] = 'TIMEOUT'
                break
        else:
            r = rf.result()
            rec['outcome'] = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(r.status, str(r.status))
            rec['error_code'] = getattr(r.result, 'error_code', None)
            rec['error_msg'] = getattr(r.result, 'error_msg', '')
        s1 = dict(self.stats)
        rec.update({'seconds': round(s1['t'] - s0['t'], 1), 'driven_m': round(s1['odo'] - s0['odo'], 2),
                    'reversals': s1['reversals'] - s0['reversals'], 'bumps': s1['bumps'] - s0['bumps'],
                    'end': [s1['x'], s1['y'], round(math.degrees(s1['th']))],
                    'end_off_m': round(math.hypot(s1['x'] - x, s1['y'] - y), 2)})
        return rec, s1

    def reset_to(self, x, y, hdeg):
        self.tele.publish(self.pose(x, y, hdeg))
        self.spin_for(0.5)
        for c in self.clears:
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())
        self.spin_for(1.0)


def main():
    argv = sys.argv[1:]
    limit = int(argv[argv.index('--limit') + 1]) if '--limit' in argv else 10 ** 6
    seed = int(argv[argv.index('--seed') + 1]) if '--seed' in argv else 7
    goals = candidates(seed)[:limit]
    rclpy.init()
    t = Tour()
    print(f'{len(goals)} goals; waiting for Nav2', flush=True)
    t.nav.wait_for_server()
    while t.stats is None:
        rclpy.spin_once(t, timeout_sec=0.1)
    t.spin_for(3.0)
    out = open(f'{D}/out/results.jsonl', 'w')
    w0 = time.monotonic()
    for i, (x, y, hd, cl) in enumerate(goals, 1):
        rec, s = t.go(x, y, hd)
        rec['clearance_m'] = cl
        rec['n'] = i
        out.write(json.dumps(rec) + '\n')
        out.flush()
        print(f"{i:3d}/{len(goals)} ({x:+.2f},{y:+.2f}) h{hd:+4d}: {rec['outcome']:9s} {rec.get('seconds', 0):5.1f} s "
              f"{rec.get('driven_m', 0):5.2f} m rev {rec.get('reversals', 0):2d} bump {rec.get('bumps', 0)} "
              f"rec {rec['recoveries']} off {rec.get('end_off_m', 0):.2f}  [{time.monotonic() - w0:5.0f} s wall]", flush=True)
        if rec['outcome'] != 'SUCCEEDED':
            t.reset_to(x, y, hd)
    print('done', flush=True)


if __name__ == '__main__':
    main()
