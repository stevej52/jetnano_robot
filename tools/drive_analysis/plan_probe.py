"""Ask Nav2's planner for a path without driving: python3 plan_probe.py PLANNER X0 Y0 H0 X1 Y1 H1
Prints the path length, the straight-line distance and the path's waypoints every 0.5 m, so a
detour shows. Headings in degrees, map frame. PLANNER = GridBased | Through."""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

planner, x0, y0, h0, x1, y1, h1 = sys.argv[1], *[float(v) for v in sys.argv[2:8]]


def pose(x, y, h):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(math.radians(h) / 2), math.cos(math.radians(h) / 2)
    return p


rclpy.init()
n = rclpy.create_node('plan_probe')
c = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
assert c.wait_for_server(timeout_sec=8.0), 'no planner action server'
g = ComputePathToPose.Goal()
g.start, g.goal, g.use_start, g.planner_id = pose(x0, y0, h0), pose(x1, y1, h1), True, planner
t0 = time.monotonic()
f = c.send_goal_async(g)
rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
h = f.result()
r = h.get_result_async()
rclpy.spin_until_future_complete(n, r, timeout_sec=30.0)
res = r.result().result if r.done() else None
if res is None or not res.path.poses:
    print(f'{planner}: NO PATH ({time.monotonic() - t0:.1f} s)' + (f' error {res.error_code}' if res is not None and hasattr(res, "error_code") else ''))
else:
    ps = [(p.pose.position.x, p.pose.position.y) for p in res.path.poses]
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(ps, ps[1:]))
    straight = math.hypot(x1 - x0, y1 - y0)
    print(f'{planner}: path {length:.2f} m for {straight:.2f} m straight, {len(ps)} poses, {time.monotonic() - t0:.1f} s')
    acc, marks = 0.0, [ps[0]]
    for a, b in zip(ps, ps[1:]):
        acc += math.hypot(b[0] - a[0], b[1] - a[1])
        if acc >= 0.5:
            marks.append(b)
            acc = 0.0
    print('   via ' + ' '.join(f'({x:+.1f},{y:+.1f})' for x, y in marks[:30]))
n.destroy_node()
rclpy.shutdown()
