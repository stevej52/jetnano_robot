#!/usr/bin/env python3
"""Nav2's route planner alone, on the saved house map, under different planning margins.

    python3 planner_margins.py            (on the robot; ROS domain 91, nothing reaches the robot)

For each setting it starts map_server (~/maps/home.yaml), a fixed map -> base_link at the leg's
start, and planner_server with the installed nav2.yaml's planner and global costmap (static layer
+ inflation only: no live lidar or camera), asks ComputePathToPose for each leg, and prints the
route: planned or not, length, whether it went round the island's WEST end, and how many
forward/reverse switches it plans. 2026-09-29: the kitchen leg round the west end failed on the
floor while Steve drives it easily ("two or three times the space she needs").
"""
import ast
import copy
import math
import os
import signal
import subprocess
import sys
import time

import yaml

NAV2 = '/home/jeston/ros2_ws/install/jetnano_navigation/share/jetnano_navigation/config/nav2.yaml'
MAP = os.environ.get('PLANNER_TEST_MAP', '/home/jeston/maps/home.yaml')
DOMAIN = '91'
LEGS = [  # (name, start x y heading_deg, goal x y heading_deg)
    ('kitchen: south of the island -> round the west end', (1.4, -7.2, 180), (-0.3, -5.0, 90)),
    ('kitchen: corridor -> south of the island', (2.5, -4.7, -90), (1.4, -7.2, 180)),
    ('living room turnaround', (0.3, 0.8, 180), (1.5, 0.0, 0)),
]
CONFIGS = [  # name, footprint_padding, inflation_radius, cost_scaling_factor, cost_penalty
    ('now', 0.03, 0.35, 3.0, 2.0),
    ('radius 0.31', 0.03, 0.31, 3.0, 2.0),
    ('radius 0.31, steep 6', 0.03, 0.31, 6.0, 2.0),
    ('radius 0.31, steep 6, penalty 1.2', 0.03, 0.31, 6.0, 1.2),
    ('pad 0.01, radius 0.29, steep 6, penalty 1.2', 0.01, 0.29, 6.0, 1.2),
]


if os.environ.get('PLANNER_TEST_CONFIGS'):
    CONFIGS = [c for c in CONFIGS if c[0] in os.environ['PLANNER_TEST_CONFIGS'].split('|')]
if os.environ.get('PLANNER_TEST_LEGS'):
    LEGS = LEGS[:int(os.environ['PLANNER_TEST_LEGS'])]


def params(pad, radius, steep, penalty, path):
    src = yaml.safe_load(open(NAV2))
    out = {'planner_server': copy.deepcopy(src['planner_server']),
           'global_costmap': copy.deepcopy(src['global_costmap']),
           'map_server': {'ros__parameters': {'yaml_filename': MAP, 'topic_name': 'map', 'frame_id': 'map'}}}
    out['planner_server']['ros__parameters']['GridBased']['cost_penalty'] = penalty
    g = out['global_costmap']['global_costmap']['ros__parameters']
    g['plugins'] = ['static_layer', 'inflation_layer']
    g['footprint_padding'] = pad
    g['static_layer']['map_topic'] = '/map'
    g['inflation_layer']['inflation_radius'] = radius
    g['inflation_layer']['cost_scaling_factor'] = steep
    yaml.safe_dump(out, open(path, 'w'))


def start(cmd, env):
    return subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def stop(procs):
    for p in procs:
        try:
            os.killpg(p.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    time.sleep(3)
    for p in procs:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def route_facts(path):
    pts = [(p.pose.position.x, p.pose.position.y,
            2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w)) for p in path.poses]
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    signs = []
    for a, b in zip(pts, pts[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        if math.hypot(dx, dy) > 1e-4:
            signs.append(1 if dx * math.cos(a[2]) + dy * math.sin(a[2]) >= 0 else -1)
    switches = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
    west = any(x < 0.2 and -6.9 < y < -5.2 for x, y, _ in pts)
    return length, switches, west


def plan_all(env):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose
    from rclpy.action import ActionClient
    rclpy.init()
    n = rclpy.create_node('margin_probe')
    client = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
    results = []
    if not client.wait_for_server(timeout_sec=120.0):
        rclpy.shutdown()
        return None

    def pose(x, y, h):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(math.radians(h) / 2), math.cos(math.radians(h) / 2)
        return p

    time.sleep(3.0)                                   # the static layer and inflation settle
    for name, s, g in LEGS:
        goal = ComputePathToPose.Goal()
        goal.start, goal.goal, goal.use_start, goal.planner_id = pose(*s), pose(*g), True, 'GridBased'
        f = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(n, f, timeout_sec=10)
        h = f.result()
        if h is None or not h.accepted:
            results.append((name, 'rejected'))
            continue
        r = h.get_result_async()
        rclpy.spin_until_future_complete(n, r, timeout_sec=15)
        res = r.result()
        if res is None or not res.result.path.poses:
            results.append((name, 'NO PATH'))
        else:
            length, switches, west = route_facts(res.result.path)
            results.append((name, f'{length:5.1f} m, {switches} switches' + (', WEST end' if west else '')))
    rclpy.shutdown()
    return results


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--child':
        res = plan_all(os.environ)
        print(repr(res))
        return
    env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN)
    for cname, pad, radius, steep, penalty in CONFIGS:
        path = f'/tmp/margins-{DOMAIN}.yaml'
        params(pad, radius, steep, penalty, path)
        s = LEGS[0][1]
        procs = [
            start(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '--params-file', path], env),
            start(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', str(s[0]), '--y', str(s[1]),
                   '--yaw', str(math.radians(s[2])), '--frame-id', 'map', '--child-frame-id', 'base_link'], env),
            start(['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file', path], env),
            start(['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args',
                   '-r', '__node:=lifecycle_manager_margins', '-p', 'autostart:=true', '-p', 'bond_timeout:=0.0',
                   '-p', 'node_names:=[map_server,planner_server]'], env),
        ]
        try:
            t0 = time.time()
            out = subprocess.run([sys.executable, __file__, '--child'], env=env, capture_output=True, text=True,
                                 timeout=200)
            res = ast.literal_eval(out.stdout.strip().splitlines()[-1]) if out.stdout.strip() else None
            print(f'== {cname}  (pad {pad}, radius {radius}, steepness {steep}, cost penalty {penalty}; '
                  f'{time.time() - t0:.0f} s)', flush=True)
            for name, r in res or [('all', 'planner did not come up')]:
                print(f'     {name:50s} {r}', flush=True)
        finally:
            stop(procs)


if __name__ == '__main__':
    main()
