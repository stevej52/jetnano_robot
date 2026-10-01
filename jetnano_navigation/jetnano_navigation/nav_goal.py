# Copyright 2026 stevej52
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Send Rosie one Nav2 goal in the map frame and report how it went.

    ros2 run jetnano_navigation nav_goal X Y HEADING_DEG [TIMEOUT_S]
    ros2 run jetnano_navigation nav_goal X Y HEADING_DEG --check     (the checks only)

Before sending it:
* waits (up to 20 s) for the motion check's lock (e_stop_motion) to be released - on
  2026-09-28 a goal sent a second after a stuck one started locked and sat 90 s;
* checks the goal against the planner's costmap: her centre must be clear of the
  furniture's margin (cost <= 20) with nothing solid within her outline's reach
  (0.32 m). If not, it moves the goal to the nearest place within 0.8 m that is, and
  says so, or refuses - that day a goal 35 cm from the TV cabinet ended with her nose
  against it.
Then it prints progress each second (from Nav2's own feedback), cancels the goal after
TIMEOUT_S (default 45), and ends with the pose error (from localization) and what the
translator sent the driver.

    ros2 run jetnano_navigation nav_goal X Y HEADING_DEG [TIMEOUT_S] --rescue
    ros2 run jetnano_navigation nav_goal X Y HEADING_DEG [TIMEOUT_S] --exact    (no margin check)
    ros2 run jetnano_navigation nav_goal X Y HEADING_DEG [TIMEOUT_S] --home     (allowed on a low battery)

A goal is refused on the bench (where_am_i) and on a low or flat battery (battery/level)
unless it is the way home (--home, nav_park's legs); "low soon" only gets a warning
(Steve, 2026-09-30: "don't start what you can't finish, and go home when the countdown plays").

--rescue (2026-09-29): if the goal fails, back out along her own track (nav_helper
~/retrace) and try again; if that fails too, ask Claude with pictures (nav_helper ~/ask:
the house map, the obstacles, a camera sweep) and do what it picks - back out further, go
by a point it chose first (checked against the costmap like any goal), wait, or give up -
at most twice.
"""

import json
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

REACH_M = 0.32            # her outline's reach from the centre, tyres turned (0.26 x 0.19)
CENTRE_MAX_COST = 20      # 0-100 costmap scale: 0 is clear of every margin
# An obstacle itself, not the margin round it. Until 2026-09-29 this was 99 (the inscribed
# margin too), which counted her size twice - the margin is already her half-width out from
# each obstacle - so a goal needed about 1.0 m of clear floor across where she needs 0.64.
# Drive 6 lost the dining room's south-east corner to that: a 1.1 m gap between the chairs and
# the wall, clear on the camera, "nowhere within 0.8 m ... clear", and the planner would have
# taken it (tools/drive_analysis/costmap_at.py).
SOLID_COST = 100
SEARCH_M = 0.8


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Grid:
    """What clear_goal reads: an OccupancyGrid's info and data (0-100, -1 unknown)."""

    def __init__(self, resolution, width, height, ox, oy, data):
        self.info = type('Info', (), {})()
        self.info.resolution, self.info.width, self.info.height = resolution, width, height
        self.info.origin = type('Pose', (), {})()
        self.info.origin.position = type('Point', (), {'x': ox, 'y': oy})()
        self.data = data


def grid_from_costmap(cm):
    """nav2_msgs/Costmap (0-255) -> Grid on the 0-100 scale, translated as Nav2 publishes it."""
    raw = np.asarray(cm.data, dtype=np.int16)
    out = np.where(raw == 0, 0, 1 + (97 * (raw - 1)) // 251)
    out[raw == 253] = 99
    out[raw == 254] = 100
    out[raw == 255] = -1
    md = cm.metadata
    return Grid(md.resolution, md.size_x, md.size_y, md.origin.position.x, md.origin.position.y, out)


def battery_verdict(level_json, home: bool):
    """(may she go?, a line to print or '') from battery_monitor's battery/level message.
    low or flat: only the way home (--home). soon: yes, with a warning. Unknown: yes."""
    try:
        rec = json.loads(level_json or '{}')
        level, why = rec.get('level', 'ok'), rec.get('why', '')
    except ValueError:
        return True, ''
    if level in ('low', 'flat'):
        if home:
            return True, f'battery {level} ({why}): allowed, this is the way home'
        return False, f'battery {level} ({why}): she is heading home, not out'
    if level == 'soon':
        return True, f'battery low soon ({why}): she may have to head home during this'
    return True, ''


def clear_goal(grid, gx, gy):
    """(x, y, moved_m) nearest (gx, gy) where she fits, or None."""
    i = grid.info
    res, ox, oy = i.resolution, i.origin.position.x, i.origin.position.y
    a = np.asarray(grid.data, dtype=np.int16).reshape(i.height, i.width)
    a = np.where(a < 0, 0, a)                     # unknown is not in the way here
    r = int(math.ceil(REACH_M / res))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    disc = (xx * xx + yy * yy) * res * res <= REACH_M * REACH_M     # cells centred within her reach

    def fits(u, v):
        if not (r <= u < i.width - r and r <= v < i.height - r):
            return False
        if a[v, u] > CENTRE_MAX_COST:
            return False
        return a[v - r:v + r + 1, u - r:u + r + 1][disc].max() < SOLID_COST

    u0, v0 = int((gx - ox) / res), int((gy - oy) / res)
    best = None
    s = int(SEARCH_M / res)
    for dv in range(-s, s + 1):
        for du in range(-s, s + 1):
            d = math.hypot(du, dv) * res
            if d > SEARCH_M or (best is not None and d >= best[2]):
                continue
            if fits(u0 + du, v0 + dv):
                best = (ox + (u0 + du + 0.5) * res, oy + (v0 + dv + 0.5) * res, d)
    return best


def main():
    check_only = '--check' in sys.argv
    rescue = '--rescue' in sys.argv
    exact = '--exact' in sys.argv          # nav_park's last leg: her own spot, however tight
    home = '--home' in sys.argv            # nav_park's legs: the way home is allowed on a low battery
    args = [a for a in sys.argv[1:] if a not in ('--check', '--rescue', '--exact', '--home')]
    gx, gy, gh = float(args[0]), float(args[1]), math.radians(float(args[2]))
    timeout = float(args[3]) if len(args) > 3 else 45.0

    rclpy.init()
    n = rclpy.create_node('nav_goal')
    st = {'cmd': [], 'lock': None, 'grid': None, 'where': None}
    n.create_subscription(Twist, '/cmd_vel_nav', lambda m: st['cmd'].append((m.linear.x, m.angular.z)), 20)
    n.create_subscription(Bool, '/e_stop_motion', lambda m: st.__setitem__('lock', m.data), 10)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # where_am_i's verdict (latched): on the bench, nothing drives (2026-09-30)
    n.create_subscription(String, '/where_am_i/state', lambda m: st.__setitem__('where', m.data),
                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    # the battery's verdict (latched): low or flat, only the way home is driven (Steve, 2026-09-30:
    # "don't start what you can't finish"); battery_home takes her there
    st['battery'] = None
    n.create_subscription(String, '/battery/level', lambda m: st.__setitem__('battery', m.data),
                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    costmap_sub = n.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                        lambda m: st.__setitem__('grid', m), latched)
    client = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=10.0):
        raise SystemExit('navigate_to_pose action server not available')

    def spin_for(seconds, until=lambda: False):
        end = time.time() + seconds
        while time.time() < end and not until():
            rclpy.spin_once(n, timeout_sec=0.05)

    # 0. not on the bench
    spin_for(1.0, lambda: st['where'] is not None)
    try:
        where = json.loads(st['where'] or '{}')
    except ValueError:
        where = {}
    if where.get('state') == 'bench':
        raise SystemExit(f'she is on the bench ({where.get("why", "")}): not sending the goal')

    # 0b. enough battery for it
    spin_for(1.0, lambda: st['battery'] is not None)
    allowed, note = battery_verdict(st['battery'], home)
    if note:
        print(note, flush=True)
    if not allowed:
        raise SystemExit('not sending the goal')

    # 1. the motion check's lock (it is re-sent every second while it holds)
    spin_for(1.5, lambda: st['lock'] is not None)
    if st['lock']:
        print('the motion check has her locked: waiting for it to be released', flush=True)
        spin_for(20.0, lambda: st['lock'] is False)
        if st['lock']:
            raise SystemExit('still locked after 20 s: not sending the goal')

    # 2. somewhere she fits, on the costmap the planner has now. The latched topic is only
    # the last FULL costmap - Nav2 sends changes separately while the size stays the same - so
    # it can be minutes behind; 2026-09-29 it refused two dining-room goals the planner's own
    # map had room for. The topic stays as the fallback.
    live = n.create_client(GetCostmap, '/global_costmap/get_costmap')

    def live_grid():
        if live.wait_for_service(timeout_sec=3.0):
            fut = live.call_async(GetCostmap.Request())
            spin_for(5.0, fut.done)
            if fut.done() and fut.result() is not None:
                st['grid'] = grid_from_costmap(fut.result().map)
        return st['grid']

    if live_grid() is None:
        spin_for(5.0, lambda: st['grid'] is not None)
    n.destroy_subscription(costmap_sub)
    if exact:
        print(f'goal ({gx:+.2f}, {gy:+.2f}) taken as it is (--exact)', flush=True)
    elif st['grid'] is None:
        print('no costmap from the planner in 5 s: goal not checked', flush=True)
    else:
        found = clear_goal(st['grid'], gx, gy)
        if found is None:
            raise SystemExit(f'nowhere within {SEARCH_M:g} m of ({gx:+.2f}, {gy:+.2f}) is clear of the '
                             'furniture for her: goal refused')
        if found[2] > 0.0:
            print(f'goal ({gx:+.2f}, {gy:+.2f}) is inside a furniture margin: moved {found[2] * 100:.0f} cm '
                  f'to ({found[0]:+.2f}, {found[1]:+.2f})', flush=True)
            gx, gy = found[0], found[1]
        else:
            print(f'goal ({gx:+.2f}, {gy:+.2f}) is clear of the furniture', flush=True)
    if check_only:
        rclpy.shutdown()
        return

    def tf_pose():
        """map -> base_footprint once, with a listener that lives only for this."""
        buf = tf2_ros.Buffer()
        lis = tf2_ros.TransformListener(buf, n)
        try:
            for _ in range(150):
                if buf.can_transform('map', 'base_footprint', rclpy.time.Time()):
                    t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                    return (t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))
                rclpy.spin_once(n, timeout_sec=0.1)
            return None
        finally:
            lis.unregister()

    goal_pub = n.create_publisher(PoseStamped, 'nav_helper/goal', 10)
    situation_pub = n.create_publisher(String, 'nav_helper/situation', 10)
    # the exact-heading planner, in case a nav_route left its any-heading "Through" selected
    planner_pub = n.create_publisher(String, 'planner_selector',
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL))
    planner_pub.publish(String(data='GridBased'))

    def drive(gx, gy, gh, label='goal'):
        """One Nav2 goal: progress each second, then the result line. -> its status."""
        start = tf_pose()
        if start is None:
            raise SystemExit('no map -> base_footprint transform after 15 s')
        print(f'start  x {start[0]:+.2f}  y {start[1]:+.2f}  heading {math.degrees(start[2]):+.0f} deg; '
              f'{label} x {gx:+.2f}  y {gy:+.2f}  heading {math.degrees(gh):+.0f} deg', flush=True)
        # her position from Nav2's feedback (a TF listener kept for this cost a third of a
        # core in Python on 2026-09-28)
        fb = {'dist': float('nan'), 'pose': None}

        def on_feedback(f):
            fb['dist'] = f.feedback.distance_remaining
            p = f.feedback.current_pose.pose
            fb['pose'] = (p.position.x, p.position.y, yaw(p.orientation))

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = n.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(gh / 2), math.cos(gh / 2)
        st['cmd'] = []
        send = client.send_goal_async(goal, feedback_callback=on_feedback)
        while not send.done():
            rclpy.spin_once(n, timeout_sec=0.1)
        handle = send.result()
        if not handle.accepted:
            print('result REJECTED', flush=True)
            return GoalStatus.STATUS_ABORTED
        res = handle.get_result_async()
        t0 = time.time()
        last = 0.0
        locked_during = False
        while not res.done():
            rclpy.spin_once(n, timeout_sec=0.05)
            locked_during = locked_during or bool(st['lock'])
            if time.time() - last >= 1.0:
                last = time.time()
                p = fb['pose'] or start
                c = st['cmd'][-1] if st['cmd'] else (0.0, 0.0)
                print(f'{time.time() - t0:5.1f} s  at x {p[0]:+.2f} y {p[1]:+.2f} h {math.degrees(p[2]):+4.0f}  '
                      f'remaining {fb["dist"]:.2f} m  driver: throttle {c[0]:+.2f} steer {c[1]:+.2f}'
                      + ('  MOTION LOCK' if st['lock'] else ''), flush=True)
            if time.time() - t0 > timeout:
                print('timeout: cancelling the goal', flush=True)
                cancel = handle.cancel_goal_async()
                while not cancel.done():
                    rclpy.spin_once(n, timeout_sec=0.1)
                break
        status = res.result().status if res.done() else GoalStatus.STATUS_CANCELED
        names = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                 GoalStatus.STATUS_CANCELED: 'CANCELED'}
        time.sleep(1.0)
        p = tf_pose() or fb['pose'] or start
        dh = math.degrees(math.atan2(math.sin(p[2] - gh), math.cos(p[2] - gh)))
        thr = [abs(c[0]) for c in st['cmd'] if c[0] != 0]
        signs = [1 if c[0] > 0 else -1 for c in st['cmd'] if abs(c[0]) > 0.01]
        switches = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
        print(f'result {names.get(status, status)} after {time.time() - t0:.1f} s'
              + (' (the motion check stopped her)' if locked_during else '')
              + f'; ended {math.hypot(p[0] - gx, p[1] - gy) * 100:.0f} cm from the {label}, '
              f'heading off by {dh:+.0f} deg; '
              f'driver got {len(st["cmd"])} commands, throttle {min(thr, default=0):.2f}-{max(thr, default=0):.2f}; '
              f'forward/reverse switches {switches}', flush=True)
        return status

    def helper(name, wait_s):
        """Call nav_helper's ~/<name> (std_srvs/Trigger) -> (success, message)."""
        cli = n.create_client(Trigger, f'/nav_helper/{name}')
        try:
            if not cli.wait_for_service(timeout_sec=5.0):
                return False, 'nav_helper is not running'
            fut = cli.call_async(Trigger.Request())
            end = time.time() + wait_s
            while not fut.done() and time.time() < end:
                rclpy.spin_once(n, timeout_sec=0.1)
            if not fut.done():
                return False, f'no reply in {wait_s:.0f} s'
            return fut.result().success, fut.result().message
        finally:
            n.destroy_client(cli)

    def set_retrace(metres):
        cli = n.create_client(SetParameters, '/nav_helper/set_parameters')
        try:
            if cli.wait_for_service(timeout_sec=3.0):
                req = SetParameters.Request()
                req.parameters = [Parameter(name='retrace_m', value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=float(metres)))]
                fut = cli.call_async(req)
                end = time.time() + 5.0
                while not fut.done() and time.time() < end:
                    rclpy.spin_once(n, timeout_sec=0.1)
        finally:
            n.destroy_client(cli)

    def announce(text):
        for _ in range(3):                               # nav_helper hears it (a new publisher)
            m = String()
            m.data = text
            situation_pub.publish(m)
            spin_for(0.2)

    gp = PoseStamped()
    gp.header.frame_id = 'map'
    gp.pose.position.x, gp.pose.position.y = gx, gy
    gp.pose.orientation.z, gp.pose.orientation.w = math.sin(gh / 2), math.cos(gh / 2)
    for _ in range(3):
        goal_pub.publish(gp)
        spin_for(0.1)

    status = drive(gx, gy, gh)
    if rescue and status != GoalStatus.STATUS_SUCCEEDED and not battery_verdict(st['battery'], home)[0]:
        print('rescue: the battery is low - not retrying; she is heading home (battery_home)', flush=True)
        rescue = False
    if rescue and status != GoalStatus.STATUS_SUCCEEDED:
        print('rescue: backing out along her own track, then trying again', flush=True)
        set_retrace(0.8)
        ok, msg = helper('retrace', 40.0)
        print(f'rescue: {msg}', flush=True)
        status = drive(gx, gy, gh)
        for ask in range(2):
            if status == GoalStatus.STATUS_SUCCEEDED:
                break
            announce(f'she was sent to map ({gx:+.2f}, {gy:+.2f}); Nav2 gave up'
                     + (', backing out along her track and trying again did not help' if ask == 0 else
                        ', and the last advice did not work either'))
            print('rescue: asking Claude (house map, obstacles, camera sweep)...', flush=True)
            ok, msg = helper('ask', 200.0)
            try:
                out = json.loads(msg)
            except ValueError:
                out = {'error': msg}
            advice = out.get('advice')
            if not advice:
                print(f'rescue: no usable advice ({out.get("error") or "unreadable answer"}); '
                      f'pictures in {out.get("dir", "?")}', flush=True)
                break
            print(f'rescue: Claude sees "{advice["what"]}"' + (' (temporary)' if advice['temporary'] else '')
                  + f' -> {advice["action"]}: {advice["why"]}'
                  + (f' Says: "{advice["say"]}"' if advice['say'] else '')
                  + f' ({out.get("seconds", "?")} s; {out.get("dir", "")})', flush=True)
            action = advice['action']
            if action == 'give_up':
                break
            if action == 'retrace':
                set_retrace(advice['retrace_m'])
                ok, msg = helper('retrace', 60.0)
                print(f'rescue: {msg}', flush=True)
            elif action == 'wait':
                print(f'rescue: waiting {advice["wait_s"]:.0f} s', flush=True)
                spin_for(advice['wait_s'])
            elif action == 'via':
                vx, vy = advice['x'], advice['y']
                if live_grid() is not None:
                    found = clear_goal(st['grid'], vx, vy)
                    if found is None:
                        print(f'rescue: ({vx:+.2f}, {vy:+.2f}) is not clear for her: not going there', flush=True)
                        continue
                    vx, vy = found[0], found[1]
                heading = math.atan2(gy - vy, gx - vx)          # facing on towards the goal
                drive(vx, vy, heading, label='waypoint')
            status = drive(gx, gy, gh)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
