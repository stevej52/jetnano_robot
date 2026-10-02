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

"""Drive Rosie through a route of waypoints without stopping at each one.

    ros2 run jetnano_navigation nav_route X Y H  X Y H  ...  [--timeout S] [--check] [--home]

Drive 11 (2026-09-30): the big lap as nine nav_goal legs stood still 89 s of 170 s - each
waypoint was a goal she stopped at, then a new process, the costmap check, a fresh plan and
the acceleration ramp. Steve: "why all the stopping?" This sends the whole lap as one Nav2
NavigateThroughPoses goal (navigate_through_poses_no_spin.xml): a waypoint is passed, not
reached - the tree drops it once she is within 0.7 m - and only the LAST one is a stop.

Nav2's only "done" test is "within 10 cm of the END of the path", so a route whose last
waypoint lies where she drives earlier (two laps, a loop that closes near its start) would end
the moment she first passes there - drive 12 (2026-09-30) ended after 18 s of a 9-waypoint lap
whose last waypoint was 0.86 m from its first. So the route is cut into stretches that do not
end on themselves (chunks), and the next stretch is sent as soon as only the current one's
last waypoint is left: Nav2 swaps goals on the move, no stop. Crossing your own path or a
figure-eight is one stretch.

The planner: on a route every waypoint is a planner goal, and the default planner plans to
its EXACT heading - drive 13 (2026-09-30): guessed headings turned the island's corners into
Reeds-Shepp reverse-and-shuffle moves the controller refused near the furniture ("detected
collision ahead", 73 s at one corner, route aborted). So this selects nav2.yaml's "Through"
planner (goal_heading_mode ALL_DIRECTION: a waypoint is passed facing whichever way the path
wants) on the tree's planner_selector topic, and hands "GridBased" back when it is done, so
nav_goal and nav_park keep their exact headings.

The checks are nav_goal's: not on the bench, enough battery (--home for the way home),
the motion check's lock released, and every waypoint clear of the furniture's margin on
the planner's live costmap (moved to the nearest clear place within 0.8 m, or refused).
Prints progress each second, the time each waypoint was passed, and nav_goal's result
line for the last one. Timeout: --timeout S (default 60 s per waypoint).
"""

import json
import math
import sys
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from jetnano_navigation.nav_goal import SEARCH_M, battery_verdict, clear_goal, grid_from_costmap, yaw


def parse_route(args):
    """['x', 'y', 'h_deg', ...] -> [(x, y, h_rad), ...]; the count must be a multiple of 3."""
    vals = [float(a) for a in args]
    if not vals or len(vals) % 3:
        raise SystemExit('a route is X Y HEADING_DEG triplets: ' + ' '.join(args))
    return [(vals[i], vals[i + 1], math.radians(vals[i + 2])) for i in range(0, len(vals), 3)]


CLOSE_M = 0.5             # the last waypoint this near an earlier leg: refused
PASSED_M = 0.7            # the tree drops a waypoint once she is within this (RemovePassedGoals radius)
WAYPOINT_CLOSE_M = 1.0    # the last waypoint this near an earlier WAYPOINT: she may pass the one
                          # anywhere within PASSED_M of it, and overshoot a corner on top


def closes_on_itself(route):
    """(metres from the LAST waypoint to the nearest earlier waypoint, to the nearest earlier leg)
    or None for a route with fewer than three waypoints. Drive 12 (2026-09-30): the lap's last
    waypoint, the hall entry, lay 0.86 m from its first, the hall exit; she passed the first,
    drifted on to the last, and Nav2's goal checker - which only asks "is she within 10 cm of
    the END of the path?" - called the whole lap done after 18 s with eight waypoints to go."""
    if len(route) < 3:
        return None
    lx, ly = route[-1][0], route[-1][1]
    d_wp = min(math.hypot(lx - x, ly - y) for x, y, _ in route[:-1])
    d_leg = None
    for (ax, ay, _), (bx, by, _) in zip(route[:-2], route[1:-1]):
        dx, dy = bx - ax, by - ay
        u = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((lx - ax) * dx + (ly - ay) * dy) / (dx * dx + dy * dy)))
        d = math.hypot(lx - (ax + u * dx), ly - (ay + u * dy))
        d_leg = d if d_leg is None else min(d_leg, d)
    return d_wp, d_leg


def route_is_closed(route):
    """A reason the route would end early, or ''."""
    close = closes_on_itself(route)
    if close is None:
        return ''
    d_wp, d_leg = close
    if d_wp < WAYPOINT_CLOSE_M:
        return f'the last waypoint is {d_wp:.2f} m from an earlier one (she may pass that one anywhere within {PASSED_M:g} m)'
    if d_leg < CLOSE_M:
        return f'the last waypoint is {d_leg:.2f} m from an earlier leg of the route'
    return ''


def chunks(route, start=None):
    """Cut the route into stretches none of which ends on itself (route_is_closed), each
    starting with the one before's last waypoint so she still goes there. Only a stretch's
    END matters - crossing your own path or a figure-eight is one stretch. Cut as LATE as
    possible: the first stretch reaches as far as its last waypoint stays clear of its own
    earlier legs, and so on. (Until 2026-10-01 evening this worked back from the end, which
    made the FIRST stretch the short one and put the handover - a stop of ~9 s while Nav2
    plans the next stretch from scratch - at the island on every lap.) `start` (x, y) is
    where she is: the leg from there to the first waypoint counts for the first stretch."""
    head = [(start[0], start[1], 0.0)] if start is not None else []
    out = []
    begin = 0
    while begin < len(route):
        end = len(route)
        while end - begin > 1 and route_is_closed((head if begin == 0 else []) + route[begin:end]):
            end -= 1
        out.append(route[begin:end])
        if end == len(route):
            break
        # the next stretch starts where this one ends - unless this one is a single waypoint,
        # which cannot be shared or the loop never moves (the audit of 2026-10-02 reproduced it
        # with [(3, 0), (0.2, 0)] from (0, 0): the second point sits on the start leg)
        begin = end - 1 if end - begin > 1 else end
    return out


DETOUR_FACTOR, DETOUR_PLUS_M, DETOUR_TRIES, DETOUR_WAIT_S = 1.4, 2.0, 6, 5.0   # ~30 s for a person to step aside


def remaining_straight(pose, remaining):
    """Metres from `pose` through the remaining waypoints as the crow flies."""
    x, y = pose[0], pose[1]
    d = 0.0
    for wx, wy, _ in remaining:
        d += math.hypot(wx - x, wy - y)
        x, y = wx, wy
    return d


def is_detour(path_m, straight_m):
    """Nav2's path is the long way round (drive 14, 2026-10-01: a transient block on the south
    side of the table, and the Through planner sent her 19 m round the north side for an 11 m
    stretch, into a chair leg at the far corner). A Reeds-Shepp path runs ~1.15x the straight
    line; 1.4x plus 2 m is a detour."""
    return path_m == path_m and path_m > DETOUR_FACTOR * straight_m + DETOUR_PLUS_M


def pose_msg(x, y, h, stamp=None):
    p = PoseStamped()
    p.header.frame_id = 'map'
    if stamp is not None:
        p.header.stamp = stamp
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(h / 2), math.cos(h / 2)
    return p


def main():
    argv = sys.argv[1:]
    check_only = '--check' in argv
    home = '--home' in argv
    timeout = None
    if '--timeout' in argv:
        i = argv.index('--timeout')
        timeout = float(argv[i + 1])
        del argv[i:i + 2]
    route = parse_route([a for a in argv if not a.startswith('--')])
    if timeout is None:
        timeout = 60.0 * len(route)

    rclpy.init()
    n = rclpy.create_node('nav_route')
    st = {'cmd': [], 'lock': None, 'grid': None, 'where': None, 'battery': None}
    n.create_subscription(Twist, '/cmd_vel_nav', lambda m: st['cmd'].append((m.linear.x, m.angular.z)), 20)
    n.create_subscription(Bool, '/e_stop_motion', lambda m: st.__setitem__('lock', m.data), 10)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(String, '/where_am_i/state', lambda m: st.__setitem__('where', m.data),
                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    n.create_subscription(String, '/battery/level', lambda m: st.__setitem__('battery', m.data),
                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    costmap_sub = n.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                        lambda m: st.__setitem__('grid', m), latched)
    # the tree's PlannerSelector listens on a latched topic: say it before the goal, every time
    planner_pub = n.create_publisher(String, 'planner_selector',
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def select_planner(name):
        m = String()
        m.data = name
        for _ in range(3):
            planner_pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.1)

    client = ActionClient(n, NavigateThroughPoses, 'navigate_through_poses')
    if not client.wait_for_server(timeout_sec=10.0):
        raise SystemExit('navigate_through_poses action server not available')

    def spin_for(seconds, until=lambda: False):
        end = time.time() + seconds
        while time.time() < end and not until():
            rclpy.spin_once(n, timeout_sec=0.05)

    # the same gate as nav_goal: bench, battery, the motion check's lock
    spin_for(1.0, lambda: st['where'] is not None)
    try:
        where = json.loads(st['where'] or '{}')
    except ValueError:
        where = {}
    if where.get('state') == 'bench':
        raise SystemExit(f'she is on the bench ({where.get("why", "")}): not sending the route')
    spin_for(1.0, lambda: st['battery'] is not None)
    allowed, note = battery_verdict(st['battery'], home)
    if note:
        print(note, flush=True)
    if not allowed:
        raise SystemExit('not sending the route')
    spin_for(1.5, lambda: st['lock'] is not None)
    if st['lock']:
        print('the motion check has her locked: waiting for it to be released', flush=True)
        spin_for(20.0, lambda: st['lock'] is False)
        if st['lock']:
            raise SystemExit('still locked after 20 s: not sending the route')

    # every waypoint somewhere she fits, on the planner's live costmap
    live = n.create_client(GetCostmap, '/global_costmap/get_costmap')
    if live.wait_for_service(timeout_sec=3.0):
        fut = live.call_async(GetCostmap.Request())
        spin_for(5.0, fut.done)
        if fut.done() and fut.result() is not None:
            st['grid'] = grid_from_costmap(fut.result().map)
    if st['grid'] is None:
        spin_for(5.0, lambda: st['grid'] is not None)
    n.destroy_subscription(costmap_sub)
    if st['grid'] is None:
        print('no costmap from the planner in 5 s: route not checked', flush=True)
    else:
        checked = []
        for k, (x, y, h) in enumerate(route, 1):
            found = clear_goal(st['grid'], x, y)
            if found is None:
                raise SystemExit(f'waypoint {k} ({x:+.2f}, {y:+.2f}): nowhere within {SEARCH_M:g} m is clear '
                                 'of the furniture for her: route refused')
            if found[2] > 0.0:
                print(f'waypoint {k} ({x:+.2f}, {y:+.2f}) is inside a furniture margin: moved {found[2] * 100:.0f} cm '
                      f'to ({found[0]:+.2f}, {found[1]:+.2f})', flush=True)
            checked.append((found[0], found[1], h))
        route = checked
        print(f'{len(route)} waypoints, all clear of the furniture', flush=True)
    if check_only:
        rclpy.shutdown()
        return

    def tf_pose():
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

    # nav_helper watches the goal: the route's end
    goal_pub = n.create_publisher(PoseStamped, 'nav_helper/goal', 10)
    for _ in range(3):
        goal_pub.publish(pose_msg(*route[-1]))
        spin_for(0.1)

    start = tf_pose()
    if start is None:
        raise SystemExit('no map -> base_footprint transform after 15 s')
    gx, gy, gh = route[-1]
    print(f'start  x {start[0]:+.2f}  y {start[1]:+.2f}  heading {math.degrees(start[2]):+.0f} deg; '
          f'route of {len(route)} to x {gx:+.2f}  y {gy:+.2f}  heading {math.degrees(gh):+.0f} deg', flush=True)

    stretches = chunks(route, (start[0], start[1]))
    if len(stretches) > 1:
        print(f'the route ends where she drives earlier: sent as {len(stretches)} stretches, '
              + ', '.join(f'{len(c)}' for c in stretches) + ' waypoints, swapped on the move', flush=True)
    passed = []                      # (waypoint number, seconds) as the tree drops each one
    fb = {'dist': float('nan'), 'pose': None, 'left': 0, 'goal': None}
    done_before = 0                  # waypoints of earlier stretches (their last is the next one's first)

    def on_feedback(f):
        if fb['goal'] is None or bytes(f.goal_id.uuid) != fb['goal']:
            return                   # the stretch before, preempted: its last words do not count
        fb['dist'] = f.feedback.distance_remaining
        p = f.feedback.current_pose.pose
        fb['pose'] = (p.position.x, p.position.y, yaw(p.orientation))
        left = f.feedback.number_of_poses_remaining
        while left < fb['left']:
            fb['left'] -= 1
            passed.append((done_before + len(stretch) - fb['left'], time.time() - t0))

    def send_stretch(stretch):
        goal = NavigateThroughPoses.Goal()
        stamp = n.get_clock().now().to_msg()
        goal.poses = [pose_msg(x, y, h, stamp) for x, y, h in stretch]
        fb['left'], fb['goal'], fb['dist'] = len(stretch), None, float('nan')   # no stale distance (drive 15)
        send = client.send_goal_async(goal, feedback_callback=on_feedback)
        while not send.done():
            rclpy.spin_once(n, timeout_sec=0.1)
        handle = send.result()
        if not handle.accepted:
            raise SystemExit('result REJECTED')
        fb['goal'] = bytes(handle.goal_id.uuid)
        return handle, handle.get_result_async()

    st['cmd'] = []
    t0 = time.time()
    last = 0.0
    said = 0
    locked_during = False
    status = GoalStatus.STATUS_ABORTED
    def cancel_goal(handle):
        cancel = handle.cancel_goal_async()
        while not cancel.done():
            rclpy.spin_once(n, timeout_sec=0.1)

    def clear_costmaps():
        from nav2_msgs.srv import ClearEntireCostmap
        for name in ('/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap'):
            cli = n.create_client(ClearEntireCostmap, name)
            if cli.wait_for_service(timeout_sec=2.0):
                fut = cli.call_async(ClearEntireCostmap.Request())
                rclpy.spin_until_future_complete(n, fut, timeout_sec=3.0)
            n.destroy_client(cli)

    # pictures whenever she stops or is sent round (Steve, 2026-10-01: "maybe a cat, they are all
    # running around - if it ever stops or diverts or detours then take some pics"): nav_helper's
    # ~/snapshot (the house map round her, a close-up, the camera sweep) into its out_dir/<time>/,
    # asked for without waiting so the loop keeps watching her; at most one per stop, 6 per route
    from std_srvs.srv import Trigger
    snap_cli = n.create_client(Trigger, '/nav_helper/snapshot')
    snaps = {'pending': None, 'why': '', 'n': 0}

    def snap(why):
        if snaps['pending'] is not None or snaps['n'] >= 6:
            return
        if not snap_cli.service_is_ready():
            print(f'   (no nav_helper for pictures: {why})', flush=True)
            snaps['n'] = 99
            return
        snaps['pending'], snaps['why'] = snap_cli.call_async(Trigger.Request()), why
        snaps['n'] += 1

    def snap_poll():
        f = snaps['pending']
        if f is not None and f.done():
            snaps['pending'] = None
            try:
                r = f.result()
                d = json.loads(r.message).get('dir', '?') if r.message else '?'
                print(f'   pictures ({snaps["why"]}): {d}' + ('' if r.success else ' - none taken'), flush=True)
            except Exception as exc:  # noqa: BLE001
                print(f'   pictures ({snaps["why"]}): failed ({str(exc)[:60]})', flush=True)

    still = {'pose': None, 'since': None, 'snapped': False}
    select_planner('Through')
    detours = 0
    for k, stretch in enumerate(stretches):
        while True:                      # once, or again after a detour with the waypoints still to go
            handle, res = send_stretch(stretch)
            handed_over = detour = False
            while not res.done():
                rclpy.spin_once(n, timeout_sec=0.05)
                locked_during = locked_during or bool(st['lock'])
                while said < len(passed):
                    num, s_ = passed[said]
                    said += 1
                    print(f'{s_:5.1f} s  passed waypoint {num}', flush=True)
                if fb['left'] <= 1 and k + 1 < len(stretches):
                    handed_over = True       # only this stretch's last is left: the next one starts with it
                    break
                if time.time() - last >= 1.0:
                    last = time.time()
                    p = fb['pose'] or start
                    c = st['cmd'][-1] if st['cmd'] else (0.0, 0.0)
                    print(f'{time.time() - t0:5.1f} s  at x {p[0]:+.2f} y {p[1]:+.2f} h {math.degrees(p[2]):+4.0f}  '
                          f'{len(route) - len(passed)} to go, '
                          f'{fb["dist"]:.2f} m  driver: throttle {c[0]:+.2f} steer {c[1]:+.2f}'
                          + ('  MOTION LOCK' if st['lock'] else ''), flush=True)
                    snap_poll()
                    # standing still 3 s with the route live (not the motion lock's doing): pictures
                    if fb['pose']:
                        moved = still['pose'] is None or math.hypot(p[0] - still['pose'][0], p[1] - still['pose'][1]) > 0.05
                        if moved:
                            still.update(pose=p, since=time.time(), snapped=False)
                        elif not still['snapped'] and not st['lock'] and time.time() - still['since'] >= 3.0:
                            still['snapped'] = True
                            print(f'{time.time() - t0:5.1f} s  stopped {time.time() - still["since"]:.0f} s at '
                                  f'({p[0]:+.2f}, {p[1]:+.2f}): taking pictures', flush=True)
                            snap(f'stopped at {p[0]:+.2f}, {p[1]:+.2f}')
                    remaining = stretch[len(stretch) - fb['left']:] if 0 < fb['left'] <= len(stretch) else []
                    if fb['pose'] and remaining and is_detour(fb['dist'], remaining_straight(fb['pose'], remaining)):
                        straight = remaining_straight(fb['pose'], remaining)
                        detours += 1
                        print(f'{time.time() - t0:5.1f} s  DETOUR: Nav2 plans {fb["dist"]:.1f} m where the way is '
                              f'{straight:.1f} m - cancelling (try {detours} of {DETOUR_TRIES})', flush=True)
                        if detours == 1:
                            snap(f'detour at {fb["pose"][0]:+.2f}, {fb["pose"][1]:+.2f}')
                        cancel_goal(handle)
                        detour = True
                        break
                if time.time() - t0 > timeout:
                    print('timeout: cancelling the route', flush=True)
                    cancel_goal(handle)
                    break
            if detour:
                if detours >= DETOUR_TRIES:
                    print('the way stays blocked: giving up', flush=True)
                    status = GoalStatus.STATUS_ABORTED
                    break
                if detours == 3:
                    print('   clearing the costmaps', flush=True)
                    clear_costmaps()
                end = time.time() + DETOUR_WAIT_S   # time for the block to pass or the costmap to settle
                while time.time() < end:
                    rclpy.spin_once(n, timeout_sec=0.1)
                done_before += len(stretch) - len(remaining)
                stretch = remaining          # the same stretch, the waypoints still to go
                continue
            break
        if detour:
            break
        if handed_over:
            done_before += len(stretch) - 1
            continue
        status = res.result().status if res.done() else GoalStatus.STATUS_CANCELED
        if status != GoalStatus.STATUS_SUCCEEDED or time.time() - t0 > timeout:
            break
    select_planner('GridBased')                            # nav_goal and nav_park: exact headings
    if status == GoalStatus.STATUS_SUCCEEDED:
        passed.append((len(route), time.time() - t0))     # the last one is reached, not passed
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
          + f'; passed {len(passed)} of {len(route)} waypoints; ended {math.hypot(p[0] - gx, p[1] - gy) * 100:.0f} cm '
          f'from the last, heading off by {dh:+.0f} deg; '
          f'driver got {len(st["cmd"])} commands, throttle {min(thr, default=0):.2f}-{max(thr, default=0):.2f}; '
          f'forward/reverse switches {switches}', flush=True)
    rclpy.shutdown()
    sys.exit(0 if status == GoalStatus.STATUS_SUCCEEDED else 1)


if __name__ == '__main__':
    main()
