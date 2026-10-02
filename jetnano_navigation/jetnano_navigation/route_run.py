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

"""A route (a lap) run by the mission controller: nav_route's behaviour as a state machine.

What nav_route.py did in a blocking loop (2026-09-30 .. 10-02, floor-proven on drives
13-19) happens here in callbacks and the controller's tick, so the controller stays
responsive to the safety gate and to commands:
- every waypoint checked against the planner's live costmap (nav_goal.clear_goal), moved
  to the nearest clear spot or the route refused;
- the route cut into stretches that do not end on themselves (nav_route.chunks), the next
  stretch sent when only the current one's last waypoint is left - swapped on the move;
- the detour guard: Nav2 planning far more than the way (nav_route.is_detour) -> cancel,
  wait DETOUR_WAIT_S, costmaps cleared on the third try, the waypoints still to go sent
  again; six tries, then the route fails;
- pictures (nav_helper ~/snapshot) on a 3 s stop or the first detour, at most six;
- a line a second with where she is and what the driver gets, the passed waypoints, and
  the result line nav_route printed.
The gate's holds are the controller's: it calls hold() (cancel, keep the rest) and
resume() (send the rest again).

Phases: checking (the costmap) -> starting (the start pose from TF) -> sending -> driving
-> [detour wait -> sending ...] | [held -> sending ...] -> done.
Nothing here spins: every wait is a future or a deadline looked at on tick().
"""

import json
import math
import time

from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.srv import GetCostmap
from std_srvs.srv import Trigger

from jetnano_navigation.nav_goal import SEARCH_M, clear_goal, grid_from_costmap, yaw
from jetnano_navigation.nav_route import (DETOUR_TRIES, DETOUR_WAIT_S, chunks, is_detour, pose_msg,
                                          remaining_straight)

STILL_M = 0.05          # not moved this far = standing still
STILL_S = 3.0           # standing still this long with the route live -> pictures
SNAPS_MAX = 6


class RouteRun:
    """One route, from the first costmap check to the result line.

    The controller (mission.Mission) provides: through (ActionClient), map_pose(), goal_watch(),
    select_planner(), clear_costmaps(), snap_cli (Trigger client), costmap_cli (GetCostmap
    client), and calls start(), tick(now, lock, cmds), hold(), resume(), abandon(); it reads
    .phase, .done, .outcome, .line, .feedback()."""

    def __init__(self, node, waypoints, timeout_s, say):
        self.n = node
        self.route = [(float(x), float(y), math.radians(float(h))) for x, y, h in waypoints]
        self.timeout_s = float(timeout_s)
        self.say = say                    # a line for the log and the client (what nav_route printed)
        self.t0 = time.monotonic()
        self.done = False
        self.outcome = None
        self.line = ''
        self.phase = 'checking'
        self.deadline = None
        self.costmap_fut = None
        self.start_pose = None
        self.stretches = []
        self.k = 0                        # the stretch in hand
        self.stretch = []
        self.done_before = 0              # waypoints of earlier stretches (their last is the next one's first)
        self.passed = []                  # (waypoint number, seconds) as the tree drops each one
        self.fb = {'dist': float('nan'), 'pose': None, 'left': 0, 'goal': None}
        self.handle = None
        self.detours = 0
        self.pending = None               # the waypoints to send again after a detour wait or a hold
        self.after_cancel = None          # 'detour' | 'hold' while our own cancel is confirmed by Nav2
        self.still = {'pose': None, 'since': None, 'snapped': False}
        self.snaps = {'pending': None, 'why': '', 'n': 0}
        self.last_line = 0.0
        self.cmds = []                    # (throttle, steer) the driver got, for the result line
        self.locked_during = False

    # ---------------------------------------------------------------- start --

    def start(self):
        """Ask the planner for its live costmap; the rest follows on tick()."""
        now = time.monotonic()
        if self.n.costmap_cli.service_is_ready():
            self.costmap_fut = self.n.costmap_cli.call_async(GetCostmap.Request())
            self.deadline = now + 5.0
        else:
            self.say('no costmap service from the planner: route not checked')
            self._checked(None)

    def _checked(self, grid):
        if grid is None:
            if self.costmap_fut is not None:
                self.say('no costmap from the planner in 5 s: route not checked')
        else:
            checked = []
            for k, (x, y, h) in enumerate(self.route, 1):
                found = clear_goal(grid, x, y)
                if found is None:
                    return self.finish('REFUSED', f'waypoint {k} ({x:+.2f}, {y:+.2f}): nowhere within {SEARCH_M:g} m '
                                                  'is clear of the furniture')
                if found[2] > 0.0:
                    self.say(f'waypoint {k} ({x:+.2f}, {y:+.2f}) is inside a furniture margin: moved {found[2] * 100:.0f} cm '
                             f'to ({found[0]:+.2f}, {found[1]:+.2f})')
                checked.append((found[0], found[1], h))
            self.route = checked
            self.say(f'{len(self.route)} waypoints, all clear of the furniture')
        self.phase = 'starting'
        self.deadline = time.monotonic() + 5.0

    def _started(self, pose):
        self.start_pose = pose
        gx, gy, gh = self.route[-1]
        self.say(f'start  x {pose[0]:+.2f}  y {pose[1]:+.2f}  heading {math.degrees(pose[2]):+.0f} deg; '
                 f'route of {len(self.route)} to x {gx:+.2f}  y {gy:+.2f}  heading {math.degrees(gh):+.0f} deg')
        self.stretches = chunks(self.route, (pose[0], pose[1]))
        if len(self.stretches) > 1:
            self.say(f'the route ends where she drives earlier: sent as {len(self.stretches)} stretches, '
                     + ', '.join(f'{len(c)}' for c in self.stretches) + ' waypoints, swapped on the move')
        self.n.goal_watch(self.route[-1])
        self.n.select_planner('Through')
        self.k = 0
        self.send(self.stretches[0])

    # ------------------------------------------------------------- stretches --

    def send(self, stretch):
        """A stretch (or what is left of one) to Nav2; a goal already running is preempted."""
        self.stretch = list(stretch)
        self.fb.update(left=len(self.stretch), goal=None, dist=float('nan'))   # no stale distance (drive 15)
        self.phase = 'sending'
        self.handle = None
        goal = NavigateThroughPoses.Goal()
        stamp = self.n.get_clock().now().to_msg()
        goal.poses = [pose_msg(x, y, h, stamp) for x, y, h in self.stretch]
        fut = self.n.through.send_goal_async(goal, feedback_callback=self.on_feedback)
        fut.add_done_callback(self._accepted)

    def _accepted(self, fut):
        handle = fut.result()
        if self.done or self.phase != 'sending':
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()                # we moved on (held, abandoned) while Nav2 was accepting
            return
        if handle is None or not handle.accepted:
            return self.finish('REJECTED', 'Nav2 rejected the stretch')
        self.handle = handle
        self.fb['goal'] = bytes(handle.goal_id.uuid)
        self.phase = 'driving'
        handle.get_result_async().add_done_callback(lambda f: self.on_result(handle, f))

    def remaining(self):
        """The waypoints of the current stretch still to go."""
        left = self.fb['left']
        return self.stretch[len(self.stretch) - left:] if 0 < left <= len(self.stretch) else list(self.stretch)

    def on_feedback(self, f):
        if self.fb['goal'] is None or bytes(f.goal_id.uuid) != self.fb['goal']:
            return                   # the stretch before, preempted: its last words do not count
        self.fb['dist'] = f.feedback.distance_remaining
        p = f.feedback.current_pose.pose
        self.fb['pose'] = (p.position.x, p.position.y, yaw(p.orientation))
        left = f.feedback.number_of_poses_remaining
        while left < self.fb['left']:
            self.fb['left'] -= 1
            num = self.done_before + len(self.stretch) - self.fb['left']
            self.passed.append((num, time.monotonic() - self.t0))
            self.say(f'{time.monotonic() - self.t0:5.1f} s  passed waypoint {num}')

    def on_result(self, handle, fut):
        if self.done or handle is not self.handle:
            return                   # an earlier stretch's result (preempted, or after a cancel): not ours
        from action_msgs.msg import GoalStatus
        status = fut.result().status
        if self.after_cancel is not None:                 # our own cancel (a detour, or the gate)
            why, self.after_cancel = self.after_cancel, None
            self.handle = None
            if why == 'hold':
                self.phase = 'held'
            else:
                self.phase = 'detour wait'                # time for the block to pass or the costmap to settle
                self.deadline = time.monotonic() + DETOUR_WAIT_S
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.passed.append((len(self.route), time.monotonic() - self.t0))   # the last is reached, not passed
            return self.finish('SUCCEEDED', '')
        names = {GoalStatus.STATUS_ABORTED: 'ABORTED', GoalStatus.STATUS_CANCELED: 'CANCELED'}
        self.finish(names.get(status, str(status)), '')

    def _cancel_then(self, why, pending):
        """Cancel the running stretch; when Nav2 confirms, wait (detour) or hold with `pending` to go."""
        self.pending = pending
        if self.handle is None:                           # still being accepted: nothing to cancel yet
            self.phase = 'held' if why == 'hold' else 'detour wait'
            self.deadline = time.monotonic() + DETOUR_WAIT_S
            return
        self.after_cancel = why
        self.phase = 'cancelling'
        self.handle.cancel_goal_async()

    def hold(self):
        """The gate stopped her: cancel, keep what is left for resume()."""
        self.locked_during = True
        if self.phase in ('checking', 'starting'):
            return self.finish('REFUSED', 'the safety gate stopped her before the start')
        self._cancel_then('hold', self.remaining())

    def resume(self):
        if self.phase == 'held' and self.pending:
            self.done_before += len(self.stretch) - len(self.pending)
            self.send(self.pending)
            self.pending = None

    def abandon(self, note):
        """Cancelled on request or by a new command: Nav2 told, the route finished."""
        self.after_cancel = None
        if self.handle is not None:
            self.handle.cancel_goal_async()
        self.finish('CANCELED', note)

    # ----------------------------------------------------------------- tick --

    def tick(self, now, lock, cmds):
        if self.done:
            return
        self.cmds.extend(cmds)
        self.locked_during = self.locked_during or bool(lock)
        if self.phase == 'checking':
            if self.costmap_fut.done():
                r = self.costmap_fut.result()
                self._checked(grid_from_costmap(r.map) if r is not None else None)
            elif now > self.deadline:
                self._checked(None)
            return
        if self.phase == 'starting':
            pose = self.n.map_pose()
            if pose is not None:
                self._started(pose)
            elif now > self.deadline:
                self.finish('REFUSED', 'no map -> base_footprint transform')
            return
        if self.phase == 'detour wait':
            if now >= self.deadline:
                self.done_before += len(self.stretch) - len(self.pending)
                self.send(self.pending)                   # the same stretch, the waypoints still to go
                self.pending = None
            return
        if self.phase not in ('driving', 'sending', 'cancelling'):
            return
        if now - self.t0 > self.timeout_s:
            self.say('timeout: cancelling the route')
            return self.abandon(f'timeout after {self.timeout_s:.0f} s')
        if self.phase == 'driving' and self.fb['left'] <= 1 and self.k + 1 < len(self.stretches):
            self.done_before += len(self.stretch) - 1     # only this stretch's last is left: the next starts with it
            self.k += 1
            self.send(self.stretches[self.k])
            return
        if now - self.last_line < 1.0:
            return
        self.last_line = now
        p = self.fb['pose'] or self.start_pose
        c = self.cmds[-1] if self.cmds else (0.0, 0.0)
        self.say(f'{now - self.t0:5.1f} s  at x {p[0]:+.2f} y {p[1]:+.2f} h {math.degrees(p[2]):+4.0f}  '
                 f'{len(self.route) - len(self.passed)} to go, {self.fb["dist"]:.2f} m  '
                 f'driver: throttle {c[0]:+.2f} steer {c[1]:+.2f}' + ('  MOTION LOCK' if lock else ''))
        self._snap_poll()
        if self.phase != 'driving':
            return
        # standing still 3 s with the route live (not the motion lock's doing): pictures
        if self.fb['pose']:
            moved = self.still['pose'] is None or math.hypot(p[0] - self.still['pose'][0], p[1] - self.still['pose'][1]) > STILL_M
            if moved:
                self.still.update(pose=p, since=now, snapped=False)
            elif not self.still['snapped'] and not lock and now - self.still['since'] >= STILL_S:
                self.still['snapped'] = True
                self.say(f'{now - self.t0:5.1f} s  stopped {now - self.still["since"]:.0f} s at ({p[0]:+.2f}, {p[1]:+.2f}): '
                         'taking pictures')
                self._snap(f'stopped at {p[0]:+.2f}, {p[1]:+.2f}')
        remaining = self.remaining()
        if self.fb['pose'] and remaining and is_detour(self.fb['dist'], remaining_straight(self.fb['pose'], remaining)):
            straight = remaining_straight(self.fb['pose'], remaining)
            self.detours += 1
            self.say(f'{now - self.t0:5.1f} s  DETOUR: Nav2 plans {self.fb["dist"]:.1f} m where the way is {straight:.1f} m '
                     f'- cancelling (try {self.detours} of {DETOUR_TRIES})')
            if self.detours == 1:
                self._snap(f'detour at {p[0]:+.2f}, {p[1]:+.2f}')
            if self.detours >= DETOUR_TRIES:
                self.say('the way stays blocked: giving up')
                return self.abandon('the way stays blocked')
            if self.detours == 3:
                self.say('   clearing the costmaps')
                self.n.clear_costmaps()
            self._cancel_then('detour', remaining)

    def feedback(self):
        """What the controller's status shows."""
        p = self.fb['pose']
        out = {'dist_m': round(self.fb['dist'], 2) if self.fb['dist'] == self.fb['dist'] else None,
               'left': len(self.route) - len(self.passed), 'stretch': self.k + 1, 'stretches': len(self.stretches),
               'detours': self.detours}
        if p:
            out.update(x=round(p[0], 2), y=round(p[1], 2), h_deg=round(math.degrees(p[2])))
        return out

    # ------------------------------------------------------------- pictures --

    def _snap(self, why):
        s = self.snaps
        if s['pending'] is not None or s['n'] >= SNAPS_MAX:
            return
        if not self.n.snap_cli.service_is_ready():
            self.say(f'   (no nav_helper for pictures: {why})')
            s['n'] = 99
            return
        s['pending'], s['why'] = self.n.snap_cli.call_async(Trigger.Request()), why
        s['n'] += 1

    def _snap_poll(self):
        f = self.snaps['pending']
        if f is not None and f.done():
            self.snaps['pending'] = None
            try:
                r = f.result()
                d = json.loads(r.message).get('dir', '?') if r.message else '?'
                self.say(f'   pictures ({self.snaps["why"]}): {d}' + ('' if r.success else ' - none taken'))
            except Exception as exc:  # noqa: BLE001
                self.say(f'   pictures ({self.snaps["why"]}): failed ({str(exc)[:60]})')

    # --------------------------------------------------------------- result --

    def finish(self, outcome, note):
        if self.done:
            return
        self.done, self.outcome, self.phase = True, outcome, 'done'
        p = self.n.map_pose() or self.fb['pose'] or self.start_pose
        gx, gy, gh = self.route[-1]
        thr = [abs(c[0]) for c in self.cmds if c[0] != 0]
        signs = [1 if c[0] > 0 else -1 for c in self.cmds if abs(c[0]) > 0.01]
        switches = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
        where = ''
        if p is not None:
            dh = math.degrees(math.atan2(math.sin(p[2] - gh), math.cos(p[2] - gh)))
            where = f'; ended {math.hypot(p[0] - gx, p[1] - gy) * 100:.0f} cm from the last, heading off by {dh:+.0f} deg'
        self.line = (f'result {outcome} after {time.monotonic() - self.t0:.1f} s'
                     + (' (the motion check stopped her)' if self.locked_during else '')
                     + (f'; {note}' if note else '')
                     + f'; passed {len(self.passed)} of {len(self.route)} waypoints' + where
                     + f'; driver got {len(self.cmds)} commands, throttle {min(thr, default=0):.2f}-{max(thr, default=0):.2f}'
                     + f'; forward/reverse switches {switches}')
        self.say(self.line)
        if self.start_pose is not None:
            self.n.select_planner('GridBased')            # nav_goal and nav_park: exact headings
