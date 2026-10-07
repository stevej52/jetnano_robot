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
  the result line nav_route printed;
- the rescue when Nav2 gives up on a waypoint (nav_goal --rescue, inside the lap since
  2026-10-02): back out along her track and retry, ask Claude with her pictures (at most
  RESCUE_ASKS), do as advised (wait, via, retrace), else skip that waypoint and go on.
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

RESCUE_ASKS = 2         # asks of Claude per route (nav_goal --rescue: the same)
RETRACE_M = 0.8         # back out this far along her own track before the first retry
ASK_WAIT_S = 200.0      # the pictures, the brain, the answer
HANDOFF_M = 0.40        # this far before the lap's end Nav2 hands over to the parking, still rolling
HANDOFF_SLACK_M = 0.35  # ... and her pose within HANDOFF_M + this of the end (straight line; the path curves)
PARK_WAIT_S = 120.0     # the parking reports within this, or the route ends without it
STALL_S = 60.0          # no 0.25 m closer to the stretch's end in this long = stuck: the rescue starts
STALL_GAIN_M = 0.25     # (house-tour sim 2026-10-05: Nav2 can loop on recoveries without ever giving up)
STILL_M = 0.05          # not moved this far = standing still
STILL_S = 3.0           # standing still this long with the route live -> pictures
SNAPS_MAX = 6
# a bump (2026-10-06, drives 43-46: the chair, the robot vacuum - pushed 3-4 times in a row): the
# motion check stops her when she pushes without moving; the spot she pushed at goes on the
# planner's map as a small box (grid_to_points, 10 min) and the route carries on round it
BUMP_AHEAD_M = 0.40      # the box's centre from hers, the way she was pushing (her nose is 0.22)
BUMPS_MAX = 3            # more than this in one route: give up
BUMP_WAIT_S = 60.0       # the motion check must let go within this


class RouteRun:
    """One route, from the first costmap check to the result line.

    The controller (mission.Mission) provides: through (ActionClient), map_pose(), goal_watch(),
    select_planner(), clear_costmaps(), snap_cli (Trigger client), costmap_cli (GetCostmap
    client), and calls start(), tick(now, lock, cmds), hold(), resume(), abandon(); it reads
    .phase, .done, .outcome, .line, .feedback()."""

    def __init__(self, node, waypoints, timeout_s, say, then_park=None):
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
        self.best_dist, self.best_t = float('inf'), time.monotonic()   # the stall rule
        self.handle = None
        self.detours = 0
        self.pending = None               # the waypoints to send again after a detour wait or a hold
        self.after_cancel = None          # 'detour' | 'hold' while our own cancel is confirmed by Nav2
        self.still = {'pose': None, 'since': None, 'snapped': False}
        self.snaps = {'pending': None, 'why': '', 'n': 0}
        self.last_line = 0.0
        self.cmds = []                    # (throttle, steer) the driver got, for the result line
        self.locked_during = False
        # the parking after the lap (2026-10-04): handed over 0.4 m before the lap's end, rolling,
        # to nav_park --serve - the lap no longer stops at the arc's start (drives 31-32: ~6 s)
        self.then_park = then_park
        self.park_t0 = None
        self.park_outcome = None
        # the rescue (nav_goal --rescue, inside the lap since 2026-10-02 drive 23: an open
        # dishwasher aborted the lap, and the rescue run by hand went round the island on
        # Claude's advice): Nav2 gives up -> back out along her track and try again -> ask
        # Claude with her pictures -> wait / via / retrace as advised, or skip the blocked
        # waypoint and carry on; the lap only fails when nothing is left to go for
        self.rescue = {'retraced': False, 'asks': 0, 'skipped': 0, 'future': None, 'advice': None}
        self.via_first = False            # the stretch in hand starts with a via point, not a waypoint
        self.skipped = []                 # waypoint numbers the rescue gave up on
        self.rescue_return = None         # the rescue phase to go back to after a gate hold
        self.bumps = 0

    # ---------------------------------------------------------------- start --

    def start(self):
        """Ask the planner for its live costmap; the rest follows on tick()."""
        now = time.monotonic()
        self.n.tf_on()                    # the map pose is needed from here to the result line
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
        self.best_dist, self.best_t = float('inf'), time.monotonic()           # a fresh stretch: a fresh stall clock
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

    def _next_overlaps(self):
        """True when the next stretch begins with this one's last waypoint, so Nav2 can swap goals
        on the move. A stretch that does NOT overlap (a lone waypoint, an out-and-back) must be
        driven to its end first (review 2026-10-03: it used to be replaced at once)."""
        if self.k + 1 >= len(self.stretches) or not self.stretch:
            return False
        nxt = self.stretches[self.k + 1][0]
        last = self.stretch[-1]
        return abs(nxt[0] - last[0]) < 1e-6 and abs(nxt[1] - last[1]) < 1e-6

    def _near_end(self):
        """Nav2's distance_remaining reads 0 while it has no path (drive 48, 2026-10-07: the planner
        said "start occupied" beside Steve's chair and the parking took over 3.4 m early, where it
        could not plan either): hand over only when her own pose is near the route's end too."""
        pose = self.fb.get('pose')
        if pose is None or not self.route:
            return False
        end = self.route[-1]
        return math.hypot(pose[0] - end[0], pose[1] - end[1]) <= HANDOFF_M + HANDOFF_SLACK_M

    def _hand_over_to_parking(self, now):
        """The lap's end is the parking arc's start: the parking takes over while she rolls."""
        self.passed.append((len(self.route), now - self.t0))           # reached, within HANDOFF_M
        self.say(f'{now - self.t0:5.1f} s  handing over to the parking {self.fb["dist"]:.2f} m before the end, rolling')
        self.n.park(dict(self.then_park, rolling=True))
        self.park_t0 = now
        self.after_cancel = 'park'
        if self.handle is not None:
            self.handle.cancel_goal_async()
        self.phase = 'parking'

    def on_park_log(self, line):
        if self.phase == 'parking':
            self.say('park: ' + line)

    def on_park_result(self, r):
        if self.phase != 'parking':
            return
        self.park_outcome = r.get('outcome')
        self.say(f'park: took {r.get("seconds", "?")} s')
        self.finish('SUCCEEDED', f'parking {r.get("outcome", "?")}: {r.get("line", "")}')

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
            if self.via_first:
                self.via_first = False                   # Claude's via point passed: not a waypoint
                continue
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
                if self.n.lock:                           # the gate's hold is the motion check: a bump
                    self._mark_bump()
            elif why == 'park':
                return                                    # handed over: the parking's result decides
            elif why == 'rescue':
                self.phase = 'driving'                    # so _rescue counts the stretch as the one in hand
                return self._rescue('the way stayed blocked')
            else:
                self.phase = 'detour wait'                # time for the block to pass or the costmap to settle
                self.deadline = time.monotonic() + DETOUR_WAIT_S
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.k + 1 < len(self.stretches):                 # a stretch that did not overlap: its end reached
                num = self.done_before + len(self.stretch)
                self.passed.append((num, time.monotonic() - self.t0))
                self.say(f'{time.monotonic() - self.t0:5.1f} s  reached waypoint {num}')
                self.done_before += len(self.stretch)
                self.k += 1
                self.handle = None
                self.send(self.stretches[self.k])
                return
            self.passed.append((len(self.route), time.monotonic() - self.t0))   # the last is reached, not passed
            return self.finish('SUCCEEDED', '')
        if status == GoalStatus.STATUS_ABORTED and self.phase == 'driving':
            self.handle = None
            return self._rescue('Nav2 gave up')
        if status == GoalStatus.STATUS_CANCELED and self.n.lock:
            # the motion check cancelled Nav2's goal itself (safety_monitor) before the gate's
            # hold reached the controller: drive 46 ended the route here
            self.handle = None
            self.pending = self.remaining()
            if not self._mark_bump():
                return
            self.phase = 'bump wait'
            self.deadline = time.monotonic() + BUMP_WAIT_S
            return
        names = {GoalStatus.STATUS_ABORTED: 'ABORTED', GoalStatus.STATUS_CANCELED: 'CANCELED'}
        self.finish(names.get(status, str(status)), '')

    # ---------------------------------------------------------------- bumps --

    def _mark_bump(self):
        """She pushed and did not move: put the spot she pushed at on the planner's map.
        -> False when that was one bump too many (the route is finished)."""
        self.bumps += 1
        if self.bumps > BUMPS_MAX:
            self.finish('ABORTED', f'bumped into something {self.bumps} times')
            return False
        p = self.n.map_pose() or self.fb['pose']
        pushing = next((c[0] for c in reversed(self.cmds) if abs(c[0]) > 0.01), 1.0)
        sign = 1.0 if pushing > 0 else -1.0
        if p is None:
            self.say('BUMP: she pushed without moving (the motion check); no pose to mark it')
            return True
        bx, by = p[0] + sign * BUMP_AHEAD_M * math.cos(p[2]), p[1] + sign * BUMP_AHEAD_M * math.sin(p[2])
        self.n.mark_bump(bx, by)
        self.say(f'{time.monotonic() - self.t0:5.1f} s  BUMP {self.bumps}: pushed {"forward" if sign > 0 else "back"} '
                 f'at ({p[0]:+.2f}, {p[1]:+.2f}) without moving; ({bx:+.2f}, {by:+.2f}) marked for the planner, '
                 'going round once the motion check lets go')
        return True

    def _bump_tick(self, now):
        if not self.n.lock and self.n.permitted()[0]:
            pending, self.pending = self.pending, None
            self.done_before += len(self.stretch) - len(pending)
            self.send(pending)
        elif now >= self.deadline:
            self.finish('ABORTED', f'the motion check held her {BUMP_WAIT_S:.0f} s after a bump')

    # --------------------------------------------------------------- rescue --

    def _rescue(self, why):
        """Nav2 could not get there: the next step of the rescue, from where she stands."""
        r = self.rescue
        remaining = self.remaining()
        if not remaining:
            return self.finish('ABORTED', why)
        num = self.done_before + len(self.stretch) - len(remaining) + 1
        if not r['retraced']:
            r['retraced'] = True
            self.say(f'rescue: {why} short of waypoint {num}: backing out along her own track, then trying again')
            self.n.set_retrace(RETRACE_M)
            r['future'] = self.n.retrace_cli.call_async(Trigger.Request()) if self.n.retrace_cli.service_is_ready() else None
            self.pending = remaining
            self.phase = 'rescue retrace'
            self.deadline = time.monotonic() + 60.0
            return
        if r['asks'] < RESCUE_ASKS and self.n.ask_cli.service_is_ready():
            r['asks'] += 1
            self.say(f'rescue: {why} again; asking Claude (house map, obstacles, camera sweep), ask {r["asks"]} of {RESCUE_ASKS}')
            self.n.load_voice()
            r['future'] = self.n.ask_cli.call_async(Trigger.Request())
            self.pending = remaining
            self.phase = 'rescue ask'
            self.deadline = time.monotonic() + ASK_WAIT_S
            return
        self._skip(remaining, f'{why} and the advice did not help')

    def _skip(self, remaining, why):
        """Drop the waypoint she cannot reach and go for the next one (Nav2 plans round)."""
        num = self.done_before + len(self.stretch) - len(remaining) + 1
        self.rescue['skipped'] += 1
        rest = remaining[1:]
        self.say(f'rescue: skipping waypoint {num} ({why}); {len(rest)} to go')
        self.skipped.append(num)                                   # dealt with, NOT reached
        if not rest:
            return self.finish('ABORTED', f'waypoint {num} unreachable and nothing after it')
        self.rescue['retraced'] = False                             # a fresh waypoint gets a fresh retrace
        self.done_before += len(self.stretch) - len(rest)
        self.send(rest)

    def _advised(self, msg):
        """Act on Claude's answer (nav_helper ~/ask): wait, via, retrace, or give up = skip."""
        try:
            out = json.loads(msg) if msg else {}
        except ValueError:
            out = {'error': msg}
        advice = out.get('advice')
        remaining = self.pending or self.remaining()
        if not advice:
            self.say(f'rescue: no usable advice ({out.get("error") or "unreadable answer"}); pictures in {out.get("dir", "?")}')
            return self._skip(remaining, 'no advice')
        self.say(f'rescue: Claude sees "{advice.get("what", "?")}"' + (' (temporary)' if advice.get('temporary') else '')
                 + f' -> {advice.get("action")}: {advice.get("why", "")}' + f' ({out.get("seconds", "?")} s; {out.get("dir", "")})')
        if advice.get('say'):
            self.n.speak(advice['say'])
        action = advice.get('action')
        self.rescue['advice'] = action
        if action == 'wait':
            wait_s = float(advice.get('wait_s') or 30.0)
            self.say(f'rescue: waiting {wait_s:.0f} s')
            self.phase = 'rescue wait'
            self.deadline = time.monotonic() + wait_s
        elif action == 'via' and 'x' in advice and 'y' in advice:
            vx, vy = float(advice['x']), float(advice['y'])
            nx, ny, _ = remaining[0]
            self.say(f'rescue: going via ({vx:+.2f}, {vy:+.2f})')
            self.done_before += len(self.stretch) - len(remaining)
            self.send([(vx, vy, math.atan2(ny - vy, nx - vx))] + remaining)
            self.done_before -= 1                                   # the via point is not a waypoint
            self.via_first = True
        elif action == 'retrace':
            self.n.set_retrace(float(advice.get('retrace_m') or RETRACE_M))
            self.rescue['future'] = self.n.retrace_cli.call_async(Trigger.Request()) if self.n.retrace_cli.service_is_ready() else None
            self.phase = 'rescue retrace'
            self.deadline = time.monotonic() + 60.0
        else:                                                       # give_up, or something new
            self._skip(remaining, f'Claude says {action or "nothing"}')

    def _rescue_tick(self, now):
        r = self.rescue
        if self.phase == 'rescue retrace':
            fut = r['future']
            if fut is not None and not fut.done() and now < self.deadline:
                return
            if fut is not None and fut.done():
                try:
                    self.say(f'rescue: {fut.result().message}')
                except Exception as exc:  # noqa: BLE001
                    self.say(f'rescue: retrace failed ({str(exc)[:60]})')
            elif fut is None:
                self.say('rescue: no nav_helper to back out with: trying again from here')
            else:
                self.say('rescue: the retrace did not answer in 60 s: trying again from here')
            r['future'] = None
            pending, self.pending = self.pending, None
            self.done_before += len(self.stretch) - len(pending)
            self.send(pending)
        elif self.phase == 'rescue ask':
            fut = r['future']
            if fut is not None and fut.done():
                r['future'] = None
                try:
                    msg = fut.result().message
                except Exception as exc:  # noqa: BLE001
                    msg = json.dumps({'error': str(exc)[:80]})
                self._advised(msg)
            elif now >= self.deadline:
                r['future'] = None
                self.say(f'rescue: no answer in {ASK_WAIT_S:.0f} s')
                self._skip(self.pending or self.remaining(), 'no answer')
        elif self.phase == 'rescue wait':
            if now >= self.deadline:
                pending, self.pending = self.pending, None
                self.done_before += len(self.stretch) - len(pending)
                self.send(pending)

    def _cancel_then(self, why, pending):
        """Cancel the running stretch; when Nav2 confirms, wait (detour) or hold with `pending` to go."""
        self.pending = pending
        if self.handle is None:                           # still being accepted: nothing to cancel yet
            if why == 'rescue':
                return self._rescue('the way stayed blocked')
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
        if self.phase.startswith('rescue'):               # nothing of Nav2's runs; the mux lock holds her
            self.rescue_return, self.phase = self.phase, 'held'
            self.say('rescue paused: the safety gate holds her')
            return
        self._cancel_then('hold', self.remaining())

    def resume(self):
        if self.phase == 'held' and self.rescue_return:
            self.phase, self.rescue_return = self.rescue_return, None
            self.say(f'rescue resumes ({self.phase})')
            return
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
        if self.phase.startswith('rescue'):
            self._rescue_tick(now)
            return
        if self.phase == 'bump wait':
            self._bump_tick(now)
            return
        if self.phase == 'parking':
            if now - self.park_t0 > PARK_WAIT_S:
                self.say(f'park: no result in {PARK_WAIT_S:.0f} s')
                return self.finish('SUCCEEDED', f'the parking did not report in {PARK_WAIT_S:.0f} s')
            return
        if self.phase not in ('driving', 'sending', 'cancelling'):
            return
        if now - self.t0 > self.timeout_s:
            self.say('timeout: cancelling the route')
            return self.abandon(f'timeout after {self.timeout_s:.0f} s')
        if self.phase == 'driving' and self.fb['dist'] == self.fb['dist']:
            # the stall rule: stuck means not getting closer, not a clock running out
            if self.fb['dist'] < self.best_dist - STALL_GAIN_M:
                self.best_dist, self.best_t = self.fb['dist'], now
            elif now - self.best_t > STALL_S:
                self.say(f'{now - self.t0:5.1f} s  no progress for {STALL_S:.0f} s ({self.fb["dist"]:.2f} m to go)')
                self.best_dist, self.best_t = self.fb['dist'], now
                return self._cancel_then('rescue', self.remaining())
        if (self.phase == 'driving' and self.then_park and self.k + 1 == len(self.stretches)
                and self.fb['left'] == 1 and self.fb['dist'] == self.fb['dist'] and self.fb['dist'] <= HANDOFF_M
                and self._near_end()):
            return self._hand_over_to_parking(now)
        if self.phase == 'driving' and self.fb['left'] <= 1 and self._next_overlaps():
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
                 f'{len(self.route) - len(self.passed) - len(self.skipped)} to go, {self.fb["dist"]:.2f} m  '
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
                self.say('the way stays blocked: the rescue takes over')
                self.detours = 0
                return self._cancel_then('rescue', remaining)
            if self.detours == 3:
                self.say('   clearing the costmaps')
                self.n.clear_costmaps()
            self._cancel_then('detour', remaining)

    def feedback(self):
        """What the controller's status shows."""
        p = self.fb['pose']
        out = {'dist_m': round(self.fb['dist'], 2) if self.fb['dist'] == self.fb['dist'] else None,
               'left': len(self.route) - len(self.passed) - len(self.skipped), 'stretch': self.k + 1, 'stretches': len(self.stretches),
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
                     + f'; passed {len(self.passed)} of {len(self.route)} waypoints'
                     + (f' ({len(self.skipped)} skipped: {self.skipped}, {self.rescue["asks"]} ask(s) of Claude)'
                        if self.rescue['asks'] or self.skipped else '')
                     + where
                     + f'; driver got {len(self.cmds)} commands, throttle {min(thr, default=0):.2f}-{max(thr, default=0):.2f}'
                     + f'; forward/reverse switches {switches}')
        self.say(self.line)
        self.n.tf_off()
        if self.start_pose is not None:
            self.n.select_planner('GridBased')            # nav_goal and nav_park: exact headings
