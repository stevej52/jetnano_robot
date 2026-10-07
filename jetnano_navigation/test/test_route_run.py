"""route_run.RouteRun: the lap's state machine, driven by hand with a fake controller and a
fake Nav2 (no ROS graph needed; the message classes only)."""
import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from jetnano_navigation import route_run  # noqa: E402
from jetnano_navigation.nav_goal import yaw  # noqa: E402


class Fut:
    def __init__(self, result=None, done=True):
        self._r, self._done, self.cbs = result, done, []

    def done(self):
        return self._done

    def result(self):
        return self._r

    def add_done_callback(self, cb):
        self.cbs.append(cb)
        if self._done:
            cb(self)

    def finish(self, result):
        self._r, self._done = result, True
        for cb in self.cbs:
            cb(self)


class Handle:
    """A Nav2 goal handle: accepted, with a result the test settles."""
    _n = 0

    def __init__(self):
        from unique_identifier_msgs.msg import UUID
        Handle._n += 1
        self.accepted = True
        self.goal_id = UUID(uuid=[Handle._n] + [0] * 15)
        self.result_fut = Fut(done=False)
        self.cancelled = False

    def get_result_async(self):
        return self.result_fut

    def cancel_goal_async(self):
        self.cancelled = True
        return Fut()


class Through:
    def __init__(self):
        self.sent = []

    def send_goal_async(self, goal, feedback_callback=None):
        h = Handle()
        self.sent.append((goal, h, feedback_callback))
        return Fut(h)


class Clock:
    def now(self):
        return rclpy.time.Time()


class Cli:
    ready = False

    def service_is_ready(self):
        return self.ready


class Svc:
    """A Trigger service the test answers by hand."""
    def __init__(self, ready=True):
        self.ready, self.calls = ready, []

    def service_is_ready(self):
        return self.ready

    def call_async(self, req):
        f = Fut(done=False)
        self.calls.append(f)
        return f


class Reply:
    def __init__(self, success, message):
        self.success, self.message = success, message


class Node:
    def __init__(self, pose=(0.0, 0.0, 0.0)):
        self.through = Through()
        self.costmap_cli = Cli()
        self.snap_cli = Cli()
        self.retrace_cli = Svc()
        self.ask_cli = Svc()
        self.retrace_m = []
        self.spoken = []
        self.voice_loads = 0
        self.pose = pose
        self.planners = []
        self.watched = []
        self.cleared = 0
        self.lock = None
        self.gate_ok = True
        self.bumps = []

    def permitted(self):
        return (self.gate_ok and not self.lock), ('' if self.gate_ok else 'motion check')

    def mark_bump(self, x, y):
        self.bumps.append((round(x, 2), round(y, 2)))

    def get_clock(self):
        return Clock()

    def map_pose(self):
        return self.pose

    def goal_watch(self, wp):
        self.watched.append(wp)

    def select_planner(self, name):
        self.planners.append(name)

    def clear_costmaps(self):
        self.cleared += 1

    def tf_on(self):
        pass

    def park(self, cmd):
        self.parked = getattr(self, 'parked', []) + [cmd]

    def tf_off(self):
        pass

    def set_retrace(self, m):
        self.retrace_m.append(m)

    def speak(self, text):
        self.spoken.append(text)

    def load_voice(self):
        self.voice_loads += 1


def feedback(handle, x, y, dist, left):
    from geometry_msgs.msg import Pose
    from nav2_msgs.action import NavigateThroughPoses

    class F:
        pass
    f = F()
    f.goal_id = handle.goal_id
    f.feedback = NavigateThroughPoses.Feedback()
    f.feedback.current_pose.pose = Pose()
    f.feedback.current_pose.pose.position.x = x
    f.feedback.current_pose.pose.position.y = y
    f.feedback.current_pose.pose.orientation.w = 1.0
    f.feedback.distance_remaining = dist
    f.feedback.number_of_poses_remaining = left
    return f


def result(status):
    class R:
        pass
    r = R()
    r.status = status
    return r


LAP = [[3, 0, 0], [3, 3, 90], [0, 3, 180], [0, 0.5, -90]]      # ends 0.5 m from the start: two stretches


def make(node=None, route=LAP, timeout=300):
    said = []
    n = node or Node()
    run = route_run.RouteRun(n, route, timeout, said.append)
    run.start()
    return n, run, said


def test_no_costmap_service_then_start_pose_then_first_stretch():
    n, run, said = make()
    assert run.phase == 'starting' and 'not checked' in said[0]
    run.tick(time.monotonic(), False, [])
    assert run.phase == 'driving'                    # accepted at once by the fake
    assert n.planners == ['Through'] and len(n.watched) == 1
    assert len(run.stretches) == 2 and len(n.through.sent) == 1
    assert [p.pose.position.x for p in n.through.sent[0][0].poses] == [3.0, 3.0, 0.0]


def test_handover_passed_lines_and_success():
    n, run, said = make()
    run.tick(time.monotonic(), False, [])
    goal, h1, fb_cb = n.through.sent[0]
    fb_cb(feedback(h1, 3.0, 0.2, 5.0, 2))            # the first waypoint dropped
    assert any('passed waypoint 1' in s for s in said)
    fb_cb(feedback(h1, 3.0, 2.9, 3.0, 1))            # only the stretch's last is left: handover
    run.tick(time.monotonic(), False, [(0.3, 0.0)])
    assert len(n.through.sent) == 2 and run.k == 1 and run.done_before == 2
    goal2, h2, fb2 = n.through.sent[1]
    assert [p.pose.position.x for p in goal2.poses] == [0.0, 0.0]
    fb_cb(feedback(h1, 3.0, 3.0, 0.1, 1))            # the old stretch's last words: ignored
    h1.result_fut.finish(result(6))                  # CANCELED by the preemption: not ours any more
    assert not run.done
    fb2(feedback(h2, 0.1, 3.0, 2.5, 1))
    assert any('passed waypoint 3' in s for s in said)
    n.pose = (0.0, 0.5, math.radians(-90))
    h2.result_fut.finish(result(4))                  # SUCCEEDED
    assert run.done and run.outcome == 'SUCCEEDED'
    assert 'passed 4 of 4 waypoints' in run.line and 'ended 0 cm' in run.line
    assert n.planners[-1] == 'GridBased'


def test_detour_cancels_waits_and_resends_the_rest(monkeypatch):
    monkeypatch.setattr(route_run, 'DETOUR_WAIT_S', 0.2)
    n, run, said = make()
    run.tick(time.monotonic(), False, [])
    goal, h1, fb_cb = n.through.sent[0]
    fb_cb(feedback(h1, 0.5, 0.0, 30.0, 3))           # Nav2 plans 30 m where the way is ~8.5 m
    run.tick(time.monotonic() + 1.5, False, [])      # the next status line is due: the guard looks
    assert h1.cancelled and run.phase == 'cancelling' and run.detours == 1
    h1.result_fut.finish(result(6))
    assert run.phase == 'detour wait'
    run.tick(time.monotonic(), False, [])
    assert len(n.through.sent) == 1
    time.sleep(0.25)
    run.tick(time.monotonic(), False, [])
    assert len(n.through.sent) == 2 and run.phase == 'driving'
    assert len(n.through.sent[1][0].poses) == 3      # the same stretch, all three still to go


def test_gate_hold_and_resume_send_what_is_left():
    n, run, said = make()
    run.tick(time.monotonic(), False, [])
    goal, h1, fb_cb = n.through.sent[0]
    fb_cb(feedback(h1, 3.0, 1.0, 4.0, 2))
    run.hold()
    assert run.phase == 'cancelling' and h1.cancelled
    h1.result_fut.finish(result(6))
    assert run.phase == 'held' and run.pending == run.stretch[1:]
    run.resume()
    assert run.phase == 'driving' and len(n.through.sent) == 2 and run.done_before == 1
    assert [p.pose.position.y for p in n.through.sent[1][0].poses] == [3.0, 3.0]


def test_hold_before_the_start_refuses():
    n, run, said = make()
    run.hold()
    assert run.done and run.outcome == 'REFUSED'


def test_timeout_abandons():
    n, run, said = make(timeout=10)
    run.tick(time.monotonic(), False, [])
    run.tick(time.monotonic() + 11.0, False, [])
    assert run.done and run.outcome == 'CANCELED' and 'timeout' in run.line
    assert n.through.sent[0][1].cancelled


def test_yaw_of_identity_is_zero():
    from geometry_msgs.msg import Quaternion
    assert yaw(Quaternion(w=1.0)) == 0.0


def test_rescue_sequence():
    import json
    from action_msgs.msg import GoalStatus
    n, run, said = make()
    run.tick(time.monotonic(), False, [])
    goal, h1, fb_cb = n.through.sent[0]
    fb_cb(feedback(h1, 2.0, 0.0, 6.0, 2))             # waypoint 1 passed, two left in this stretch
    h1.result_fut.finish(result(GoalStatus.STATUS_ABORTED))
    assert run.phase == 'rescue retrace' and n.retrace_m == [route_run.RETRACE_M] and len(n.retrace_cli.calls) == 1
    n.retrace_cli.calls[0].finish(Reply(True, 'backed out 0.80 m of 0.80'))
    run.tick(time.monotonic(), False, [])
    assert run.phase == 'driving' and len(n.through.sent) == 2, 'retried with what was left'
    assert [p.pose.position.x for p in n.through.sent[1][0].poses] == [3.0, 0.0]
    goal2, h2, fb2 = n.through.sent[1]
    h2.result_fut.finish(result(GoalStatus.STATUS_ABORTED))
    assert run.phase == 'rescue ask' and len(n.ask_cli.calls) == 1 and n.voice_loads == 1
    advice = {'advice': {'what': 'an open dishwasher', 'temporary': True, 'action': 'via', 'x': 1.5, 'y': 1.5,
                         'why': 'go north of the island', 'say': 'I will go round the island.'}, 'seconds': 12, 'dir': '/tmp/x'}
    n.ask_cli.calls[0].finish(Reply(True, json.dumps(advice)))
    run.tick(time.monotonic(), False, [])
    assert n.spoken == ['I will go round the island.']
    assert run.phase == 'driving' and len(n.through.sent) == 3
    xs = [(round(p.pose.position.x, 1), round(p.pose.position.y, 1)) for p in n.through.sent[2][0].poses]
    assert xs == [(1.5, 1.5), (3.0, 3.0), (0.0, 3.0)], 'the via point first, then what was left'
    goal3, h3, fb3 = n.through.sent[2]
    fb3(feedback(h3, 1.5, 1.5, 4.0, 2))                 # the via point passed: not a waypoint
    fb3(feedback(h3, 3.0, 3.0, 2.0, 1))                 # waypoint 2 passed
    assert any('passed waypoint 2' in s for s in said) and not any('passed waypoint 3' in s for s in said)
    assert [num for num, _ in run.passed] == [1, 2], 'the via point is not counted as a waypoint'
    assert [num for num, _ in run.passed] == [1, 2], 'the via point is not counted as a waypoint'


def test_rescue_gives_up_by_skipping_the_waypoint():
    import json
    from action_msgs.msg import GoalStatus
    n, run, said = make()
    run.tick(time.monotonic(), False, [])
    goal, h1, fb_cb = n.through.sent[0]
    fb_cb(feedback(h1, 2.0, 0.0, 6.0, 2))             # waypoint 1 passed, two left in this stretch
    h1.result_fut.finish(result(GoalStatus.STATUS_ABORTED))
    n.retrace_cli.calls[0].finish(Reply(True, 'backed out'))
    run.tick(time.monotonic(), False, [])
    n.through.sent[1][1].result_fut.finish(result(GoalStatus.STATUS_ABORTED))
    n.ask_cli.calls[0].finish(Reply(True, json.dumps({'advice': {'what': 'a wall', 'action': 'give_up', 'why': 'no way', 'say': ''}})))
    run.tick(time.monotonic(), False, [])
    assert run.rescue['skipped'] == 1 and len(n.through.sent) == 3
    assert [p.pose.position.x for p in n.through.sent[2][0].poses] == [0.0], 'waypoint 2 skipped, on to 3'
    assert any('skipping waypoint 2' in s for s in said)


def test_rescue_with_no_answer_skips():
    from action_msgs.msg import GoalStatus
    n, run, said = make()
    run.tick(time.monotonic(), False, [])
    n.through.sent[0][1].result_fut.finish(result(GoalStatus.STATUS_ABORTED))
    n.retrace_cli.calls[0].finish(Reply(True, 'backed out'))
    run.tick(time.monotonic(), False, [])
    n.through.sent[1][1].result_fut.finish(result(GoalStatus.STATUS_ABORTED))
    assert run.phase == 'rescue ask'
    run.tick(time.monotonic() + route_run.ASK_WAIT_S + 1, False, [])
    assert run.rescue['skipped'] == 1 and run.phase == 'driving'


def test_a_lone_waypoint_stretch_is_driven_to_its_end_before_the_next():
    # the review's example: out 3 m and back to 0.2 m - two stretches that do not overlap
    from action_msgs.msg import GoalStatus
    n, run, said = make(route=[[3, 0, 0], [0.2, 0, 0]])
    run.tick(time.monotonic(), False, [])
    assert len(run.stretches) == 2 and len(n.through.sent) == 1
    goal, h1, fb_cb = n.through.sent[0]
    fb_cb(feedback(h1, 1.0, 0.0, 2.0, 1))             # one left (it is the only one): NOT a handover
    run.tick(time.monotonic() + 1.5, False, [])
    assert len(n.through.sent) == 1, 'the lone waypoint is driven to its end'
    h1.result_fut.finish(result(GoalStatus.STATUS_SUCCEEDED))
    assert any('reached waypoint 1' in s for s in said) and len(n.through.sent) == 2
    assert [p.pose.position.x for p in n.through.sent[1][0].poses] == [0.2]
    n.through.sent[1][1].result_fut.finish(result(GoalStatus.STATUS_SUCCEEDED))
    assert run.done and run.outcome == 'SUCCEEDED' and [num for num, _ in run.passed] == [1, 2]


def test_a_skipped_waypoint_is_not_reported_as_reached():
    import json
    from action_msgs.msg import GoalStatus
    n, run, said = make()
    run.tick(time.monotonic(), False, [])
    goal, h1, fb_cb = n.through.sent[0]
    fb_cb(feedback(h1, 2.0, 0.0, 6.0, 2))
    h1.result_fut.finish(result(GoalStatus.STATUS_ABORTED))
    n.retrace_cli.calls[0].finish(Reply(True, 'backed out'))
    run.tick(time.monotonic(), False, [])
    n.through.sent[1][1].result_fut.finish(result(GoalStatus.STATUS_ABORTED))
    n.ask_cli.calls[0].finish(Reply(True, json.dumps({'advice': {'what': 'a wall', 'action': 'give_up', 'why': 'no way', 'say': ''}})))
    run.tick(time.monotonic(), False, [])
    assert run.skipped == [2] and [num for num, _ in run.passed] == [1]
    n.pose = (0.0, 0.5, 0.0)
    for h in (n.through.sent[2][1],):
        h.result_fut.finish(result(GoalStatus.STATUS_SUCCEEDED))
    # the last stretch (an overlapping one) follows; finish it
    while not run.done and len(n.through.sent) > 3:
        n.through.sent[-1][1].result_fut.finish(result(GoalStatus.STATUS_SUCCEEDED))
    if not run.done:
        run.finish('SUCCEEDED', '')
    assert 'skipped: [2]' in run.line and '2 of 4 waypoints' not in run.line.replace('skipped', '')


def test_gate_hold_during_the_rescue_pauses_it_and_resumes_in_place():
    from action_msgs.msg import GoalStatus
    n, run, said = make()
    run.tick(time.monotonic(), False, [])
    n.through.sent[0][1].result_fut.finish(result(GoalStatus.STATUS_ABORTED))
    assert run.phase == 'rescue retrace'
    run.hold()
    assert run.phase == 'held' and run.rescue_return == 'rescue retrace'
    run.tick(time.monotonic(), True, [])
    assert run.phase == 'held', 'nothing moves while held'
    run.resume()
    assert run.phase == 'rescue retrace'


def test_the_lap_hands_over_to_the_parking_while_rolling():
    from action_msgs.msg import GoalStatus
    route = [[3, 0, 0], [3, 3, 90]]                      # one stretch, ends far from the start
    n, run, said = make(route=route)
    run.then_park = {'x': 0.0, 'y': 0.0, 'heading_deg': 0.0}
    run.tick(time.monotonic(), False, [])
    goal, h1, fb_cb = n.through.sent[0]
    fb_cb(feedback(h1, 3.0, 2.0, 1.2, 1))               # last waypoint, 1.2 m to go: keep driving
    run.tick(time.monotonic() + 1.5, False, [])
    assert run.phase == 'driving' and not getattr(n, 'parked', [])
    fb_cb(feedback(h1, 3.0, 2.7, 0.3, 1))               # 0.3 m before the end: hand over
    run.tick(time.monotonic() + 3.0, False, [])
    assert run.phase == 'parking' and n.parked[0]['rolling'] is True and h1.cancelled
    h1.result_fut.finish(result(GoalStatus.STATUS_CANCELED))   # our own cancel: not an outcome
    assert not run.done
    run.on_park_log('-- reversing in')
    run.on_park_result({'outcome': 'PARKED', 'line': 'ended 3 cm from the spot', 'seconds': 14.2})
    assert run.done and run.outcome == 'SUCCEEDED' and 'parking PARKED' in run.line
    assert any(x.startswith('park: -- reversing in') for x in said) and any('park: took 14.2 s' in x for x in said)
    assert [num for num, _ in run.passed] == [1, 2]


def test_a_bump_marks_the_spot_and_carries_on_when_the_motion_check_lets_go():
    """Drive 46 (2026-10-06): the motion check cancelled the goal itself and the route ended."""
    n, run, said = make()
    run.tick(time.monotonic(), False, [])
    goal, h1, fb_cb = n.through.sent[0]
    fb_cb(feedback(h1, 3.0, 1.0, 4.0, 2))
    run.tick(time.monotonic(), False, [(0.15, 0.0)])          # pushing forward
    n.pose = (3.0, 1.0, math.radians(90))
    n.lock = True                                              # the motion check: pushed, did not move
    h1.result_fut.finish(result(5))                            # CANCELED, not by us
    assert not run.done and run.phase == 'bump wait'
    assert n.bumps == [(3.0, 1.4)]                             # 0.4 m ahead of her, the way she pushed
    assert any('BUMP 1' in s for s in said)
    run.tick(time.monotonic(), True, [])
    assert len(n.through.sent) == 1                            # still held
    n.lock = False
    run.tick(time.monotonic(), False, [])
    assert run.phase == 'driving' and len(n.through.sent) == 2
    assert [p.pose.position.y for p in n.through.sent[1][0].poses] == [3.0, 3.0]


def test_a_bump_while_reversing_is_marked_behind_and_too_many_end_the_route(monkeypatch):
    n, run, said = make()
    for k in range(route_run.BUMPS_MAX + 1):
        run.tick(time.monotonic(), False, [(-0.15, 0.0)])     # backing
        goal, h, fb_cb = n.through.sent[-1]
        fb_cb(feedback(h, 1.0, 0.0, 4.0, 3))
        n.lock = True
        h.result_fut.finish(result(5))
        n.lock = False
        if run.done:
            break
        run.tick(time.monotonic(), False, [])
    assert n.bumps[0] == (-0.4, 0.0)                           # behind her: she faced +x
    assert run.done and run.outcome == 'ABORTED' and 'bumped' in run.line
