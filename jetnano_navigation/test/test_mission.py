"""mission: the pure decisions - resume policy, the remaining route."""
from jetnano_navigation import mission


def test_short_holds_resume_on_their_own_long_ones_ask():
    assert mission.resume_policy(3.0) == 'resume'
    assert mission.resume_policy(30.0) == 'resume'
    assert mission.resume_policy(31.0) == 'ask'
    assert mission.resume_policy(10.0, resume_within_s=5.0) == 'ask'


def test_remaining_route_from_nav2s_count():
    wps = [[1, 0, 0], [2, 0, 0], [3, 0, 0], [4, 0, 0]]
    assert mission.remaining_route(wps, 2) == [[3, 0, 0], [4, 0, 0]]
    assert mission.remaining_route(wps, 4) == wps
    assert mission.remaining_route(wps, None) == wps          # no feedback yet: all of it again
    assert mission.remaining_route(wps, 0) == wps
    assert mission.remaining_route(wps, 9) == wps


def test_permission_expires_with_the_gates_silence():
    import time
    from std_msgs.msg import String
    pytest = __import__('pytest')
    pytest.importorskip('rclpy')
    import rclpy
    rclpy.init()
    try:
        n = mission.Mission()
        assert n.permitted() == (False, 'no verdict from the safety gate')
        n._on_gate(String(data='{"permit": true, "state": "ok", "reasons": []}'))
        assert n.permitted() == (True, '')
        n.gate_at = time.monotonic() - mission.PERMIT_MAX_AGE_S - 1
        ok, why = n.permitted()
        assert not ok and 'has not spoken' in why
        n._on_gate(String(data='{"permit": false, "state": "stopped", "reasons": ["STOP on the page"]}'))
        assert n.permitted() == (False, 'STOP on the page')
        n.destroy_node()
    finally:
        rclpy.shutdown()


def test_a_person_driving_ends_the_mission_for_good():
    """Drive 41 (2026-10-06): Steve drove her back mid-rescue and the route carried on later."""
    from types import SimpleNamespace
    from geometry_msgs.msg import Twist
    said, abandoned, cancelled = [], [], []
    run = SimpleNamespace(abandon=abandoned.append)
    m = {'run': run}
    fake = SimpleNamespace(active=m, held={'mission': m}, say=lambda mi, line: said.append(line),
                           _cancel=cancelled.append)
    idle = Twist()                                      # the page's idle zeros: not a takeover
    mission.Mission._on_person(fake, 'cmd_vel_web', idle)
    assert not abandoned and fake.held is not None
    knob = Twist()
    knob.linear.x = 0.2
    mission.Mission._on_person(fake, 'cmd_vel_web', knob)
    assert abandoned == ['a person took the wheel (cmd_vel_web)']
    assert fake.held is None                            # nothing resumes on its own
    assert 'took the wheel' in said[0]
    mission.Mission._on_person(fake, 'cmd_vel_web', knob)   # more knob: said once
    assert len(abandoned) == 1
    plain = {'run': None}                               # a plain Nav2 goal: cancelled through Nav2
    fake2 = SimpleNamespace(active=plain, held=None, say=lambda mi, line: None, _cancel=cancelled.append)
    turn = Twist()
    turn.angular.z = 0.5
    mission.Mission._on_person(fake2, 'cmd_vel_teleop', turn)
    assert cancelled == ['a person took the wheel (cmd_vel_teleop)']
    mission.Mission._on_person(SimpleNamespace(active=None, held=None), 'cmd_vel_web', knob)   # idle: nothing


# ---- review of 2026-10-13 (an outside AI read mission.py; the findings that held) ----

def test_a_cancel_also_drops_a_command_waiting_to_replace_the_mission():
    from types import SimpleNamespace
    from std_msgs.msg import String
    cancelled = []
    fake = SimpleNamespace(held=None, queued={'do': 'route'}, _cancel=cancelled.append)
    mission.Mission._on_command(fake, String(data='{"do": "cancel"}'))
    assert fake.queued is None and cancelled == ['cancelled on request']


def test_a_person_taking_over_drops_the_queued_command_and_stops_the_parking():
    from types import SimpleNamespace
    from geometry_msgs.msg import Twist
    abandoned, stopped = [], []
    m = {'run': SimpleNamespace(phase='parking', abandon=abandoned.append), 'id': 'x1'}
    fake = SimpleNamespace(active=m, held=None, queued={'do': 'goal'}, say=lambda mi, line: None,
                           _stop_parking=stopped.append)
    knob = Twist()
    knob.linear.x = 0.3
    mission.Mission._on_person(fake, 'cmd_vel_web', knob)
    assert fake.queued is None and stopped == [m] and len(abandoned) == 1


def test_a_cancel_during_the_parking_stops_the_parking_too():
    from types import SimpleNamespace
    abandoned, stopped = [], []
    m = {'run': SimpleNamespace(phase='parking', abandon=abandoned.append), 'id': 'x2'}
    fake = SimpleNamespace(active=m, held=None, _stop_parking=stopped.append)
    mission.Mission._cancel(fake, 'cancelled on request')
    assert stopped == [m] and abandoned == ['cancelled on request'] and m['cancel_why'] == 'cancelled on request'
    m2 = {'run': SimpleNamespace(phase='driving', abandon=abandoned.append), 'id': 'x3'}
    fake2 = SimpleNamespace(active=m2, held=None, _stop_parking=stopped.append)
    mission.Mission._cancel(fake2, 'replaced by a new command')
    assert len(stopped) == 1, 'only a parking is told to stop'


def test_a_cancelled_goal_reports_why():
    from types import SimpleNamespace
    from action_msgs.msg import GoalStatus
    finished = []
    m = {'cancel_why': 'timeout after 45 s'}
    fake = SimpleNamespace(active=m, held=None, _finish=lambda mi, outcome, note: finished.append((outcome, note)))
    fut = SimpleNamespace(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    mission.Mission._on_result(fake, m, fut)
    assert finished == [('CANCELED', 'timeout after 45 s')]
    fut2 = SimpleNamespace(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    mission.Mission._on_result(fake, m, fut2)
    assert finished[-1] == ('SUCCEEDED', ''), 'a success carries no cancel reason'
