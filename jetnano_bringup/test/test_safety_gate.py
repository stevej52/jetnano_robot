"""safety_gate.decide: the one decision, without a robot."""
from types import SimpleNamespace

from jetnano_bringup.safety_gate import DEGRADED_SPEED_PCT, SafetyGate, decide, hard_outputs

OK = {'scan_age': 0.1, 'odom_age': 0.01, 'vo_age': 0.02, 'battery': {'level': 'ok', 'volts': 12.1},
      'where': 'placed', 'tilt': 'armed -> safe: level', 'guard_flow': 'ok', 'camera': 'ok', 'watchdog': []}


def test_starts_inhibited_until_it_knows_enough():
    assert decide(0, {})[0] == 'inhibited'
    assert decide(0, {**OK, 'scan_age': None})[0] == 'inhibited'
    assert decide(0, {**OK, 'odom_age': 3.0})[0] == 'inhibited'
    assert decide(0, {**OK, 'battery': None})[0] == 'inhibited'
    state, reasons, _ = decide(0, {**OK, 'battery': {'level': 'none', 'volts': 0.04}})
    assert state == 'inhibited' and 'bench' in reasons[0]
    assert decide(0, {**OK, 'where': 'bench'})[0] == 'inhibited'


def test_ok_when_everything_is_fresh_and_valid():
    assert decide(0, OK) == ('ok', [], 100.0)


def test_locks_and_guards_stop_her():
    assert decide(0, {**OK, 'lock_e_stop_web': True})[0] == 'stopped'
    state, reasons, speed = decide(0, {**OK, 'lock_e_stop_motion': True, 'lock_e_stop': True})
    assert state == 'stopped' and len(reasons) == 2 and speed == 0.0
    assert decide(0, {**OK, 'guard_flow': 'holding: commands go in, nothing comes out'})[0] == 'stopped'
    assert decide(0, {**OK, 'tilt': 'armed -> recovering: pitch 31 deg'})[0] == 'stopped'
    assert decide(0, {**OK, 'tilt': 'recovering -> safe: level again'})[0] == 'ok'
    assert decide(0, {**OK, 'battery': {'level': 'flat', 'volts': 9.5}})[0] == 'stopped'


def test_degraded_means_slower_with_a_reason():
    state, reasons, speed = decide(0, {**OK, 'camera': 'stale 73 s'})
    assert state == 'degraded' and speed == DEGRADED_SPEED_PCT and 'lidar only' in reasons[0]
    assert decide(0, {**OK, 'vo_age': 5.0})[0] == 'degraded'
    assert decide(0, {**OK, 'battery': {'level': 'low', 'volts': 10.6}})[0] == 'degraded'
    assert decide(0, {**OK, 'watchdog': ['lidar odometry: silent for 9 s']})[0] == 'degraded'


def test_a_stop_outranks_degraded():
    state, reasons, _ = decide(0, {**OK, 'camera': 'stale 10 s', 'lock_e_stop_joy': True})
    assert state == 'stopped' and reasons == ['STOP on the joystick']


def test_bench_driving_can_be_allowed_as_degraded():
    bench = {**OK, 'battery': {'level': 'none', 'volts': 0.04}, 'where': 'bench'}
    assert decide(0, bench)[0] == 'inhibited'
    state, reasons, speed = decide(0, {**bench, 'allow_bench': True})
    assert state == 'degraded' and 'wheels up' in reasons[0] and speed == DEGRADED_SPEED_PCT


def test_verdicts_expire_and_a_frozen_stamp_counts_as_stale():
    # the battery monitor and the camera's health report go quiet: degraded with the reason
    state, reasons, speed = decide(0, {**OK, 'battery_age': 25.0})
    assert state == 'degraded' and 'battery verdict 25 s old' in reasons[0] and speed == DEGRADED_SPEED_PCT
    state, reasons, _ = decide(0, {**OK, 'camera_age': 9.0})
    assert state == 'degraded' and 'no health report for 9 s' in reasons[0]
    assert decide(0, {**OK, 'battery_age': 3.0, 'camera_age': 1.0}) == ('ok', [], 100.0)
    # the node adds the measurement's lag to a stream's age: a scan arriving with a 3 s old stamp
    # is a 3 s old scan, so the pure decision sees scan_age 3 and inhibits
    assert decide(0, {**OK, 'scan_age': 3.0})[0] == 'inhibited'


def test_no_freshness_report_means_inhibited():
    state, reasons, _ = decide(0, {**OK, 'fresh_report': False, 'scan_age': None, 'odom_age': None})
    assert state == 'inhibited' and reasons[0].startswith('stream ages unknown')
    assert decide(0, {**OK, 'fresh_report': True}) == ('ok', [], 100.0)


def test_stream_age_from_topic_watch():
    from types import SimpleNamespace
    from jetnano_bringup.safety_gate import SafetyGate
    g = SimpleNamespace(fresh={'/scan': (0.10, 0.02), '/vo': (0.01, float('nan')), '/odometry/filtered': (-1.0, 0.0),
                               '/frozen': (0.01, 5.0), '/future': (0.01, -3.0)}, fresh_at=100.0)
    age = lambda topic, report=True: SafetyGate._age(g, topic, 100.5, report)  # noqa: E731
    assert abs(age('/scan') - 0.62) < 1e-9          # its age + the report's age + the stamp lag
    assert abs(age('/vo') - 0.51) < 1e-9            # no stamp: arrival only
    assert age('/odometry/filtered') is None        # never seen
    assert age('/missing') is None
    assert abs(age('/frozen') - 5.51) < 1e-9        # arriving, but stamped 5 s ago: stale
    assert abs(age('/future') - 0.51) < 1e-9        # a stamp from the future counts as arrival only
    assert age('/scan', report=False) is None


UP = {**OK, 'uptime': 200.0, 'tilt_age': 30.0, 'guard_flow_age': 0.2, 'watchdog_age': 2.0, 'camera_age': 0.5,
      'fresh_report': True}


def test_silence_is_not_health_after_the_grace():
    assert decide(0, UP) == ('ok', [], 100.0)
    # starting up: missing reports are still parts on their way
    assert decide(0, {**UP, 'uptime': 60.0, 'tilt_age': None, 'watchdog_age': None})[0] == 'ok'
    state, reasons, _ = decide(0, {**UP, 'tilt_age': None})
    assert state == 'inhibited' and 'tilt guard has not reported' in reasons
    for change, words in (({'guard_flow_age': 5.0}, 'guard_flow silent'), ({'guard_flow_age': None}, 'guard_flow silent'),
                          ({'watchdog_age': None}, 'watchdog silent'), ({'watchdog_age': 40.0}, 'watchdog silent'),
                          ({'camera_age': None}, 'no health report ever'), ({'vo_age': None}, 'never seen'),
                          ({'tilt': 'off -> off: tilt guard switched off'}, 'tilt guard switched off')):
        state, reasons, speed = decide(0, {**UP, **change})
        assert state == 'degraded' and speed == DEGRADED_SPEED_PCT and any(words in r for r in reasons), change


def test_hard_stop_sends_off_then_repeats_then_on_once():
    """STOP is a hard all-stop (2026-10-15): /pca9685/output_enable false, again every second."""
    assert hard_outputs(False, False, True, 0.0, 0.0) is True      # the start: on, once
    assert hard_outputs(False, False, False, 0.2, 0.0) is None     # never a stream of True
    assert hard_outputs(True, False, False, 5.0, 0.0) is False     # STOP pressed: cut
    assert hard_outputs(True, True, False, 5.5, 5.0) is None
    assert hard_outputs(True, True, False, 6.0, 5.0) is False      # held: repeated every second
    assert hard_outputs(False, True, False, 7.0, 6.0) is True      # GO: back on, once
    assert hard_outputs(True, False, True, 0.0, 0.0) is False      # started mid-STOP: cut at once


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, m):
        self.sent.append(m.data)


class _Log:
    def warning(self, *a):
        pass

    def info(self, *a):
        pass


def test_go_on_the_page_does_not_undo_the_joysticks_stop():
    g = SimpleNamespace(locks={}, hard_keys=['e_stop_web', 'e_stop_joy'], hard=False, hard_first=False,
                        last_hard_pub=0.0, outputs_pub=_Pub(), get_logger=lambda: _Log())
    tick = lambda now: SafetyGate._hard_stop(g, now)            # noqa: E731
    g.locks['e_stop_web'] = True
    tick(1.0)
    g.locks['e_stop_joy'] = True
    tick(1.2)
    g.locks['e_stop_web'] = False                               # GO on the page
    tick(1.4)
    assert g.outputs_pub.sent == [False], 'the joystick still holds STOP: outputs stay cut'
    g.locks['e_stop_joy'] = False
    tick(1.6)
    assert g.outputs_pub.sent == [False, True]


def test_battery_and_motion_check_stay_software_stops():
    g = SimpleNamespace(locks={'e_stop': True, 'e_stop_motion': True}, hard_keys=['e_stop_web', 'e_stop_joy'],
                        hard=False, hard_first=False, last_hard_pub=0.0, outputs_pub=_Pub(),
                        get_logger=lambda: _Log())
    SafetyGate._hard_stop(g, 1.0)
    assert g.outputs_pub.sent == []

