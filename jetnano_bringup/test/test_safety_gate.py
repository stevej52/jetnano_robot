"""safety_gate.decide: the one decision, without a robot."""
from jetnano_bringup.safety_gate import DEGRADED_SPEED_PCT, decide

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
