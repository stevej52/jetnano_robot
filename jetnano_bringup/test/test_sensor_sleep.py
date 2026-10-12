"""sensor_sleep's decision and the flag the watchdogs read, without a robot."""
import os

from jetnano_bringup import sensor_flag
from jetnano_bringup.safety_gate import decide
from jetnano_bringup.sensor_sleep import due_to_sleep


def test_sleeps_after_idle_sooner_on_the_bench():
    assert due_to_sleep(599, False, False, 600, 90) == ''
    assert due_to_sleep(600, False, False, 600, 90) == 'idle 10 min'
    assert due_to_sleep(89, False, True, 600, 90) == ''
    assert due_to_sleep(90, False, True, 600, 90).startswith('on the bench')


def test_never_while_a_goal_runs():
    assert due_to_sleep(10_000, True, True, 600, 90) == ''


def test_flag_skips_lidar_and_the_camera_only_while_switching(tmp_path):
    p = str(tmp_path / 'asleep')
    assert sensor_flag.not_judged(p) == ()
    sensor_flag.write('asleep', 20.0, path=p, now=1000.0)
    os.utime(p, (1000.0, 1000.0))
    assert 'vo' in sensor_flag.not_judged(p, now=1030.0)          # asleep: the camera rests too
    sensor_flag.write('waking', 20.0, path=p, now=1100.0)
    os.utime(p, (1100.0, 1100.0))
    assert 'vo' in sensor_flag.not_judged(p, now=1110.0)
    skip = sensor_flag.not_judged(p, now=1130.0)
    assert 'lidar' in skip and 'lidar_odom' in skip and 'vo' not in skip
    # a writer that died: its flag no longer blinds the watchdog
    assert sensor_flag.not_judged(p, now=1100.0 + sensor_flag.STALE_S + 1) == ()
    # awake, the keeper switching the projector: the camera's streams only, for a while
    sensor_flag.write('camera', 20.0, path=p, now=1200.0)
    os.utime(p, (1200.0, 1200.0))
    skip = sensor_flag.not_judged(p, now=1210.0)
    assert 'vo' in skip and 'lidar' not in skip
    assert sensor_flag.not_judged(p, now=1230.0) == ()
    sensor_flag.clear(p)
    assert sensor_flag.not_judged(p) == ()


def test_gate_says_asleep_not_no_lidar():
    ok = {'odom_age': 0.01, 'battery': {'level': 'ok', 'volts': 12.1}, 'where': 'placed'}
    state, reasons, _ = decide(0, {**ok, 'sensors': 'asleep', 'scan_age': 30.0})
    assert state == 'inhibited' and reasons == ['sensors asleep: a drive command wakes them']
    state, reasons, _ = decide(0, {**ok, 'sensors': 'awake', 'scan_age': 30.0})
    assert reasons[0].startswith('lidar silent')
