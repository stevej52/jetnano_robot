import numpy as np

from jetnano_navigation.near_cap import Ceiling, cap_twist, zone_counts


def test_cruise_is_capped_when_something_is_near_ahead_and_the_curve_is_kept():
    v, w = cap_twist(0.44, 0.88, True, False, 0.24)
    assert v == 0.24
    assert abs(w / v - 0.88 / 0.44) < 1e-9


def test_no_cap_when_clear_or_already_slow():
    assert cap_twist(0.44, 0.3, False, False, 0.24) == (0.44, 0.3)
    assert cap_twist(0.20, 0.3, True, False, 0.24) == (0.20, 0.3)


def test_only_the_side_she_drives_towards_counts():
    assert cap_twist(0.44, 0.0, False, True, 0.24) == (0.44, 0.0)      # something behind, going forward
    assert cap_twist(-0.40, 0.2, False, True, 0.24)[0] == -0.24        # reversing towards it


def test_zone_counts_ahead_behind_and_beside():
    px = np.array([0.44, 0.45, 0.46, 0.47, -0.30, 0.44, 0.80])
    py = np.array([0.05, 0.06, 0.07, 0.08, 0.00, 0.40, 0.00])            # 0.40 beside, 0.80 too far
    assert zone_counts(px, py, 0.12, 0.55, 0.19) == (4, 1)


def test_ceiling_caps_at_once_holds_then_ramps_back():
    c = Ceiling(0.24, hold_s=0.3, accel=1.0)
    assert c.step(0.44, 0.88, False, False, 0.0) == (0.44, 0.88)
    v, w = c.step(0.44, 0.88, True, False, 0.05)                 # near: down at once, same curve
    assert v == 0.24 and abs(w / v - 2.0) < 1e-9
    assert c.step(0.44, 0.0, False, False, 0.30)[0] == 0.24      # clear, but still holding
    v1 = c.step(0.44, 0.0, False, False, 0.40)[0]                # holding over: lifts 1.0 m/s^2
    v2 = c.step(0.44, 0.0, False, False, 0.45)[0]
    assert 0.24 < v1 < v2 <= 0.44 and abs(v2 - v1 - 0.05) < 1e-9
    for i in range(10):
        v = c.step(0.44, 0.0, False, False, 0.5 + 0.05 * i)[0]
    assert v == 0.44 and c.lim is None


def test_ceiling_flicker_does_not_release():
    c = Ceiling(0.24, hold_s=0.3, accel=1.0)
    for i in range(20):                                           # near every other scan
        v = c.step(0.44, 0.0, i % 2 == 0, False, 0.1 * i)[0]
        assert v == 0.24


def test_ceiling_drops_when_she_changes_direction():
    c = Ceiling(0.24, hold_s=0.3, accel=1.0)
    c.step(0.44, 0.0, True, False, 0.0)
    assert c.step(-0.40, 0.0, False, False, 0.05) == (-0.40, 0.0)     # nothing behind
    assert c.step(-0.40, 0.0, False, True, 0.10)[0] == -0.24           # something behind
