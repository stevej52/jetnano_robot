import numpy as np

from jetnano_navigation.near_cap import cap_twist, zone_counts


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
