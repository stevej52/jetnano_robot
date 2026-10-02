"""meet: the geometry and choices that need no robot."""
import math

from jetnano_navigation import meet


def test_nearest_needs_a_distance():
    msg = {'people': [{'id': 0, 'dist_m': None}, {'id': 1, 'dist_m': 2.4}, {'id': 2, 'dist_m': 1.1}]}
    assert meet.nearest(msg)['id'] == 2
    assert meet.nearest({'people': [{'id': 0, 'dist_m': None}]}) is None
    assert meet.nearest({'people': []}) is None


def test_stand_off_stops_short_facing_them():
    x, y, h = meet.stand_off((0.0, 0.0), (3.0, 0.0), 1.2)
    assert (round(x, 3), round(y, 3)) == (1.8, 0.0) and abs(h) < 1e-9
    x, y, h = meet.stand_off((1.0, 1.0), (1.0, 4.0), 1.2)
    assert (round(x, 3), round(y, 3)) == (1.0, 2.8) and abs(h - math.pi / 2) < 1e-9


def test_stand_off_never_backs_past_her():
    x, y, _ = meet.stand_off((0.0, 0.0), (0.5, 0.0), 1.2)
    assert (round(x, 3), round(y, 3)) == (0.0, 0.0)


def test_head_step_is_smooth_and_clamped():
    pan, tilt = meet.head_step((0.0, 0.0), (40.0, -20.0))
    assert (pan, tilt) == (20.0, -10.0)
    pan, tilt = meet.head_step((100.0, 70.0), (200.0, 200.0), gain=1.0)
    assert (pan, tilt) == (meet.PAN_RANGE[1], meet.TILT_RANGE[1])
