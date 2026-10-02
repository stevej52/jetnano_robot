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
