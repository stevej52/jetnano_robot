from jetnano_bringup.stop_settle import StopWatch, plan


def test_settles_once_after_driving_then_stopping():
    w = StopWatch(still_s=0.6, quiet_s=0.3)
    assert not w.due(10.0, False)                 # never drove: nothing to settle
    w.heard(0.0, True)
    w.heard(0.5, False)                           # a driver still talking (stop messages)
    assert not w.due(0.7, False)                  # stopped 0.7 s but a driver spoke 0.2 s ago
    assert w.due(0.9, False)
    assert not w.due(0.9, True)                   # a lock: never


def test_driving_again_restarts_the_wait():
    w = StopWatch(still_s=0.6, quiet_s=0.3)
    w.heard(0.0, True)
    w.heard(0.5, True)
    assert not w.due(1.0, False)
    assert w.due(1.2, False)


def test_plan_goes_past_centre_then_back():
    steps = plan(-0.327, 0.6, 0.5, 0.05)
    assert steps[:12] == [-0.327] * 12 and steps[12:] == [0.0] * 10


def test_a_masked_settle_goes_again_once_quiet():
    """Drive 56: nav_park's last zero took twist_mux 0.2 s into the hold."""
    w = StopWatch(still_s=0.6, quiet_s=0.3, max_tries=3)
    w.heard(0.0, True)
    assert w.due(1.0, False)
    w.start()
    assert not w.due(1.1, False)                  # once per stop
    w.heard(1.2, False)                           # park's zero, mid-settle
    assert w.masked()
    assert not w.due(1.4, False)                  # park spoke 0.2 s ago
    assert w.due(1.5, False)


def test_masked_settles_stop_at_max_tries_until_she_drives_again():
    w = StopWatch(still_s=0.6, quiet_s=0.3, max_tries=2)
    w.heard(0.0, True)
    w.start()
    assert w.masked()
    w.start()
    assert not w.masked()                         # a driver that keeps speaking: no endless twitching
    assert not w.due(9.0, False)
    w.heard(10.0, True)                           # she drives again: a fresh stop, fresh tries
    assert w.due(11.0, False)
    w.start()
    assert w.masked()
