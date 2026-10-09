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
    steps = plan(-0.327, 0.4, 0.3, 0.05)
    assert steps[:8] == [-0.327] * 8 and steps[8:] == [0.0] * 6
