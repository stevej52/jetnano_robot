from jetnano_navigation.nav_translator import (SPEED_FWD, TURN_SPEED_CAP, scrub_factor, throttle_for,
                                              turn_speed_cap)


def test_scrub_factor_falls_with_lock_as_measured():
    assert scrub_factor(0.0, 0.35) == 1.0
    assert abs(scrub_factor(0.6, 0.35) - 0.79) < 1e-9            # drives 50-52: 0.77 at 0.6
    assert abs(scrub_factor(1.0, 0.35) - 0.65) < 1e-9
    assert scrub_factor(2.0, 0.35) == scrub_factor(1.0, 0.35)     # past full lock: as full lock
    assert scrub_factor(1.0, 0.9) == 0.4                          # never asks for 2.5x


def test_turning_asks_the_table_for_more_throttle():
    straight = throttle_for(0.25, SPEED_FWD)
    at_lock = throttle_for(0.25 / scrub_factor(0.9, 0.35), SPEED_FWD)
    assert at_lock > straight + 0.02


def test_steering_trim_is_added_within_lock():
    """Drives 52-54 needed -0.20 (now in pca9685.yaml's homes); a trim still stays within lock."""
    from types import SimpleNamespace
    from jetnano_navigation.nav_translator import NavTranslator
    n = SimpleNamespace(max_steer=2.4, steer_trim=-0.20, steer=0.0)
    assert abs(NavTranslator.steered(n) + 0.20) < 1e-9
    n.steer = -2.4
    assert NavTranslator.steered(n) == -2.4                 # never past full lock
    n.steer = 2.4
    assert abs(NavTranslator.steered(n) - 2.2) < 1e-9


def test_turn_speed_cap_follows_the_measured_limits():
    """2026-10-14: above ~1 m radius she turns as asked; tighter only slower (drives 55, 59-62)."""
    assert turn_speed_cap(0.3, TURN_SPEED_CAP) is None            # gentle: no cap
    assert turn_speed_cap(-0.6, TURN_SPEED_CAP) is None
    assert abs(turn_speed_cap(1.0, TURN_SPEED_CAP) - 0.25) < 1e-9
    assert abs(turn_speed_cap(-1.0, TURN_SPEED_CAP) - 0.25) < 1e-9  # right turns the same
    assert abs(turn_speed_cap(0.8, TURN_SPEED_CAP) - 0.345) < 1e-9  # between points: linear
    assert turn_speed_cap(2.5, TURN_SPEED_CAP) == 0.12            # full lock: the slowest
    assert turn_speed_cap(1.0, []) is None                        # [] turns it off


def test_the_cap_only_ever_slows_her():
    for k in (0.0, 0.5, 0.7, 1.1, 1.4, 1.8, 3.0):
        cap = turn_speed_cap(k, TURN_SPEED_CAP)
        assert cap is None or 0.1 <= cap <= 0.44
    caps = [turn_speed_cap(k / 10, TURN_SPEED_CAP) or 0.44 for k in range(0, 30)]
    assert all(a >= b for a, b in zip(caps, caps[1:])), 'tighter never faster'

