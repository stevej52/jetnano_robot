from jetnano_navigation.nav_translator import SPEED_FWD, scrub_factor, throttle_for


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
    """Drives 52-54: she goes straight at -0.20, not 0."""
    from types import SimpleNamespace
    from jetnano_navigation.nav_translator import NavTranslator
    n = SimpleNamespace(max_steer=2.4, steer_trim=-0.20, steer=0.0)
    assert abs(NavTranslator.steered(n) + 0.20) < 1e-9
    n.steer = -2.4
    assert NavTranslator.steered(n) == -2.4                 # never past full lock
    n.steer = 2.4
    assert abs(NavTranslator.steered(n) - 2.2) < 1e-9
