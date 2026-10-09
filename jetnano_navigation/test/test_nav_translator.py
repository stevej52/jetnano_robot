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
