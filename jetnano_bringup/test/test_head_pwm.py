"""head_pwm: the pan-tilt calibration carried over from pca9685.yaml (2026-09-26)."""
from jetnano_bringup.head_pwm import pulse_us

PAN = {'min_pulse_us': 700.0, 'max_pulse_us': 2450.0, 'min_limit': 5.0, 'max_limit': 180.0}
TILT = {'min_pulse_us': 750.0, 'max_pulse_us': 2250.0, 'min_limit': 12.0, 'max_limit': 168.0}


def test_dead_ahead_and_level_match_the_measured_pulses():
    assert abs(pulse_us(74.6, PAN) - 1425.3) < 0.5      # "that's dead ahead"
    assert abs(pulse_us(90.0, TILT) - 1500.0) < 0.01


def test_angles_are_clamped_to_the_clean_range():
    assert abs(pulse_us(0.0, PAN) - pulse_us(5.0, PAN)) < 1e-9          # never into the 600 us hunt
    assert abs(pulse_us(200.0, TILT) - 2150.0) < 0.5                    # 168 deg = 2150 us, camera well down
    assert abs(pulse_us(-10.0, TILT) - 850.0) < 0.5                     # 12 deg, camera well up
