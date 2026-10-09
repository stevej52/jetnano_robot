import numpy as np

from jetnano_bringup.buzz_source import prominence, score, verdict


def test_prominence_sees_a_tone_and_not_noise():
    mask = np.ones(40, bool)
    flat = np.ones(40)
    tone = flat.copy()
    tone[[10, 11, 12]] = 100.0
    assert abs(prominence(flat, mask)) < 1e-9
    assert abs(prominence(tone, mask) - 20.0) < 1e-9


def test_verdict_names_the_output_that_quiets_it():
    assert verdict([15, 17, 16], [5, 6, 4]).startswith('SOURCE')
    assert verdict([15, 17, 16], [15, 16, 17]).startswith('not it')
    assert verdict([4, 5], [4, 5]) == 'no buzz heard to judge by'
    assert verdict([None], [None]) == 'no data'


def test_score_counts_only_trials_heard_and_buzzing_before():
    nb, nq, aq, n, med = score([(15, 5), (16, 12), (5, 4), (None, 3)])
    assert (nb, nq, aq, n) == (2, 1, 2, 3)
    assert med == 5
    assert score([(None, None)]) is None
