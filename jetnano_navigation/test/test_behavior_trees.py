import os
import xml.etree.ElementTree as ET

import pytest

TREES = os.path.join(os.path.dirname(__file__), '..', 'behavior_trees')


@pytest.mark.parametrize('name', ['navigate_through_poses_no_spin.xml', 'navigate_to_pose_no_spin.xml'])
def test_start_occupied_backs_off_before_the_round_robin(name):
    """2026-10-14: on START_OCCUPIED she backs off 15 cm at once, before clear / wait / back-up."""
    root = ET.parse(os.path.join(TREES, name)).getroot()
    fallback = root.find(".//ReactiveFallback[@name='RecoveryFallback']")
    kids = [c.tag + ':' + c.get('name', '') for c in fallback]
    assert kids == ['GoalUpdated:', 'Sequence:StartOccupiedBackOff', 'RoundRobin:RecoveryActions']
    seq = fallback.find("Sequence[@name='StartOccupiedBackOff']")
    check, backup = list(seq)
    assert check.tag == 'AreErrorCodesPresent' and check.get('error_codes_to_check') == '205;305'
    assert backup.tag == 'BackUp' and float(backup.get('backup_dist')) == 0.15
    assert float(backup.get('backup_speed')) <= 0.10
