"""watchdog: a node that leaves the graph while its process lives on is hung, and gets the
ladder; a node whose process died gets no action (its respawn brings it back)."""
import re

import pytest

pytest.importorskip('rclpy')
from jetnano_bringup import watchdog  # noqa: E402


def test_hung_node_gets_acted_on_dead_node_does_not():
    why, act = watchdog.node_gone_verdict(52.0, process_alive=True)
    assert act and 'hung' in why and '52 s' in why
    why, act = watchdog.node_gone_verdict(52.0, process_alive=False)
    assert not act and why == 'gone for 52 s'


def test_every_hung_candidate_is_a_watched_node_with_a_pgrep_pattern():
    for node, pattern in watchdog.NODE_PROCS.items():
        assert node in watchdog.NODES, node
        re.compile(pattern)
        assert pattern.startswith('/lib/') and pattern.endswith('( |$)'), pattern
    assert watchdog.NODE_PROCS['ekf_filter_node'] == '/lib/robot_localization/ekf_node( |$)'
