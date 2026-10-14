from types import SimpleNamespace

from jetnano_navigation.mission_cmd import controller_listening, taken


def node_with(*names):
    infos = [SimpleNamespace(node_name=n) for n in names]
    return SimpleNamespace(get_subscriptions_info_by_topic=lambda topic: infos)


def test_only_the_controller_counts_as_listening():
    """Drive 58: sensor_sleep matched first and the route went to nobody that acts on it."""
    assert not controller_listening(node_with())
    assert not controller_listening(node_with('sensor_sleep'))
    assert controller_listening(node_with('sensor_sleep', 'mission'))


def test_taken_means_the_status_shows_our_id():
    assert not taken(None, 'abc')
    assert not taken({'mission': 'idle', 'id': None}, 'abc')
    assert not taken({'mission': 'route', 'id': 'xyz'}, 'abc')
    assert taken({'mission': 'route', 'id': 'abc'}, 'abc')
