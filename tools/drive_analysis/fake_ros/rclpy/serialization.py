"""Test stand-in: decode with the schema the fake rosbag2_py hands over alongside the bytes."""
from mcap_ros2.decoder import DecoderFactory

_factory = DecoderFactory()
_decoders = {}


def deserialize_message(data, msg_type):
    schema, raw = data
    d = _decoders.get(schema.id)
    if d is None:
        d = _decoders[schema.id] = _factory.decoder_for('cdr', schema)
    return d(raw)
