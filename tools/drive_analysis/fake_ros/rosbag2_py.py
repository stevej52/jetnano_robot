"""Test stand-in for rosbag2_py on a machine without ROS: just enough of SequentialReader
for jetnano_bringup's drive_report, over one MCAP file (mcap + mcap-ros2-support)."""
import glob
import os
from types import SimpleNamespace

from mcap.reader import make_reader


class StorageOptions:
    def __init__(self, uri, storage_id=''):
        self.uri = uri


class ConverterOptions:
    def __init__(self, *a):
        pass


class StorageFilter:
    def __init__(self, topics):
        self.topics = list(topics)


class SequentialReader:
    def open(self, storage, converter):
        path = storage.uri
        if os.path.isdir(path):
            path = glob.glob(os.path.join(path, '*.mcap'))[0]
        self.path = path
        with open(path, 'rb') as f:
            s = make_reader(f).get_summary()
        self.channels = s.channels
        self.schemas = s.schemas
        self.counts = s.statistics.channel_message_counts
        self.topics = None
        self._it = None

    def get_all_topics_and_types(self):
        return [SimpleNamespace(name=c.topic, type=self.schemas[c.schema_id].name) for c in self.channels.values()]

    def get_metadata(self):
        return SimpleNamespace(topics_with_message_count=[
            SimpleNamespace(topic_metadata=SimpleNamespace(name=c.topic), message_count=self.counts.get(cid, 0))
            for cid, c in self.channels.items()])

    def set_filter(self, f):
        self.topics = f.topics

    def _gen(self):
        with open(self.path, 'rb') as f:
            for schema, channel, msg in make_reader(f).iter_messages(topics=self.topics):
                yield channel.topic, (schema, msg.data), msg.log_time

    def has_next(self):
        if self._it is None:
            self._it = self._gen()
            self._next = next(self._it, None)
        return self._next is not None

    def read_next(self):
        out = self._next
        self._next = next(self._it, None)
        return out
