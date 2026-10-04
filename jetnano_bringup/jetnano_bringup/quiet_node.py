# Copyright 2026 stevej52
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A Node whose subscriptions and publishers carry no default QoS event handlers.

rclpy gives every subscription and publisher two default event handlers (incompatible QoS,
and the like), and its executor rebuilds the whole wait set - every subscription, timer,
service and event handler - on every wake. In housekeeping (four nodes, ~40 subscriptions,
~210 messages a second) that rebuild was 73 % of a core, all of it inside
`_wait_for_ready_callbacks` -> `event_handler.add_to_wait_set` (py-spy, 2026-10-03). The
default handlers only log; without them the wait set is a third the size.
"""

from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.node import Node


class QuietNode(Node):
    """Node without the default per-entity QoS event handlers."""

    def create_subscription(self, *args, **kwargs):
        kwargs.setdefault('event_callbacks', SubscriptionEventCallbacks(use_default_callbacks=False))
        return super().create_subscription(*args, **kwargs)

    def create_publisher(self, *args, **kwargs):
        kwargs.setdefault('event_callbacks', PublisherEventCallbacks(use_default_callbacks=False))
        return super().create_publisher(*args, **kwargs)
