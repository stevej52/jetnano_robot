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

"""bno055_lean publishes the same imu/imu as the stock driver, from the same bytes, and
skips the other messages only while nobody subscribes. Needs ros-jazzy-bno055."""

import random

import pytest

bno055 = pytest.importorskip('bno055')

from bno055.sensor.SensorService import SensorService  # noqa: E402
from jetnano_bringup.bno055_lean import LeanSensorService  # noqa: E402


class _Value:
    def __init__(self, value):
        self.value = value


class _Params:
    frame_id = _Value('imu_link')
    acc_factor = _Value(100.0)
    gyr_factor = _Value(900.0)
    mag_factor = _Value(16000000.0)
    grav_factor = _Value(100.0)
    variance_orientation = _Value([0.0159, 0.0159, 0.0159])
    variance_acc = _Value([0.017, 0.017, 0.017])
    variance_angular_vel = _Value([0.04, 0.04, 0.04])
    variance_mag = _Value([0.0, 0.0, 0.0])


class _Publisher:
    def __init__(self, subscribers):
        self.subscribers = subscribers
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)

    def get_subscription_count(self):
        return self.subscribers


class _Clock:
    class _Now:
        def to_msg(self):
            from builtin_interfaces.msg import Time
            return Time(sec=1, nanosec=2)

    def now(self):
        return self._Now()


class _Node:
    def get_clock(self):
        return _Clock()


class _Connector:
    def __init__(self, data):
        self.data = data

    def receive(self, reg_addr, length):
        assert length == 45
        return bytearray(self.data)


def _service(cls, data, subscribers):
    service = cls.__new__(cls)  # no ROS node: just the fields get_sensor_data uses
    service.node = _Node()
    service.con = _Connector(data)
    service.param = _Params()
    for name in ('pub_imu', 'pub_imu_raw', 'pub_mag', 'pub_grav', 'pub_temp'):
        setattr(service, name, _Publisher(subscribers))
    return service


def _imu_fields(msg):
    return (msg.header.frame_id,
            tuple(getattr(msg.orientation, a) for a in 'xyzw'),
            tuple(getattr(msg.linear_acceleration, a) for a in 'xyz'),
            tuple(getattr(msg.angular_velocity, a) for a in 'xyz'),
            tuple(msg.orientation_covariance), tuple(msg.linear_acceleration_covariance),
            tuple(msg.angular_velocity_covariance))


@pytest.mark.parametrize('seed', range(20))
def test_same_messages_as_the_stock_driver(seed):
    rng = random.Random(seed)
    data = bytes(rng.randrange(256) for _ in range(45))
    stock = _service(SensorService, data, subscribers=1)
    lean = _service(LeanSensorService, data, subscribers=1)
    stock.get_sensor_data()
    lean.get_sensor_data()
    assert _imu_fields(lean.pub_imu.sent[0]) == _imu_fields(stock.pub_imu.sent[0])
    assert _imu_fields(lean.pub_imu_raw.sent[0]) == _imu_fields(stock.pub_imu_raw.sent[0])
    s, m = stock.pub_mag.sent[0], lean.pub_mag.sent[0]
    assert [getattr(m.magnetic_field, a) for a in 'xyz'] == \
        [getattr(s.magnetic_field, a) for a in 'xyz']
    assert tuple(m.magnetic_field_covariance) == tuple(s.magnetic_field_covariance)
    s, m = stock.pub_grav.sent[0], lean.pub_grav.sent[0]
    assert (m.x, m.y, m.z) == (s.x, s.y, s.z)
    assert lean.pub_temp.sent[0].temperature == stock.pub_temp.sent[0].temperature


def test_unread_topics_are_skipped():
    data = bytes(range(1, 46))
    lean = _service(LeanSensorService, data, subscribers=0)
    lean.get_sensor_data()
    assert len(lean.pub_imu.sent) == 1
    for name in ('pub_imu_raw', 'pub_mag', 'pub_grav', 'pub_temp'):
        assert getattr(lean, name).sent == []


def test_all_zero_quaternion_still_raises_for_the_stock_main_to_skip():
    lean = _service(LeanSensorService, bytes(45), subscribers=0)
    with pytest.raises(ZeroDivisionError):
        lean.get_sensor_data()
