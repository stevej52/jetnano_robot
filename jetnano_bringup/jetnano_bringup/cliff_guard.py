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

"""Drops: four VL53L0X time-of-flight sensors looking straight down at the corners.

    ros2 run jetnano_bringup cliff_guard

The sensors all wake up at I2C address 0x29, so they sit behind a TCA9548A
multiplexer, one per channel. Each is read in turn (~30 ms each); the floor
distance is learned per sensor from the first readings at start-up (the robot
must be on flat floor then - its parking spot), and a reading ``drop_mm``
longer than that, or no return at all, is a drop.

Rather than stop the robot itself, this publishes the drops as obstacles the
collision guard already understands: a small cluster of points just outside
the corner that sees the drop, on ``cliff/points`` (PointCloud2 in
base_footprint). A drop at a front corner then lands in the guard's forward
stop zone - forward is refused, reversing is allowed - and the guard's page
indicator, its switch and its fail-safe (source_timeout) all apply. The
cloud is published every cycle, empty when all is well, so the guard can
tell "no drop" from "sensor dead". drive.launch.py cliff:=true adds the
source to the guard.

``simulate: true`` reads nothing and publishes drops for the corners listed
in ``simulated_drops`` - for testing the guard's reaction without the
hardware. ``cliff/ranges`` (Float32MultiArray, mm per sensor) is for a
human watching.

The VL53L0X driver is Adafruit's (adafruit-circuitpython-vl53l0x, with the
tca9548a and extended-bus packages), in a venv with system site-packages so
rclpy is still importable: sensors.launch.py runs this node with that venv's
python. Nothing is installed into the system Python.
"""

import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Float32MultiArray

NO_RETURN = 8000    # the VL53L0X reports 8190/8191 when nothing is in range


class Sensors:
    """The four sensors behind the multiplexer. Import errors surface here, not at import time."""

    def __init__(self, bus: int, mux_address: int, channels):
        from adafruit_extended_bus import ExtendedI2C
        import adafruit_tca9548a
        import adafruit_vl53l0x
        i2c = ExtendedI2C(bus)
        mux = adafruit_tca9548a.TCA9548A(i2c, address=mux_address)
        self.devices = []
        for ch in channels:
            dev = adafruit_vl53l0x.VL53L0X(mux[ch])
            dev.measurement_timing_budget = 33000   # us: ~30 Hz, default accuracy
            self.devices.append(dev)

    def read_mm(self):
        return [float(d.range) for d in self.devices]


class CliffGuard(Node):

    def __init__(self):
        super().__init__('cliff_guard')
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('mux_address', 0x70)
        self.declare_parameter('names', ['front_left', 'front_right', 'rear_left', 'rear_right'])
        self.declare_parameter('channels', [0, 1, 2, 3])
        # Where each sensor is, base_footprint, x forward y left (the body is 0.44 x 0.22).
        self.declare_parameter('xs', [0.20, 0.20, -0.20, -0.20])
        self.declare_parameter('ys', [0.09, -0.09, 0.09, -0.09])
        self.declare_parameter('drop_mm', 60.0)
        self.declare_parameter('calibration_samples', 20)
        self.declare_parameter('rate', 8.0)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('point_height', 0.2)       # inside the guard's source min/max height
        self.declare_parameter('simulate', False)
        self.declare_parameter('simulated_drops', [''])

        self.names = [str(n) for n in self.get_parameter('names').value]
        self.xs = [float(v) for v in self.get_parameter('xs').value]
        self.ys = [float(v) for v in self.get_parameter('ys').value]
        self.drop_mm = float(self.get_parameter('drop_mm').value)
        self.n_cal = int(self.get_parameter('calibration_samples').value)
        self.frame = str(self.get_parameter('base_frame').value)
        self.z = float(self.get_parameter('point_height').value)
        self.simulate = bool(self.get_parameter('simulate').value)
        self.sim_drops = {str(s) for s in self.get_parameter('simulated_drops').value if s}

        self.points_pub = self.create_publisher(PointCloud2, 'cliff/points', 10)
        self.ranges_pub = self.create_publisher(Float32MultiArray, 'cliff/ranges', 10)
        self.drop_pub = self.create_publisher(Bool, 'cliff/drop', 10)

        self.sensors = None
        self.floor = None            # per-sensor floor distance, mm
        self.samples = [[] for _ in self.names]
        self._last_error = 0.0
        self._dropped = set()
        if not self.simulate:
            self._open()
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)

    def _open(self) -> bool:
        try:
            self.sensors = Sensors(int(self.get_parameter('i2c_bus').value),
                                   int(self.get_parameter('mux_address').value),
                                   [int(c) for c in self.get_parameter('channels').value])
            self.get_logger().info(f'{len(self.names)} VL53L0X behind the TCA9548A; learning the floor '
                                   f'from the first {self.n_cal} readings - the robot must be on flat floor')
            return True
        except Exception as exc:  # noqa: B902 - missing package or hardware: say so, keep trying
            self.sensors = None
            now = time.monotonic()
            if now - self._last_error > 30:
                self.get_logger().warning(f'cliff sensors not available: {exc} (retrying)')
                self._last_error = now
            return False

    def _read(self):
        if self.simulate:
            return [NO_RETURN + 190.0 if n in self.sim_drops else 85.0 for n in self.names]
        if self.sensors is None and not self._open():
            return None
        try:
            return self.sensors.read_mm()
        except Exception as exc:  # noqa: B902
            self.get_logger().warning(f'cliff sensor read failed: {exc}')
            self.sensors = None
            return None

    def _tick(self):
        ranges = self._read()
        if ranges is None:
            return                     # nothing published: the guard's source_timeout stops the robot
        if self.floor is None:
            for buf, r in zip(self.samples, ranges):
                if r < NO_RETURN:
                    buf.append(r)
            if all(len(b) >= self.n_cal for b in self.samples):
                self.floor = [sorted(b)[len(b) // 2] for b in self.samples]
                self.get_logger().info('floor learned: ' + ', '.join(
                    f'{n} {f:.0f} mm' for n, f in zip(self.names, self.floor)))
            elif not self.simulate:
                return
            else:
                self.floor = [85.0] * len(self.names)

        dropped = [r >= NO_RETURN or r > f + self.drop_mm for r, f in zip(ranges, self.floor)]
        now_set = {n for n, d in zip(self.names, dropped) if d}
        for n in now_set - self._dropped:
            self.get_logger().warning(f'DROP at {n}')
        for n in self._dropped - now_set:
            self.get_logger().info(f'{n} sees floor again')
        self._dropped = now_set

        # One cluster of points 5 cm outside each dropped corner, along the
        # corner's own side, so it lands in the guard's forward/backward zone.
        pts = []
        for n, x, y, d in zip(self.names, self.xs, self.ys, dropped):
            if d:
                ahead = 1.0 if x > 0 else -1.0
                for dx in (0.05, 0.10, 0.15):
                    for dy in (-0.05, 0.0, 0.05):
                        pts.append((x + ahead * dx, y + dy, self.z))
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = self.frame
        cloud.height, cloud.width = 1, len(pts)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate('xyz')]
        cloud.point_step, cloud.row_step = 12, 12 * len(pts)
        cloud.is_dense, cloud.is_bigendian = True, False
        cloud.data = b''.join(struct.pack('<fff', *p) for p in pts)
        self.points_pub.publish(cloud)

        arr = Float32MultiArray()
        arr.data = [float(r) for r in ranges]
        self.ranges_pub.publish(arr)
        flag = Bool()
        flag.data = bool(now_set)
        self.drop_pub.publish(flag)


def main(args=None):
    rclpy.init(args=args)
    node = CliffGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
