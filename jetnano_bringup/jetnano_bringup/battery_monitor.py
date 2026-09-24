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

"""The battery, from an INA219 on the I2C bus.

    ros2 run jetnano_bringup battery_monitor

Publishes sensor_msgs/BatteryState on ``battery`` twice a second: pack
voltage, current (negative while discharging, per the message's convention),
a percentage from the LiPo discharge curve, and a status. Below
``warn_v_per_cell`` it warns; below ``stop_v_per_cell`` it raises the
``e_stop`` lock every second, which twist_mux honours until the pack is
charged - GO on the page does not win an argument with a flat battery.

Wiring notes that cost a board if ignored:

- The INA219's default address 0x40 is the PCA9685's. Bridge A0 -> 0x41.
- The breakout's 0.1 ohm shunt is good for ~3.2 A. The motor rail pulls ten
  times that on a stall. Either measure voltage only (battery + to Vin-,
  nothing through the shunt; ``voltage_only: true``) or put the board on the
  electronics feed. A motor-rail current reading needs an external 10 mohm
  shunt and ``shunt_ohms`` to match.

A pack that is not there (the robot on its wall supply) reads under 6 V and
is reported as "no battery"; nothing is raised for that.

``simulate: true`` publishes a slowly draining pack for testing the rest of
the system without the board.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

try:
    import smbus
except ImportError:  # pragma: no cover
    smbus = None

# INA219 registers (16-bit, big-endian on the wire)
REG_CONFIG, REG_SHUNT, REG_BUS = 0x00, 0x01, 0x02
# 32 V bus range, +-320 mV shunt range (gain /8), 12-bit ADCs, continuous shunt+bus
CONFIG = 0x399F

# LiPo per-cell voltage -> remaining, roughly, at light load.
CURVE = [(4.20, 100), (4.10, 90), (4.00, 78), (3.90, 62), (3.80, 45), (3.70, 28),
         (3.60, 15), (3.50, 8), (3.40, 4), (3.30, 2), (3.00, 0)]


def percent(v_cell: float) -> float:
    if v_cell >= CURVE[0][0]:
        return 100.0
    for (v_hi, p_hi), (v_lo, p_lo) in zip(CURVE, CURVE[1:]):
        if v_cell >= v_lo:
            return p_lo + (p_hi - p_lo) * (v_cell - v_lo) / (v_hi - v_lo)
    return 0.0


def swap16(word: int) -> int:
    return ((word & 0xFF) << 8) | (word >> 8)


class BatteryMonitor(Node):

    def __init__(self):
        super().__init__('battery_monitor')
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('address', 0x41)
        self.declare_parameter('shunt_ohms', 0.1)
        self.declare_parameter('voltage_only', False)
        self.declare_parameter('cells', 3)
        self.declare_parameter('warn_v_per_cell', 3.5)
        self.declare_parameter('stop_v_per_cell', 3.3)
        self.declare_parameter('present_above_v', 6.0)
        self.declare_parameter('rate', 2.0)
        self.declare_parameter('simulate', False)

        self.address = int(self.get_parameter('address').value)
        self.shunt = float(self.get_parameter('shunt_ohms').value)
        self.voltage_only = bool(self.get_parameter('voltage_only').value)
        self.cells = int(self.get_parameter('cells').value)
        self.warn_v = float(self.get_parameter('warn_v_per_cell').value) * self.cells
        self.stop_v = float(self.get_parameter('stop_v_per_cell').value) * self.cells
        self.present_v = float(self.get_parameter('present_above_v').value)
        self.simulate = bool(self.get_parameter('simulate').value)

        self.pub = self.create_publisher(BatteryState, 'battery', 10)
        self.stop_pub = self.create_publisher(Bool, 'e_stop', 10)
        self.bus = None
        self._last_error = 0.0
        self._warned = False
        self._stopped = False
        self._sim_v = 12.4
        if not self.simulate:
            self._open()
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)

    def _open(self) -> bool:
        if smbus is None:
            self.get_logger().error('python3-smbus is not installed')
            return False
        try:
            self.bus = smbus.SMBus(int(self.get_parameter('i2c_bus').value))
            self.bus.write_word_data(self.address, REG_CONFIG, swap16(CONFIG))
            self.get_logger().info(f'INA219 at 0x{self.address:02x} on bus {self.get_parameter("i2c_bus").value}'
                                   + (' (voltage only)' if self.voltage_only else f', shunt {self.shunt} ohm'))
            return True
        except OSError as exc:
            self.bus = None
            now = time.monotonic()
            if now - self._last_error > 30:
                self.get_logger().warning(f'no INA219 at 0x{self.address:02x}: {exc} (retrying)')
                self._last_error = now
            return False

    def _read(self):
        """(voltage V, current A) - current positive = discharging."""
        raw_bus = swap16(self.bus.read_word_data(self.address, REG_BUS))
        voltage = (raw_bus >> 3) * 0.004
        if self.voltage_only:
            return voltage, float('nan')
        raw_shunt = swap16(self.bus.read_word_data(self.address, REG_SHUNT))
        if raw_shunt > 0x7FFF:
            raw_shunt -= 0x10000
        return voltage, raw_shunt * 1e-5 / self.shunt

    def _tick(self):
        if self.simulate:
            self._sim_v = max(9.0, self._sim_v - 0.002)
            voltage, current = self._sim_v, 1.2
        else:
            if self.bus is None and not self._open():
                return
            try:
                voltage, current = self._read()
            except OSError as exc:
                self.get_logger().warning(f'INA219 read failed: {exc}')
                self.bus = None
                return

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(voltage)
        msg.current = -float(current) if current == current else float('nan')   # NaN stays NaN
        msg.design_capacity = float('nan')
        msg.capacity = float('nan')
        msg.charge = float('nan')
        msg.temperature = float('nan')
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        present = voltage > self.present_v
        msg.present = present
        if present:
            msg.percentage = percent(voltage / self.cells) / 100.0
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            if voltage <= self.stop_v:
                msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
            elif voltage <= self.warn_v:
                msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            else:
                msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        else:
            msg.percentage = float('nan')
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        self.pub.publish(msg)

        if not present:
            self._warned = self._stopped = False
            return
        if voltage <= self.stop_v:
            if not self._stopped:
                self.get_logger().error(f'battery {voltage:.2f} V is below {self.stop_v:.2f} V: stopping the robot')
                self._stopped = True
            stop = Bool()
            stop.data = True
            self.stop_pub.publish(stop)
        elif voltage <= self.warn_v:
            if not self._warned:
                self.get_logger().warning(f'battery {voltage:.2f} V ({msg.percentage * 100:.0f} %): head home')
                self._warned = True
        else:
            self._warned = self._stopped = False


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
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
