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
a percentage from the LiPo discharge curve, and a status. ``battery/level``
(latched, JSON) is the verdict in a word, by Steve's rule on the last minute
of readings (judge_level): ``low`` when every reading was under
``warn_v_per_cell`` with no recovery, ``soon`` when it was under on average but
still climbed above between sags, else ``ok``; ``flat`` below
``stop_v_per_cell`` (a 5 s average), when it also raises the ``e_stop`` lock
every second, which twist_mux honours until the pack is
charged - GO on the page does not win an argument with a flat battery. At or
below ``poweroff_v_per_cell`` for ``poweroff_after_s`` it powers her off
(``poweroff_cmd``): parked she still draws ~20 W, and a forgotten pack goes flat.

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

import json
import shlex
import subprocess
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

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


def judge_reading(v_slow, recent, voltage: float, jump_v: float = 2.0, agree_v: float = 0.3):
    """Whether an instant reading counts, and whether the slow average restarts from it.

    Returns (counts, restart): restart is the value to restart the average at, or None
    to update it normally. A reading more than jump_v from the average is a spike and does
    not count - 2026-09-30, on the floor: one 7.37 V between 12 V readings (the INA219's
    config register read in place of the bus voltage) became the judged voltage, "flat",
    motors locked - unless the last three readings (recent, this one last) agree within
    agree_v: then the pack was swapped and the average restarts at their mean.
    """
    if v_slow is None:
        return True, voltage
    if abs(voltage - v_slow) <= jump_v:
        return True, None
    if len(recent) >= 3 and max(recent[-3:]) - min(recent[-3:]) <= agree_v:
        return True, sum(recent[-3:]) / 3.0
    return False, None


def judge_level(readings, warn_v: float, window_s: float = 60.0):
    """Steve's rule (2026-09-30) on the last minute of instant readings [(t, volts)]:
    "If it's low for over a minute straight with no sag, then it's low. If it's low for
    over a minute with sag, then it's gonna be low pretty soon."
    -> 'low'   every reading of a full minute is under warn_v: nothing brings it back
       'soon'  a full minute in which readings have been under warn_v, but between the sags
               it still climbs above: the pack at rest is just over the line
       'ok'    otherwise, or less than a minute of readings yet"""
    if len(readings) < 2 or readings[-1][0] - readings[0][0] < window_s - 1.0:
        return 'ok'
    volts = [v for _, v in readings]
    if max(volts) <= warn_v:
        return 'low'
    # under the line for the whole minute on average, above it at the peaks: sag
    if sum(volts) / len(volts) <= warn_v and min(volts) <= warn_v:
        return 'soon'
    return 'ok'


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
        # 2026-09-29: parked she still draws ~20.7 W at the battery (Steve's meter; the Jetson
        # module is 9.5 W of it), so a forgotten Rosie empties a 5200 mAh 3S pack in ~2.5 h and
        # nothing turned her off. Now a pack at or below poweroff_v_per_cell for poweroff_after_s
        # (driving sag passes; a reading 0.05 V/cell above resets it) powers her off, cleanly.
        self.declare_parameter('poweroff', True)
        self.declare_parameter('poweroff_v_per_cell', 3.3)
        self.declare_parameter('poweroff_after_s', 60.0)
        self.declare_parameter('poweroff_cmd', 'sudo -n systemctl poweroff')
        self.declare_parameter('simulate_start_v', 12.4)
        self.declare_parameter('present_above_v', 6.0)
        self.declare_parameter('rate', 2.0)
        self.declare_parameter('slow_s', 5.0)            # the thresholds judge this long an average
        self.declare_parameter('level_window_s', 60.0)   # judge_level: a minute of readings
        self.declare_parameter('simulate', False)

        self.address = int(self.get_parameter('address').value)
        self.shunt = float(self.get_parameter('shunt_ohms').value)
        self.voltage_only = bool(self.get_parameter('voltage_only').value)
        self.cells = int(self.get_parameter('cells').value)
        self.warn_v = float(self.get_parameter('warn_v_per_cell').value) * self.cells
        self.stop_v = float(self.get_parameter('stop_v_per_cell').value) * self.cells
        self.present_v = float(self.get_parameter('present_above_v').value)
        self.simulate = bool(self.get_parameter('simulate').value)
        self.poweroff_on = bool(self.get_parameter('poweroff').value)
        self.poweroff_v = float(self.get_parameter('poweroff_v_per_cell').value) * self.cells
        self.poweroff_after = float(self.get_parameter('poweroff_after_s').value)
        self.poweroff_cmd = str(self.get_parameter('poweroff_cmd').value)
        self.rate = float(self.get_parameter('rate').value)
        self.slow_s = float(self.get_parameter('slow_s').value)
        self._v_slow = None
        self._low_since = None
        self._powering_off = False
        self.window_s = float(self.get_parameter('level_window_s').value)
        self.readings = deque()          # (monotonic t, instant volts), the last minute
        self._recent = deque(maxlen=3)   # the last three instant readings, for judge_reading
        self.level = None                # none | ok | soon | low | flat
        self._level_sent = 0.0

        self.pub = self.create_publisher(BatteryState, 'battery', 10)
        # the verdict in a word, latched: sounds plays the countdown on it, the page shows it
        self.level_pub = self.create_publisher(
            String, 'battery/level', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.stop_pub = self.create_publisher(Bool, 'e_stop', 10)
        self.bus = None
        self._last_error = 0.0
        self._warned = False
        self._stopped = False
        self._sim_v = float(self.get_parameter('simulate_start_v').value)
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
                # the first time a warning, then a quiet note every 10 min: the board
                # may simply not be fitted yet (2026-09-27: 165 warnings in 80 minutes)
                if not self._last_error:
                    self.get_logger().warning(f'no INA219 at 0x{self.address:02x}: {exc} (retrying quietly)')
                elif now - self._last_error > 600:
                    self.get_logger().info(f'still no INA219 at 0x{self.address:02x}')
                else:
                    return False
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

        # The thresholds judge a slow average (about 5 s), not the instant: a motor stall
        # sags a 3S pack by half a volt for a second, and that must not lock her at 9.9 V
        # from a pack resting at 10.3. The message carries the instant reading.
        self._recent.append(voltage)
        counts, restart = judge_reading(self._v_slow, list(self._recent), voltage)
        if not counts:
            # 2026-09-30: a single 7.37 V between 12 V readings (the INA219's config register
            # read in place of the bus voltage) reset the average, said "flat" and locked
            # the motors on the floor. A jump only counts when three readings agree.
            self.get_logger().warning(f'INA219 reading {voltage:.2f} V ignored: {self._v_slow:.2f} V a moment ago '
                                      '(a spike, or the wrong register on the bus)')
            return
        if restart is not None:
            self._v_slow = restart                    # first reading, or a pack swapped
        else:
            self._v_slow += (voltage - self._v_slow) * min(1.0, 1.0 / (self.slow_s * self.rate))
        judged = self._v_slow
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(voltage)
        msg.current = -float(current) if current == current else float('nan')   # NaN stays NaN
        msg.design_capacity = float('nan')
        msg.capacity = float('nan')
        msg.charge = float('nan')
        msg.temperature = float('nan')
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        present = judged > self.present_v
        msg.present = present
        if present:
            msg.percentage = percent(judged / self.cells) / 100.0
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            if judged <= self.stop_v:
                msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
            else:
                msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        else:
            msg.percentage = float('nan')
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        self.pub.publish(msg)

        now = time.monotonic()
        self.readings.append((now, float(voltage)))
        while self.readings and now - self.readings[0][0] > self.window_s:
            self.readings.popleft()
        if not present:
            self._warned = self._stopped = False
            self._low_since = None
            self.readings.clear()
            self._say_level('none', 'no battery (under %.1f V)' % self.present_v, voltage)
            return
        self._poweroff_check(judged)
        if judged <= self.stop_v:
            if not self._stopped:
                self.get_logger().error(f'battery {judged:.2f} V is below {self.stop_v:.2f} V: stopping the robot')
                self._stopped = True
            stop = Bool()
            stop.data = True
            self.stop_pub.publish(stop)
            self._say_level('flat', f'{judged:.2f} V, under {self.stop_v:.2f}: motors locked', voltage)
            return
        if self._stopped:
            # twist_mux keeps the last value of a lock: what we locked, we must unlock
            # (2026-09-30: the lock outlived the reading that set it)
            release = Bool()
            release.data = False
            self.stop_pub.publish(release)
            self.get_logger().warning(f'battery {judged:.2f} V is back above {self.stop_v:.2f} V: motors released')
        self._stopped = False
        level = judge_level(self.readings, self.warn_v, self.window_s)
        volts = [v for _, v in self.readings]
        why = {'low': f'under {self.warn_v:.1f} V for a minute with no recovery (peak {max(volts):.2f})',
               'soon': f'under {self.warn_v:.1f} V on average for a minute, but still up to {max(volts):.2f} between sags',
               'ok': f'{judged:.2f} V'}[level]
        self._say_level(level, why, voltage)
        if level == 'low' and not self._warned:
            self.get_logger().warning(f'battery low: {why}: head home')
            self._warned = True
        elif level == 'ok':
            self._warned = False

    def _say_level(self, level: str, why: str, volts: float) -> None:
        """battery/level on every change, and again every 10 s."""
        now = time.monotonic()
        if level == self.level and now - self._level_sent < 10.0:
            return
        if level != self.level:
            self.get_logger().info(f'battery level {level}: {why}')
        self.level, self._level_sent = level, now
        msg = String()
        msg.data = json.dumps({'level': level, 'why': why, 'volts': round(volts, 2),
                               'avg': round(self._v_slow or 0.0, 2)})
        self.level_pub.publish(msg)


    def _poweroff_check(self, voltage: float) -> None:
        if not self.poweroff_on or self._powering_off:
            return
        now = time.monotonic()
        if voltage <= self.poweroff_v:
            if self._low_since is None:
                self._low_since = now
                self.get_logger().error(f'battery {voltage:.2f} V is at or below {self.poweroff_v:.2f} V: '
                                        f'powering off in {self.poweroff_after:.0f} s unless it recovers')
            elif now - self._low_since >= self.poweroff_after:
                self._powering_off = True
                self.get_logger().error(f'battery {voltage:.2f} V for {self.poweroff_after:.0f} s: powering off now '
                                        f'to save the pack ({self.poweroff_cmd})')
                stop = Bool()
                stop.data = True
                self.stop_pub.publish(stop)
                try:
                    subprocess.Popen(shlex.split(self.poweroff_cmd))
                except OSError as exc:
                    self.get_logger().error(f'could not power off: {exc}')
        elif voltage > self.poweroff_v + 0.05 * self.cells and self._low_since is not None:
            self.get_logger().warning(f'battery back to {voltage:.2f} V: not powering off')
            self._low_since = None


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
