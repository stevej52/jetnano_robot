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

"""Joystick teleoperation that works out for itself which controller is plugged in.

Two very different controllers drive this robot:

* a Thrustmaster HOTAS set, where the stick and the throttle are **two
  separate USB devices**, and
* an Xbox pad, which is one device with two sticks.

Rather than a program per controller, this node reads a profile file, looks at
what is actually attached, and uses the first profile whose required devices
are all present. Unplug one and plug in the other and it follows, because it
rescans whenever it has no usable profile.

It publishes ``geometry_msgs/Twist`` on ``cmd_vel_teleop`` and a
``std_msgs/Bool`` on ``e_stop``. Both go to twist_mux: the Twist as the
high-priority input, the Bool as a lock that blocks *everything* including
Nav2.

Safety, in the order it is applied every tick:

1. No profile matched, or a required device vanished  -> zero + e_stop true
2. No event from a device within ``timeout``          -> zero + e_stop true
3. E-stop button held                                 -> zero + e_stop true
4. Dead-man ("arm") button not held                   -> zero, e_stop false
5. Otherwise                                          -> the commanded Twist

The zero Twist is published continuously rather than simply stopping, so the
robot is actively told to stop rather than being left to time out. The driver's
own 0.5 s channel timeout sits underneath this as a second line of defence.

Axis and button numbers are **indices into the device's sorted capability
list**, which is exactly what ``ros2 run jetnano_teleop list_devices`` prints.
They are not raw evdev codes.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import evdev
except ImportError:  # pragma: no cover - checked at runtime with a clear message
    evdev = None


@dataclass
class AxisSpec:
    """How one physical axis maps to one robot control."""

    index: int
    scale: float = 1.0
    deadzone: float = 0.05
    expo: float = 0.0
    mode: str = 'centred'

    def shape(self, raw: float, lo: float, hi: float) -> float:
        """Normalise a raw axis reading, then apply deadzone, expo and scale."""
        if hi <= lo:
            return 0.0
        span = hi - lo
        if self.mode == 'unipolar':
            # A throttle lever: rest at one end, full travel at the other.
            value = (raw - lo) / span
        else:
            value = 2.0 * (raw - lo) / span - 1.0

        if abs(value) < self.deadzone:
            return 0.0
        # Rescale what is left so the usable range still reaches 1.0.
        sign = math.copysign(1.0, value)
        value = sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

        if self.expo > 0.0:
            value = sign * ((1.0 - self.expo) * abs(value) + self.expo * value * value)

        return max(-1.0, min(1.0, value * self.scale))


@dataclass
class DeviceSpec:
    """One physical device a profile expects to find."""

    role: str
    match: str
    required: bool = True
    axes: dict = field(default_factory=dict)
    buttons: dict = field(default_factory=dict)


@dataclass
class Profile:
    name: str
    devices: list


class OpenDevice:
    """An opened evdev device plus the decoding it needs."""

    def __init__(self, dev, spec: DeviceSpec):
        self.dev = dev
        self.spec = spec
        self.last_event = 0.0
        self.axis_raw: dict[int, float] = {}
        self.button_state: dict[int, bool] = {}

        caps = dev.capabilities(absinfo=True)
        abs_caps = caps.get(evdev.ecodes.EV_ABS, [])
        key_caps = caps.get(evdev.ecodes.EV_KEY, [])

        # Index -> evdev code, matching what list_devices prints.
        self.abs_codes = [code for code, _info in sorted(abs_caps, key=lambda c: c[0])]
        self.abs_range = {
            code: (info.min, info.max)
            for code, info in sorted(abs_caps, key=lambda c: c[0])
        }
        for code, info in sorted(abs_caps, key=lambda c: c[0]):
            self.axis_raw[code] = float(info.value)
        self.key_codes = sorted(key_caps)

    def code_for_axis(self, index: int):
        if 0 <= index < len(self.abs_codes):
            return self.abs_codes[index]
        return None

    def code_for_button(self, index: int):
        if 0 <= index < len(self.key_codes):
            return self.key_codes[index]
        return None

    def pump(self, now: float) -> bool:
        """Drain pending events. False means the device went away."""
        try:
            while True:
                event = self.dev.read_one()
                if event is None:
                    break
                if event.type == evdev.ecodes.EV_ABS:
                    self.axis_raw[event.code] = float(event.value)
                    self.last_event = now
                elif event.type == evdev.ecodes.EV_KEY:
                    self.button_state[event.code] = event.value != 0
                    self.last_event = now
        except (OSError, IOError):
            return False
        return True

    def axis(self, name: str) -> float | None:
        spec_dict = self.spec.axes.get(name)
        if spec_dict is None:
            return None
        spec = AxisSpec(**spec_dict)
        code = self.code_for_axis(spec.index)
        if code is None or code not in self.abs_range:
            return None
        lo, hi = self.abs_range[code]
        return spec.shape(self.axis_raw.get(code, 0.0), lo, hi)

    def button(self, name: str) -> bool | None:
        index = self.spec.buttons.get(name)
        if index is None:
            return None
        code = self.code_for_button(int(index))
        if code is None:
            return None
        return self.button_state.get(code, False)

    def close(self):
        try:
            self.dev.close()
        except Exception:
            pass


class TeleopNode(Node):

    def __init__(self):
        super().__init__('jetnano_teleop')

        default_profiles = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'joysticks.yaml')

        self.declare_parameter('profiles_file', default_profiles)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_teleop')
        self.declare_parameter('e_stop_topic', 'e_stop')
        self.declare_parameter('turbo_scale', 2.0)

        path = self.get_parameter('profiles_file').value
        self.config = self._load(path)

        self.publish_rate = float(self.config.get('publish_rate', 50.0))
        self.timeout = float(self.config.get('timeout', 0.5))
        self.max_linear = float(self.config.get('max_linear', 1.0))
        self.max_angular = float(self.config.get('max_angular', 3.0))
        self.turbo_scale = float(self.get_parameter('turbo_scale').value)

        self.profiles = self._parse_profiles(self.config.get('profiles', {}))
        if not self.profiles:
            self.get_logger().error(f'no profiles defined in {path}')

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.stop_pub = self.create_publisher(
            Bool, self.get_parameter('e_stop_topic').value, 10)

        self.active: Profile | None = None
        self.open_devices: dict[str, OpenDevice] = {}
        self._last_reason = ''

        if evdev is None:
            self.get_logger().fatal(
                'python3-evdev is not installed. sudo apt install python3-evdev')

        self.create_timer(1.0 / self.publish_rate, self._tick)
        self.create_timer(2.0, self._rescan_if_idle)
        self._rescan()

    # ------------------------------------------------------------------ config --

    def _load(self, path: str) -> dict:
        try:
            with open(path, 'r') as handle:
                return yaml.safe_load(handle) or {}
        except OSError as exc:
            self.get_logger().error(f'cannot read {path}: {exc}')
            return {}

    def _parse_profiles(self, raw: dict) -> list:
        profiles = []
        for name, body in (raw or {}).items():
            specs = []
            for role, dev in (body.get('devices') or {}).items():
                specs.append(DeviceSpec(
                    role=role,
                    match=dev.get('match', ''),
                    required=bool(dev.get('required', True)),
                    axes=dev.get('axes') or {},
                    buttons=dev.get('buttons') or {}))
            if specs:
                profiles.append(Profile(name=name, devices=specs))
        return profiles

    # ------------------------------------------------------------- device scan --

    def _rescan(self) -> None:
        if evdev is None:
            return
        self._close_all()

        try:
            attached = [evdev.InputDevice(p) for p in evdev.list_devices()]
        except OSError as exc:
            self.get_logger().warning(f'cannot enumerate input devices: {exc}')
            return

        import re
        for profile in self.profiles:
            chosen: dict[str, object] = {}
            ok = True
            for spec in profile.devices:
                found = None
                for dev in attached:
                    if spec.match and re.search(spec.match, dev.name, re.IGNORECASE):
                        found = dev
                        break
                if found is None:
                    if spec.required:
                        ok = False
                        break
                else:
                    chosen[spec.role] = (found, spec)
            if ok and chosen:
                self.active = profile
                for role, (dev, spec) in chosen.items():
                    self.open_devices[role] = OpenDevice(dev, spec)
                names = ', '.join(f'{r}="{d.dev.name}"' for r, d in self.open_devices.items())
                self.get_logger().info(f'using profile "{profile.name}": {names}')
                for dev in attached:
                    if dev not in [d.dev for d in self.open_devices.values()]:
                        dev.close()
                return

        for dev in attached:
            dev.close()
        self.active = None
        self._say_once('no matching controller attached; holding the robot stopped')

    def _rescan_if_idle(self) -> None:
        if self.active is None:
            self._rescan()

    def _close_all(self) -> None:
        for dev in self.open_devices.values():
            dev.close()
        self.open_devices.clear()
        self.active = None

    def _say_once(self, message: str) -> None:
        if message != self._last_reason:
            self.get_logger().warning(message)
            self._last_reason = message

    # -------------------------------------------------------------------- loop --

    def _tick(self) -> None:
        now = self.now_seconds()
        twist = Twist()
        e_stop = False

        if self.active is None:
            e_stop = True
        else:
            alive = True
            for role, dev in list(self.open_devices.items()):
                if not dev.pump(now):
                    self._say_once(f'{role} disappeared; stopping')
                    alive = False
                    break
                if dev.last_event and (now - dev.last_event) > self.timeout:
                    self._say_once(f'{role} silent for {self.timeout:g}s; stopping')
                    alive = False
                    break

            if not alive:
                self._close_all()
                e_stop = True
            else:
                e_stop, twist = self._command()

        self.cmd_pub.publish(twist)
        message = Bool()
        message.data = e_stop
        self.stop_pub.publish(message)

    def _command(self):
        """Fold every attached device's axes and buttons into one command."""
        steer = 0.0
        throttle = 0.0
        armed = None
        stop = False
        turbo = False

        for dev in self.open_devices.values():
            value = dev.axis('steer')
            if value is not None:
                steer = value
            value = dev.axis('throttle')
            if value is not None:
                throttle = value

            pressed = dev.button('e_stop')
            if pressed:
                stop = True
            pressed = dev.button('arm')
            if pressed is not None:
                armed = bool(pressed) or bool(armed)
            pressed = dev.button('turbo')
            if pressed:
                turbo = True

        twist = Twist()
        if stop:
            self._say_once('e-stop pressed')
            return True, twist

        # No arm button configured anywhere means "always live".
        if armed is False:
            self._last_reason = ''
            return False, twist

        self._last_reason = ''
        limit = self.max_linear * (self.turbo_scale if turbo else 1.0)
        twist.linear.x = throttle * limit
        twist.angular.z = steer * self.max_angular
        return False, twist

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the robot commanded to stop, not merely abandoned.
        try:
            stop = Bool()
            stop.data = True
            node.stop_pub.publish(stop)
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node._close_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
