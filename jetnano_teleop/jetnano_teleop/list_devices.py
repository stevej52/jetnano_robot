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

"""Print what is plugged in, so joysticks.yaml can be filled in from fact.

    ros2 run jetnano_teleop list_devices          # describe every device
    ros2 run jetnano_teleop list_devices --watch  # and show what moves

The index columns are what joysticks.yaml wants. They are positions in the
device's sorted capability list, not raw evdev codes - the code is printed
beside each one so there is no ambiguity.

Run it on the machine the controller is plugged into. If it finds nothing,
your user is probably not in the ``input`` group:

    sudo usermod -aG input $USER     # then log out and back in
"""

from __future__ import annotations

import sys

try:
    import evdev
except ImportError:
    print('python3-evdev is not installed.  sudo apt install python3-evdev')
    sys.exit(1)


def describe(dev) -> None:
    caps = dev.capabilities(absinfo=True)
    abs_caps = sorted(caps.get(evdev.ecodes.EV_ABS, []), key=lambda c: c[0])
    key_caps = sorted(caps.get(evdev.ecodes.EV_KEY, []))

    print(f'\n{"=" * 72}')
    print(f'name : {dev.name}')
    print(f'path : {dev.path}')
    print(f'{"=" * 72}')

    if abs_caps:
        print('\n  AXES   (use "index" in joysticks.yaml)')
        print('  index  evdev code            min      max   rest')
        for index, (code, info) in enumerate(abs_caps):
            names = evdev.ecodes.ABS.get(code, code)
            if isinstance(names, list):
                names = names[0]
            print(f'  {index:>5}  {str(names):<18} {info.min:>7} {info.max:>8} {info.value:>6}')
    else:
        print('\n  no absolute axes')

    if key_caps:
        print('\n  BUTTONS   (use "index" in joysticks.yaml)')
        print('  index  evdev code')
        for index, code in enumerate(key_caps):
            names = evdev.ecodes.bytype[evdev.ecodes.EV_KEY].get(code, code)
            if isinstance(names, list):
                names = names[0]
            print(f'  {index:>5}  {names}')
    else:
        print('\n  no buttons')


def watch(devices) -> None:
    from select import select

    print('\nMove an axis or press a button. Ctrl-C to stop.\n')
    fds = {dev.fd: dev for dev in devices}
    index_of_abs = {}
    index_of_key = {}
    for dev in devices:
        caps = dev.capabilities(absinfo=True)
        abs_caps = sorted(caps.get(evdev.ecodes.EV_ABS, []), key=lambda c: c[0])
        key_caps = sorted(caps.get(evdev.ecodes.EV_KEY, []))
        index_of_abs[dev.path] = {code: i for i, (code, _) in enumerate(abs_caps)}
        index_of_key[dev.path] = {code: i for i, code in enumerate(key_caps)}

    try:
        while True:
            ready, _, _ = select(fds, [], [])
            for fd in ready:
                dev = fds[fd]
                for event in dev.read():
                    if event.type == evdev.ecodes.EV_ABS:
                        index = index_of_abs[dev.path].get(event.code, '?')
                        print(f'{dev.name:<34} AXIS   index {index:<3} value {event.value}')
                    elif event.type == evdev.ecodes.EV_KEY:
                        index = index_of_key[dev.path].get(event.code, '?')
                        state = 'down' if event.value else 'up'
                        print(f'{dev.name:<34} BUTTON index {index:<3} {state}')
    except KeyboardInterrupt:
        print()


def main() -> int:
    paths = evdev.list_devices()
    if not paths:
        print('No input devices readable.')
        print('If a controller IS plugged in, you are probably not in the input group:')
        print('    sudo usermod -aG input $USER     # then log out and back in')
        return 1

    devices = [evdev.InputDevice(p) for p in paths]
    for dev in devices:
        describe(dev)

    print(f'\n{len(devices)} device(s).')
    if '--watch' in sys.argv:
        watch(devices)
    else:
        print('Re-run with --watch to see live axis and button numbers.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
