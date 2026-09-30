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

""""Diane is sleeping" mode: nothing out of the speaker until it is switched off again.

    ros2 run jetnano_bringup quiet on        # silent: sounds, spoken replies, uh-ohs, hello, goodbye
    ros2 run jetnano_bringup quiet off
    ros2 run jetnano_bringup quiet status

It is one file, ~/voice/quiet (sounds.py quiet_file), so it holds across reboots. The sounds
node checks it every second; the power-on and goodbye hooks (robot-environment
rosie-poweron, rosie-goodbye) look for it too. Servos and motors are not sounds: driving is
as loud as ever. The driving page still shows problems while she is silent.
"""

import os
import sys
import time

FLAG = os.path.expanduser('~/voice/quiet')


def main():
    want = sys.argv[1] if len(sys.argv) > 1 else 'status'
    if want == 'on':
        os.makedirs(os.path.dirname(FLAG), exist_ok=True)
        with open(FLAG, 'w') as f:
            f.write(time.strftime('%Y-%m-%d %H:%M:%S\n'))
    elif want == 'off':
        try:
            os.remove(FLAG)
        except FileNotFoundError:
            pass
    elif want != 'status':
        print(__doc__)
        return 2
    if os.path.exists(FLAG):
        with open(FLAG) as f:
            since = f.read().strip()
        print(f'quiet mode ON since {since}: nothing plays (ros2 run jetnano_bringup quiet off)')
    else:
        print('quiet mode off: she makes her sounds')
    return 0


if __name__ == '__main__':
    sys.exit(main())
