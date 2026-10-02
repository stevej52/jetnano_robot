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

"""The universal off switch for her voice and hearing (Steve, 2026-10-01).

    ros2 run jetnano_bringup voice on        -> start jetnano-voice.service (ears, settle, sounds,
                                                speak, listen: launch/voice.launch.py), ~15 s to words
    ros2 run jetnano_bringup voice off       -> stop it: the microphone's USB audio stream, the
                                                speaker and 1.4 GB of RAM go away; nothing listens
    ros2 run jetnano_bringup voice status    -> "loaded" / "unloaded", and what is running

Off is the default: the service is not enabled at boot, and drive.sh switches it off before a
lap. ~/voice/off is written while it is off, for the watchdog (it stops watching the voice
parts) and the driving page (its VOICE button). The same flag file is read by nothing else:
the TALK / QUIET / ROBOT ONLY modes are separate (~/voice/quiet, diane, robot_only).
"""

import os
import subprocess
import sys
import time

SERVICE = 'jetnano-voice.service'
OFF_FLAG = os.path.expanduser('~/voice/off')
VOICE_NODES = ('ears', 'settle', 'sounds', 'speak', 'listen')


def loaded() -> bool:
    return subprocess.run(['systemctl', 'is-active', '-q', SERVICE]).returncode == 0


def running() -> list:
    out = subprocess.run(['pgrep', '-af', 'lib/jetnano_bringup/(ears|settle|sounds|speak|listen)( |$)'],
                         capture_output=True, text=True).stdout
    return sorted({ln.split('/')[-1].split(' ')[0] for ln in out.splitlines() if ln.strip()})


def switch(on: bool, quiet: bool = False) -> bool:
    """Start or stop the service and set the flag. -> True if it ended up as asked."""
    os.makedirs(os.path.dirname(OFF_FLAG), exist_ok=True)
    if on:
        try:
            os.remove(OFF_FLAG)
        except FileNotFoundError:
            pass
        r = subprocess.run(['sudo', '-n', 'systemctl', 'start', SERVICE], capture_output=True, text=True)
    else:
        with open(OFF_FLAG, 'w') as f:
            f.write(time.strftime('%Y-%m-%d %H:%M:%S') + '\n')
        r = subprocess.run(['sudo', '-n', 'systemctl', 'stop', SERVICE], capture_output=True, text=True)
    if r.returncode != 0 and not quiet:
        print(f'systemctl: {r.stderr.strip() or r.stdout.strip()}', file=sys.stderr)
    return loaded() == on


def main(args=None):
    argv = sys.argv[1:] if args is None else args
    want = argv[0] if argv else 'status'
    if want == 'on':
        ok = switch(True)
        print('voice: loading (ears, sounds, speak, listen - about 15 s to words)' if ok else 'voice: could not start the service')
        return 0 if ok else 1
    if want == 'off':
        ok = switch(False)
        print('voice: unloaded (nothing listens, nothing speaks)' if ok else 'voice: could not stop the service')
        return 0 if ok else 1
    if want in ('status', 'st'):
        print(f'voice: {"loaded" if loaded() else "unloaded"}; running: {", ".join(running()) or "nothing"}'
              + ('; flag ~/voice/off set' if os.path.exists(OFF_FLAG) else ''))
        return 0
    print('usage: voice on|off|status', file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main())
