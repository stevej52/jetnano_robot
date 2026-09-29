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

"""Reading the reSpeaker's XVF3800 over USB: which way a voice came from and whether the
chip hears speech, while ALSA streams the audio (a separate USB interface).

The protocol is the vendor control transfer of Seeed's xvf_host.py (reSpeaker_Flex repo):
IN, request 0, wValue 0x80 | command id, wIndex resource id; the reply is a status byte
(0 ok, 64 busy: ask again) and little-endian values. Access needs the udev rule in
robot-environment (system/udev-99-rosie-respeaker.rules) or root.
"""

import struct

VID, PID = 0x2886, 0x0022
OK, RETRY = 0, 64
# name: (resource id, command id, count, type) - from xvf_host.py's table
COMMANDS = {
    'DOA_VALUE': (20, 18, 2, 'H'),                      # degrees 0-359 (0-180 on a line array), speech 0/1
    'AEC_SPENERGY_VALUES': (33, 80, 4, 'f'),             # speech energy: beam 1, beam 2, free, auto-selected
    'AEC_AZIMUTH_VALUES': (33, 75, 4, 'f'),              # radians, the same four beams
    'AUDIO_MGR_SELECTED_AZIMUTHS': (35, 11, 2, 'f'),     # radians: processed DoA, auto-selected beam
}
SIZE = {'H': 2, 'f': 4, 'B': 1}


class XVF:
    def __init__(self):
        import usb.core
        self.dev = usb.core.find(idVendor=VID, idProduct=PID)
        if self.dev is None:
            raise OSError('no reSpeaker XVF3800 (2886:0022) on USB')

    def read(self, name):
        res, cmd, n, t = COMMANDS[name]
        length = n * SIZE[t] + 1
        for _ in range(100):
            r = bytes(self.dev.ctrl_transfer(0xC0, 0, 0x80 | cmd, res, length, 500))
            if r and r[0] == OK:
                return struct.unpack('<' + t * n, r[1:1 + n * SIZE[t]])
            if not r or r[0] != RETRY:
                raise OSError(f'{name}: status {r[0] if r else "none"}')
        raise OSError(f'{name}: busy')

    def close(self):
        import usb.util
        usb.util.dispose_resources(self.dev)
