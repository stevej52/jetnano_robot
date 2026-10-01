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

"""Reading and tuning the reSpeaker's XVF3800 over USB: which way a voice came from and
whether the chip hears speech, while ALSA streams the audio (a separate USB interface); and
the chip's echo / noise / gain processing (PROFILES, applied by ears or by hand):

    ros2 run jetnano_bringup xvf_tune status          # the controls that matter, as they are
    ros2 run jetnano_bringup xvf_tune clean           # the "hear better" profile (2026-10-01)
    ros2 run jetnano_bringup xvf_tune shipped         # back to the factory values
    ros2 run jetnano_bringup xvf_tune set PP_GAMMA_ENL 2.5

Nothing on the chip survives a power cycle: ears applies its board_profile parameter when
it opens the chip (robot.launch.py board_profile:=clean).

The protocol is the vendor control transfer of Seeed's xvf_host.py (reSpeaker_Flex repo):
IN, request 0, wValue 0x80 | command id, wIndex resource id; the reply is a status byte
(0 ok, 64 busy: ask again) and little-endian values. Access needs the udev rule in
robot-environment (system/udev-99-rosie-respeaker.rules) or root.
"""

import struct
import sys

VID, PID = 0x2886, 0x0022
OK, RETRY = 0, 64
# name: (resource id, command id, count, type) - from xvf_host.py's table
COMMANDS = {
    'DOA_VALUE': (20, 18, 2, 'H'),                      # degrees 0-359 (0-180 on a line array), speech 0/1
    'AEC_SPENERGY_VALUES': (33, 80, 4, 'f'),             # speech energy: beam 1, beam 2, free, auto-selected
    'AEC_AZIMUTH_VALUES': (33, 75, 4, 'f'),              # radians, the same four beams
    'AUDIO_MGR_SELECTED_AZIMUTHS': (35, 11, 2, 'f'),     # radians: processed DoA, auto-selected beam
    # The processing on the chip's two outputs. ch0 (OP_L) is the post-processed talk output:
    # echo cancelled, non-linear echo attenuated, noise suppressed, limited, AGC. ch1 (OP_R)
    # is the echo canceller's linear output alone, fixed gain - what ears read until
    # 2026-10-01, when her own voice through the new speaker at 100 % came back at -40 dBFS
    # (as loud as Steve): distortion is non-linear, and ch1 skips the stage that removes it.
    'AUDIO_MGR_OP_L': (35, 15, 2, 'B'),                 # category, source of output ch0
    'AUDIO_MGR_OP_R': (35, 19, 2, 'B'),                 # category, source of output ch1
    'AUDIO_MGR_MIC_GAIN': (35, 0, 1, 'f'),
    'AUDIO_MGR_REF_GAIN': (35, 1, 1, 'f'),              # the echo reference (what she plays)
    'AEC_AECCONVERGED': (33, 3, 1, 'i'),
    'AEC_ASROUTGAIN': (33, 36, 1, 'f'),                 # ch1's fixed gain
    'PP_AGCONOFF': (17, 10, 1, 'i'),                    # AGC on ch0: off makes ch0 fixed-gain
    'PP_AGCMAXGAIN': (17, 11, 1, 'f'),
    'PP_MIN_NS': (17, 21, 1, 'f'),                      # stationary noise suppression floor (gain 0-1)
    'PP_MIN_NN': (17, 22, 1, 'f'),                      # non-stationary noise suppression floor
    'PP_ECHOONOFF': (17, 23, 1, 'i'),
    'PP_GAMMA_E': (17, 24, 1, 'f'),                     # echo over-subtraction: direct and early (0-2)
    'PP_GAMMA_ETAIL': (17, 25, 1, 'f'),                 # tail (0-2)
    'PP_GAMMA_ENL': (17, 26, 1, 'f'),                   # NON-LINEAR echo, the speaker's distortion (0-5)
    'PP_NLATTENONOFF': (17, 27, 1, 'i'),
    'PP_DTSENSITIVE': (17, 31, 1, 'i'),                 # 0-5 echo first .. 10-15 double-talk first
    'PP_ATTNS_MODE': (17, 32, 1, 'i'),                  # extra gain reduction while nobody speaks
    'PP_ATTNS_NOMINAL': (17, 33, 1, 'f'),
}
SIZE = {'H': 2, 'f': 4, 'B': 1, 'i': 4}
TUNABLE = [n for n in COMMANDS if n.startswith(('PP_', 'AUDIO_MGR_MIC', 'AUDIO_MGR_REF', 'AEC_ASROUTGAIN'))]
STATUS = ['AUDIO_MGR_OP_L', 'AUDIO_MGR_OP_R', 'AUDIO_MGR_MIC_GAIN', 'AUDIO_MGR_REF_GAIN', 'AEC_AECCONVERGED',
          'AEC_ASROUTGAIN'] + [n for n in COMMANDS if n.startswith('PP_')]
# What each profile writes. "shipped" is what the chip reads after a power cycle (read
# 2026-10-01 00:50); "clean" is the hear-better profile for ch0 to be tested with Steve.
PROFILES = {
    'shipped': {'PP_AGCONOFF': 1, 'PP_ATTNS_MODE': 0, 'PP_ATTNS_NOMINAL': 1.0, 'PP_GAMMA_E': 1.0,
                'PP_GAMMA_ETAIL': 1.0, 'PP_GAMMA_ENL': 1.1, 'PP_MIN_NS': 0.15, 'PP_MIN_NN': 0.51,
                'PP_DTSENSITIVE': 12, 'PP_NLATTENONOFF': 1, 'PP_ECHOONOFF': 1},
    'clean': {'PP_AGCONOFF': 0,            # fixed gain on ch0, so a level meter and a recogniser can use it
              'PP_ATTNS_MODE': 1,          # and quieter between words
              'PP_GAMMA_ENL': 2.5,         # hit the speaker's distortion harder (range 0-5)
              'PP_GAMMA_ETAIL': 1.3,
              'PP_MIN_NS': 0.10,           # noise floor -20 dB (was -16.5)
              'PP_DTSENSITIVE': 5,         # echo suppression before double-talk (10-15 adds a detector)
              'PP_NLATTENONOFF': 1, 'PP_ECHOONOFF': 1},
}


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

    def write(self, name, *values):
        """Set a control (xvf_host.py's write: OUT, request 0, wValue = command id, wIndex = resource)."""
        res, cmd, n, t = COMMANDS[name]
        if len(values) != n:
            raise ValueError(f'{name} takes {n} value(s)')
        conv = float if t == 'f' else int
        payload = struct.pack('<' + t * n, *(conv(v) for v in values))
        self.dev.ctrl_transfer(0x40, 0, cmd, res, payload, 500)

    def apply(self, profile):
        """Write a PROFILES entry -> {name: value read back}."""
        out = {}
        for name, value in PROFILES[profile].items():
            self.write(name, value)
            out[name] = self.read(name)[0]
        return out

    def status(self):
        return {name: self.read(name) for name in STATUS}

    def close(self):
        import usb.util
        usb.util.dispose_resources(self.dev)


def main(argv=None):
    """xvf_tune: status | <profile> | set NAME VALUE..."""
    argv = sys.argv[1:] if argv is None else argv
    what = argv[0] if argv else 'status'
    chip = XVF()
    try:
        if what == 'status':
            for name, vals in chip.status().items():
                print(f'{name:28s} {" ".join(f"{v:g}" if isinstance(v, float) else str(v) for v in vals)}')
        elif what in PROFILES:
            for name, val in chip.apply(what).items():
                print(f'{name:28s} -> {val:g}')
            print(f'profile {what} applied (until the next power cycle)')
        elif what == 'set' and len(argv) >= 3:
            chip.write(argv[1], *argv[2:])
            print(f'{argv[1]} -> {chip.read(argv[1])}')
        else:
            print(__doc__)
            return 2
    finally:
        chip.close()
    return 0
