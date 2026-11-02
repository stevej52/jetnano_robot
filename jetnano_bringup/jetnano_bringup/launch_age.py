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

"""How long ago a supervised process started, so a restart does not shoot one still coming up.

The camera pipeline's launch needs ~25 s after a respawn before it publishes: the safety monitor
stops it 5 s into a /vo silence and the wrapper (cuvslam_vo.sh) is respawned 10 s later. A
second restart in that window doubles the outage (2026-09-27, the watchdog's). Both the watchdog
and guard_flow restart that launch, so both ask here (2026-11-01 review).
"""

import os


def started_ago(pids):
    """Seconds since the oldest of ``pids`` started (a wrapper forks subshells with its own
    command line: its own start is the oldest); None if /proc cannot say."""
    try:
        with open('/proc/uptime') as f:
            up = float(f.read().split()[0])
        tick = os.sysconf('SC_CLK_TCK')
        starts = []
        for pid in pids:
            with open(f'/proc/{pid}/stat') as f:
                starts.append(int(f.read().rsplit(')', 1)[1].split()[19]) / tick)
        return up - min(starts) if starts else None
    except (OSError, ValueError, IndexError):
        return None


def young(pids, young_s):
    """True while the process is between exit and respawn (no pids: its launch brings it back)
    or started under ``young_s`` ago. False if its age cannot be read."""
    if not pids:
        return True
    age = started_ago(pids)
    return age is not None and age < young_s
