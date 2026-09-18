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

"""Save the map, in both forms that matter.

    ros2 run jetnano_navigation save_map ~/maps/home

Writes two different things, because they do two different jobs:

  home.posegraph + home.data   slam_toolbox's serialised pose-graph. THIS is
                               the one you reload to keep mapping, because it
                               preserves the graph, the scans and the loop
                               closures. An image cannot.

  home.pgm + home.yaml         the ordinary occupancy grid. Good for looking
                               at, for AMCL, and for anything else that wants
                               a plain map.

Both come from slam_toolbox's own services, so slam_toolbox must be running
and must have mapped something.

Service definitions this relies on (checked against Jazzy):

    slam_toolbox/srv/SerializePoseGraph   string filename  ->  uint8 result
    slam_toolbox/srv/SaveMap              std_msgs/String name -> uint8 result

In BOTH, result == 0 means success (RESULT_SUCCESS=0). Do not read a zero as
a failure; that mistake costs an evening.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys

SUCCESS = 0


def _call(service: str, srv_type: str, request: str, what: str) -> bool:
    print(f'--> {what}')
    try:
        proc = subprocess.run(
            ['ros2', 'service', 'call', service, srv_type, request],
            capture_output=True, text=True, timeout=120)
    except subprocess.TimeoutExpired:
        print('    TIMED OUT - is slam_toolbox running?')
        return False
    except FileNotFoundError:
        print('    ros2 not on PATH - source your workspace first')
        return False

    blob = (proc.stdout or '') + (proc.stderr or '')

    if proc.returncode != 0 or 'invalid' in blob.lower():
        print(f'    FAILED (exit {proc.returncode})')
        for line in blob.splitlines()[:5]:
            print(f'      {line.strip()}')
        return False

    # The response prints as e.g. "slam_toolbox.srv.SerializePoseGraph_Response(result=0)"
    match = re.search(r'result=(\d+)', blob.replace(' ', ''))
    if match is None:
        print('    no result code in the reply - service may not have run')
        for line in blob.splitlines()[-4:]:
            print(f'      {line.strip()}')
        return False

    code = int(match.group(1))
    if code == SUCCESS:
        print('    ok')
        return True
    print(f'    service reported failure (result={code})')
    return False


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    target = os.path.expanduser(sys.argv[1])
    for suffix in ('.posegraph', '.data', '.yaml', '.pgm'):
        if target.endswith(suffix):
            target = target[: -len(suffix)]
            break
    target = os.path.abspath(target)

    os.makedirs(os.path.dirname(target) or '.', exist_ok=True)
    print(f'saving map as "{target}"\n')

    graph_ok = _call(
        '/slam_toolbox/serialize_map',
        'slam_toolbox/srv/SerializePoseGraph',
        f"{{filename: '{target}'}}",
        'serialising the pose-graph (reloadable, keeps loop closures)')

    image_ok = _call(
        '/slam_toolbox/save_map',
        'slam_toolbox/srv/SaveMap',
        f"{{name: {{data: '{target}'}}}}",
        'writing the occupancy grid image')

    print()
    written = [p for p in (target + '.posegraph', target + '.data',
                           target + '.yaml', target + '.pgm') if os.path.exists(p)]
    if written:
        print('files written:')
        for path in written:
            print(f'  {path}  ({os.path.getsize(path)} bytes)')
    else:
        print('nothing was written.')

    if graph_ok:
        print('\nreload it with:')
        print(f'  ros2 launch jetnano_navigation navigation.launch.py '
              f'mode:=continue map:={target}')
    return 0 if (graph_ok and image_ok) else 1


if __name__ == '__main__':
    sys.exit(main())
