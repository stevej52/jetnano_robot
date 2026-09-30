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

"""What the bag does not hold, logged through a drive until it is told to stop.

    drive_log <folder> [--cameras d435,front,rear] [--stop-after 7200 --stop-cmd CMD]

`drive_record.sh start full` runs it beside the recorder and `drive_record.sh stop`
stops it (SIGTERM). Not a ROS node: it reads /proc and fetches pictures over HTTP,
so it adds nothing to DDS. Writes into <folder>:

    system.csv            every second: CPU per core, load, memory, swap, the disk's
                          writes, the Wi-Fi's access point, signal, link quality, bit
                          rates and bytes in/out
    processes.csv         every 5 s: each process using over 0.5 % of a core, and its
                          resident memory
    frames/<camera>/HHMMSS.s.jpg
                          a picture a second from each camera, from csi_cameras on
                          :8082 (the D435's turned upright, as the pages turn it)
    drive_log.txt         what it did: cameras found and lost, the stop

tegrastats (GPU, temperatures, power rails) runs beside it, also from drive_record.sh.
--stop-after is the backstop for a drive nobody stopped: past it, CMD is run detached
(drive_record.sh passes its own `stop`) and this exits.
"""

import argparse
import json
import os
import re
import signal
import subprocess
import threading
import time
import urllib.request

try:
    import cv2
    import numpy as np
except ImportError:                     # pictures are then kept as they come
    cv2 = None

CAMERA_URL = 'http://127.0.0.1:8082'
PAGE = os.sysconf('SC_PAGE_SIZE')
TICK = os.sysconf('SC_CLK_TCK')


def cpu_times():
    """{cpu: (busy, total)} jiffies from /proc/stat, 'cpu' being all of them."""
    out = {}
    with open('/proc/stat') as f:
        for line in f:
            if not line.startswith('cpu'):
                break
            name, *v = line.split()
            v = [int(x) for x in v[:8]]
            idle = v[3] + v[4]                          # idle + iowait
            out[name] = (sum(v) - idle, sum(v))
    return out


def meminfo():
    m = {}
    with open('/proc/meminfo') as f:
        for line in f:
            k, v = line.split(':', 1)
            m[k] = int(v.split()[0]) // 1024            # MB
    return m


def net_bytes(iface):
    with open('/proc/net/dev') as f:
        for line in f:
            name, _, rest = line.partition(':')
            if name.strip() == iface:
                v = rest.split()
                return int(v[0]), int(v[8])
    return 0, 0


def disk_written(dev='nvme0n1'):
    with open('/proc/diskstats') as f:
        for line in f:
            v = line.split()
            if v[2] == dev:
                return int(v[9]) * 512                  # sectors written
    return 0


def wifi_iface():
    with open('/proc/net/wireless') as f:
        for line in f.readlines()[2:]:
            return line.split(':')[0].strip()
    return None


def wifi_quality(iface):
    with open('/proc/net/wireless') as f:
        for line in f.readlines()[2:]:
            name, _, rest = line.partition(':')
            if name.strip() == iface:
                v = rest.split()
                return float(v[1].rstrip('.')), float(v[2].rstrip('.'))
    return None, None


def wifi_link(iface):
    """Access point, signal and bit rates from `iw` (every 5 s: it is a process start)."""
    try:
        out = subprocess.run(['iw', 'dev', iface, 'link'], capture_output=True, text=True,
                             timeout=3).stdout
    except (OSError, subprocess.TimeoutExpired):
        return {}
    info = {}
    m = re.search(r'Connected to ([0-9a-f:]{17})', out)
    info['ap'] = m.group(1) if m else 'none'
    for key, pat in (('signal', r'signal:\s*(-?\d+)'), ('tx_mbit', r'tx bitrate:\s*([\d.]+)'),
                     ('rx_mbit', r'rx bitrate:\s*([\d.]+)')):
        m = re.search(pat, out)
        info[key] = m.group(1) if m else ''
    return info


def process_name(pid, comm):
    """A name that tells the Python nodes and the containers apart."""
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            args = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
    except OSError:
        return comm
    if not args:
        return comm
    name = comm
    if comm.startswith(('python', 'ros2')) or comm.startswith('component_con'):
        scripts = [a for a in args[1:] if '/' in a and not a.startswith('-')]
        if scripts:
            name = f'{comm}:{os.path.basename(scripts[0])}'
    for a in args:
        if a.startswith('__node:='):
            name += f'[{a[8:]}]'
            break
    return name.replace(',', ';')


def process_times():
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/stat') as f:
                stat = f.read()
        except OSError:
            continue
        comm = stat[stat.index('(') + 1:stat.rindex(')')]
        v = stat[stat.rindex(')') + 2:].split()
        out[int(pid)] = (comm, int(v[11]) + int(v[12]), int(v[21]) * PAGE // 2**20)
    return out


class DriveLog:

    def __init__(self, folder, cameras, stop_after, stop_cmd):
        self.folder, self.cameras = folder, cameras
        self.stop_after, self.stop_cmd = stop_after, stop_cmd
        self.done = threading.Event()
        self.t0 = time.time()
        self.note_lock = threading.Lock()
        self.rotate = {}
        os.makedirs(folder, exist_ok=True)

    def note(self, text):
        line = f'{time.strftime("%H:%M:%S")} {text}'
        with self.note_lock, open(os.path.join(self.folder, 'drive_log.txt'), 'a') as f:
            f.write(line + '\n')

    def system(self):
        iface = wifi_iface()
        cpus = cpu_times()
        cores = sorted((c for c in cpus if c != 'cpu'), key=lambda c: int(c[3:]))
        with open(os.path.join(self.folder, 'system.csv'), 'w') as f:
            f.write('epoch,time,cpu_pct,' + ','.join(f'{c}_pct' for c in cores)
                    + ',load1,mem_used_mb,mem_avail_mb,swap_used_mb,disk_write_mb_s,'
                    'wifi_ap,wifi_signal_dbm,wifi_link_quality,wifi_tx_mbit,wifi_rx_mbit,'
                    'net_rx_kb_s,net_tx_kb_s\n')
            last_cpu, last_net, last_disk = cpus, net_bytes(iface), disk_written()
            last_t, link, n = time.monotonic(), wifi_link(iface) if iface else {}, 0
            while not self.done.wait(1.0 - (time.monotonic() - last_t) % 1.0):
                now, cpus, net, disk = time.monotonic(), cpu_times(), net_bytes(iface), disk_written()
                dt = max(1e-3, now - last_t)
                n += 1
                if n % 5 == 0 and iface:
                    link = wifi_link(iface)

                def pct(c):
                    busy, total = cpus[c][0] - last_cpu[c][0], cpus[c][1] - last_cpu[c][1]
                    return f'{100.0 * busy / total:.0f}' if total > 0 else ''
                with open('/proc/loadavg') as la:
                    load1 = la.read().split()[0]
                m = meminfo()
                quality, _ = wifi_quality(iface)
                f.write(f'{time.time():.1f},{time.strftime("%H:%M:%S")},{pct("cpu")},'
                        + ','.join(pct(c) for c in cores)
                        + f',{load1},{m["MemTotal"] - m["MemAvailable"]},{m["MemAvailable"]},'
                        f'{m["SwapTotal"] - m["SwapFree"]},{(disk - last_disk) / dt / 1e6:.2f},'
                        f'{link.get("ap", "")},{link.get("signal", "")},'
                        f'{"" if quality is None else f"{quality:.0f}"},'
                        f'{link.get("tx_mbit", "")},{link.get("rx_mbit", "")},'
                        f'{(net[0] - last_net[0]) / dt / 1e3:.1f},{(net[1] - last_net[1]) / dt / 1e3:.1f}\n')
                f.flush()
                last_cpu, last_net, last_disk, last_t = cpus, net, disk, now

    def processes(self):
        with open(os.path.join(self.folder, 'processes.csv'), 'w') as f:
            f.write('epoch,time,pid,process,cpu_pct_of_a_core,rss_mb\n')
            last, last_t = process_times(), time.monotonic()
            while not self.done.wait(5.0):
                now, cur = time.monotonic(), process_times()
                dt = now - last_t
                stamp = f'{time.time():.1f},{time.strftime("%H:%M:%S")}'
                rows = []
                for pid, (comm, ticks, rss) in cur.items():
                    if pid in last and last[pid][0] == comm:
                        pct = 100.0 * (ticks - last[pid][1]) / TICK / dt
                        if pct >= 0.5:
                            rows.append((pct, pid, comm, rss))
                for pct, pid, comm, rss in sorted(rows, reverse=True):
                    f.write(f'{stamp},{pid},{process_name(pid, comm)},{pct:.1f},{rss}\n')
                f.flush()
                last, last_t = cur, now

    def camera(self, name):
        folder = os.path.join(self.folder, 'frames', name)
        os.makedirs(folder, exist_ok=True)
        url = f'{CAMERA_URL}/{name}.jpg?w=640&q=75'
        failing, saved = False, 0
        next_t = time.monotonic()
        while not self.done.is_set():
            try:
                with urllib.request.urlopen(url, timeout=10) as r:
                    jpeg = r.read() if r.status == 200 else b''
            except OSError as e:
                jpeg = b''
                err = str(e)
            else:
                err = 'no picture'
            if jpeg:
                if failing:
                    self.note(f'{name}: pictures again')
                    failing = False
                if self.rotate.get(name) == 180 and cv2 is not None:
                    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        ok, buf = cv2.imencode('.jpg', cv2.rotate(img, cv2.ROTATE_180),
                                               [cv2.IMWRITE_JPEG_QUALITY, 75])
                        jpeg = buf.tobytes() if ok else jpeg
                now = time.time()
                stamp = time.strftime('%H%M%S', time.localtime(now)) + f'.{int(now * 10) % 10}'
                with open(os.path.join(folder, stamp + '.jpg'), 'wb') as out:
                    out.write(jpeg)
                saved += 1
                next_t += 1.0
            else:
                if not failing:
                    self.note(f'{name}: no picture ({err}); trying again every 30 s')
                    failing = True
                next_t = time.monotonic() + 30.0
            next_t = max(next_t, time.monotonic())
            self.done.wait(next_t - time.monotonic())
        self.note(f'{name}: {saved} pictures')

    def backstop(self):
        if self.done.wait(self.stop_after):
            return
        self.note(f'still running after {self.stop_after / 60:.0f} min: stopping the drive log')
        if self.stop_cmd:
            with open(os.path.join(self.folder, 'backstop_stop.txt'), 'w') as out:
                subprocess.Popen(['bash', '-c', self.stop_cmd], stdout=out, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL, start_new_session=True)
        self.done.set()

    def run(self):
        try:
            with urllib.request.urlopen(f'{CAMERA_URL}/config.json', timeout=3) as r:
                if json.loads(r.read()).get('d435_rotate') == 180:
                    self.rotate['d435'] = 180
        except (OSError, ValueError):
            pass
        # only the cameras the server has: its front page lists them ("cameras: front.mjpg
        # front.jpg, ..."). The rear one went off 2026-09-30 (broken); asking for it every
        # second would only log misses.
        try:
            with urllib.request.urlopen(f'{CAMERA_URL}/', timeout=3) as r:
                listed = {w.split('.')[0] for w in r.read().decode(errors='replace').replace(',', ' ').split()
                          if w.endswith('.jpg')}
            missing = [c for c in self.cameras if c not in listed]
            if missing:
                self.note(f'not on the camera server, skipped: {", ".join(missing)}')
                self.cameras = [c for c in self.cameras if c in listed]
        except (OSError, ValueError):
            pass
        self.note(f'started: cameras {", ".join(self.cameras) or "none"}, '
                  f'stops by itself after {self.stop_after / 60:.0f} min')
        threads = [threading.Thread(target=self.system, daemon=True),
                   threading.Thread(target=self.processes, daemon=True),
                   threading.Thread(target=self.backstop, daemon=True)]
        threads += [threading.Thread(target=self.camera, args=(c,), daemon=True) for c in self.cameras]
        for t in threads:
            t.start()
        self.done.wait()
        for t in threads:
            t.join(timeout=12.0)
        self.note(f'stopped after {(time.time() - self.t0) / 60:.1f} min')


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('folder')
    p.add_argument('--cameras', default='d435,front,rear')
    p.add_argument('--stop-after', type=float, default=7200.0, help='seconds')
    p.add_argument('--stop-cmd', default='')
    a = p.parse_args()
    cams = [c for c in a.cameras.split(',') if c]
    try:                    # only the cameras the relay runs (rear is off since 2026-09-29)
        import json
        with urllib.request.urlopen(CAMERA_URL + '/config.json', timeout=5) as r:
            running = json.loads(r.read()).get('cameras')
        if running:
            cams = [c for c in cams if c in running]
    except (OSError, ValueError):
        pass
    log = DriveLog(a.folder, cams, a.stop_after, a.stop_cmd)
    signal.signal(signal.SIGTERM, lambda *_: log.done.set())
    signal.signal(signal.SIGINT, lambda *_: log.done.set())
    log.run()


if __name__ == '__main__':
    main()
