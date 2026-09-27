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

"""The two CSI cameras as MJPEG streams for a browser, only while someone watches.

    ros2 run jetnano_bringup csi_cameras
    http://<robot>:8082/front.mjpg     the pan-tilt camera (sensor-id 0, port A)
    http://<robot>:8082/rear.mjpg      the rear camera     (sensor-id 1, port C)
    http://<robot>:8082/front.jpg      one picture

Both Raspberry-Pi-style IMX219 modules sit upside down, hence flip-method=2.
Each camera runs one GStreamer pipeline (nvarguscamerasrc -> nvvidconv ->
nvjpegenc, the Jetson's own JPEG encoder: ~13 fps at 960x540 for next to no
CPU, 2026-09-27) while at least one browser is connected, and stops a few
seconds after the last one leaves, so a camera nobody watches costs nothing.
Frames are cut from the encoder's output at the JPEG end marker.

White balance: Argus's automatic balance is fine in daylight but goes magenta
under the living room's lamps at night, where "warm fluorescent" (4) measured
most neutral (2026-09-26). ``wbmode`` -1 picks by the clock: auto (1) from
``day_from`` to ``day_until``, 4 otherwise; any other value is used as is.

Plain HTTP on the robot's own network, no login - like the driving page.
"""

import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node

BOUNDARY = b'rosieframe'


class Camera:
    """One sensor: a pipeline that runs while there are viewers, and its latest frame."""

    def __init__(self, node, name, sensor, width, height, fps, quality):
        self.node, self.name, self.sensor = node, name, sensor
        self.width, self.height, self.fps, self.quality = width, height, fps, quality
        self.cond = threading.Condition()
        self.frame, self.seq = None, 0
        self.viewers, self.last_viewer = 0, 0.0
        self.proc = None
        threading.Thread(target=self._run, daemon=True, name=f'cam-{name}').start()

    def _wbmode(self) -> int:
        wb = int(self.node.get_parameter('wbmode').value)
        if wb >= 0:
            return wb
        hour = time.localtime().tm_hour + time.localtime().tm_min / 60.0
        day = float(self.node.get_parameter('day_from').value) <= hour < float(self.node.get_parameter('day_until').value)
        return 1 if day else 4

    def _command(self):
        return ['gst-launch-1.0', '-q', 'nvarguscamerasrc', f'sensor-id={self.sensor}', f'wbmode={self._wbmode()}',
                '!', 'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1',
                '!', 'nvvidconv', 'flip-method=2',
                '!', f'video/x-raw(memory:NVMM),width={self.width},height={self.height},format=I420',
                '!', 'videorate', 'drop-only=true',
                '!', f'video/x-raw(memory:NVMM),framerate={self.fps}/1',
                '!', 'nvjpegenc', f'quality={self.quality}', '!', 'fdsink', 'fd=1']

    def _run(self):
        while True:
            with self.cond:
                while self.viewers == 0:
                    self.cond.wait(1.0)
            cmd = self._command()
            self.node.get_logger().info(f'{self.name}: starting (sensor {self.sensor}, '
                                        f'{self.width}x{self.height} at {self.fps} fps, wbmode {cmd[4][7:]})')
            try:
                self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            except OSError as exc:
                self.node.get_logger().error(f'{self.name}: {exc}')
                time.sleep(10)
                continue
            buf = b''
            while True:
                chunk = self.proc.stdout.read1(65536) if hasattr(self.proc.stdout, 'read1') else self.proc.stdout.read(65536)
                if not chunk:
                    break
                buf += chunk
                while True:
                    end = buf.find(b'\xff\xd9')
                    if end < 0:
                        break
                    start = buf.find(b'\xff\xd8')
                    if 0 <= start < end:
                        with self.cond:
                            self.frame, self.seq = buf[start:end + 2], self.seq + 1
                            self.cond.notify_all()
                    buf = buf[end + 2:]
                if len(buf) > 4_000_000:
                    buf = b''
                with self.cond:
                    idle = self.viewers == 0 and time.monotonic() - self.last_viewer > 5.0
                if idle:
                    break
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None
            with self.cond:
                self.frame = None
            self.node.get_logger().info(f'{self.name}: stopped (nobody watching)')
            time.sleep(1.0)

    def watch(self):
        with self.cond:
            self.viewers += 1
            self.cond.notify_all()

    def leave(self):
        with self.cond:
            self.viewers -= 1
            self.last_viewer = time.monotonic()

    def next_frame(self, after, timeout=5.0):
        with self.cond:
            self.cond.wait_for(lambda: self.frame is not None and self.seq != after, timeout)
            return self.seq, self.frame


def make_handler(cameras):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):
            pass

        def do_GET(self):
            path = self.path.split('?', 1)[0].strip('/')
            name, _, kind = path.partition('.')
            cam = cameras.get(name)
            if cam is None or kind not in ('mjpg', 'jpg'):
                body = b'front.mjpg, rear.mjpg, front.jpg, rear.jpg'
                self.send_response(HTTPStatus.NOT_FOUND)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            cam.watch()
            try:
                if kind == 'jpg':
                    _, frame = cam.next_frame(-1, timeout=8.0)
                    body = frame or b''
                    self.send_response(HTTPStatus.OK if frame else HTTPStatus.SERVICE_UNAVAILABLE)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(body)))
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(body)
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=' + BOUNDARY.decode())
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Connection', 'close')
                self.end_headers()
                seq = -1
                while True:
                    seq, frame = cam.next_frame(seq)
                    if frame is None:
                        continue
                    self.wfile.write(b'--' + BOUNDARY + b'\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass
            finally:
                cam.leave()

    return Handler


class CsiCameras(Node):

    def __init__(self):
        super().__init__('csi_cameras')
        self.declare_parameter('port', 8082)
        self.declare_parameter('wbmode', -1)          # -1: auto by day, warm fluorescent at night
        self.declare_parameter('day_from', 7.5)
        self.declare_parameter('day_until', 18.5)
        self.declare_parameter('front', [0, 960, 540, 15, 80])     # sensor, width, height, fps, quality
        self.declare_parameter('rear', [1, 640, 360, 10, 70])
        cams = {}
        for name in ('front', 'rear'):
            s, w, h, f, q = [int(v) for v in self.get_parameter(name).value]
            cams[name] = Camera(self, name, s, w, h, f, q)
        port = int(self.get_parameter('port').value)
        self.server = ThreadingHTTPServer(('0.0.0.0', port), make_handler(cams))
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True, name='csi-http').start()
        self.get_logger().info(f'CSI cameras on http://0.0.0.0:{port}/front.mjpg and /rear.mjpg (started on demand)')


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = CsiCameras()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
