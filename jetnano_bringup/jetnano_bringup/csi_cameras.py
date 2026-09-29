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

"""Every camera as MJPEG for a browser, only while someone watches.

    ros2 run jetnano_bringup csi_cameras
    http://<robot>:8082/front.mjpg     the pan-tilt camera (sensor-id 0, port A)
    http://<robot>:8082/rear.mjpg      the rear camera     (sensor-id 1, port C)
    http://<robot>:8082/d435.mjpg      the RealSense colour stream (relayed, see below)
    http://<robot>:8082/config.json    {"d435_rotate": 180}: how the pages must turn the
                                       RealSense picture (param d435_rotate; the camera is
                                       mounted upside down since 2026-09-27). The browser
                                       turns it (CSS), so it costs the robot nothing.
    http://<robot>:8082/front.jpg      one picture (any camera: <name>.jpg)
    ?fps=5 &w=320 &q=50                per viewer: fewer frames, smaller, rougher

The D435's colour stream is already JPEG on /camera/color/image_raw/compressed
(made inside the Isaac container). Browsers used to get it from NVIDIA's
web_video_server, which on 2026-09-27 hung three times in an hour: with the
phone on weak Wi-Fi its page fell back to a snapshot a second, the requests
piled up (132 open connections) and the server stopped answering and ignored
SIGINT. Here one subscription feeds every viewer the latest frame; a slow
viewer simply skips frames, and one that stops reading for 10 s is dropped.

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

import json
import signal
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - then every viewer gets full size
    cv2 = None

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
        # Capture at the rate we send (2026-09-27): capturing at 30 fps and dropping frames
        # after the colour conversion cost the Argus daemon and each pipeline for every
        # dropped frame - ~30 % of a core per camera - and that CPU came out of the camera
        # odometry (29 -> 21 Hz with the dashboard open). If a sensor will not start at the
        # low rate (twice in a row, no frame), fall back to 30 fps + videorate.
        self.native, self.native_failures = True, 0
        threading.Thread(target=self._run, daemon=True, name=f'cam-{name}').start()

    def _wbmode(self) -> int:
        wb = int(self.node.get_parameter('wbmode').value)
        if wb >= 0:
            return wb
        hour = time.localtime().tm_hour + time.localtime().tm_min / 60.0
        day = float(self.node.get_parameter('day_from').value) <= hour < float(self.node.get_parameter('day_until').value)
        return 1 if day else 4

    def _command(self):
        # -e: on SIGINT, an end-of-stream through the pipeline before exiting (see _stop)
        head = ['gst-launch-1.0', '-e', '-q', 'nvarguscamerasrc', f'sensor-id={self.sensor}',
                f'wbmode={self._wbmode()}']
        tail = ['!', 'nvjpegenc', f'quality={self.quality}', '!', 'fdsink', 'fd=1']
        if self.native:      # the sensor itself runs at the rate we send
            return head + ['!', f'video/x-raw(memory:NVMM),width=1280,height=720,framerate={self.fps}/1',
                           '!', 'nvvidconv', 'flip-method=2',
                           '!', f'video/x-raw(memory:NVMM),width={self.width},height={self.height},format=I420'] + tail
        return head + ['!', 'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1',
                       '!', 'nvvidconv', 'flip-method=2',
                       '!', f'video/x-raw(memory:NVMM),width={self.width},height={self.height},format=I420',
                       '!', 'videorate', 'drop-only=true',
                       '!', f'video/x-raw(memory:NVMM),framerate={self.fps}/1'] + tail

    def _stop(self, proc):
        """End the pipeline the way GStreamer wants: SIGINT, which with -e is an end-of-stream
        through every element - the camera closed, the NVMM buffers freed - before it exits.

        SIGTERM, what it got until 2026-09-28, kills gst-launch at once with its buffers still
        mapped, and the kernel tears them down at exit; that day, after 791 such stops, the GPU
        driver oopsed doing it (NULL dereference in nvgpu_mem_get_addr, via gk20a_as_dev_release)
        and the kernel panicked with Rosie parked. SIGTERM and SIGKILL are only the fallbacks.
        """
        if proc.poll() is not None:
            return                                          # it ended by itself
        # keep reading while it winds down: fdsink must be able to write out what the
        # end-of-stream flushes, or it blocks on a full pipe and never finishes
        drain = threading.Thread(target=lambda: [None for _ in iter(lambda: proc.stdout.read(65536), b'')],
                                 daemon=True)
        drain.start()
        proc.send_signal(signal.SIGINT)
        for sig, wait_s in ((None, 8.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 3.0)):
            if sig is not None:
                self.node.get_logger().warning(f'{self.name}: the pipeline did not end on its own, {sig.name}')
                proc.send_signal(sig)
            try:
                proc.wait(timeout=wait_s)
                break
            except subprocess.TimeoutExpired:
                continue
        drain.join(timeout=1.0)

    def _run(self):
        while True:
            with self.cond:
                while self.viewers == 0:
                    self.cond.wait(1.0)
            cmd = self._command()
            self.node.get_logger().info(f'{self.name}: starting (sensor {self.sensor}, '
                                        f'{self.width}x{self.height} at {self.fps} fps, '
                                        f'wbmode {next(a for a in cmd if a.startswith("wbmode="))[7:]}, '
                                        f'{"sensor at that rate" if self.native else "sensor at 30 fps, frames dropped"})')
            got = ended_idle = False
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
                        got = True
                        with self.cond:
                            self.frame, self.seq = buf[start:end + 2], self.seq + 1
                            self.cond.notify_all()
                    buf = buf[end + 2:]
                if len(buf) > 4_000_000:
                    buf = b''
                with self.cond:
                    idle = self.viewers == 0 and time.monotonic() - self.last_viewer > 5.0
                if idle:
                    ended_idle = True
                    break
            self._stop(self.proc)
            self.proc = None
            with self.cond:
                self.frame = None
            if got:
                self.native_failures = 0
                self.node.get_logger().info(f'{self.name}: stopped (nobody watching)')
            elif ended_idle:
                self.node.get_logger().info(f'{self.name}: stopped (nobody watching)')
            elif self.native:
                self.native_failures += 1
                if self.native_failures >= 2:
                    self.native = False
                    self.node.get_logger().warning(
                        f'{self.name}: sensor {self.sensor} gave no picture at {self.fps} fps twice; '
                        'capturing at 30 fps and dropping frames instead')
                else:
                    self.node.get_logger().warning(f'{self.name}: pipeline ended without a picture; retrying')
            else:
                self.node.get_logger().warning(f'{self.name}: pipeline ended without a picture')
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


class RosCamera:
    """A JPEG topic as a camera: subscribed while there are viewers (the node's timer
    does the subscribing - rclpy is not touched from the HTTP threads)."""

    def __init__(self, node, name, topic):
        self.node, self.name, self.topic = node, name, topic
        self.cond = threading.Condition()
        self.frame, self.seq = None, 0
        self.viewers, self.last_viewer = 0, 0.0
        self.sub = None
        self.small = {}                     # (seq, w, q) -> jpeg, the latest resized copies
        node.create_timer(1.0, self._manage)

    def _manage(self):
        with self.cond:
            wanted = self.viewers > 0 or time.monotonic() - self.last_viewer < 5.0
        if wanted and self.sub is None:
            self.sub = self.node.create_subscription(CompressedImage, self.topic, self._on_image,
                                                     qos_profile_sensor_data)
            self.node.get_logger().info(f'{self.name}: relaying {self.topic}')
        elif not wanted and self.sub is not None:
            self.node.destroy_subscription(self.sub)
            self.sub = None
            with self.cond:
                self.frame = None
            self.node.get_logger().info(f'{self.name}: stopped (nobody watching)')

    def _on_image(self, msg):
        with self.cond:
            self.frame, self.seq = bytes(msg.data), self.seq + 1
            self.small.clear()
            self.cond.notify_all()

    def watch(self):
        with self.cond:
            self.viewers += 1

    def leave(self):
        with self.cond:
            self.viewers -= 1
            self.last_viewer = time.monotonic()

    def next_frame(self, after, timeout=5.0):
        with self.cond:
            self.cond.wait_for(lambda: self.frame is not None and self.seq != after, timeout)
            return self.seq, self.frame


def shrink(cam, seq, frame, width, quality):
    """A smaller, rougher copy for a slow link, made once per frame and size."""
    if cv2 is None or not frame or (width <= 0 and quality >= 90):
        return frame
    key = (seq, width, quality)
    small = cam.small.get(key) if hasattr(cam, 'small') else None
    if small is not None:
        return small
    img = cv2.imdecode(np.frombuffer(frame, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return frame
    if 0 < width < img.shape[1]:
        img = cv2.resize(img, (width, int(img.shape[0] * width / img.shape[1])), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    small = buf.tobytes() if ok else frame
    if hasattr(cam, 'small'):
        cam.small[key] = small
    return small


def make_handler(cameras, config=None):
    config_body = json.dumps(config or {}).encode()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):
            pass

        def do_GET(self):
            path, _, query = self.path.partition('?')
            args = {k: v[-1] for k, v in parse_qs(query).items()}
            if path.strip('/') == 'config.json':
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(config_body)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(config_body)
                return
            name, _, kind = path.strip('/').partition('.')
            cam = cameras.get(name)
            if cam is None or kind not in ('mjpg', 'jpg'):
                body = ('cameras: ' + ', '.join(f'{c}.mjpg {c}.jpg' for c in cameras)
                        + '  (?fps=N&w=WIDTH&q=QUALITY)').encode()
                self.send_response(HTTPStatus.OK if path.strip('/') == '' else HTTPStatus.NOT_FOUND)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            try:
                fps = max(0.2, min(30.0, float(args.get('fps', 30))))
                width, quality = int(args.get('w', 0)), int(args.get('q', 90))
            except ValueError:
                fps, width, quality = 30.0, 0, 90
            self.connection.settimeout(10.0)          # a viewer that stops reading is dropped
            cam.watch()
            try:
                if kind == 'jpg':
                    seq, frame = cam.next_frame(-1, timeout=8.0)
                    body = shrink(cam, seq, frame, width, quality) or b''
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
                seq, sent = -1, 0.0
                while True:
                    seq, frame = cam.next_frame(seq)
                    if frame is None:
                        continue
                    now = time.monotonic()
                    if now - sent < 1.0 / fps - 0.005:
                        continue
                    sent = now
                    frame = shrink(cam, seq, frame, width, quality)
                    self.wfile.write(b'--' + BOUNDARY + b'\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
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
        # The CSI cameras that run at all (the D435 relay always does). 2026-09-29, Steve: "What
        # about turning off the rear view for now?" - with the iPad watching, the CSI streams
        # cost gst-launch 15 % + nvargus-daemon 13 % of a core and nvargus held 744 MB; rear
        # (9.7 fps) was the least used. Add 'rear' back here to bring it back everywhere.
        self.declare_parameter('cameras', ['front'])
        self.declare_parameter('d435_topic', '/camera/color/image_raw/compressed')
        self.declare_parameter('d435_rotate', 0)      # degrees the pages turn the RealSense picture
        cams = {}
        wanted = [str(c) for c in self.get_parameter('cameras').value]
        for name in ('front', 'rear'):
            if name not in wanted:
                continue
            s, w, h, f, q = [int(v) for v in self.get_parameter(name).value]
            cams[name] = Camera(self, name, s, w, h, f, q)
        cams['d435'] = RosCamera(self, 'd435', str(self.get_parameter('d435_topic').value))
        port = int(self.get_parameter('port').value)
        # the pages and drive_log ask which cameras there are, and show / photograph only those
        config = {'d435_rotate': int(self.get_parameter('d435_rotate').value) % 360, 'cameras': list(cams)}
        self.server = ThreadingHTTPServer(('0.0.0.0', port), make_handler(cams, config))
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True, name='csi-http').start()
        self.get_logger().info(f'cameras on http://0.0.0.0:{port}/: {", ".join(cams)} (.mjpg / .jpg, on demand)')


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
