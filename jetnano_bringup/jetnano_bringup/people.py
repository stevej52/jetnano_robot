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

"""People: who is in front of her, how far, and where their face is.

    ros2 run jetnano_bringup people
    ros2 topic echo /people            {"count": 1, "people": [{"id": 0, "conf": 0.91, "frame": "camera_color_optical_frame",
                                        "box": [x, y, w, h], "dist_m": 1.8, "pos": [x, y, z], "face_px": [u, v],
                                        "eyes": true, "face_deg": [left, up]}]}  at up to rate_hz
    http://<robot>:8082/people.mjpg    (later) the boxes drawn, through csi_cameras

YOLOv8n-pose as a TensorRT engine on the GPU (~10 ms a frame, fp16) on the fixed D435's colour
stream; the depth image, registered to the left imager, gives each person's distance at the torso
(median of the patch between the shoulders and hips, or the middle of the box). The colour and
depth imagers are 15 mm apart, ignored: at a metre or more that is a pixel. The person's position
is given in the camera's optical frame ("frame"); meet puts it on the map (a TransformListener
in here would cost a quarter of a core just following /tf). The face is the nose keypoint, or the
midpoint of the eyes; "eyes" means both eyes were seen, i.e. they are facing her.

The engine: export yolov8n-pose.pt to ONNX (ultralytics, on H2-Host) and build with
/usr/src/tensorrt/bin/trtexec --onnx=... --saveEngine=... --fp16 on the robot (TensorRT 10).
For the meet behaviour (2026-10-01): see / drive up / look / talk.
"""

import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

ENGINE = os.path.expanduser('~/vision/models/yolov8n-pose-fp16.engine')
SKELETON = [(5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12), (11, 12), (11, 13), (13, 15), (12, 14), (14, 16)]


class Cuda:
    """The four CUDA runtime calls we need, straight from libcudart through ctypes: the robot has
    TensorRT's Python module but neither cuda-python nor pycuda, and this needs no build."""

    H2D, D2H = 1, 2

    def __init__(self):
        import ctypes
        import ctypes.util
        name = ctypes.util.find_library('cudart') or '/usr/local/cuda/lib64/libcudart.so'
        self.lib = ctypes.CDLL(name)
        self.c = ctypes
        self.lib.cudaSetDeviceFlags(ctypes.c_uint(4))   # cudaDeviceScheduleBlockingSync: sleep, don't spin, while the GPU works
        self.lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.lib.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.lib.cudaMemcpyAsync.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int, ctypes.c_void_p]
        self.lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.lib.cudaGetErrorString.restype = ctypes.c_char_p

    def _ok(self, err, what):
        if err != 0:
            raise RuntimeError(f'{what}: {self.lib.cudaGetErrorString(err).decode()}')

    def malloc(self, n: int) -> int:
        ptr = self.c.c_void_p()
        self._ok(self.lib.cudaMalloc(self.c.byref(ptr), n), 'cudaMalloc')
        return ptr.value

    def stream(self) -> int:
        s = self.c.c_void_p()
        self._ok(self.lib.cudaStreamCreate(self.c.byref(s)), 'cudaStreamCreate')
        return s.value

    def copy(self, dst: int, src: int, n: int, kind: int, stream: int) -> None:
        self._ok(self.lib.cudaMemcpyAsync(dst, src, n, kind, stream), 'cudaMemcpyAsync')

    def sync(self, stream: int) -> None:
        self._ok(self.lib.cudaStreamSynchronize(stream), 'cudaStreamSynchronize')


class TrtPose:
    """YOLOv8-pose through TensorRT 10: input images (1,3,640,640) float32, output (1,56,8400)."""

    def __init__(self, path: str):
        import tensorrt as trt
        self.cuda = Cuda()
        logger = trt.Logger(trt.Logger.WARNING)
        with open(path, 'rb') as f, trt.Runtime(logger) as rt:
            self.engine = rt.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f'TensorRT could not load {path}')
        self.ctx = self.engine.create_execution_context()
        self.names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        self.inp = next(n for n in self.names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT)
        self.out = next(n for n in self.names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT)
        self.in_shape = tuple(self.engine.get_tensor_shape(self.inp))
        self.out_shape = tuple(self.engine.get_tensor_shape(self.out))
        self.h_in = np.zeros(self.in_shape, dtype=np.float32)
        self.h_out = np.zeros(self.out_shape, dtype=np.float32)
        self.d_in = self.cuda.malloc(self.h_in.nbytes)
        self.d_out = self.cuda.malloc(self.h_out.nbytes)
        self.ctx.set_tensor_address(self.inp, self.d_in)
        self.ctx.set_tensor_address(self.out, self.d_out)
        self.stream = self.cuda.stream()

    def infer(self, blob: np.ndarray) -> np.ndarray:
        np.copyto(self.h_in, blob)
        c = self.cuda
        c.copy(self.d_in, self.h_in.ctypes.data, self.h_in.nbytes, Cuda.H2D, self.stream)
        if not self.ctx.execute_async_v3(self.stream):
            raise RuntimeError('TensorRT enqueue failed')
        c.copy(self.h_out.ctypes.data, self.d_out, self.h_out.nbytes, Cuda.D2H, self.stream)
        c.sync(self.stream)
        return self.h_out


def letterbox(img: np.ndarray, size: int = 640):
    """-> (blob NCHW float32 RGB/255, scale, (pad_x, pad_y))."""
    h, w = img.shape[:2]
    s = size / max(h, w)
    nw, nh = int(round(w * s)), int(round(h * s))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    px, py = (size - nw) // 2, (size - nh) // 2
    canvas[py:py + nh, px:px + nw] = resized
    blob = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return blob.transpose(2, 0, 1)[None], s, (px, py)


def decode(out: np.ndarray, scale: float, pad, conf_min: float, iou: float = 0.5):
    """(1,56,8400) -> [(box xywh in image px, conf, keypoints (17,3) in image px)]."""
    p = out[0].T                                   # (8400, 56)
    keep = p[:, 4] >= conf_min
    p = p[keep]
    if not len(p):
        return []
    boxes = p[:, :4].copy()                        # cx cy w h in the 640 frame
    boxes[:, 0] -= boxes[:, 2] / 2
    boxes[:, 1] -= boxes[:, 3] / 2
    idx = cv2.dnn.NMSBoxes(boxes.tolist(), p[:, 4].tolist(), conf_min, iou)
    idx = [int(i) for i in np.asarray(idx).reshape(-1)]
    px, py = pad
    people = []
    for i in idx:
        x, y, w, h = boxes[i]
        box = [(x - px) / scale, (y - py) / scale, w / scale, h / scale]
        kp = p[i, 5:].reshape(17, 3).copy()
        kp[:, 0] = (kp[:, 0] - px) / scale
        kp[:, 1] = (kp[:, 1] - py) / scale
        people.append((box, float(p[i, 4]), kp))
    people.sort(key=lambda t: -t[0][2] * t[0][3])  # biggest first
    return people


class People(Node):

    def __init__(self):
        super().__init__('people')
        self.declare_parameter('engine', ENGINE)
        self.declare_parameter('color_topic', '/camera/color/image_raw/compressed')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('color_info', '/camera/color/camera_info')
        self.declare_parameter('depth_info', '/camera/depth/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('rotate_180', True)          # the D435 is mounted upside down (2026-09-27)
        self.declare_parameter('conf', 0.45)
        self.declare_parameter('rate_hz', 8.0)          # with someone in view
        self.declare_parameter('idle_hz', 3.0)          # nobody seen for 5 s
        self.declare_parameter('kp_conf', 0.3)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.conf, self.kp_conf = float(p('conf')), float(p('kp_conf'))
        self.period = 1.0 / float(p('rate_hz'))
        self.idle_period = 1.0 / float(p('idle_hz'))
        self.seen_at = 0.0
        cv2.setNumThreads(1)                            # the resize/colour work on one core, not all six
        self.rotate = bool(p('rotate_180'))
        self.frame = str(p('camera_frame'))
        t0 = time.monotonic()
        self.net = TrtPose(os.path.expanduser(str(p('engine'))))
        self.get_logger().info(f'YOLOv8n-pose engine ready in {time.monotonic() - t0:.1f} s ({self.net.in_shape} -> {self.net.out_shape})')
        self.pub = self.create_publisher(String, 'people', 10)
        self.img_pub = self.create_publisher(CompressedImage, 'people/image/compressed', 1)
        self.K_c = self.K_d = None
        self.size = (640, 480)
        self.depth = None
        self.depth_stamp = 0.0
        self.last = 0.0
        self.said = 0.0
        self.busy = 0.0
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Every message through rclpy's executor costs ~0.5 ms before any callback runs, so the
        # node takes as few as it can (profiled 2026-10-01: the executor was most of its CPU):
        # the intrinsics once each (then unsubscribed), the depth stream only while someone is
        # in view, and both images raw (serialized bytes), deserialized only for a frame it
        # looks at (3-8 a second of the 30 sent).
        self.info_subs = {}
        for key, topic in (('K_c', str(p('color_info'))), ('K_d', str(p('depth_info')))):
            self.info_subs[key] = self.create_subscription(CameraInfo, topic, lambda m, key=key: self._on_info(key, m), best_effort)
        self.depth_topic, self.depth_qos = str(p('depth_topic')), best_effort
        self.depth_sub = None
        self.depth_raw = None
        self.create_subscription(CompressedImage, str(p('color_topic')), self._on_color, best_effort, raw=True)
        self.create_timer(1.0, self._depth_wanted)
        # no TransformListener here: in Python it costs a core's quarter just keeping up with
        # /tf (the EKF's 100 Hz and the rest); meet puts the camera-frame point on the map

    def _on_info(self, key: str, m: CameraInfo) -> None:
        setattr(self, key, np.array(m.k).reshape(3, 3))
        sub = self.info_subs.pop(key, None)
        if sub is not None:
            self.destroy_subscription(sub)

    def _depth_wanted(self) -> None:
        want = time.monotonic() - self.seen_at < 5.0
        if want and self.depth_sub is None:
            self.depth_sub = self.create_subscription(Image, self.depth_topic, self._on_depth, self.depth_qos, raw=True)
        elif not want and self.depth_sub is not None:
            self.destroy_subscription(self.depth_sub)
            self.depth_sub = self.depth_raw = None

    def _on_depth(self, raw: bytes) -> None:
        self.depth_raw = raw
        self.depth_stamp = time.monotonic()

    def _depth_image(self):
        """The latest depth image as uint16 metres/1000, deserialized on demand."""
        if self.depth_raw is None or time.monotonic() - self.depth_stamp > 1.0:
            return None
        if getattr(self, '_depth_src', None) is not self.depth_raw:
            m = deserialize_message(self.depth_raw, Image)
            if m.encoding != '16UC1':
                return None
            self.depth = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)
            self._depth_src = self.depth_raw
        return self.depth

    def _distance(self, box, kp):
        """Metres at the torso from the depth image, or None."""
        depth = self._depth_image()
        if depth is None or self.K_c is None or self.K_d is None:
            return None
        x, y, w, h = box
        if kp[5, 2] > self.kp_conf and kp[6, 2] > self.kp_conf and kp[11, 2] > self.kp_conf and kp[12, 2] > self.kp_conf:
            u0, u1 = min(kp[5, 0], kp[6, 0]), max(kp[5, 0], kp[6, 0])
            v0, v1 = min(kp[5, 1], kp[6, 1]), max(kp[11, 1], kp[12, 1])
        else:
            u0, u1, v0, v1 = x + 0.3 * w, x + 0.7 * w, y + 0.25 * h, y + 0.65 * h
        # colour pixels -> depth pixels (same optical axis to within the 15 mm baseline); the
        # intrinsics belong to the sensor's own orientation, so undo the upright rotation first
        (u0, v0), (u1, v1) = self._sensor_px(u0, v0), self._sensor_px(u1, v1)
        u0, u1, v0, v1 = min(u0, u1), max(u0, u1), min(v0, v1), max(v0, v1)
        fx_c, fy_c, cx_c, cy_c = self.K_c[0, 0], self.K_c[1, 1], self.K_c[0, 2], self.K_c[1, 2]
        fx_d, fy_d, cx_d, cy_d = self.K_d[0, 0], self.K_d[1, 1], self.K_d[0, 2], self.K_d[1, 2]
        dh, dw = depth.shape
        du0 = int(np.clip((u0 - cx_c) / fx_c * fx_d + cx_d, 0, dw - 1))
        du1 = int(np.clip((u1 - cx_c) / fx_c * fx_d + cx_d, 0, dw - 1))
        dv0 = int(np.clip((v0 - cy_c) / fy_c * fy_d + cy_d, 0, dh - 1))
        dv1 = int(np.clip((v1 - cy_c) / fy_c * fy_d + cy_d, 0, dh - 1))
        patch = depth[dv0:dv1 + 1, du0:du1 + 1]
        valid = patch[(patch > 300) & (patch < 8000)]
        if valid.size < 20:
            return None
        return float(np.median(valid)) / 1000.0

    def _sensor_px(self, u, v):
        """An upright-image pixel as the sensor saw it (the image was turned 180 deg)."""
        if self.rotate:
            return self.size[0] - 1 - u, self.size[1] - 1 - v
        return u, v

    def _angles(self, u, v):
        """Where an upright-image pixel is, as seen from the camera: (left, up) in degrees."""
        fx, fy, cx, cy = self.K_c[0, 0], self.K_c[1, 1], self.K_c[0, 2], self.K_c[1, 2]
        su, sv = self._sensor_px(u, v)
        left, up = math.degrees(math.atan2(su - cx, fx)), math.degrees(math.atan2(sv - cy, fy))
        if not self.rotate:                       # right-handed sensor: +u is right, +v is down
            left, up = -left, -up
        return round(left, 1), round(up, 1)

    def _position(self, u, v, z):
        """The point for pixel (u, v) at depth z in the camera's optical frame (x right, y down,
        z forward, as the sensor sits) -> (frame, [x, y, z])."""
        fx, fy, cx, cy = self.K_c[0, 0], self.K_c[1, 1], self.K_c[0, 2], self.K_c[1, 2]
        u, v = self._sensor_px(u, v)
        X, Y = (u - cx) / fx * z, (v - cy) / fy * z
        return self.frame, [round(X, 3), round(Y, 3), round(z, 3)]

    def _on_color(self, raw: bytes) -> None:
        now = time.monotonic()
        if now - self.last < (self.period if now - self.seen_at < 5.0 else self.idle_period):
            return
        self.last = now
        m = deserialize_message(raw, CompressedImage)
        img = cv2.imdecode(np.frombuffer(m.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        if self.rotate:
            img = cv2.rotate(img, cv2.ROTATE_180)
        self.size = (img.shape[1], img.shape[0])
        t0 = time.monotonic()
        blob, scale, pad = letterbox(img, self.net.in_shape[-1])
        out = self.net.infer(blob)
        found = decode(out, scale, pad, self.conf)
        self.busy = 0.9 * self.busy + 0.1 * (time.monotonic() - t0)
        people = []
        for k, (box, conf, kp) in enumerate(found):
            dist = self._distance(box, kp)
            face = None
            if kp[0, 2] > self.kp_conf:
                face = [float(kp[0, 0]), float(kp[0, 1])]
            elif kp[1, 2] > self.kp_conf and kp[2, 2] > self.kp_conf:
                face = [float((kp[1, 0] + kp[2, 0]) / 2), float((kp[1, 1] + kp[2, 1]) / 2)]
            eyes = bool(kp[1, 2] > self.kp_conf and kp[2, 2] > self.kp_conf)
            frame, pos = (None, None)
            if dist is not None:
                u, v = box[0] + box[2] / 2, box[1] + box[3] * 0.4
                frame, pos = self._position(u, v, dist)
            look = face if face is not None else [box[0] + box[2] / 2, box[1] + 0.12 * box[3]]   # no face seen: the top of them
            people.append({'id': k, 'conf': round(conf, 2), 'box': [round(float(b), 1) for b in box],
                           'dist_m': None if dist is None else round(dist, 2), 'frame': frame, 'pos': pos,
                           'face_px': None if face is None else [round(face[0], 1), round(face[1], 1)], 'eyes': eyes,
                           'face_deg': list(self._angles(*look)) if self.K_c is not None else None})
        if people:
            self.seen_at = now
        msg = String()
        msg.data = json.dumps({'stamp': time.time(), 'count': len(people), 'image': [img.shape[1], img.shape[0]],
                               'infer_ms': round(self.busy * 1000, 1), 'people': people})
        self.pub.publish(msg)
        if people and now - self.said > 10.0:
            self.said = now
            near = min((q for q in people if q['dist_m'] is not None), key=lambda q: q['dist_m'], default=None)
            self.get_logger().info(f'{len(people)} person(s)' + (f', nearest {near["dist_m"]:.1f} m' if near else ', distance unknown')
                                   + f' ({self.busy * 1000:.0f} ms a frame)')
        if self.img_pub.get_subscription_count() > 0:
            self._draw(img, found, people)

    def _draw(self, img, found, people):
        for (box, conf, kp), q in zip(found, people):
            x, y, w, h = [int(v) for v in box]
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 200, 255), 2)
            label = f'{conf:.2f}' + (f' {q["dist_m"]:.1f} m' if q['dist_m'] else '')
            cv2.putText(img, label, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            for a, b in SKELETON:
                if kp[a, 2] > self.kp_conf and kp[b, 2] > self.kp_conf:
                    cv2.line(img, (int(kp[a, 0]), int(kp[a, 1])), (int(kp[b, 0]), int(kp[b, 1])), (255, 120, 0), 2)
            if q['face_px']:
                cv2.circle(img, (int(q['face_px'][0]), int(q['face_px'][1])), 6, (0, 255, 0) if q['eyes'] else (0, 0, 255), 2)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            m = CompressedImage()
            m.header.stamp = self.get_clock().now().to_msg()
            m.format = 'jpeg'
            m.data = jpg.tobytes()
            self.img_pub.publish(m)


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = People()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
