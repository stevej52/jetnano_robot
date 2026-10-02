"""people: the detector's maths without a GPU - letterbox, output decoding, pixel angles."""
import numpy as np
import pytest

from jetnano_bringup import people


def test_letterbox_scales_and_pads_the_wider_side():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    blob, scale, (px, py) = people.letterbox(img, 640)
    assert blob.shape == (1, 3, 640, 640) and blob.dtype == np.float32
    assert scale == 1.0 and (px, py) == (0, 80)                 # 480 rows centred in 640
    assert blob[0, 0, 0, 0] == pytest.approx(114 / 255) and blob[0, 0, 100, 0] == 0.0


def _output(dets):
    """A fake (1, 56, 8400) YOLOv8-pose output with the given (cx, cy, w, h, conf) boxes."""
    out = np.zeros((1, 56, 8400), dtype=np.float32)
    for i, (cx, cy, w, h, conf) in enumerate(dets):
        out[0, :5, i] = [cx, cy, w, h, conf]
        out[0, 5:, i] = np.tile([cx, cy - h / 2 + 10, 0.9], 17)   # every keypoint near the head
    return out


def test_decode_maps_back_to_the_image_and_drops_duplicates():
    out = _output([(320, 320, 100, 200, 0.9), (322, 318, 100, 200, 0.6), (100, 100, 20, 20, 0.2)])
    found = people.decode(out, scale=1.0, pad=(0, 80), conf_min=0.45)
    assert len(found) == 1                                        # the overlap suppressed, the weak one gated
    box, conf, kp = found[0]
    assert conf == pytest.approx(0.9)
    assert [round(v) for v in box] == [270, 140, 100, 200]        # x = cx - w/2, y = cy - h/2 - pad
    assert kp.shape == (17, 3) and round(kp[0, 1]) == 320 - 100 + 10 - 80


def test_decode_biggest_first_and_scale():
    out = _output([(100, 100, 20, 40, 0.8), (300, 300, 200, 300, 0.7)])
    found = people.decode(out, scale=0.5, pad=(0, 0), conf_min=0.45)
    assert [round(f[1], 1) for f in found] == [0.7, 0.8]
    assert [round(v) for v in found[0][0]] == [400, 300, 400, 600]


class _Maths(people.People):
    def __init__(self):            # no ROS, no engine: just the pixel maths
        self.rotate = True
        self.size = (640, 480)
        self.frame = 'camera_color_optical_frame'
        self.K_c = np.array([[600.0, 0, 320.0], [0, 600.0, 240.0], [0, 0, 1]])


def test_angles_left_and_up_are_positive_on_the_upright_image():
    m = _Maths()
    assert m._sensor_px(0, 0) == (639, 479)                       # the sensor hangs upside down
    assert m._angles(319, 239) == (0.0, 0.0)
    left, up = m._angles(0, 239)
    assert left > 25 and up == 0.0
    left, up = m._angles(319, 0)
    assert left == 0.0 and up > 20
    left, up = m._angles(639, 479)
    assert left < -25 and up < -20


def test_position_is_in_the_sensors_optical_frame():
    m = _Maths()
    frame, (x, y, z) = m._position(319, 239, 2.0)                 # the upright centre, 2 m away
    assert frame == 'camera_color_optical_frame'
    assert abs(x) < 0.01 and abs(y) < 0.01 and z == 2.0
    _, (x, y, _) = m._position(0, 239, 2.0)                       # upright left = the sensor's +x
    assert x > 0.9 and abs(y) < 0.01
