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

"""csi_cameras, 2026-11-01 review: the CSI cameras keep their resized copies like the D435's,
one per frame and size, and drop them with the next frame."""

import numpy as np
import pytest

from jetnano_bringup import csi_cameras

cv2 = pytest.importorskip('cv2')


@pytest.fixture
def cam(monkeypatch):
    class NoThread:
        def __init__(self, *a, **kw):
            pass

        def start(self):
            pass
    monkeypatch.setattr(csi_cameras.threading, 'Thread', NoThread)     # no pipeline
    return csi_cameras.Camera(None, 'front', 0, 960, 540, 10, 85)


def jpeg(shade):
    return cv2.imencode('.jpg', np.full((540, 960, 3), shade, np.uint8))[1].tobytes()


def test_a_resized_frame_is_made_once(cam, monkeypatch):
    cam.frame, cam.seq = jpeg(90), 1
    decodes = []
    real = cv2.imdecode
    monkeypatch.setattr(csi_cameras.cv2, 'imdecode', lambda *a: decodes.append(1) or real(*a))
    first = csi_cameras.shrink(cam, cam.seq, cam.frame, 640, 75)
    again = csi_cameras.shrink(cam, cam.seq, cam.frame, 640, 75)
    assert again is first and len(decodes) == 1
    assert cv2.imdecode(np.frombuffer(first, np.uint8), cv2.IMREAD_COLOR).shape[1] == 640


def test_the_next_frame_drops_the_copies(cam):
    cam._store(jpeg(90))
    csi_cameras.shrink(cam, cam.seq, cam.frame, 640, 75)
    csi_cameras.shrink(cam, cam.seq, cam.frame, 320, 50)
    assert len(cam.small) == 2
    cam._store(jpeg(160))
    assert cam.small == {} and cam.seq == 2
