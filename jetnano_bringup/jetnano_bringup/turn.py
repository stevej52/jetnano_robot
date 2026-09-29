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

"""Is he done talking? For listen's turn-taking.

Steve, 2026-09-28: "I have been known to pause for like 15 seconds trying to
find the right word ... A human would know by the context already given that
you weren't done talking." A fixed pause cannot do that - half a second cuts
him off mid-thought, fifteen would make every answer wait fifteen - so at each
short pause two things are asked:

* the WORDS (``unfinished``): a sentence that stops on "and", "the", "to",
  "because", "my", an "um" or a trailing comma is not over, however long the
  pause;
* the SOUND (``SmartTurn``): Pipecat's open Smart Turn v3.2 model (BSD-2,
  https://github.com/pipecat-ai/smart-turn, Whisper-tiny based, 8 MB int8)
  hears the intonation - a voice that trails off to think against one that
  lands - and gives the probability that the turn is complete.

The model wants Whisper's log-mel features of the last 8 s; they are computed
here in numpy (the same maths as transformers' WhisperFeatureExtractor with
chunk_length=8, checked against it on Rosie 2026-09-28) so listen does not
need transformers.
"""

import os
import re

import numpy as np

RATE = 16000
SECONDS = 8
N_FFT, HOP, N_MELS = 400, 160, 80
DEFAULT_MODEL = os.path.expanduser('~/voice/models/smart-turn/smart-turn-v3.2-cpu.onnx')

# A sentence ending on one of these is not finished, however long the pause.
DANGLING = {
    'and', 'or', 'but', 'so', 'because', 'cause', 'then', 'than', 'if', 'when', 'while', 'unless', 'until',
    'although', 'though', 'whether', 'where', 'which', 'who', 'whose', 'that', 'what',
    'the', 'a', 'an', 'this', 'these', 'those', 'some', 'any', 'every', 'each', 'another', 'no',
    'my', 'your', 'his', 'her', 'its', 'our', 'their',
    'to', 'of', 'for', 'with', 'in', 'on', 'at', 'from', 'into', 'onto', 'about', 'by', 'over', 'under',
    'between', 'through', 'toward', 'towards', 'like', 'as', 'around', 'behind', 'near', 'past',
    'is', 'are', 'was', 'were', 'be', 'been', 'am', 'will', 'would', 'should', 'could', 'can', 'do', 'does',
    'did', 'have', 'has', 'had', 'gonna', 'wanna', 'i', 'we', "i'm", "we're", "let's",
    'um', 'uh', 'er', 'erm', 'hmm', 'mm', 'uhm', 'ah', 'well',
}
FILLER = re.compile(r"(\.\.\.|…|,|;|:|-)\s*$")


def unfinished(text: str) -> bool:
    """The words stop mid-sentence (see DANGLING) or on a comma, colon or dash."""
    t = text.strip()
    if not t:
        return False
    if FILLER.search(t):
        return True
    words = re.findall(r"[a-z']+", t.lower())
    return bool(words) and words[-1] in DANGLING


def _hz_to_mel(f):
    f = np.asarray(f, dtype=np.float64)
    lin = 3.0 * f / 200.0
    log = 15.0 + np.log(np.maximum(f, 1e-10) / 1000.0) * (27.0 / np.log(6.4))
    return np.where(f >= 1000.0, log, lin)


def _mel_to_hz(m):
    m = np.asarray(m, dtype=np.float64)
    lin = 200.0 * m / 3.0
    log = 1000.0 * np.exp(np.log(6.4) * (m - 15.0) / 27.0)
    return np.where(m >= 15.0, log, lin)


def _mel_filters() -> np.ndarray:
    """(201 x 80) slaney mel filter bank, 0-8 kHz: as transformers' mel_filter_bank."""
    fft_freqs = np.linspace(0, RATE // 2, 1 + N_FFT // 2)
    filter_freqs = _mel_to_hz(np.linspace(_hz_to_mel(0.0), _hz_to_mel(RATE / 2), N_MELS + 2))
    diff = np.diff(filter_freqs)
    slopes = filter_freqs[None, :] - fft_freqs[:, None]
    down = -slopes[:, :-2] / diff[:-1]
    up = slopes[:, 2:] / diff[1:]
    fb = np.maximum(0.0, np.minimum(down, up))
    return fb * (2.0 / (filter_freqs[2:N_MELS + 2] - filter_freqs[:N_MELS]))[None, :]


_FB = _mel_filters()
_WINDOW = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(N_FFT) / N_FFT)      # periodic Hann


def last_seconds(audio: np.ndarray) -> np.ndarray:
    """The last 8 s, zero-padded at the front to exactly 8 s (Smart Turn's audio_utils)."""
    x = np.asarray(audio, dtype=np.float32)
    n = SECONDS * RATE
    return x[-n:] if len(x) >= n else np.pad(x, (n - len(x), 0))


def features(audio: np.ndarray) -> np.ndarray:
    """Whisper log-mel (1 x 80 x 800) of 8 s of 16 kHz audio, normalised like
    WhisperFeatureExtractor(chunk_length=8)(..., padding='max_length', do_normalize=True)."""
    x = last_seconds(audio).astype(np.float64)
    x = (x - x.mean()) / np.sqrt(x.var() + 1e-7)
    x = np.pad(x, (N_FFT // 2, N_FFT // 2), mode='reflect')                 # centred frames
    n = 1 + (len(x) - N_FFT) // HOP
    frames = np.lib.stride_tricks.as_strided(x, shape=(n, N_FFT), strides=(x.strides[0] * HOP, x.strides[0]))
    power = np.abs(np.fft.rfft(frames * _WINDOW, n=N_FFT, axis=1)) ** 2       # (n, 201)
    mel = np.maximum(power @ _FB, 1e-10).T                                    # (80, n)
    log = np.log10(mel)[:, :-1]
    log = np.maximum(log, log.max() - 8.0)
    return ((log + 4.0) / 4.0).astype(np.float32)[None, :, :]


class SmartTurn:
    """``complete(audio)`` -> probability that the speaker has finished (0-1)."""

    def __init__(self, path: str = DEFAULT_MODEL, threads: int = 1):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        so.inter_op_num_threads = 1
        so.intra_op_num_threads = threads
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(path, sess_options=so, providers=['CPUExecutionProvider'])

    def complete(self, audio: np.ndarray) -> float:
        out = self.session.run(None, {'input_features': features(audio)})
        return float(np.asarray(out[0]).reshape(-1)[0])
