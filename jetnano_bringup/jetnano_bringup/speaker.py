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

"""Whose voice is that?

    voices                       who she knows, and how many samples of each
    voices forget Jill
    voices rename Gill Jill
    voices test a.wav b.wav      what she makes of recordings (16 kHz mono wav)

(``voices`` = ``~/venv-voice/bin/python3 -m jetnano_bringup.speaker``, the
same Python listen runs in; plain ``ros2 run`` lacks sherpa-onnx.)

A voice print (a 512-number embedding from WeSpeaker's CAM++ model, run by
sherpa-onnx) is taken from every utterance listen.py hears, right after the
speech-to-text, and compared with the prints she has been taught. Cosine
similarity is the score: the same person usually lands around 0.5-0.8,
another person around 0.1-0.3, so ``MATCH`` 0.45 means "confidently that
person" and ``SAME`` 0.35 "the voice from a moment ago". The percentage she
quotes is that score stretched over 0.15..0.65. Both thresholds are
parameters of listen; check them against Steve and his wife once both are
enrolled (``voices test``). It is not security - a good recording of Steve
would pass - it is knowing who is in the room (Steve, 2026-09-26: "not Fort
Knox, but a fairly confident percentage").

Prints live in ``~/voice/speakers/<Name>.npz`` (all samples; the centroid is
recomputed on load). Cost: ~75 ms of one core per second of speech, only
when someone has spoken. The first voice she is taught is the owner's.
"""

import glob
import os
import sys
import wave

import numpy as np

DEFAULT_MODEL = os.path.expanduser('~/voice/models/speaker/wespeaker_en_voxceleb_CAM++.onnx')
DEFAULT_DIR = os.path.expanduser('~/voice/speakers')
MATCH = 0.55            # confidently this person (2026-09-27: Steve live >= 0.65; other voices 0.18-0.44)
SAME = 0.35             # the same voice as a moment ago
MIN_SECONDS = 0.6       # shorter than this, no print: too little to go on
SURE_SECONDS = 1.0      # shorter than this, a print is a hint, never a confident match
TOP_K = 2               # a score is the better of: cosine to the centroid, mean of the K closest samples
PERCENT_LOW, PERCENT_HIGH = 0.15, 0.65
MAX_SAMPLES = 30


def cosine(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.dot(a, b) / ((np.linalg.norm(a) * np.linalg.norm(b)) + 1e-9))


def percent(score: float) -> int:
    return int(round(100.0 * min(1.0, max(0.0, (score - PERCENT_LOW) / (PERCENT_HIGH - PERCENT_LOW)))))


class Who:
    """What one utterance sounded like: the closest known voice and how close."""

    __slots__ = ('name', 'score', 'emb', 'match', 'seconds', 'audio')

    def __init__(self, name, score: float, emb, match: float = MATCH, seconds: float = SURE_SECONDS, audio=None):
        self.name, self.score, self.emb, self.match, self.seconds = name, score, emb, match, seconds
        self.audio = audio              # the sound itself, kept only while a lesson may want to save it

    @property
    def confident(self) -> bool:
        return self.name is not None and self.score >= self.match and self.seconds >= SURE_SECONDS

    @property
    def percent(self) -> int:
        return percent(self.score)

    def is_owner(self, owner: str) -> bool:
        return self.confident and self.name.lower() == owner.lower()

    @property
    def label(self) -> str:
        return f'{self.name if self.confident else "unknown"} {self.percent}%' + (
            f' (nearest {self.name})' if self.name and not self.confident else '')

    def describe(self, owner: str) -> str:
        """For the brain's prompt."""
        if self.is_owner(owner):
            return f'{owner}, your owner ({self.percent} percent sure by voice)'
        if self.confident:
            return f'{self.name}, someone whose voice you know ({self.percent} percent sure)'
        return 'someone whose voice you do not know: a guest, or a voice from the TV'


class Voices:

    def __init__(self, model: str = DEFAULT_MODEL, directory: str = DEFAULT_DIR, threads: int = 1,
                 match: float = MATCH, same: float = SAME):
        import sherpa_onnx
        cfg = sherpa_onnx.SpeakerEmbeddingExtractorConfig(model=model, num_threads=threads, debug=False,
                                                           provider='cpu')
        self.extractor = sherpa_onnx.SpeakerEmbeddingExtractor(cfg)
        self.dir = directory
        self.match, self.same = match, same
        self.samples = {}           # name -> (n, dim) array
        self.centroids = {}         # name -> unit vector
        self.load()

    # ---------------------------------------------------------------- files --

    def load(self) -> None:
        self.samples, self.centroids = {}, {}
        for path in sorted(glob.glob(os.path.join(self.dir, '*.npz'))):
            name = os.path.splitext(os.path.basename(path))[0]
            try:
                s = np.load(path)['samples']
            except (OSError, ValueError, KeyError):
                continue
            if s.ndim == 2 and len(s):
                self.samples[name] = s.astype(np.float32)
                self._centroid(name)

    def _centroid(self, name: str) -> None:
        c = self.samples[name].mean(axis=0)
        self.centroids[name] = c / (np.linalg.norm(c) + 1e-9)

    def _save(self, name: str) -> None:
        os.makedirs(self.dir, exist_ok=True)
        np.savez(os.path.join(self.dir, f'{name}.npz'), samples=self.samples[name])

    def names(self):
        return sorted(self.samples)

    def has(self, name: str) -> bool:
        return any(n.lower() == name.lower() for n in self.samples)

    def canonical(self, name: str) -> str:
        return next((n for n in self.samples if n.lower() == name.lower()), name)

    def add(self, name: str, emb: np.ndarray) -> int:
        name = self.canonical(name)
        e = np.asarray(emb, dtype=np.float32)[None, :]
        self.samples[name] = np.concatenate([self.samples.get(name, e[:0]), e])[-MAX_SAMPLES:]
        self._centroid(name)
        self._save(name)
        return len(self.samples[name])

    def forget(self, name: str) -> bool:
        name = self.canonical(name)
        if name not in self.samples:
            return False
        del self.samples[name], self.centroids[name]
        try:
            os.remove(os.path.join(self.dir, f'{name}.npz'))
        except OSError:
            pass
        return True

    def rename(self, old: str, new: str) -> bool:
        old = self.canonical(old)
        if old not in self.samples or self.has(new):
            return False
        self.samples[new] = self.samples.pop(old)
        self.centroids[new] = self.centroids.pop(old)
        try:
            os.rename(os.path.join(self.dir, f'{old}.npz'), os.path.join(self.dir, f'{new}.npz'))
        except OSError:
            self._save(new)
        return True

    # ---------------------------------------------------------------- sound --

    def embed(self, samples: np.ndarray):
        """A unit-length voice print, or None when there is too little sound."""
        if len(samples) < MIN_SECONDS * 16000:
            return None
        s = self.extractor.create_stream()
        s.accept_waveform(sample_rate=16000, waveform=np.asarray(samples, dtype=np.float32))
        s.input_finished()
        e = np.array(self.extractor.compute(s), dtype=np.float32)
        return e / (np.linalg.norm(e) + 1e-9)

    def who(self, samples: np.ndarray):
        """The closest known voice (its name and score, even when not close), or None if too short."""
        emb = self.embed(samples)
        if emb is None:
            return None
        w = self.identify(emb, len(samples) / 16000.0)
        w.audio = samples
        return w

    def save_samples(self, name: str, audios) -> int:
        """Keep the recordings a voice was learned from (``voices test`` can then re-score them
        when the thresholds are tuned). 16 kHz mono wav under samples/<Name>-<n>.wav."""
        d = os.path.join(self.dir, 'samples')
        os.makedirs(d, exist_ok=True)
        n = len(glob.glob(os.path.join(d, f'{name}-*.wav')))
        for x in audios:
            if x is None:
                continue
            n += 1
            with wave.open(os.path.join(d, f'{name}-{n}.wav'), 'wb') as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(16000)
                w.writeframes((np.clip(np.asarray(x, dtype=np.float32), -1, 1) * 32767).astype(np.int16).tobytes())
        return n

    def score(self, emb: np.ndarray, name: str) -> float:
        """How much like this person: the better of the centroid and the closest few samples
        (one print of a voice varies a lot sentence to sentence; the centroid alone under-scores)."""
        sims = np.sort(self.samples[name] @ emb)[::-1]
        return max(cosine(emb, self.centroids[name]), float(sims[:TOP_K].mean()))

    def identify(self, emb: np.ndarray, seconds: float = SURE_SECONDS) -> Who:
        best, score = None, -1.0
        for name in self.centroids:
            s = self.score(emb, name)
            if s > score:
                best, score = name, s
        return Who(best, score, emb, self.match, seconds)

    def cosine(self, a, b) -> float:
        return cosine(a, b)


# ------------------------------------------------------------------- tool --

def _read_wav(path: str) -> np.ndarray:
    with wave.open(path) as w:
        rate, ch = w.getframerate(), w.getnchannels()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float32) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    if rate != 16000:
        idx = np.minimum((np.arange(int(len(x) * 16000 / rate)) * rate / 16000).astype(int), len(x) - 1)
        x = x[idx]
    return x.astype(np.float32)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] in ('-h', '--help'):
        print(__doc__.split('\n\n')[0])
        return 0
    try:
        v = Voices()
    except ModuleNotFoundError as exc:
        print(f'{exc}: run this with ~/venv-voice/bin/python3 -m jetnano_bringup.speaker')
        return 1
    if not argv:
        if not v.names():
            print('she knows nobody yet: say "Rosie, learn my voice"')
        for n in v.names():
            print(f'{n}: {len(v.samples[n])} samples')
        return 0
    cmd, args = argv[0], argv[1:]
    if cmd == 'forget' and args:
        print('forgotten' if v.forget(args[0]) else f'no such voice: {args[0]}')
    elif cmd == 'rename' and len(args) == 2:
        print('renamed' if v.rename(args[0], args[1]) else 'cannot rename (unknown, or the new name exists)')
    elif cmd == 'test' and args:
        for path in args:
            x = _read_wav(path)
            w = v.who(x)
            if w is None:
                print(f'{path}: too short ({len(x) / 16000:.1f} s)')
                continue
            scores = ', '.join(f'{n} {v.score(w.emb, n):.2f}' for n in v.centroids)
            print(f'{path}: {w.label}   [{scores}]')
    else:
        print(__doc__.split('\n\n')[0])
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
