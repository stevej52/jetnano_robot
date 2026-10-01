#!/usr/bin/env python3
"""Rosie's voice server: Kokoro (sherpa-onnx, CPU) on H2-Host, for the speak node on the robot.

    POST /tts   JSON {"text": "...", "voice": "sky", "speed": 1.0}  ->  audio/wav (16-bit mono)
    GET  /health                                                    ->  {"ok": true, "voices": [...]}

Runs as the systemd user service rosie-tts (port 8092). Rosie's speak node tries this first and
falls back to its local Piper voice when the server does not answer. Names the English G2P gets
wrong are respelled before synthesis (RESPELL), the cheap fix until a proper lexicon.
"""
import io
import json
import os
import re
import threading
import time
import wave
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import sherpa_onnx

MODEL = os.path.expanduser('~/voice/models/kokoro-en-v0_19')
PORT = int(os.environ.get('ROSIE_TTS_PORT', '8092'))
THREADS = int(os.environ.get('ROSIE_TTS_THREADS', '8'))
VOICES = {'af': 0, 'bella': 1, 'nicole': 2, 'sarah': 3, 'sky': 4, 'adam': 5, 'michael': 6, 'emma': 7, 'isabella': 8, 'george': 9, 'lewis': 10}
DEFAULT_VOICE = os.environ.get('ROSIE_TTS_VOICE', 'nicole')
# the household's names as the G2P should say them
RESPELL = {'Rosie': 'Rozie', 'Rosa': 'Rozah', 'Diane': 'Dyann', 'Queenie': 'Kweenie', 'Sadie': 'Saydee', 
           'Scrunchy': 'Scrunchee', 'Frankie': 'Frankie', 'Annabelle Lee': 'Anabelee', 'Annabelle': 'Anabel', 'Pumpkin': 'Pumpkin', 'Beans': 'Beans', 'Totoro': 'Toto', 'Baby': 'Baby',
           'Jetson': 'Jetson', 'ESC': 'E S C', 'IMU': 'I M U', 'Nav2': 'Nav two', 'lidar': 'lie-dar', 'LIDAR': 'lie-dar'}
_respell = re.compile(r'\b(' + '|'.join(re.escape(k) for k in RESPELL) + r')\b')
import hashlib
RESPELL_VERSION = hashlib.md5(json.dumps(RESPELL, sort_keys=True).encode()).hexdigest()[:8]   # into the robot's cache key

lock = threading.Lock()
tts = sherpa_onnx.OfflineTts(sherpa_onnx.OfflineTtsConfig(model=sherpa_onnx.OfflineTtsModelConfig(
    kokoro=sherpa_onnx.OfflineTtsKokoroModelConfig(model=f'{MODEL}/model.onnx', voices=f'{MODEL}/voices.bin', tokens=f'{MODEL}/tokens.txt',
                                                   data_dir=f'{MODEL}/espeak-ng-data'),
    num_threads=THREADS, provider='cpu')))
print(f'kokoro ready: {tts.num_speakers} voices, {THREADS} threads, port {PORT}', flush=True)


def synth(text, voice, speed):
    text = _respell.sub(lambda m: RESPELL[m.group(1)], text)
    sid = VOICES.get(voice, VOICES[DEFAULT_VOICE])
    with lock:
        a = tts.generate(text, sid=sid, speed=float(speed))
    y = np.asarray(a.samples, dtype=np.float32)
    y = y / (float(np.max(np.abs(y))) or 1.0) * 0.8
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(a.sample_rate)
        w.writeframes((y * 32767).astype(np.int16).tobytes())
    return buf.getvalue(), len(y) / a.sample_rate


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _json(self, obj, status=HTTPStatus.OK):
        body = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith('/health'):
            self._json({'ok': True, 'engine': 'kokoro-en-v0_19', 'voices': sorted(VOICES), 'default': DEFAULT_VOICE, 'respell': RESPELL_VERSION})
        else:
            self._json({'error': 'not found'}, HTTPStatus.NOT_FOUND)

    def do_POST(self):
        if not self.path.startswith('/tts'):
            self._json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
            return
        n = int(self.headers.get('Content-Length', '0'))
        try:
            req = json.loads(self.rfile.read(n) or b'{}')
            text = str(req.get('text', '')).strip()
            if not text:
                raise ValueError('empty text')
            t0 = time.time()
            wav, seconds = synth(text, str(req.get('voice', DEFAULT_VOICE)), req.get('speed', 1.0))
            print(f'{seconds:4.1f} s audio in {time.time() - t0:4.2f} s: "{text[:70]}"', flush=True)
        except Exception as exc:  # noqa: BLE001
            self._json({'error': str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', 'audio/wav')
        self.send_header('Content-Length', str(len(wav)))
        self.send_header('X-Audio-Seconds', f'{seconds:.2f}')
        self.end_headers()
        self.wfile.write(wav)


if __name__ == '__main__':
    ThreadingHTTPServer(('0.0.0.0', PORT), Handler).serve_forever()
