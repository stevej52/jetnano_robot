# The voice switch

Since 2026-10-01 her voice and hearing are a separate service, `jetnano-voice.service`,
**off by default**. The robot service does not start it.

## Why

The microphone's USB audio stream panicked the kernel in the middle of drive 20
(`xhci_invalidate_cancelled_tds`, a bug in the Tegra kernel). Tests and laps run without it;
it is loaded when she is to be shown off.

## Using it

```
ros2 run jetnano_bringup voice on       load: ears, settle, sounds, speak, listen, people
ros2 run jetnano_bringup voice off      unload them all
ros2 run jetnano_bringup voice status
```

On the driving page the VOICE row has LOAD / UNLOAD, and TALK loads the stack before she
greets. `meet` loads it before she looks at a person. `drive.sh` unloads it before a lap.

While it is off, a flag `~/voice/off` exists: the watchdog does not count the microphone or
the listener as missing, and the people detector is not running either (it lives in the voice
launch, since nothing but `meet` uses it).

## What is in it

`jetnano_bringup/launch/voice.launch.py`: `people`, `sounds`, `ears`, `settle`, `speak`,
`listen`. The service binds to the robot service (it stops with it) and reads `VOICE_ARGS`
from `/etc/default/jetnano-robot` (root only; it holds keys, never print it whole).

Loaded, the stack costs about half a core more and about 2 GB of memory (the speech model is
1.2 GB of it, the people detector 0.8 GB), measured on the bench on 2026-10-01.
