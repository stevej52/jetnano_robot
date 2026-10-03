# Rosie's relay watchdog - CircuitPython for a Seeed XIAO RP2040 (docs/relay-watchdog.md).
#
# Sits between the Jetson's header pin 7 and the safety relay on the PCA9685's OE pin.
# The relay (every servo and throttle signal) is ON only while pin 7 keeps TOGGLING: the
# servo node toggles it from its own timer (ros2_pca9685 output_enable_heartbeat_hz: 20).
# A steady pin - high or low - means something stopped: the node, its executor, or the
# whole kernel (a frozen Jetson keeps its GPIO where it was, and the PCA9685 keeps its last
# pulses, so without this Rosie drove on at her last command until the hardware watchdog
# reset the board, up to 20 s). No edge for TIMEOUT_NS -> relay off, within ~0.2 s.
#
# Its own failures fail safe: no power, a crash (the RP2040's watchdog resets it in 1 s)
# or a reset all leave the relay pin floating or low, and the relay off.
import time

import board
import digitalio
import microcontroller
from watchdog import WatchDogMode

HEARTBEAT_PIN = board.D0      # from the Jetson's header pin 7
RELAY_PIN = board.D1          # to the relay module's IN (jumper on H: high = on)
TIMEOUT_NS = 200_000_000      # 20 Hz heartbeat: an edge every 25 ms; 8 missed edges = off
# (integer nanoseconds: CircuitPython's float time.monotonic() coarsens to tens of ms after days)
EDGES_TO_ARM = 10             # a steady stream first, not one glitch, before the relay closes

hb = digitalio.DigitalInOut(HEARTBEAT_PIN)
hb.direction = digitalio.Direction.INPUT
hb.pull = digitalio.Pull.DOWN
relay = digitalio.DigitalInOut(RELAY_PIN)
relay.direction = digitalio.Direction.OUTPUT
relay.value = False

# The board's RGB pixel as a pulse (Steve, 2026-10-02: "green and red LEDs all over this
# thing, another green light doesn't mean a whole lot"):
#   BLUE, swelling and fading like a heartbeat (lub-dub, once a second) while the heartbeat
#          is alive and the relay is closed - nothing else on her is blue and pulsing
#   AMBER, one short blink every two seconds, while the board has power but no heartbeat
#          (the Jetson booting, the servo node down, a wire off)
#   dark   no power to the board, which also means no relay
# The board's plain green/red LEDs stay off: they look like every other LED on the robot.
try:
    import neopixel_write
    _px_power = digitalio.DigitalInOut(board.NEOPIXEL_POWER)
    _px_power.direction = digitalio.Direction.OUTPUT
    _px_power.value = True
    px = digitalio.DigitalInOut(board.NEOPIXEL)
    px.direction = digitalio.Direction.OUTPUT
except (ImportError, AttributeError, ValueError):
    px = None
px_last = None


def bump(t, start, width):
    """1 at `start`, fading to 0 over `width` ms; 0 outside."""
    if start <= t < start + width:
        return 1.0 - (t - start) / width
    return 0.0


def heartbeat_level(ms):
    """0..1 brightness within a 1000 ms beat: the big lub, then the smaller dub."""
    t = ms % 1000
    return max(bump(t, 0, 260), 0.55 * bump(t, 280, 260))


def waiting_level(ms):
    return 1.0 if (ms % 2000) < 80 else 0.0


def show(level, colour):
    """Light the pixel at `level` (0..1) of `colour` (r, g, b); only writes on a change."""
    global px_last
    if px is None:
        return
    k = level * level                 # squared: the fade looks even to the eye
    frame = bytes((int(colour[1] * k), int(colour[0] * k), int(colour[2] * k)))   # GRB
    if frame != px_last:
        px_last = frame
        neopixel_write.neopixel_write(px, frame)


BLUE = (0, 40, 255)
AMBER = (255, 90, 0)

wd = microcontroller.watchdog
wd.timeout = 1.0
wd.mode = WatchDogMode.RESET

prev = hb.value
last_edge = time.monotonic_ns()
edges = 0
while True:
    now = time.monotonic_ns()
    v = hb.value
    if v != prev:
        prev = v
        last_edge = now
        edges = min(edges + 1, EDGES_TO_ARM)
    alive = now - last_edge < TIMEOUT_NS
    if not alive:
        edges = 0
    on = alive and edges >= EDGES_TO_ARM
    if relay.value != on:
        relay.value = on
    ms = now // 1_000_000
    if on:
        show(heartbeat_level(ms), BLUE)
    else:
        show(0.35 * waiting_level(ms), AMBER)
    wd.feed()
