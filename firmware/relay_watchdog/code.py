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

# The board's own LEDs (active low: False lights them), as a pulse (Steve, 2026-10-02):
#   green  lub-dub, two short blips a second, while the heartbeat is alive and the relay is
#          closed - a pulse you can see from across the room
#   red    one short blink every two seconds while the board has power but no heartbeat
#          (the Jetson booting, the servo node down, a wire off)
#   dark   no power to the board, which also means no relay


def _led(name):
    try:
        pin = digitalio.DigitalInOut(getattr(board, name))
        pin.direction = digitalio.Direction.OUTPUT
        pin.value = True
        return pin
    except (AttributeError, ValueError):
        return None


green = _led('LED_GREEN')
red = _led('LED_RED')
LUB_DUB = ((0, 70), (200, 70))        # (start ms, length ms) within a 1000 ms beat
WAITING = ((0, 80),)                  # within a 2000 ms cycle


def lit(ms, pattern, period):
    t = ms % period
    for start, length in pattern:
        if start <= t < start + length:
            return True
    return False

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
    if green is not None:
        green.value = not (on and lit(ms, LUB_DUB, 1000))
    if red is not None:
        red.value = not ((not on) and lit(ms, WAITING, 2000))
    wd.feed()
