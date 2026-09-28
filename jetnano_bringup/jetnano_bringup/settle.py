"""Settle the steering servos after a stop, by ear.

On carpet the tyres wind up when the wheels turn, and a steering servo holding
that load chatters - a buzz that can go on for ever once she has stopped. A
small back-and-forth lets the load off and it goes quiet, but not reliably:
measured 2026-09-27 with her own microphone, every fixed wiggle (1.5 to 12
degrees, single or double) sometimes left the buzz behind, and the buzz itself
comes in bursts. So this node does what Steve asked for - "listen to yourself
with the microphone and keep doing it until you don't hear nothing":

* it listens to ``sound/audio`` (the ears' 16 kHz stream) for the buzz's
  signature, a cluster of tones around 1.6 kHz: the prominence of the loudest
  bins over the band's median, per half second, reference free so the room
  and the TV do not shift it (buzzing +15..+18 dB, faint bursts +10, room +3..+8);
* once she has been still for a moment (no cmd_vel with anything in it) and
  the last three seconds hold two or more buzzing half seconds, it wiggles the
  steering a few degrees each way through twist_mux (``cmd_vel_settle``, the
  lowest priority, so a driver, Nav2 or an e-stop lock always wins);
* then listens again, wiggling a little bigger each time, four goes a round;
  if it still buzzes it comes back for another round 20 s later, up to three
  rounds per stop; the result is logged so the numbers can be tuned from the journal.

2026-09-28, on the floor with Nav2 driving: 4-6-8-8 degrees gave up twice in five
minutes (12-16 dB left) and the buzz went on until she moved; Steve: "wiggle a
little harder". Now 6-10-14-18 degrees, and rounds instead of one try. Bigger alone
did not do it (two rounds, 15-19 dB left), so each wiggle is now Steve's "turn it fast
and then bring it back slow": a snap to one side to break the tyres loose, held a
quarter second, then a slow ramp back to centre so the load does not wind up again;
alternate tries go to alternate sides.

Publishes ``steering/buzz`` (dB per half second) for watching it live.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int16MultiArray

RATE_HZ = 16000
CHUNK = 1600                       # the ears send 100 ms a message


class Settle(Node):
    def __init__(self) -> None:
        super().__init__('settle')
        self.declare_parameter('band_hz', [1400.0, 1800.0])   # where the servo buzz lives
        self.declare_parameter('buzz_db', 8.0)                # prominence that counts as buzzing (room 3-6, faint buzz 8-10)
        self.declare_parameter('hits', 2)                      # ... in this many of the last
        self.declare_parameter('windows', 4)                   # ... half-second windows (2 s: Steve, 2026-09-27,
                                                                #     "jittered for a long time")
        self.declare_parameter('after_stop_s', 1.0)            # still this long before listening
        self.declare_parameter('between_s', 2.0)               # listen this long after a wiggle
        self.declare_parameter('wiggle_deg', [6.0, 10.0, 14.0, 18.0])  # each try, a little bigger
        self.declare_parameter('rounds', 3)                    # rounds of those per stop ...
        self.declare_parameter('retry_s', 20.0)                # ... this far apart
        self.declare_parameter('step_s', 0.25)                 # the snap, held this long
        self.declare_parameter('return_s', 1.2)                # then back to centre this slowly
        self.declare_parameter('tick_s', 0.05)                 # one steering command every
        # ros2_pca9685 steers by cmd_vel angular.z: |twist.angular_z| in pca9685.yaml
        self.declare_parameter('deg_per_rad_s', 18.33)

        lo, hi = self.get_parameter('band_hz').value
        freqs = np.fft.rfftfreq(CHUNK, 1.0 / RATE_HZ)
        self.band = (freqs >= lo) & (freqs <= hi)
        self.window = np.hanning(CHUNK).astype(np.float32)
        self.powers: list[np.ndarray] = []      # chunk spectra of the current half second
        self.recent: list[float] = []           # buzz dB of the last few half seconds
        self.locked = {'e_stop': False, 'e_stop_web': False, 'e_stop_joy': False}
        self.last_drive = self._now()
        self.quiet_from = self.last_drive       # windows before this hold her own motion, not a buzz
        self.next_check = self.last_drive
        self.attempt = 0
        self.round = 1
        self.gave_up_at = None                  # the end of a round that did not quiet it
        self.reported = True
        self.plan: list[float] = []
        self.side = 1.0                         # the next snap's side; flips every wiggle
        self.step_timer = None
        self.talking = False                    # someone speaking (listen), or her own voice
        self.speaking = False
        self.hush_until = 0.0                   # ... and a second after, or after a bang

        self.buzz_pub = self.create_publisher(Float32, 'steering/buzz', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_settle', 10)
        self.create_subscription(Int16MultiArray, 'sound/audio', self._on_audio, 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        for topic in self.locked:
            self.create_subscription(Bool, topic, self._on_lock(topic), 10)
        # Voices and bangs have energy in the buzz band too: on 2026-09-27 people
        # talking beside her set off four wiggles in a minute with no buzz at all.
        self.create_subscription(Bool, 'speech/active', self._on_talking, 10)
        self.create_subscription(Bool, 'sound/speaking', self._on_speaking, 10)
        self.create_subscription(Bool, 'sound/loud', self._on_loud, 10)
        self.get_logger().info(
            f'listening for steering buzz at {lo:g}-{hi:g} Hz over {self.get_parameter("buzz_db").value:g} dB; '
            f'wiggles {", ".join(f"{d:g}" for d in self.get_parameter("wiggle_deg").value)} deg')

    # ------------------------------------------------------------------ inputs --
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_lock(self, topic: str):
        def callback(msg: Bool) -> None:
            self.locked[topic] = msg.data
        return callback

    def _on_talking(self, msg: Bool) -> None:
        if self.talking and not msg.data:
            self.hush_until = self._now() + 1.0
        self.talking = msg.data

    def _on_speaking(self, msg: Bool) -> None:
        if self.speaking and not msg.data:
            self.hush_until = self._now() + 1.0
        self.speaking = msg.data

    def _on_loud(self, msg: Bool) -> None:
        if msg.data:
            self.hush_until = self._now() + 1.0

    def _on_cmd(self, msg: Twist) -> None:
        if self.plan or self.step_timer is not None:
            return                                          # our own wiggle coming back round
        if any(abs(v) > 1e-6 for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
            if self.attempt and not self.reported:
                self._report('moved on')
            self.last_drive = self._now()
            self.quiet_from = self.last_drive + float(self.get_parameter('after_stop_s').value)
            self.attempt = 0
            self.round = 1
            self.gave_up_at = None
            self.reported = True
            self.recent.clear()
            self.powers.clear()

    def _on_audio(self, msg: Int16MultiArray) -> None:
        if len(msg.data) < CHUNK:
            return
        x = np.asarray(msg.data[:CHUNK], dtype=np.float32) * (1.0 / 32768.0)
        self.powers.append(np.abs(np.fft.rfft(x * self.window)) ** 2)
        if len(self.powers) < 5:
            return
        power = np.mean(self.powers, axis=0)
        self.powers.clear()
        band = np.sort(power[self.band])
        buzz = float(10.0 * math.log10(max(band[-3:].mean(), 1e-20) / max(float(np.median(band)), 1e-20)))
        self.buzz_pub.publish(Float32(data=buzz))
        if self.step_timer is not None or self._now() < self.quiet_from:
            self.recent.clear()                             # her own motion is not a buzz
            return
        if self.talking or self.speaking or self._now() < self.hush_until:
            self.recent.clear()                             # a voice or a bang is not a buzz
            return
        windows = int(self.get_parameter('windows').value)
        self.recent = (self.recent + [buzz])[-windows:]
        self._decide()

    # ---------------------------------------------------------------- decision --
    def _buzzing(self) -> bool:
        threshold = float(self.get_parameter('buzz_db').value)
        return sum(b >= threshold for b in self.recent) >= int(self.get_parameter('hits').value)

    def _decide(self) -> None:
        now = self._now()
        wiggles = list(self.get_parameter('wiggle_deg').value)
        if now < self.next_check:
            return
        if any(self.locked.values()):
            return                                          # stopped means nothing moves
        if self._buzzing():
            if self.attempt >= len(wiggles):
                rounds = int(self.get_parameter('rounds').value)
                if self.gave_up_at is None:
                    self.gave_up_at = now
                    self._report(f'round {self.round} of {rounds} did not quiet it'
                                 if self.round < rounds else 'giving up until she moves')
                if self.round >= rounds or now - self.gave_up_at < float(self.get_parameter('retry_s').value):
                    return
                self.round += 1
                self.attempt = 0
                self.gave_up_at = None
            deg = wiggles[self.attempt]
            self.attempt += 1
            self.reported = False
            self.get_logger().info(
                f'steering buzzing ({max(self.recent):.0f} dB): round {self.round}, '
                f'wiggle {self.attempt} of {len(wiggles)}, {deg:g} deg')
            self._wiggle(deg)
            return
        elif self.attempt and not self.reported and len(self.recent) >= int(self.get_parameter('windows').value):
            self._report('quiet')

    def _report(self, outcome: str) -> None:
        self.reported = True
        level = max(self.recent) if self.recent else float('nan')
        done = (self.round - 1) * len(self.get_parameter('wiggle_deg').value) + self.attempt
        text = f'{outcome} after {done} wiggle{"s" if done != 1 else ""} ({level:.0f} dB)'
        (self.get_logger().warning if outcome.startswith('giving up') else self.get_logger().info)(text)

    # ------------------------------------------------------------------ wiggle --
    def _wiggle(self, deg: float) -> None:
        # out fast, back slow: the snap held for step_s, then a ramp to centre over return_s
        z = self.side * deg / float(self.get_parameter('deg_per_rad_s').value)
        self.side = -self.side
        tick = float(self.get_parameter('tick_s').value)
        hold = max(1, round(float(self.get_parameter('step_s').value) / tick))
        ramp = max(1, round(float(self.get_parameter('return_s').value) / tick))
        self.plan = [z] * hold + [z * (1.0 - k / ramp) for k in range(1, ramp + 1)]
        self.recent.clear()
        self.powers.clear()
        self.next_check = (self._now() + len(self.plan) * tick
                           + float(self.get_parameter('between_s').value))
        self._step()
        self.step_timer = self.create_timer(tick, self._step)

    def _step(self) -> None:
        if not self.plan:
            if self.step_timer is not None:
                self.step_timer.cancel()
                self.step_timer = None
                self.quiet_from = self._now() + 0.5      # the wheels coming back to centre
            return
        msg = Twist()
        msg.angular.z = self.plan.pop(0)
        self.cmd_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = Settle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
