"""The camera pan-tilt on the Jetson's own hardware PWM (since 2026-10-06).

Until then the two micro servos were PCA9685 channels 8 and 9. That board's outputs are gated by
the pin-7 safety relay, whose coil (and the XIAO watchdog) are on the servo rail - dead on the
bench with no pack - so the head could not look round on the bench. The servos are now powered
by their own Castle BEC and their signals come from header pins:

    pin 32  pwm7  32e0000.pwm  PAN      pin 33  pwm5  32c0000.pwm  TILT      pin 34  GND
    (as wired 2026-10-06, confirmed by Steve watching the head)

(robot-environment rosie-hdr40-pin7-output.dts muxes the two pins.) The relay still cuts the
ESC and steering; the head is no longer behind it, which is the point.

Same interface as before, so web_teleop /look, motion_watch, meet and nav_helper are unchanged:

    /pca9685/pan/angle   std_msgs/Float64, degrees 0-180 (higher = LEFT)
    /pca9685/tilt/angle  std_msgs/Float64, degrees 0-180 (higher = DOWN)

The angle-to-pulse calibration is the one measured on 2026-09-26 (the defaults below, moved
from pca9685.yaml; each can be overridden as a parameter, e.g. pan.min_limit): pulse = min_pulse + (max_pulse - min_pulse) * angle / 180, clamped to the limits.
Nothing moves at start-up; a command holds until the next (no timeout); on shutdown the pulses
stop (servos limp), as "on shutdown off" did on the PCA9685.
"""

import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

# 200 Hz, not the textbook 50: the Orin's PWM has only 8 bits of pulse width per period (256 steps)
# and runs from a 408 MHz clock, so its longest period is ~5.1 ms. At 5 ms a step is 19.5 us (~2 deg
# of pan); at 20 ms (with a slower clock) it would be 78 us, ~8 deg - too coarse to aim a camera.
# The micro servos follow a 200 Hz frame (camera-tested 2026-10-06).
PERIOD_NS = 5_000_000


def find_chip(device):
    """/sys/class/pwm/pwmchipN whose device is `device` (e.g. '32c0000.pwm'), or None."""
    root = '/sys/class/pwm'
    for name in sorted(os.listdir(root)):
        path = os.path.join(root, name)
        if os.path.basename(os.path.realpath(os.path.join(path, 'device'))) == device:
            return path
    return None


def pulse_us(angle, cfg):
    """Degrees -> microseconds, clamped to the servo's limits."""
    a = max(cfg['min_limit'], min(cfg['max_limit'], float(angle)))
    return cfg['min_pulse_us'] + (cfg['max_pulse_us'] - cfg['min_pulse_us']) * a / 180.0


class Servo:
    def __init__(self, chip):
        self.dir = os.path.join(chip, 'pwm0')
        if not os.path.isdir(self.dir):
            with open(os.path.join(chip, 'export'), 'w') as f:
                f.write('0')
        # a fresh export: udev hands pwm0's files to the gpio group a moment later (2026-10-06,
        # first boot: "Permission denied" on enable)
        for _ in range(50):
            if os.access(os.path.join(self.dir, 'enable'), os.W_OK) and os.access(os.path.join(self.dir, 'duty_cycle'), os.W_OK):
                break
            time.sleep(0.1)
        # the tegra PWM driver's rules (2026-10-06, tried by hand): nothing can be written before
        # the period; it will not enable with a duty of 0. So: period now, and the first command
        # sets the pulse and enables; limp = disabled (the pin's pull-down holds it low).
        self._write('period', PERIOD_NS)
        self.enabled = False
        try:
            self._write('enable', 0)
        except OSError:
            pass

    def _write(self, name, value):
        with open(os.path.join(self.dir, name), 'w') as f:
            f.write(str(int(value)))

    def set_us(self, us):
        self._write('duty_cycle', us * 1000)
        if not self.enabled:
            self._write('enable', 1)
            self.enabled = True

    def limp(self):
        try:
            self._write('enable', 0)
            self.enabled = False
        except OSError:
            pass


class HeadPwm(Node):
    def __init__(self):
        super().__init__('head_pwm')
        self.servos, self.cfg = {}, {}
        for joint, device, mn, mx, lo, hi in (('pan', '32e0000.pwm', 700.0, 2450.0, 5.0, 180.0),
                                              ('tilt', '32c0000.pwm', 750.0, 2250.0, 12.0, 168.0)):
            p = lambda k, d, j=joint: self.declare_parameter(f'{j}.{k}', d).value  # noqa: E731
            cfg = {'device': p('device', device), 'min_pulse_us': float(p('min_pulse_us', mn)),
                   'max_pulse_us': float(p('max_pulse_us', mx)), 'min_limit': float(p('min_limit', lo)),
                   'max_limit': float(p('max_limit', hi)), 'topic': p('topic', f'/pca9685/{joint}/angle')}
            chip = find_chip(cfg['device'])
            if chip is None:
                self.get_logger().error(f'{joint}: no PWM controller {cfg["device"]} in /sys/class/pwm')
                continue
            try:
                self.servos[joint] = Servo(chip)
            except OSError as exc:
                self.get_logger().error(f'{joint}: cannot set up {chip}: {exc}')
                continue
            self.cfg[joint] = cfg
            self.create_subscription(Float64, cfg['topic'], lambda m, j=joint: self.on_angle(j, m.data), 10)
            # as pca9685's <name>/pulse_width: a pulse in us, 0 = limp (the buzz hunt, 2026-10-08)
            self.create_subscription(Float64, cfg['topic'].rsplit('/', 1)[0] + '/pulse_width',
                                     lambda m, j=joint: self.on_pulse(j, m.data), 10)
            self.get_logger().info(f'{joint}: {cfg["device"]} ({os.path.basename(chip)}), '
                                   f'{cfg["min_pulse_us"]:.0f}-{cfg["max_pulse_us"]:.0f} us over 0-180 deg, '
                                   f'limits {cfg["min_limit"]:.0f}..{cfg["max_limit"]:.0f}, on {cfg["topic"]}')

    def on_angle(self, joint, angle):
        try:
            self.servos[joint].set_us(pulse_us(angle, self.cfg[joint]))
        except OSError as exc:
            self.get_logger().warning(f'{joint}: {exc}', throttle_duration_sec=5.0)

    def on_pulse(self, joint, us):
        cfg = self.cfg[joint]
        try:
            if us <= 0.0:
                self.servos[joint].limp()
            else:
                self.servos[joint].set_us(min(max(us, cfg['min_pulse_us']), cfg['max_pulse_us']))
        except OSError as exc:
            self.get_logger().warning(f'{joint}: {exc}', throttle_duration_sec=5.0)

    def limp(self):
        for s in self.servos.values():
            s.limp()


def main(args=None):
    rclpy.init(args=args)
    node = HeadPwm()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.limp()
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: BLE001 - the context is gone after an external shutdown
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
