# Relay watchdog (fitted 2026-10-02)

## Why

The safety relay on the PCA9685's OE pin is driven straight from the Jetson's
header pin 7. That covers a clean shutdown and a panic (the board reboots in
5 s and the pin resets with it), but not a **freeze**: a hung kernel keeps
pin 7 where it was, and the PCA9685 keeps sending its last pulses. Rosie would
keep driving at her last command until the hardware watchdog resets the board,
which can take up to 20 s.

The fix is a small microcontroller between pin 7 and the relay. The servo node
toggles pin 7 at 20 Hz from its own timer. The microcontroller closes the relay
only while those edges keep arriving, and opens it 0.2 s after they stop. A
frozen kernel, a stuck node or a dead executor all stop the edges.

## Parts

| Part | Link | Note |
|---|---|---|
| Seeed Studio XIAO RP2040, Pre-Soldered | https://www.seeedstudio.com/Seeed-Studio-XIAO-RP2040-Pre-Soldered-p-6333.html | the header pins are already soldered |
| 4 female-female jumper wires | - | |
| 10 kΩ resistor (optional) | - | pin 7 to GND, so pin 7 reads low while the Jetson boots |

## Wiring

| From | To | Why |
|---|---|---|
| Jetson header pin 7 | XIAO **D0** | the heartbeat (3.3 V on both sides) |
| XIAO **D1** | relay **IN** (take it off pin 7) | the XIAO switches the relay now |
| XIAO **5V** (VCC on the back) | PCA9685 **V+** (a servo header's middle pin) | powers the XIAO |
| XIAO **GND** | PCA9685 **GND** | common ground with the Jetson and the relay |

The relay's own wiring doesn't change: DC+ on V+, jumper on H, COM to OE, NC to
VCC, NO to GND.

**Don't plug the XIAO's USB into a computer while its 5V pin is wired to V+.**
Its 5V pin is connected directly to USB power, so the two supplies would fight.
Unplug the 5V wire first whenever you reprogram it.

## Installing the program

1. Hold the XIAO's **B** (boot) button while plugging it into the laptop over
   USB-C. A drive called `RPI-RP2` appears.
2. Download CircuitPython for the XIAO RP2040 from
   https://circuitpython.org/board/seeeduino_xiao_rp2040/ and drag the `.uf2`
   file onto `RPI-RP2`. The board restarts as a drive called `CIRCUITPY`.
3. Copy `firmware/relay_watchdog/code.py` onto `CIRCUITPY`, replacing the
   `code.py` that is already there.
4. Unplug it, then wire it as in the table above.

The XIAO's RGB pixel is the indicator (firmware 54048e8, to load at the next USB session):
**blue, swelling and fading once a second like a heartbeat** while the heartbeat is alive and
the relay is closed; a short **amber blink every two seconds** while the board has power but
no heartbeat; dark means no power and no relay. The relay module's own LED shows the relay
state too (dimmer than before: the XIAO drives IN at 3.3 V).

## The heartbeat

On since 2026-10-02 in `jetnano_bringup/config/pca9685.yaml` (fitted that evening; the
servo node's setting had never been copied into its settings before, ros2_pca9685 537846c):

```yaml
    output_enable_heartbeat_hz: 20.0
```

Then restart the robot service. Without the XIAO, a toggling pin 7 would click
the relay 20 times a second. The line is in the file already, commented out.

The node's log says `outputs ENABLED (output-enable pin 7 high, heartbeat 20 Hz)`.

## Tests (wheels up)

1. **Start-up:** restart the robot service. The relay clicks on a quarter of a
   second after the servo node enables its outputs (10 edges), and the XIAO's green LED lights.
2. **Stop from ROS:** `ros2 topic pub --once /pca9685/output_enable
   std_msgs/Bool "{data: false}"`. The relay drops within 0.2 s. Publish
   `true` and it comes back.
3. **Frozen node:** freeze the servo node for 2 s, then let it go:

   ```bash
   pid=$(pgrep -n -f '^/usr/bin/python3 .*pca9685_node')
   kill -STOP "$pid"; sleep 2; kill -CONT "$pid"
   ```

   The relay drops within 0.2 s of the STOP, then comes back after the
   CONT.
4. **XIAO unplugged:** pull its 5V wire. The relay drops, and every servo and
   the ESC go limp.

What the pca9685 node's own logs say during a test is in `journalctl -u
jetnano-robot`.

## Fitted and proven (2026-10-02, 21:16-22:04)

Stop from ROS, a servo node frozen for 2 s, and re-enable all go through the XIAO; the
relay follows within 0.2 s and comes back on release. Two lessons from the fitting:

- The relay's **IN** terminal must carry one wire only, from the XIAO's D1. On the first
  try the D1 wire sat on another relay terminal while the old pin-7 wire still fed IN: the
  relay chattered at 20 Hz (a flickering light, servos stuttering) and looked as if it
  "worked". Pulling the D1 wire with the relay still pulsing gave it away.
- A cut of the relay chops whatever servo pulse is in flight, and a chopped pulse sends a
  servo to its end stop (pan hard right, steering hard left). Every stop the servo node
  commands now silences all channels first, waits one PWM period, then drops the pin
  (ros2_pca9685). A cut from outside - the XIAO on a frozen node, a pulled wire - is
  abrupt by nature and may twitch a servo once.
