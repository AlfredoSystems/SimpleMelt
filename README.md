# SimpleMelt

Meltybrain combat robot control for the ESP32, made for the
[Rotini](https://github.com/AlfredoSystems) control board.

A meltybrain spins its whole body as the weapon and steers by pulsing the
motors at the right point of each revolution. Everything depends on knowing
the heading: this library estimates it from a high-g accelerometer and a
magnetometer, turns the sticks and that heading into motor powers, and runs
the Rotini V4 board's hardware.

```
  sticks ─┐                                     ┌─ motor powers ─┐
          ▼                                     ▼                ▼
  RotiniV4 ──► HeadingEstimator ──► MeltyDrive ──► RotiniV4 (ESCs, LEDs)
  (receiver,    (heading)            (steering)
   sensors)
```

## What's in it

| | |
|---|---|
| `MeltyDrive` | Drive modes (stop, arcade, melty, disconnected). Inputs are the sticks, spin power, direction and a heading; outputs are two motor powers and the LED states. Works on any board. |
| `HeadingEstimator` | Spin rate from the accelerometer's centripetal reading, integrated into a heading and corrected by the magnetometer with a complementary filter. Learns the accelerometer radius, the magnetometer's circle centre and the accelerometer offset while it runs. Works on any board. |
| `RotiniV4` | The Rotini V4 board: CRSF receiver, H3LIS331 accelerometer and MMC5983MA magnetometer on their interrupt lines, two AM32 ESCs over bidirectional DShot600 with telemetry, LEDs and battery sense. |
| `Button` | A button or switch position on a CRSF channel. |

`#include <SimpleMelt.h>` gets all of it; the individual headers are there for
boards that only want the maths.

## Quick start

`examples/Rotini-V4` is the complete firmware for a Rotini V4 robot with a
RadioMaster Zorro, and the place to start. Its loop is the whole architecture:

```cpp
board.update();                                    // receiver, sensors, battery
readController();                                  // sticks and switches -> drive inputs
heading.update(dt, board.accel_z, board.mag_fresh, board.mag_x, board.mag_y, board.mag_z, spin_dir, motors_off);
drive.heading = heading.theta;
drive.update();
board.setMotors(drive.motor_foo, drive.motor_bar);
board.setLeds(drive.melty_led, drive.status_led);
```

The example also streams about thirty channels (sensors, heading estimator
internals, ESC rpm, temperature, voltage and current) over
[AlfredoTelemetry](https://github.com/AlfredoSystems/AlfredoTelemetry), and
its `README.md` holds the bench notes: measured sensor rates and why they are
what they are, and a magnetometer glitch traced to the ESP32's radio.

## Requirements

- ESP32 Arduino core 3.x.
- [AlfredoCRSF](https://github.com/AlfredoSystems/AlfredoCRSF),
  [AlfredoFusion](https://github.com/AlfredoSystems/AlfredoFusion) (sensor drivers),
  [AlfredoDShot](https://github.com/AlfredoSystems/AlfredoDShot) and
  [AlfredoTelemetry](https://github.com/AlfredoSystems/AlfredoTelemetry).
- AM32 ESCs with "3D mode" on. See the AlfredoDShot README for the signal
  wiring and the configurator link.

## Rotini V3

Version 1.x of this library supports the Rotini V3 board (OneShot125 ESCs,
accelerometer-only heading). It is tagged `1.0.0` and stays as it is.
