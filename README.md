# SimpleMelt

Meltybrain combat robot control for the ESP32, made for the
[Rotini](https://github.com/AlfredoSystems) control board.

A meltybrain spins its whole body as the weapon and steers by pulsing the
motors at the right point of each revolution. Everything depends on knowing
the heading: this library estimates it from a high-g accelerometer and a
magnetometer, and turns the sticks and that heading into motor powers. It
knows nothing about any particular board; the Rotini V4 firmware in
`examples/` shows how a board plugs in.

```
  sticks ─┐                                     ┌─ motor powers ─┐
          ▼                                     ▼                ▼
  board ──► HeadingEstimator ──► MeltyDrive ──► board (ESCs, LEDs)
  (receiver,  (heading)           (steering)
   sensors)
```

## What's in it

| | |
|---|---|
| `MeltyDrive` | Drive modes (stop, arcade, melty, disconnected). Inputs are the sticks, spin power, direction and a heading; outputs are two motor powers and the LED states. Given the body's spin rate it also holds the spin command to what the wheels can use without sliding the tread (the slip limit), and it brakes before reversing on a direction change. |
| `HeadingEstimator` | Spin rate from the accelerometer's centripetal reading, integrated into a heading and corrected by the magnetometer with a complementary filter. Learns the accelerometer radius, the magnetometer's circle centre and the accelerometer offset while it runs. |

Both are plain maths on floats; `#include <SimpleMelt.h>` gets both. Switches,
buttons and stick axes come from AlfredoCRSF (`CrsfSwitch`, `getAxis()`).

## Quick start

`examples/Rotini-V4` is the complete firmware for a Rotini V4 robot with a
RadioMaster Zorro, and the place to start. Its `RotiniV4` class (in the
example folder, not the library) wraps the board: CRSF receiver, H3LIS331
accelerometer and MMC5983MA magnetometer on their interrupt lines, two AM32
ESCs over bidirectional DShot600 with telemetry, LEDs and battery sense. To
run on another board, write the equivalent of that class; the loop stays the
same:

```cpp
board.update();                                    // receiver, sensors, battery
readController();                                  // sticks and switches -> drive inputs
heading.update(dt, board.accel_z, board.mag_fresh, board.mag_x, board.mag_y, board.mag_z, spin_dir, motors_off);
drive.heading = heading.theta;
drive.spin_rate = heading.w;                       // slip limit: the body's spin rate
drive.battery = board.vin;                         // and the battery voltage
drive.update();
board.setMotors(drive.motor_foo, drive.motor_bar);
board.setLeds(drive.melty_led, drive.status_led);
```

`spin_dir` comes from `drive.turning_reversed`, the direction the body is
really turning, not from the direction switch: after a direction change the
drive brakes first and reports the new direction only once the body has slowed.

The example also streams about thirty channels (sensors, heading estimator
internals, ESC rpm, temperature, voltage and current) over
[AlfredoTelemetry](https://github.com/AlfredoSystems/AlfredoTelemetry), and
its `README.md` holds the bench notes: measured sensor rates and why they are
what they are, and a magnetometer glitch traced to the ESP32's radio.

## Requirements

The library itself needs only the Arduino core. The Rotini V4 example needs:

- ESP32 Arduino core 3.x.
- [AlfredoCRSF](https://github.com/AlfredoSystems/AlfredoCRSF) 2.5 or newer,
  [AlfredoFusion](https://github.com/AlfredoSystems/AlfredoFusion) (sensor drivers),
  [AlfredoDShot](https://github.com/AlfredoSystems/AlfredoDShot) and
  [AlfredoTelemetry](https://github.com/AlfredoSystems/AlfredoTelemetry).
- AM32 ESCs with "3D mode" on. See the AlfredoDShot README for the signal
  wiring and the configurator link.

## Rotini V3

`examples/Rotini-V3` is the same firmware for the Rotini V3 board: its
`RotiniV3` class has the V3 pins and drives OneShot125 ESCs (`OneShot125.h`,
in the example folder) instead of DShot, so there is no ESC telemetry. The
original V3 firmware, with the 1.x `SimpleMelt` class and accelerometer-only
heading, is tagged `1.0.0`.
