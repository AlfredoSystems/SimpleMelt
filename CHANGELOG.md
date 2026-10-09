# Changelog

## 2.0.0 (unreleased)

The library is now organised around what is reusable and what is board-specific.
Sketches written for 1.x need updating; Rotini V3 stays on 1.x.

- `MeltyDrive` replaces the `SimpleMelt` class. One `update()` call dispatches on
  `mode`; the heading is an input, supplied by a `HeadingEstimator`. The
  accelerometer-only dead reckoning (`meltyStateUpdate`) is gone: a
  `HeadingEstimator` that never sees a magnetometer sample does the same thing.
- `RotiniV4` is new: the whole board (receiver, both sensors on their INT
  lines, both AM32 ESCs over bidirectional DShot with EDT, LEDs, battery sense)
  behind `begin()`, `update()`, `setMotors()`, `setLeds()` and `printStatus()`.
- `Button` moved to its own header. `SimpleMeltUtility` and `OneShot125` are
  gone (the V4 ESCs run DShot).
- `SimpleMelt.h` is now an umbrella header that includes everything.
- The Rotini V4 example shrank to the controller mapping, calibration numbers
  and telemetry channel list. The Rotini V3 example and the bench logs under
  `extras/` were removed; see the 1.x tags for both.
- Depends on AlfredoFusion (sensor drivers), AlfredoDShot and AlfredoTelemetry.

## 1.0.0

Rotini V3 and V4 firmware with the original `SimpleMelt` class and the first
`HeadingEstimator`.
