# Changelog

## 2.0.0 (unreleased)

The library is now board-agnostic maths; everything hardware-specific lives in
the examples. Sketches written for 1.x need updating.

- `MeltyDrive` replaces the `SimpleMelt` class. One `update()` call dispatches on
  `mode`; the heading is an input, supplied by a `HeadingEstimator`. The
  accelerometer-only dead reckoning (`meltyStateUpdate`) is gone: a
  `HeadingEstimator` that never sees a magnetometer sample does the same thing.
- `SimpleMelt.h` is now an umbrella header for `MeltyDrive` and `HeadingEstimator`.
  The library's own code depends on nothing but the Arduino core.
- `Button` is gone, along with `SimpleMeltUtility` and `OneShot125` (the V4
  ESCs run DShot). Switches, buttons and stick axes come from AlfredoCRSF 2.5:
  `CrsfSwitch` and `getAxis()`.
- The Rotini V4 example gained a `RotiniV4` class (in the example folder) that
  wraps the whole board: receiver, both sensors on their INT lines, both AM32
  ESCs over bidirectional DShot with EDT, LEDs and battery sense, behind
  `begin()`, `update()`, `setMotors()`, `setLeds()` and `printStatus()`. The
  sketch itself is the controller mapping, calibration numbers and telemetry
  channel list.
- The Rotini V3 example was rebuilt the same way: a `RotiniV3` class with the
  V3 pins and OneShot125 ESCs (`OneShot125.h`, in the example folder). The
  bench logs under `extras/` were removed; see the 1.x tags.
- The example depends on AlfredoCRSF 2.5 or newer, AlfredoFusion (sensor
  drivers), AlfredoDShot and AlfredoTelemetry.
- `MeltyDrive` has a slip limit. A step in spin power used to put the wheels at
  their commanded speed in milliseconds while the body took seconds to follow,
  sliding the tread the whole way (55 m per wheel on one full-power spin-up of
  Rotini V4). With `spin_rate` and `battery` supplied, the spin command is now
  the command that rolls the wheels at the body's speed, `volts_per_rad_s *
  spin_rate / battery`, plus `spin_push`, which does the pushing. `spin_power`
  is read as a target speed, `spin_power * battery / volts_per_rad_s`: the full lead
  is kept until the body is within `spin_push_fade_rad_s` of it, so the push does not tail
  off on the way up, and the command sits above `spin_power` by up to the lead
  while the body is below target. The limit acts on spin-up, after a hit, in a
  pin and on a restart while still turning. Setting `spin_push` to 1 switches
  it off. The translation waveform is applied to the limited command afterwards.
- A direction change while spinning no longer reverses the motors at full
  command. The drive brakes with the wheels just below rolling speed, carries
  the body through zero, and only then reports the new direction in
  `turning_reversed`, which is what the heading estimator should be given.
- The Rotini V4 example logs `spin_power`, `spin_power_sent`, `spin_power_rolling`,
  `spin_target_rad_s` and `spin_reversing`, and exposes `spin_push`, `volts_per_rad_s` and
  `spin_push_fade_rad_s` as live tunables, which it also logs.

## 1.0.0

Rotini V3 and V4 firmware with the original `SimpleMelt` class and the first
`HeadingEstimator`.
