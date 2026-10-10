# Rotini V4 notes

## Sensor sample rates (2026-10-08)

Both sensors are read from their INT lines (LIS331 DRDY on GPIO 37, MMC5983MA
measurement-done on GPIO 36). The once-a-second USB status line prints the
interrupt rates; measured on the bench:

| Sensor | Setting | Measured | Why |
|---|---|---|---|
| LIS331 | 400 Hz ODR | 415/s | internal oscillator ~4 % fast |
| MMC5983MA | 1000 Hz, BW 800, Auto SET/RESET on | 618/s | see below |
| MMC5983MA | same, Auto SET/RESET off | 1234/s | oscillator ~23 % fast |

Auto SET/RESET halves the magnetometer rate: the chip takes a SET and a RESET
measurement per output sample to cancel its own offset. It stays on, because
the heading estimator was tuned with it and it is what keeps the chip offset
from drifting with temperature; 618/s is more than the filter uses.

The driver's continuous-mode rates are 1000, 200, 100, 50, 20, 10 and 1 Hz
(nothing between 200 and 1000), and the bandwidth setting caps the real rate
by fixing the measurement time: BW 800 = 0.5 ms (the only one that supports
1000 Hz), BW 400 = 2 ms, BW 200 = 4 ms, BW 100 = 8 ms. Lower bandwidth is
lower noise (datasheet: ~0.4 mG RMS at BW 800, ~0.12 mG at BW 100) but more
lag per fix. 200 Hz was not measured with Auto SET/RESET on.

With the reads interrupt-driven, `loop()` runs at ~13.9 kHz (it was ~875/s
when it polled the accelerometer every pass).

## Magnetometer Z glitch: caused by the ESP32 radio (2026-10-08)

### Symptom

In the 2026-10-07 telemetry logs the MMC5983MA Z axis jumps by +2 to +6 uT
(typically +3.5 uT) on 17 to 38 % of fresh samples, always in the positive
direction, usually for 1 to 3 samples in a row. X dips by about 4 % of the Z
jump; Y is untouched. At rest Z is the `v` input of the heading `atan2(u, v)`
and happens to be small, so a +3.5 uT step swings the heading 10 to 18 deg,
always the same way. While spinning the same hit is a 3 uT error on a 20 uT
circle, so it looks like ordinary noise and biases the tracked centre `cv` by
about 1 uT (a fixed heading offset of roughly 3 deg, absorbed by the LED trim).

### What it is not

Rebuilding the raw 18-bit counts from the logged uT values ruled out the
firmware and the SPI read:

- The jump is not a fixed 512 counts. It ranges smoothly from 300 to 1100
  counts, so it is not a stuck or flipped bit.
- The two low bits (register 0x06) are evenly distributed in glitched and
  normal samples, so the last byte is not torn.
- X and Y always change by normal noise on a glitched sample, so it is a real
  new measurement, not a corrupted re-read.
- With the radio off, the sensor at the firmware's exact settings (1000 Hz,
  BW 800, automatic SET/RESET, SPI) is clean: Z noise 0.15 uT, zero jumps.
  Auto SET/RESET off, 200 Hz, and accelerometer traffic on the shared SPI bus
  are also clean.

### What it is

Every ESP-NOW transmission by the ESP32-S3 puts a one-sided +Z pulse on the
magnetometer. A standalone test sketch (magnetometer only, same driver and
pins, raw counts streamed over USB) sent broadcast packets at known times.
The pulse starts about 1.5 ms after `esp_now_send()` and lasts the packet's
airtime plus about 1 ms. It scales with transmit power and with airtime.
Measured at 50 packets/s of 250 bytes, robot sitting still:

| Radio setting                     | Samples hit | Mean jump | Pulse length |
|-----------------------------------|-------------|-----------|--------------|
| 1 Mbps, 11 dBm                    | 17 %        | +7.0 uT   | ~3 ms        |
| 12 Mbps, 11 dBm (library default) | 4 %         | +2.6 uT   | ~1 ms        |
| 54 Mbps or MCS7, 11 dBm           | < 1 %       | +1.7 uT   | ~1 ms        |
| 12 Mbps, 2 dBm                    | 1.6 %       | +1.7 uT   | ~1 ms        |
| MCS7, 2 dBm                       | 0 %         | none      | none         |
| 21 dBm (max), 1 Mbps              | 18 %        | +12.8 uT  | ~3 ms        |
| Radio off                         | 0 %         | none      | none         |

The logs look worse than the 12 Mbps default row because AlfredoTelemetry's
adaptive ladder steps down to 6, 2 and 1 Mbps after a few first-try failures
(signal was -82 to -85 dBm) and unicast packets wait for the dongle's ACK and
retry, so a packet can transmit more than once.

Note for anyone correlating with loop timing: the 10 ms loop stalls in the
logs are the 100 Hz CRSF packets, not the sends. `radioSend()` blocks the loop
while the packet is in flight, so a send never shows up as a gap between
samples.

### Decision (2026-10-08)

No fix for now. The complementary filter averages the hits while spinning and
the LED trim covers the small `cv` bias. Options if it ever matters:

1. Blank magnetometer samples for a few ms after each send. The library knows
   when it transmits; rejecting fresh samples 0 to ~3 ms after a send removed
   every jump on the bench and keeps most of the 600 samples/s.
2. Pin the radio at a fast rate (54 Mbps or MCS7) and drop the power. Cuts the
   hits below 1 %; MCS7 at 2 dBm was perfectly clean, at the cost of range.
3. Hardware: more distance between the ESP32 module and the magnetometer on a
   future board revision.

### Reproducing the bench test

Standalone sketch for the Alfredo NoU3 (`esp32:esp32:alfredo-nou3`): copy the
`mmc*.h/.cpp` driver files next to it, bring the MMC5983MA up as `beginMag()`
does here (BW 800, 1000 Hz, auto SET/RESET, continuous), hold the ESC pins
40 and 41 low so the AM32 bootloader exits, and in `loop()`:

- every 1 ms read `readFieldsXYZ()`, print `micros,x,y,z` when the triple
  changes (fresh sample);
- in the radio stages, start Wi-Fi STA + ESP-NOW with the broadcast peer,
  call `esp_wifi_set_max_tx_power()` and `esp_now_set_peer_rate_config()`,
  and every 20 ms print `T,micros` then `esp_now_send(broadcast, buf, 250)`.

Capture the USB serial stream to a file, then for each stage take the Z median,
count samples more than 250 counts (1.5 uT) above it, and bin those by
milliseconds since the last `T` line. 800 uT full scale over 2^18 counts is
0.0061 uT per count.

## Slip limit (2026-10-09)

### Symptom

Tread wear. A step in spin power puts the wheels at their commanded speed in
about 0.1 s, the body takes seconds to catch up, and the tread slides the
whole time.

### Measurements

From telemetry logs on the shop floor. Wheel diameter 41 mm, spin axis to tyre
contact 109.5 mm. "Tread slid" is the distance the tread slid across the floor
per wheel: wheel surface speed minus rolling speed, added up over the event.

- Rolling: 51 wheel rpm per rad/s of body spin. Geometry and logs agree.
- No-load wheel speed: about 20,000 rpm per unit of command at 17 V. The
  660 KV in the ESC config is not what the motor does.
- Steady spin: `spin_power_sent * vin / est_omega` = 0.0447 V per rad/s (0.043 to
  0.048 between 35 and 85 rad/s). This is `volts_per_rad_s`.
- Accelerating with grip: `volts = 0.0447 * omega + 0.0408 * alpha`. A command
  0.05 above the rolling command gives about 20 rad/s2 with the wheels 5 to
  10 % above rolling speed. This is `spin_push`.
- Grip: the body accelerates at 20 to 27 rad/s2 whether the wheels grip or
  slide at 1 to 10 m/s. Only a full-power burnout, 35 to 40 m/s of slip, gives
  more: 35 to 43 rad/s2.

Spin-up from rest, time and tread slid:

| | Cruise 0.18, to 50 rad/s | Full power, to 60 rad/s |
|---|---|---|
| Step | 2.1 s, 7.0 m | 1.6 s, 55 m |
| Timed ramp, 0.15 per second | 2.7 s, 5.7 m | 2.9 s, 14 m |
| Timed ramp, 0.05 per second | 3.8 s, 1.7 m | 4.1 s, 2.2 m |

The 0.05 ramp loses about a second at the start: a gripping wheel runs about
0.04 of command below its no-load speed, so a ramp from zero spends its first
0.8 s building that lead before the wheel pushes at all.

Hits (seven in the logs): the body drops to 12 to 17 rad/s, the wheels stay at
cruise speed at about three times rolling, and it takes 2.5 to 3.9 s to get
back to 80 % speed. 6 to 14 m of tread per hit at cruise, 36 m for the one hit
at full power. A timed ramp cannot see a hit or a pin, and it blunts the
full-power switch (16 s from cruise to full at 0.05 per second).

### On the robot, after balancing (2026-10-09, late)

The first feed-forward runs were on a poorly balanced robot: above 70 rad/s the
bar wheel ran 13 to 23 % over rolling speed while foo did the work, and full
power took 7.9 s to reach 100 rad/s. An off-centre mass loads the wheels
unevenly with a force that grows as speed squared. Balance the robot before
tuning anything here. Everything below is from the balanced robot.

- Steady spin: 0.0402 to 0.0409 V per rad/s (74 to 75 rad/s at spin power 0.18
  and 16.7 V). `volts_per_rad_s` is 0.0405.
- The knee, from full-power spin-ups with `volts_per_rad_s` at the old 0.0447, which
  adds about 0.015 of hidden lead at 60 rad/s:

  | `spin_push` as set | Push, 40 to 100 rad/s | 60 rad/s in | Tread slid to 60 | Wheels vs rolling |
  |---|---|---|---|---|
  | 0.05 | 22 to 25 rad/s2 | 2.83 s | 1.5 m | 1.09 |
  | 0.06 | 25 to 29 rad/s2 | 2.50 s | 1.8 m | 1.15 |
  | 0.10 | 25 to 29 rad/s2 | 2.48 s | 5.6 m | 1.25 to 1.55 |

  Past 0.06 the push does not rise and the slip triples. With `volts_per_rad_s`
  corrected that knee is a lead of about 0.07.
- With spin power as a plain cap the push faded on the way to cruise: 26 rad/s2
  at 45 rad/s, 15 at 57, 8 at 65, 2 at 75. 50 rad/s came in 2.3 s but 75 took
  5.4 s. That is what `spin_push_fade_rad_s` fixes.
- Direction change at cruise: 5.3 s of braking from 69 to 20 rad/s with the
  wheels at 0.93 of rolling speed, 0.75 s through zero, and 68 rad/s the other
  way 10.4 s after the switch. Heading error stayed between 5 and 17 degrees.

### What the firmware does

`MeltyDrive` reads spin power as a target speed and pushes toward it with the
rolling command for the measured spin rate plus a lead:

    rolling command = volts_per_rad_s * spin_rate / battery
    target speed    = spin_power * battery / volts_per_rad_s
    spin_power_sent        = rolling command + spin_push      while below the target

- The full lead is kept until the body is within `spin_push_fade_rad_s` (5 rad/s) of the
  target, then fades to nothing at the target, where `spin_power_sent` is `spin_power`
  again. While the body is below target `spin_power_sent` sits above `spin_power`, by
  up to the lead. At or above the target it is `spin_power`.
- The wheel is never asked to outrun the floor by more than the lead, about
  3 m/s at 0.07. That holds on spin-up, after a hit, in a pin, and when
  restarting while still turning. The full-power switch is limited too.
- A spin power at or below the lead is applied directly.
- `volts_per_rad_s` now sets the speed each spin power means. If it is set too high
  the push stops short of the real steady speed and the last part creeps; too
  low and the robot settles faster than spin power alone would hold, with the
  command sitting above spin power.
- A direction change brakes with the wheels just below rolling speed, carries
  the body through zero at the lead alone, and reports the new direction
  (`turning_reversed`, which the heading estimator uses) once the body is
  under 10 rad/s.
- There is no fallback if the spin rate is wrong. No accelerometer dropout or
  stuck reading shows in any of the eleven logs checked. A spin rate stuck at
  zero would hold the robot at the lead.
- The translation waveform is applied to the limited command afterwards.

Logged: `spin_power`, `spin_power_sent`, `spin_power_rolling`, `spin_target_rad_s`, `spin_reversing`, and
ten times a second the tunables `spin_push`, `volts_per_rad_s`, `spin_push_fade_rad_s`. All
three are tunable from the viewer and return to the sketch values on boot.

To check the tuning in a log:

- `volts_per_rad_s`: hold a steady cruise, then `spin_power_sent * vin / est_omega`.
- `spin_push`: compare `foo_rpm / 51` and `bar_rpm / 51` with `est_omega`
  during a full-power spin-up. The wheel line should sit 10 to 15 % above the
  body line. More than that is slip for no extra push.

### Open

- The speed-target version has been checked against a model, not yet on the
  robot. A hit and a pin have not been tried on the robot with any version.
- Direction-change braking is about half as hard as the lead should give. The
  ESCs brake weakly when the command is only just below rolling speed.
- ESC braking at zero command. On a stop from cruise the bar wheel drops to
  about a fifth of rolling speed and the body takes 4 to 5 s to stop. The foo
  wheel has reported 0 rpm during stops since the 20:53 log; in the 19:58 log
  both wheels reported and stops took 2 to 2.6 s. The AM32 brake settings need
  a second look.

## Translation waveform (2026-10-10)

The waveform pushes one wheel harder and eases the other for half of each
turn: push side spin power plus 3 parts, ease side minus 1 part, each part
`spin_power_sent * stick / 2`.

Measured in the 00:54 log, 7 s of holding the stick at cruise:

- The spin rate climbed from 77 to 96 rad/s, because the average power rises
  with the stick.
- The motor powers alternated between 0.12 and 0.35. At the higher spin rate
  that is about 0.11 above and below the rolling power. The spin-up tests put
  the limit of grip at about 0.07.
- Each wheel swung between 0.83 and 1.21 of rolling speed and slid 8.6 m of
  tread, 1.2 m per second. A whole spin-up slides about 2.3 m.

Tried and taken out the same night: an equal-and-opposite waveform, spin power
plus and minus `0.07 * stick`, which holds the spin rate steady and keeps the
wheels within their grip. It did not feel right to drive, so the original
3-to-1 waveform is back. `throttle` (the stick) is still logged so a recording
shows when the robot was translating.

Ideas for another day: the symmetric swing with a larger size, pushing only
within about 60 degrees of the drive direction (keeps about 87 % of the drive
for two thirds of the slip), and advancing the switch angle with spin rate.
