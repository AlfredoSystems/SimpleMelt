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
