#ifndef ROTINIV4_H
#define ROTINIV4_H

#include <Arduino.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <AlfredoCRSF.h>
#include <AlfredoDShot.h>
#include <AlfredoFusion_H3LIS.h>
#include <AlfredoFusion_MMC5983.h>

// Everything on the Rotini V4 board: the receiver, the two sensors, the two
// ESCs, the LEDs and the battery sense. The sketch reads sensor values from
// here and hands motor powers back; it never touches a pin.
//
//   board.begin();               in setup()
//   board.update();              every loop: receiver, sensors, battery
//   board.setMotors(foo, bar);   -1..1 each; one DShot frame per motor per ms
//   board.setLeds(melty, status);
//
// Sensors: the H3LIS331 accelerometer (400 Hz, +/-400 g) and the MMC5983MA
// magnetometer each raise an INT line when a sample is ready. The ISRs only
// set a flag; the SPI reads happen in update(), which sets accel_fresh /
// mag_fresh on the loop a new sample came in. If an INT line never fires,
// that sensor is polled at 100 Hz instead and printStatus() shows 0/s for it.
//
// ESCs: AM32 over bidirectional DShot600, push-pull (the V4 signal path has a
// level shifter and weak pull-ups). "3D mode" must be ON in the AM32
// configurator. Extended DShot Telemetry (temperature, volts, amps) is turned
// on automatically once each ESC answers.

// The sensor ISRs. They live outside the class because an ISR has to be a
// plain function in IRAM, and a function defined inside a class is inline,
// which the ESP32 linker can't place there.
namespace {
  volatile bool rotini_accel_ready = false, rotini_mag_ready = false;
  volatile uint32_t rotini_accel_ints = 0, rotini_mag_ints = 0;  // for printStatus()
  void IRAM_ATTR rotiniAccelReady() { rotini_accel_ready = true; rotini_accel_ints++; }
  void IRAM_ATTR rotiniMagReady() { rotini_mag_ready = true; rotini_mag_ints++; }
}

class RotiniV4 {
  public:
    // ---- pins ----
    static constexpr int PIN_SPI_SCK = 6, PIN_SPI_MISO = 7, PIN_SPI_MOSI = 5;
    static constexpr int PIN_ACCEL_CS = 47, PIN_ACCEL_INT = 37;
    static constexpr int PIN_MAG_CS = 48, PIN_MAG_INT = 36;
    static constexpr int PIN_CRSF_RX = 18, PIN_CRSF_TX = 17;
    static constexpr int PIN_MOTOR_FOO = 40, PIN_MOTOR_BAR = 41;
    static constexpr int PIN_MELTY_LED = 15, PIN_STATUS_LED = 4;
    static constexpr int PIN_VIN_SENSE = 10;

    // ---- settings, change before begin() ----
    uint16_t accel_rate_hz = 400;  // 50, 100, 400 or 1000. Also sets the chip's low-pass (292 Hz at 400).
    uint16_t mag_rate_hz = 1000;   // 1000 gives ~618 samples/s with auto set/reset on (see README.md)
    bool foo_reversed = true;      // flip a motor for the way it is mounted
    bool bar_reversed = false;
    uint8_t motor_poles = 14;      // magnet count, for rpm from the ESC's telemetry

    // ---- peripherals, public so the sketch can read channels and ESC telemetry ----
    AlfredoCRSF crsf;
    AlfredoH3LIS accelerometer;
    AlfredoMMC5983 magnetometer;
    AlfredoDShot foo;
    AlfredoDShot bar;

    // ---- latest readings ----
    float accel_x = 0, accel_y = 0, accel_z = 0;  // m/s^2
    bool accel_fresh = false;                     // a new sample arrived this loop
    float mag_x = 0, mag_y = 0, mag_z = 0;        // uT
    bool mag_fresh = false;
    float vin = 0;                                // battery, V, updated 10 times a second
    bool accel_ok = false;                        // sensor answered at begin()
    bool mag_ok = false;

    // Holds the ESC lines low first so a rebooting AM32 leaves its bootloader
    // (about 2.5 s), then starts everything. False if a sensor or ESC is missing;
    // the robot still runs with what it has.
    bool begin() {
      // Must be first, see AlfredoDShot.h. holdMs = 0 leaves FOO low and moves on,
      // so both ESCs share one hold.
      AlfredoDShot::releaseBootloader(PIN_MOTOR_FOO, 0);
      AlfredoDShot::releaseBootloader(PIN_MOTOR_BAR);

      pinMode(PIN_MELTY_LED, OUTPUT);
      pinMode(PIN_STATUS_LED, OUTPUT);
      setLeds(false, false);

      crsf_serial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
      crsf.begin(crsf_serial);

      SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

      accel_ok = accelerometer.begin(SPI, PIN_ACCEL_CS);
      if (accel_ok) {
        accelerometer.configure(accel_rate_hz, 400);
        accelerometer.enableInterrupt();  // data-ready on INT1; a read clears it
        pinMode(PIN_ACCEL_INT, INPUT);
        attachInterrupt(digitalPinToInterrupt(PIN_ACCEL_INT), rotiniAccelReady, RISING);
        rotini_accel_ready = true;  // in case DRDY was already high
      }

      for (int tries = 0; tries < 3 && !mag_ok; tries++) {  // the chip sometimes needs a moment after power-up
        mag_ok = magnetometer.begin(SPI, PIN_MAG_CS);
        if (!mag_ok) delay(100);
      }
      if (mag_ok) {
        // Auto set/reset cancels the chip's own offset at the cost of half the rate
        magnetometer.configure(mag_rate_hz, 800, true);
        magnetometer.enableInterrupt();  // measurement-done; readRaw() clears it
        pinMode(PIN_MAG_INT, INPUT);
        attachInterrupt(digitalPinToInterrupt(PIN_MAG_INT), rotiniMagReady, RISING);
        rotini_mag_ready = true;
      }

      // AM32 arms after ~1 s of zero throttle; send() forces zero until then.
      bool escs_ok = foo.begin(PIN_MOTOR_FOO, DSHOT600, true, motor_poles);
      escs_ok &= bar.begin(PIN_MOTOR_BAR, DSHOT600, true, motor_poles);
      foo.setPushPull(true);
      bar.setPushPull(true);

      return accel_ok && mag_ok && escs_ok;
    }

    void update() {
      loops++;
      crsf.update();
      accel_fresh = accel_ok && readAccel();
      mag_fresh = mag_ok && readMag();

      if (millis() - last_vin_ms >= 100) {
        last_vin_ms = millis();
        vin = analogReadMilliVolts(PIN_VIN_SENSE) * 0.001f * 8.21f;  // the board's divider
        sendBatteryTelemetry();
      }
    }

    // One frame per motor per millisecond. send() also harvests the ESC's reply
    // to the previous frame, which is where rpm and the EDT values come from.
    void setMotors(float foo_power, float bar_power) {
      if ((int32_t)(micros() - next_dshot_us) < 0) return;
      next_dshot_us = micros() + DSHOT_PERIOD_US;

      foo.send(throttle3D(foo_reversed ? -foo_power : foo_power));
      bar.send(throttle3D(bar_reversed ? -bar_power : bar_power));
      updateEdt(foo, foo_power, foo_edt_ms);
      updateEdt(bar, bar_power, bar_edt_ms);
    }

    void setLeds(bool melty, bool status) {
      digitalWrite(PIN_MELTY_LED, melty);
      digitalWrite(PIN_STATUS_LED, status);
    }

    // Three lines: sensor sample rates and loop rate, then each ESC's link state.
    // Expect about 415/s accel and 618/s mag; 0 means that INT line is dead.
    void printStatus(Print &out) {
      float seconds = (millis() - stat_ms) * 0.001f;
      if (seconds <= 0) seconds = 1;
      out.printf("sensors: accel %.0f/s%s, mag %.0f/s%s | loop %.0f/s\n",
                 (rotini_accel_ints - stat_accel) / seconds, accel_ok ? "" : " (MISSING)",
                 (rotini_mag_ints - stat_mag) / seconds, mag_ok ? "" : " (MISSING)",
                 (loops - stat_loops) / seconds);
      stat_ms = millis();
      stat_accel = rotini_accel_ints;
      stat_mag = rotini_mag_ints;
      stat_loops = loops;

      printEsc(out, "foo", foo);
      printEsc(out, "bar", bar);
    }

  private:
    static constexpr uint32_t SENSOR_TIMEOUT_US = 10000;  // poll a sensor whose INT line is silent this long
    static constexpr uint32_t DSHOT_PERIOD_US = 1000;
    static constexpr float G = 9.80665f;

    bool readAccel() {
      if (!rotini_accel_ready && micros() - last_accel_us < SENSOR_TIMEOUT_US) return false;
      rotini_accel_ready = false;
      last_accel_us = micros();

      float x, y, z;
      if (!accelerometer.read(x, y, z)) return false;  // g
      accel_x = x * G;
      accel_y = y * G;
      accel_z = z * G;
      return true;
    }

    bool readMag() {
      if (!rotini_mag_ready && micros() - last_mag_us < SENSOR_TIMEOUT_US) return false;
      rotini_mag_ready = false;
      last_mag_us = micros();

      uint32_t raw[3];
      if (!magnetometer.readRaw(raw[0], raw[1], raw[2])) return false;
      if (raw[0] == prev_mag_raw[0] && raw[1] == prev_mag_raw[1] && raw[2] == prev_mag_raw[2]) return false;  // nothing new
      for (int i = 0; i < 3; i++) prev_mag_raw[i] = raw[i];

      const float k = 800.0f / 131072.0f;  // uT per count, 131072 is zero field
      mag_x = ((float)raw[0] - 131072.0f) * k;
      mag_y = ((float)raw[1] - 131072.0f) * k;
      mag_z = ((float)raw[2] - 131072.0f) * k;
      return true;
    }

    // Shows the battery on the transmitter
    void sendBatteryTelemetry() {
      crsf_sensor_battery_t battery = {
        .voltage = htobe16((uint16_t)(vin * 10)),
      };
      crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &battery, sizeof(battery));
    }

    // Motor power (-1..1) to a DShot value for an ESC in 3D mode:
    //   0            stop
    //   48..1047     reverse, slowest to fastest
    //   1048..2047   forward, slowest to fastest
    static uint16_t throttle3D(float power) {
      if (power > 1) power = 1;
      if (power < -1) power = -1;
      if (!(fabsf(power) > 0)) return 0;  // also catches NaN
      uint16_t steps = (uint16_t)(fabsf(power) * 999.0f + 0.5f);  // 0..999
      return (power > 0 ? 1048 : 48) + steps;
    }

    // Sends DSHOT_CMD_EDT_ENABLE once the ESC is answering and stopped. A command
    // sent while AM32 is still booting is lost, so it is re-sent every second until
    // EDT frames arrive. Commands replace the throttle for a few frames, which is
    // why this only runs with the motor stopped.
    void updateEdt(AlfredoDShot &esc, float power, uint32_t &sent_ms) {
      if (power != 0 || !esc.telemetryValid() || esc.commandPending()) return;
      if (esc.edtSeen() || (sent_ms && millis() - sent_ms < 1000)) return;
      esc.command(DSHOT_CMD_EDT_ENABLE);
      sent_ms = millis();
    }

    // echo: 31 = wiring good, 0 = nothing on the line, 1-30 = weak pull-up
    void printEsc(Print &out, const char *name, AlfredoDShot &esc) {
      const char *status = "IDLE";
      switch (esc.status()) {
        case DSHOT_RX_OK:       status = "OK"; break;
        case DSHOT_RX_NO_REPLY: status = "NO-REPLY"; break;
        case DSHOT_RX_FRAMING:  status = "FRAMING"; break;
        case DSHOT_RX_BAD_GCR:  status = "BAD-GCR"; break;
        case DSHOT_RX_BAD_CRC:  status = "BAD-CRC"; break;
        default: break;
      }
      out.printf("%s: %s echo %u %-8s rpm %6.0f loss %5.1f%%\n",
                 name, esc.isArmed() ? "armed " : "arming", esc.echoPulses(),
                 status, esc.rpm(), esc.lossPercent());
    }

    HardwareSerial crsf_serial{1};
    uint32_t last_accel_us = 0, last_mag_us = 0;
    uint32_t prev_mag_raw[3] = {0, 0, 0};
    uint32_t next_dshot_us = 0;
    uint32_t foo_edt_ms = 0, bar_edt_ms = 0;
    uint32_t last_vin_ms = 0;
    uint32_t loops = 0;
    uint32_t stat_ms = 0, stat_accel = 0, stat_mag = 0, stat_loops = 0;
};

#endif
