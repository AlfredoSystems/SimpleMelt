#ifndef ROTINIV3_H
#define ROTINIV3_H

#include <Arduino.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <AlfredoCRSF.h>
#include <AlfredoFusion_H3LIS.h>
#include <AlfredoFusion_MMC5983.h>
#include "OneShot125.h"

// Everything on the Rotini V3 board: the receiver, the two sensors, the two
// ESCs, the LEDs and the battery sense. The sketch reads sensor values from
// here and hands motor powers back; it never touches a pin.
//
//   board.begin();               in setup()
//   board.update();              every loop: receiver, sensors, battery
//   board.setMotors(foo, bar);   -1..1 each
//   board.setLeds(melty, status);
//
// Sensors: the H3LIS331 accelerometer (400 Hz, +/-400 g) and the MMC5983MA
// magnetometer each raise an INT line when a sample is ready. The ISRs only
// set a flag; the SPI reads happen in update(), which sets accel_fresh /
// mag_fresh on the loop a new sample came in. If an INT line never fires,
// that sensor is polled at 100 Hz instead and printStatus() shows 0/s for it.
// A V3 without a magnetometer runs on the accelerometer alone.
//
// ESCs: OneShot125 (see OneShot125.h), no telemetry back.

// The sensor ISRs. They live outside the class because an ISR has to be a
// plain function in IRAM, and a function defined inside a class is inline,
// which the ESP32 linker can't place there.
namespace {
  volatile bool rotini_accel_ready = false, rotini_mag_ready = false;
  volatile uint32_t rotini_accel_ints = 0, rotini_mag_ints = 0;  // for printStatus()
  void IRAM_ATTR rotiniAccelReady() { rotini_accel_ready = true; rotini_accel_ints++; }
  void IRAM_ATTR rotiniMagReady() { rotini_mag_ready = true; rotini_mag_ints++; }
}

class RotiniV3 {
  public:
    // ---- pins ----
    static constexpr int PIN_SPI_SCK = 17, PIN_SPI_MISO = 1, PIN_SPI_MOSI = 14;
    static constexpr int PIN_ACCEL_CS = 11, PIN_ACCEL_INT = 12;
    static constexpr int PIN_MAG_CS = 2, PIN_MAG_INT = 13;
    static constexpr int PIN_CRSF_RX = 7, PIN_CRSF_TX = 8;
    static constexpr int PIN_MOTOR_FOO = 34, PIN_MOTOR_BAR = 35;
    static constexpr int PIN_MELTY_LED = 42, PIN_STATUS_LED = 41;
    static constexpr int PIN_VIN_SENSE = 3;

    // ---- settings, change before begin() ----
    uint16_t accel_rate_hz = 400;  // 50, 100, 400 or 1000. Also sets the chip's low-pass (292 Hz at 400).
    uint16_t mag_rate_hz = 1000;   // 1000 gives ~618 samples/s with auto set/reset on
    bool foo_reversed = true;      // flip a motor for the way it is mounted
    bool bar_reversed = false;

    // ---- peripherals, public so the sketch can read channels ----
    AlfredoCRSF crsf;
    AlfredoH3LIS accelerometer;
    AlfredoMMC5983 magnetometer;
    OneShot125 foo;
    OneShot125 bar;

    // ---- latest readings ----
    float accel_x = 0, accel_y = 0, accel_z = 0;  // m/s^2
    bool accel_fresh = false;                     // a new sample arrived this loop
    float mag_x = 0, mag_y = 0, mag_z = 0;        // uT
    bool mag_fresh = false;
    float vin = 0;                                // battery, V, updated 10 times a second
    bool accel_ok = false;                        // sensor answered at begin()
    bool mag_ok = false;

    // False if a sensor or ESC is missing; the robot still runs with what it has.
    bool begin() {
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

      bool escs_ok = foo.begin(PIN_MOTOR_FOO);
      escs_ok &= bar.begin(PIN_MOTOR_BAR);
      foo.setReversed(foo_reversed);
      bar.setReversed(bar_reversed);

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

    void setMotors(float foo_power, float bar_power) {
      foo.set(foo_power);
      bar.set(bar_power);
    }

    void setLeds(bool melty, bool status) {
      digitalWrite(PIN_MELTY_LED, melty);
      digitalWrite(PIN_STATUS_LED, status);
    }

    // One line: sensor sample rates and loop rate. Expect about 415/s accel
    // and 618/s mag; 0 means that INT line is dead.
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
    }

  private:
    static constexpr uint32_t SENSOR_TIMEOUT_US = 10000;  // poll a sensor whose INT line is silent this long
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

    HardwareSerial crsf_serial{1};
    uint32_t last_accel_us = 0, last_mag_us = 0;
    uint32_t prev_mag_raw[3] = {0, 0, 0};
    uint32_t last_vin_ms = 0;
    uint32_t loops = 0;
    uint32_t stat_ms = 0, stat_accel = 0, stat_mag = 0, stat_loops = 0;
};

#endif
