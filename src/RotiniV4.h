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
    uint16_t mag_rate_hz = 1000;   // 1000 gives ~618 samples/s with auto set/reset on (see the example README)
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
    bool begin();
    void update();
    void setMotors(float foo_power, float bar_power);
    void setLeds(bool melty, bool status);

    // Two lines: sensor sample rates and loop rate, then each ESC's link state.
    // Expect about 415/s accel and 618/s mag; 0 means that INT line is dead.
    void printStatus(Print &out);

  private:
    bool readAccel();
    bool readMag();
    void sendBatteryTelemetry();
    void updateEdt(AlfredoDShot &esc, float power, uint32_t &sent_ms);
    void printEsc(Print &out, const char *name, AlfredoDShot &esc);
    static uint16_t throttle3D(float power);
    static void IRAM_ATTR onAccelReady();
    static void IRAM_ATTR onMagReady();

    static volatile bool accel_ready, mag_ready;
    static volatile uint32_t accel_ints, mag_ints;  // ISR counts, for printStatus()

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
