#include <Arduino.h>
#include <SPI.h>
#include <HardwareSerial.h>

#include "SimpleMelt.h"
#include "SimpleMeltUtility.h"
#include "HeadingEstimator.h"

#include "AlfredoCRSF.h"
#include <AlfredoDShot.h>
#include <AlfredoTelemetry.h>
#include <AlfredoFusion.h>  // H3LIS331 accelerometer + MMC5983MA magnetometer drivers

const int PIN_SPI_SCK = 6;
const int PIN_SPI_MISO = 7;
const int PIN_SPI_MOSI = 5;
const int PIN_ACCELEROMETER_CS = 47;
const int PIN_ACCELEROMETER_INT = 37;
const int PIN_MAGNETOMETER_CS = 48;
const int PIN_MAGNETOMETER_INT = 36;
const int PIN_CRSF_RX = 18;
const int PIN_CRSF_TX = 17;
const int PIN_MOTOR_FOO = 40;
const int PIN_MOTOR_BAR = 41;
const int PIN_MELTY_LED = 15;
const int PIN_STATUS_LED = 4;
const int PIN_SNS_VIN = 10;

SimpleMelt Rotini;

HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

AlfredoH3LIS accelerometer;

AlfredoMMC5983 magnetometer;
const uint16_t MAG_RATE_HZ = 1000;  // continuous mode rate: 1000, 200, 100, 50, 20, 10 or 1
bool mag_ok = false;

// Both sensors raise an INT line when a new sample is ready (H3LIS331 DRDY on
// INT1, MMC5983MA measurement-done). The ISRs only set a flag; the SPI reads
// happen in loop(). If an INT line never fires (wrong pin, or GPIO 36/37 taken
// by octal PSRAM), the read falls back to a slow poll after SENSOR_TIMEOUT_US
// and the once-a-second status line shows 0 interrupts/s for that sensor.
volatile bool accel_ready = false;
volatile bool mag_ready = false;
volatile uint32_t accel_int_count = 0;  // ISR fires, for the status line
volatile uint32_t mag_int_count = 0;
uint32_t mag_fresh_count = 0, mag_stale_count = 0, loop_count = 0;  // status line rates
const uint32_t SENSOR_TIMEOUT_US = 10000;

void IRAM_ATTR accelISR() { accel_ready = true; accel_int_count++; }
void IRAM_ATTR magISR() { mag_ready = true; mag_int_count++; }

// Heading from accelerometer + magnetometer (see HeadingEstimator.h). Runs in every
// drive mode so the compass holds the heading while parked. Set USE_HEADING_ESTIMATOR
// false to fall back to the accelerometer-only dead reckoning.
HeadingEstimator Heading;
const bool USE_HEADING_ESTIMATOR = true;
// +1 if the compass heading increases when `reversed` is false, -1 if it decreases.
// Both 2026-10-07 logs spun with the compass heading increasing; if the LED arc
// runs backwards after a direction change, flip this.
const int MAG_SPIN_SIGN = 1;

// ESCs run AM32 over bidirectional DShot. "3D mode" must be ON in the AM32
// configurator: the DShot range is split in two halves, one per direction.
// Rotini V4's signal path (BSS138 level shifter, 10k pull-up on the ESP side,
// 5.1k on the ESC side, ~470 ohm inside the ESC) needs push-pull drive for
// DSHOT600; with PUSH_PULL off, use DSHOT300. See AlfredoDShot's
// Rotini_V4_Telemetry example.
const DShotMode DSHOT_RATE = DSHOT600;
const bool PUSH_PULL = true;
const uint8_t MOTOR_POLES = 14;        // magnet count, for rpm telemetry
const uint32_t DSHOT_PERIOD_US = 1000; // one frame per motor at 1 kHz
const bool FOO_REVERSED = true;        // was foo.setReversed(true) with OneShot125
const bool BAR_REVERSED = false;

AlfredoDShot foo;
AlfredoDShot bar;

void setup() {
  // Must be first, see AlfredoDShot.h: holds the ESC signal lines low so a
  // rebooting AM32 leaves its bootloader. holdMs = 0 leaves FOO low and moves
  // on, so both ESCs share one 2.5 s hold.
  AlfredoDShot::releaseBootloader(PIN_MOTOR_FOO, 0);
  AlfredoDShot::releaseBootloader(PIN_MOTOR_BAR);

  Rotini.melty_led_offset_CW = 2.22;     // radians (CCW is positive)
  Rotini.melty_led_offset_CCW = 4.02;    // radians (CCW is positive)
  Rotini.turn_speed = 1.1;              // rotations per second
  Rotini.accelerometer_radius = 0.118;  // meters, accelerometer-only fallback only (was 0.090 + 0.032 trim)

  // Start values, measured from the 2026-10-07/08 logs. The estimator learns the
  // radius and the circle center while spinning and the accel offset while parked,
  // so these only shape the first second; the mag center is a body property
  // (the robot's own magnets) and B_NOM is the local horizontal Earth field.
  Heading.R0 = 0.117;     // m, what the logs learned (the old 0.090 + 0.032 trim gave 0.122)
  Heading.CU0 = 6.4;      // uT, mag circle center at cruise power
  Heading.CV0 = 2.4;      // uT
  Heading.B_NOM = 19.7;   // uT, in-plane field size while spinning (gate 1 compares against it)
  Heading.begin();

  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_MELTY_LED, OUTPUT);
  pinMode(PIN_ACCELEROMETER_CS, OUTPUT);
  pinMode(PIN_MAGNETOMETER_CS, OUTPUT);

  digitalWrite(PIN_STATUS_LED, LOW);
  digitalWrite(PIN_MELTY_LED, LOW);
  digitalWrite(PIN_ACCELEROMETER_CS, HIGH);
  digitalWrite(PIN_MAGNETOMETER_CS, HIGH);

  Serial.begin(115200);

  // Started before the sensors so a sensor problem can't keep the robot off the viewer.
  // 200 frames/s instead of one per loop (~875/s) and a 6 Mbps radio rate: about a
  // tenth of the airtime, so the link keeps a multi-second buffer and the adaptive
  // fallback rates (down to LR) stay available in a noisy arena. The heading filter
  // itself still runs every loop; only the log is decimated.
  Telemetry.setMaxRate(200);
  Telemetry.setRadioRate(WIFI_PHY_RATE_6M);
  Telemetry.begin("Rotini");

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
  crsf.begin(crsfSerial);

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  if (!accelerometer.begin(SPI, PIN_ACCELEROMETER_CS)) {
    Serial.println("H3LIS331 did not respond");
  }
  accelerometer.configure(400, 400);  // 400 Hz (chip low-pass 292 Hz), +/-400 g
  accelerometer.enableInterrupt();    // data-ready on INT1, active high, push-pull
  pinMode(PIN_ACCELEROMETER_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ACCELEROMETER_INT), accelISR, RISING);
  accel_ready = true;  // DRDY may already be high; the first read clears it

  mag_ok = beginMag();

  // AM32 arms after ~1 s of zero throttle; send() forces zero until then.
  if (!foo.begin(PIN_MOTOR_FOO, DSHOT_RATE, true, MOTOR_POLES) ||
      !bar.begin(PIN_MOTOR_BAR, DSHOT_RATE, true, MOTOR_POLES)) {
    Serial.println("esc.begin() failed - out of RMT channels?");
  }
  foo.setPushPull(PUSH_PULL);
  bar.setPushPull(PUSH_PULL);
}

void loop() {

  crsf.update();
  Rotini.rotation = channel_to_axis(1);  //axis_right_x
  Rotini.throttle = channel_to_axis(3);  //axis_left_y
  //axis_right_y = channel_to_axis(2);
  //axis_left_x = channel_to_axis(4);

  //bumpers
  static Button left_bumper;
  left_bumper.update(crsf.getChannel(11), 1000);
  static Button right_bumper;
  right_bumper.update(crsf.getChannel(11), 2000);
  //trim buttons
  static Button left_left_arrow;
  left_left_arrow.update(crsf.getChannel(10), 1000);
  static Button left_right_arrow;
  left_right_arrow.update(crsf.getChannel(10), 2000);
  static Button left_up_arrow;
  left_up_arrow.update(crsf.getChannel(9), 2000);
  static Button left_down_arrow;
  left_down_arrow.update(crsf.getChannel(9), 1000);
  static Button right_left_arrow;
  right_left_arrow.update(crsf.getChannel(7), 1000);
  static Button right_right_arrow;
  right_right_arrow.update(crsf.getChannel(7), 2000);
  static Button right_up_arrow;
  right_up_arrow.update(crsf.getChannel(8), 2000);
  static Button right_down_arrow;
  right_down_arrow.update(crsf.getChannel(8), 1000);
  // SWA - Zorro Switch B
  static Button SWA_backward;
  SWA_backward.update(crsf.getChannel(6), 1000);
  static Button SWA_neutral;
  SWA_neutral.update(crsf.getChannel(6), 1500);
  static Button SWA_forward;
  SWA_forward.update(crsf.getChannel(6), 2000);
  // SWD - Zorro Switch F
  static Button SWD_backward;
  SWD_backward.update(crsf.getChannel(5), 1000);
  static Button SWD_forward;
  SWD_forward.update(crsf.getChannel(5), 2000);
  // SWB - Zorro Switch C
  static Button SWB_backward;
  SWB_backward.update(crsf.getChannel(12), 1000);
  static Button SWB_neutral;
  SWB_neutral.update(crsf.getChannel(12), 1500);
  static Button SWB_forward;
  SWB_forward.update(crsf.getChannel(12), 2000);

  if (crsf.isLinkUp()) {
    if (SWA_backward.is_held()) Rotini.drive_mode = STOP;
    else if (SWA_neutral.is_held()) Rotini.drive_mode = ARCADE;
    else if (SWA_forward.is_held()) Rotini.drive_mode = MELTY;
  } else {
    Rotini.drive_mode = NO_CONNECTION;
  }

  // Reads the accelerometer once per new sample (DRDY interrupt), in every
  // drive mode. The read clears DRDY. Between samples the last values hold.
  static float accel_x = 0, accel_y = 0, accel_z = 0;  // m/s^2
  static uint32_t last_accel_us = 0;
  if (accel_ready || micros() - last_accel_us > SENSOR_TIMEOUT_US) {
    accel_ready = false;
    last_accel_us = micros();
    float gx, gy, gz;
    if (accelerometer.read(gx, gy, gz)) {  // g
      accel_x = gx * 9.80665f;
      accel_y = gy * 9.80665f;
      accel_z = gz * 9.80665f;
    }
    Telemetry.add("accel_x", accel_x);  // m/s^2
    Telemetry.add("accel_y", accel_y);
    Telemetry.add("accel_z", accel_z);
  }

  float mag_x = 0, mag_y = 0, mag_z = 0;
  bool mag_fresh = mag_ok && updateMag(mag_x, mag_y, mag_z);
  loop_count++;
  if (mag_fresh) {
    mag_fresh_count++;
    Telemetry.add("mag_x", mag_x);  // uT
    Telemetry.add("mag_y", mag_y);
    Telemetry.add("mag_z", mag_z);
  }

  // Heading estimate, every loop in every mode (the compass holds it while parked)
  static uint32_t last_heading_us = micros();
  uint32_t now_us = micros();
  float heading_dt = (now_us - last_heading_us) * 0.000001f;
  last_heading_us = now_us;
  if (heading_dt > 0.5f) heading_dt = 0;  // first loop or a stall: skip the step
  int spin_dir = (Rotini.reversed ? -1 : 1) * MAG_SPIN_SIGN;
  bool motors_off = Rotini.motor_power_foo == 0 && Rotini.motor_power_bar == 0;  // the offset is learned only then
  Heading.update(heading_dt, accel_z, mag_fresh, mag_x, mag_y, mag_z, spin_dir, motors_off);

  if (Rotini.drive_mode == MELTY) {
	// Accelerometer info for the accelerometer-only fallback (meltyStateUpdate)
    Rotini.accelerometer_x = 0; // accel_x;
    Rotini.accelerometer_y = 0; // accel_y;
    Rotini.accelerometer_z = accel_z;

	// Spin_power is the average power that motors are set to.
    if (right_bumper.just_pressed())
      Rotini.spin_power += 0.02; // increase power by 2%
    if (left_bumper.just_pressed())
      Rotini.spin_power -= 0.02; // decrease power by 2%
    if (right_up_arrow.is_held() || right_down_arrow.is_held())
      Rotini.spin_power = 0;     // 0% power, the robot does not spin
    else if (right_left_arrow.is_held() || right_right_arrow.just_released())
      Rotini.spin_power = 0.18;  // 18% power, "cruising power" is what I am using most of the time during a match
    else if (right_right_arrow.is_held())
      Rotini.spin_power = 1;     // 100% power, I press this to accelerate from stand still, to get out of pins, or deal extra damage

	// Controls what direction the motors are spinning in
    if (SWD_backward.is_held())
      Rotini.reversed = false;
    else if (SWD_forward.is_held())
      Rotini.reversed = true;

	// SWB has three states, the state of SWB controls what certain trim buttons do.
	// The back position used to trim the accel radius; the HeadingEstimator learns
	// the radius itself, so that position is free now.
    if (SWB_neutral.is_held()) {  // middle pos trims led offset
      if (left_left_arrow.just_pressed())
        Rotini.melty_led_offset_CW += 0.1;  // 0.1 radians = 5.729 degrees
      else if (left_right_arrow.just_pressed())
        Rotini.melty_led_offset_CW -= 0.1;  // 0.1 radians = 5.729 degrees
      else if (left_up_arrow.is_held())
        Rotini.melty_led_offset_CW = 0.698132 + 0.09 * 9.0;
    } else if (SWB_forward.is_held()) {  // up pos trims lag angle
      if (left_left_arrow.just_pressed())
        Rotini.melty_led_offset_CCW += 0.1;  // 0.1 radians = 5.729 degrees
      else if (left_right_arrow.just_pressed())
        Rotini.melty_led_offset_CCW -= 0.1;  // 0.1 radians = 5.729 degrees
      else if (left_up_arrow.is_held())
        Rotini.melty_led_offset_CCW = 0.698132 - 0.09 * 6.0;
    }

    if (USE_HEADING_ESTIMATOR) Rotini.meltyHeadingStateUpdate(Heading.theta);
    else Rotini.meltyStateUpdate();

  } else if (Rotini.drive_mode == ARCADE) {
    Rotini.arcadeStateUpdate();

  } else if (Rotini.drive_mode == STOP) {
    Rotini.stopStateUpdate();

  } else {  //Rotini.drive_mode == DISCONNECTED
    Rotini.disconnectedStateUpdate();
  }

  // Actually commanding LEDs and motors. DShot frames go out at a steady
  // rate: each send() also harvests the ESC's reply to the previous frame.
  static uint32_t next_dshot_us = micros();
  if ((int32_t)(micros() - next_dshot_us) >= 0) {
    next_dshot_us = micros() + DSHOT_PERIOD_US;
    foo.send(throttle3D(Rotini.motor_power_foo * (FOO_REVERSED ? -1 : 1)));
    bar.send(throttle3D(Rotini.motor_power_bar * (BAR_REVERSED ? -1 : 1)));
  }

  digitalWrite(PIN_MELTY_LED, Rotini.melty_led);
  digitalWrite(PIN_STATUS_LED, Rotini.status_led);

  // Turns on Extended DShot Telemetry once each ESC is answering
  updateEdt(foo, Rotini.motor_power_foo);
  updateEdt(bar, Rotini.motor_power_bar);

  //send telemetry 10 times a second
  static uint32_t last_telem_ms = 0;
  if (millis() - last_telem_ms > 100) {
    float vin = read_voltage(PIN_SNS_VIN);
    //Serial.println(vin);
    Telemetry.add("vin", vin);  // volts
    send_telemetry(vin);

    // EDT values are NAN until the ESC sends them; AM32 updates them at a few Hz
    Telemetry.add("foo_temp_c", foo.temperatureC());
    Telemetry.add("foo_volts", foo.voltage());
    Telemetry.add("foo_amps", foo.current());
    Telemetry.add("bar_temp_c", bar.temperatureC());
    Telemetry.add("bar_volts", bar.voltage());
    Telemetry.add("bar_amps", bar.current());
    last_telem_ms = millis();
  }

  Telemetry.add("drive_mode", Rotini.drive_mode);
  Telemetry.add("heading", Rotini.angle * RAD_TO_DEG);  // 0..360, only updates in melty mode
  // Heading estimator internals: enough to replay the filter offline
  Telemetry.add("est_heading", Heading.theta * RAD_TO_DEG);     // 0..360, every mode
  Telemetry.add("est_mag_heading", Heading.thetaMag * RAD_TO_DEG);
  Telemetry.add("est_innov", Heading.e * RAD_TO_DEG);           // compass minus estimate at the last fix
  Telemetry.add("est_omega", Heading.w);                        // rad/s
  Telemetry.add("est_r_mm", Heading.r * 1000);                  // learned radius
  Telemetry.add("est_cu", Heading.cu);                          // learned circle center, uT
  Telemetry.add("est_cv", Heading.cv);
  Telemetry.add("est_a_off", Heading.aOff);                     // learned accel Z offset, m/s2
  Telemetry.add("est_mag_ok", Heading.accepted ? 1 : 0);        // last fix passed gate 1
  Telemetry.add("motor_foo", Rotini.motor_power_foo);  // -1..1, 0 is stop
  Telemetry.add("motor_bar", Rotini.motor_power_bar);
  // Shaft rpm from the ESC's DShot reply; a gap while it isn't answering
  Telemetry.add("foo_rpm", foo.telemetryValid() ? foo.rpm() : NAN);
  Telemetry.add("bar_rpm", bar.telemetryValid() ? bar.rpm() : NAN);
  Telemetry.send();

  // Link diagnostics over USB, once a second
  static uint32_t last_status_ms = 0;
  if (millis() - last_status_ms >= 1000) {
    Telemetry.printStatus(Serial);
    printMotor("foo", foo);
    printMotor("bar", bar);
    if (!mag_ok) Serial.println("magnetometer: not found");
    // Expect about 415 and 618 (see beginMag). 0 means that INT line isn't
    // reaching the ESP and the sensor is being polled at 100 Hz instead.
    static uint32_t accel_int_last = 0, mag_int_last = 0, mag_fresh_last = 0, mag_stale_last = 0, loop_last = 0;
    Serial.printf("sensor interrupts: accel %lu/s, mag %lu/s (fresh %lu, stale %lu) | loop %lu/s\n",
                  (unsigned long)(accel_int_count - accel_int_last),
                  (unsigned long)(mag_int_count - mag_int_last),
                  (unsigned long)(mag_fresh_count - mag_fresh_last),
                  (unsigned long)(mag_stale_count - mag_stale_last),
                  (unsigned long)(loop_count - loop_last));
    accel_int_last = accel_int_count;
    mag_int_last = mag_int_count;
    mag_fresh_last = mag_fresh_count;
    mag_stale_last = mag_stale_count;
    loop_last = loop_count;
    last_status_ms = millis();
  }
}

/////////////////////ESC Code//////////////////////////////////////////////////////////

// Motor power (-1..1) to a DShot value for an ESC in 3D mode:
//   0            stop
//   48..1047     reverse, slowest to fastest
//   1048..2047   forward, slowest to fastest
uint16_t throttle3D(float power) {
  power = fconstrain(power, -1, 1);
  if (!(fabsf(power) > 0)) return 0;  // also catches NaN
  uint16_t steps = (uint16_t)(fabsf(power) * 999.0f + 0.5f);  // 0..999
  return (power > 0 ? 1048 : 48) + steps;
}

// Sends DSHOT_CMD_EDT_ENABLE once the ESC is answering and stopped: a command
// sent while AM32 is still booting is silently lost, so it is re-sent every
// second until EDT frames arrive. Commands replace the throttle for a few
// frames, which is why this only runs with the motor stopped.
void updateEdt(AlfredoDShot &esc, float power) {
  static uint32_t sent_ms[2] = {0, 0};
  uint32_t &last_ms = sent_ms[&esc == &foo ? 0 : 1];
  if (power != 0 || !esc.telemetryValid() || esc.commandPending()) return;
  if (esc.edtSeen() || (last_ms && millis() - last_ms < 1000)) return;
  esc.command(DSHOT_CMD_EDT_ENABLE);
  last_ms = millis();
}

const char *dshotStatusName(DShotRxStatus s) {
  switch (s) {
    case DSHOT_RX_OK: return "OK";
    case DSHOT_RX_NO_REPLY: return "NO-REPLY";
    case DSHOT_RX_FRAMING: return "FRAMING";
    case DSHOT_RX_BAD_GCR: return "BAD-GCR";
    case DSHOT_RX_BAD_CRC: return "BAD-CRC";
    default: return "IDLE";
  }
}

// echo: 31 = wiring good, 0 = nothing on the line, 1-30 = weak pull-up
void printMotor(const char *name, AlfredoDShot &esc) {
  Serial.printf("%s: %s echo %u %-8s rpm %6.0f loss %5.1f%%\n",
                name, esc.isArmed() ? "armed " : "arming", esc.echoPulses(),
                dshotStatusName(esc.status()), esc.rpm(), esc.lossPercent());
}

/////////////////////Magnetometer Code/////////////////////////////////////////////////

// Gives up after a few tries so a missing magnetometer doesn't stop the robot from starting
bool beginMag() {
  int tries = 0;
  while (magnetometer.begin(SPI, PIN_MAGNETOMETER_CS) == false) {
    if (++tries >= 5) {
      Serial.println("MMC5983MA did not respond, running without it");
      return false;
    }
    Serial.println("MMC5983MA did not respond. Retrying...");
    delay(1000);
  }
  Serial.println("MMC5983MA connected");

  // Continuous mode, 800 Hz bandwidth. Auto SET/RESET halves the output
  // rate: the chip takes a SET and a RESET measurement per sample to cancel
  // its own offset. Measured 2026-10-08 on Rotini V4 at the 1000 Hz setting:
  // 618 samples/s with it, 1234/s without.
  magnetometer.configure(MAG_RATE_HZ, 800, true);

  // Measurement-done interrupt; read() clears it before each read
  magnetometer.enableInterrupt();
  pinMode(PIN_MAGNETOMETER_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_MAGNETOMETER_INT), magISR, RISING);
  mag_ready = true;  // in case the first edge came before attachInterrupt
  return true;
}

// Reads the magnetometer once per measurement-done interrupt. Returns true
// only for a NEW sample, with the field in uT (full scale +/-8 G = 800 uT).
// The stale check stays as a guard for the timeout fallback.
bool updateMag(float &x, float &y, float &z) {
  static uint32_t last_read_us = 0;
  static uint32_t prev_x = 0, prev_y = 0, prev_z = 0;
  if (!mag_ready && micros() - last_read_us < SENSOR_TIMEOUT_US) return false;
  mag_ready = false;
  last_read_us = micros();

  uint32_t raw_x, raw_y, raw_z;
  // readRaw() clears the measurement-done flag first, re-arming the INT line
  if (!magnetometer.readRaw(raw_x, raw_y, raw_z)) return false;
  if (raw_x == prev_x && raw_y == prev_y && raw_z == prev_z) { mag_stale_count++; return false; }  // stale sample
  prev_x = raw_x; prev_y = raw_y; prev_z = raw_z;
  x = ((float)raw_x - 131072.0f) / 131072.0f * 800.0f;
  y = ((float)raw_y - 131072.0f) / 131072.0f * 800.0f;
  z = ((float)raw_z - 131072.0f) / 131072.0f * 800.0f;
  return true;
}

//////////////////// Helper functions ////////////////////////////////////////////////////////////////////

float channel_to_axis(unsigned int channel) {
  float axis = min(1.f, max(-1.f, (crsf.getChannel(channel) / 500.f) - 3));  // Map 1000-2000 channel value to -1..1
  if (fabs(axis) < 0.03) axis = 0;                                           // Apply deadzone
  return axis;
}

float read_voltage(int pin_sense_voltage) {
  float adc_vin = analogReadMilliVolts(pin_sense_voltage) * 0.001 * 8.21;
  return adc_vin;
}

void send_telemetry(float telem_voltage) {
  crsf_sensor_battery_t battery_data = {
    .voltage = htobe16((uint16_t)(telem_voltage * 10)),
  };
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &battery_data, sizeof(crsf_sensor_battery_t));

  // crsf_sensor_gps_t telemetry_data = {
  //   // Cram telemetry data into GPS struct
  //   .latitude = htobe32((int32_t)(RotiniPtr->centripetal_acceleration * 1000)),  // 10,000,000 big endian
  //   .longitude = htobe32((int32_t)(dRotiniPtr->angular_velocity * 1000)),        // 10,000,000 big endian
  //   .groundspeed = htobe16((uint16_t)(RotiniPtr->percent_power * 100)),          // 10 big endian
  //   .heading = htobe16((uint16_t)(RotiniPtr->accelerometer_radius_trim * 1000))  // 10 big endian
  // };
}