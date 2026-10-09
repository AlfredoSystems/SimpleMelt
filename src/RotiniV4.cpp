#include "RotiniV4.h"

static const uint32_t SENSOR_TIMEOUT_US = 10000;  // poll a sensor whose INT line is silent this long
static const uint32_t DSHOT_PERIOD_US = 1000;
static const float G = 9.80665f;

volatile bool RotiniV4::accel_ready = false;
volatile bool RotiniV4::mag_ready = false;
volatile uint32_t RotiniV4::accel_ints = 0;
volatile uint32_t RotiniV4::mag_ints = 0;

void IRAM_ATTR RotiniV4::onAccelReady() { accel_ready = true; accel_ints++; }
void IRAM_ATTR RotiniV4::onMagReady() { mag_ready = true; mag_ints++; }

bool RotiniV4::begin() {
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
    attachInterrupt(digitalPinToInterrupt(PIN_ACCEL_INT), onAccelReady, RISING);
    accel_ready = true;  // in case DRDY was already high
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
    attachInterrupt(digitalPinToInterrupt(PIN_MAG_INT), onMagReady, RISING);
    mag_ready = true;
  }

  // AM32 arms after ~1 s of zero throttle; send() forces zero until then.
  bool escs_ok = foo.begin(PIN_MOTOR_FOO, DSHOT600, true, motor_poles);
  escs_ok &= bar.begin(PIN_MOTOR_BAR, DSHOT600, true, motor_poles);
  foo.setPushPull(true);
  bar.setPushPull(true);

  return accel_ok && mag_ok && escs_ok;
}

void RotiniV4::update() {
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

bool RotiniV4::readAccel() {
  if (!accel_ready && micros() - last_accel_us < SENSOR_TIMEOUT_US) return false;
  accel_ready = false;
  last_accel_us = micros();

  float x, y, z;
  if (!accelerometer.read(x, y, z)) return false;  // g
  accel_x = x * G;
  accel_y = y * G;
  accel_z = z * G;
  return true;
}

bool RotiniV4::readMag() {
  if (!mag_ready && micros() - last_mag_us < SENSOR_TIMEOUT_US) return false;
  mag_ready = false;
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
void RotiniV4::sendBatteryTelemetry() {
  crsf_sensor_battery_t battery = {
    .voltage = htobe16((uint16_t)(vin * 10)),
  };
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &battery, sizeof(battery));
}

// One frame per motor per millisecond. send() also harvests the ESC's reply
// to the previous frame, which is where rpm and the EDT values come from.
void RotiniV4::setMotors(float foo_power, float bar_power) {
  if ((int32_t)(micros() - next_dshot_us) < 0) return;
  next_dshot_us = micros() + DSHOT_PERIOD_US;

  foo.send(throttle3D(foo_reversed ? -foo_power : foo_power));
  bar.send(throttle3D(bar_reversed ? -bar_power : bar_power));
  updateEdt(foo, foo_power, foo_edt_ms);
  updateEdt(bar, bar_power, bar_edt_ms);
}

// Motor power (-1..1) to a DShot value for an ESC in 3D mode:
//   0            stop
//   48..1047     reverse, slowest to fastest
//   1048..2047   forward, slowest to fastest
uint16_t RotiniV4::throttle3D(float power) {
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
void RotiniV4::updateEdt(AlfredoDShot &esc, float power, uint32_t &sent_ms) {
  if (power != 0 || !esc.telemetryValid() || esc.commandPending()) return;
  if (esc.edtSeen() || (sent_ms && millis() - sent_ms < 1000)) return;
  esc.command(DSHOT_CMD_EDT_ENABLE);
  sent_ms = millis();
}

void RotiniV4::setLeds(bool melty, bool status) {
  digitalWrite(PIN_MELTY_LED, melty);
  digitalWrite(PIN_STATUS_LED, status);
}

void RotiniV4::printStatus(Print &out) {
  float seconds = (millis() - stat_ms) * 0.001f;
  if (seconds <= 0) seconds = 1;
  out.printf("sensors: accel %.0f/s%s, mag %.0f/s%s | loop %.0f/s\n",
             (accel_ints - stat_accel) / seconds, accel_ok ? "" : " (MISSING)",
             (mag_ints - stat_mag) / seconds, mag_ok ? "" : " (MISSING)",
             (loops - stat_loops) / seconds);
  stat_ms = millis();
  stat_accel = accel_ints;
  stat_mag = mag_ints;
  stat_loops = loops;

  printEsc(out, "foo", foo);
  printEsc(out, "bar", bar);
}

// echo: 31 = wiring good, 0 = nothing on the line, 1-30 = weak pull-up
void RotiniV4::printEsc(Print &out, const char *name, AlfredoDShot &esc) {
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
