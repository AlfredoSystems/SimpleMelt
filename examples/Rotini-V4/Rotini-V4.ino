/*
  Rotini V4 - meltybrain firmware for the Rotini V4 board.

  The library does the work: RotiniV4 owns the hardware, HeadingEstimator
  turns the sensors into a heading, MeltyDrive turns the sticks and that
  heading into motor powers. This sketch is the part that is yours to change:
  the controller mapping, the calibration numbers, and what gets logged.

  Controller: ExpressLRS over CRSF, mapped for a RadioMaster Zorro.
    left stick Y      throttle          right stick X   turn
    switch B (SWA)    stop / arcade / melty
    switch F (SWD)    spin direction
    bumpers           spin power -2% / +2%
    right arrows      spin power: up or down 0%, left 18% (cruise), right 100%
    switch C (SWB)    what the left arrows trim: mid = clockwise LED offset,
                      up = counter-clockwise LED offset, back = nothing

  Telemetry: AlfredoTelemetry over ESP-NOW. Flash its Dongle example to a
  second ESP32, open extras/TelemetryViewer/index.html, and pair with "Rotini".
  The channel list is in logTelemetry(). See README.md for bench notes.
*/

#include <SimpleMelt.h>
#include <AlfredoTelemetry.h>

RotiniV4 board;
MeltyDrive drive;
HeadingEstimator heading;

// +1 if the compass heading increases while spinning with `reversed` false,
// -1 if it decreases. If the LED arc runs backwards after a direction change, flip this.
const int MAG_SPIN_SIGN = 1;

// ---- controller ----
Button left_bumper, right_bumper;                      // channel 11
Button left_left, left_right, left_up, left_down;      // left arrows, channels 10 and 9
Button right_left, right_right, right_up, right_down;  // right arrows, channels 7 and 8
Button swa_back, swa_mid, swa_fwd;                     // switch B, channel 6
Button swd_back, swd_fwd;                              // switch F, channel 5
Button swb_back, swb_mid, swb_fwd;                     // switch C, channel 12

void setup() {
  Serial.begin(115200);

  // Telemetry first, so a hardware problem can't keep the robot off the viewer.
  // 200 frames/s and a 6 Mbps radio rate: about a tenth of the airtime, so the
  // link keeps a multi-second buffer and the slower fallback rates stay available
  // in a noisy arena.
  Telemetry.setMaxRate(200);
  Telemetry.setRadioRate(WIFI_PHY_RATE_6M);
  Telemetry.begin("Rotini");

  if (!board.begin()) Serial.println("board.begin(): something is missing, see the status lines");

  drive.led_offset_cw = 2.22;   // rad
  drive.led_offset_ccw = 4.02;  // rad
  drive.turn_speed = 1.1;       // rev/s

  // Start values from the 2026-10-07/08 logs. The estimator learns the radius
  // and the mag circle center while spinning and the accel offset while parked,
  // so these only shape the first second. The mag center is a body property
  // (the robot's own magnets); B_NOM is the local horizontal Earth field.
  heading.R0 = 0.117;    // m
  heading.CU0 = 6.4;     // uT
  heading.CV0 = 2.4;     // uT
  heading.B_NOM = 19.7;  // uT
  heading.begin();
}

void loop() {
  board.update();
  readController();

  // Heading, every loop in every mode, so the compass holds it while parked
  static uint32_t last_us = micros();
  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;
  if (dt > 0.5f) dt = 0;  // first loop, or a stall
  int spin_dir = (drive.reversed ? -1 : 1) * MAG_SPIN_SIGN;
  bool motors_off = drive.motor_foo == 0 && drive.motor_bar == 0;
  heading.update(dt, board.accel_z, board.mag_fresh, board.mag_x, board.mag_y, board.mag_z, spin_dir, motors_off);

  drive.heading = heading.theta;
  drive.update();

  board.setMotors(drive.motor_foo, drive.motor_bar);
  board.setLeds(drive.melty_led, drive.status_led);

  logTelemetry();

  static uint32_t last_status_ms = 0;  // link diagnostics over USB, once a second
  if (millis() - last_status_ms >= 1000) {
    last_status_ms = millis();
    Telemetry.printStatus(Serial);
    board.printStatus(Serial);
  }
}

// Sticks, switches and buttons to drive inputs
void readController() {
  AlfredoCRSF &rx = board.crsf;

  drive.rotation = axis(rx.getChannel(1));  // right stick X
  drive.throttle = axis(rx.getChannel(3));  // left stick Y

  left_bumper.update(rx.getChannel(11), 1000);
  right_bumper.update(rx.getChannel(11), 2000);
  left_left.update(rx.getChannel(10), 1000);
  left_right.update(rx.getChannel(10), 2000);
  left_up.update(rx.getChannel(9), 2000);
  left_down.update(rx.getChannel(9), 1000);
  right_left.update(rx.getChannel(7), 1000);
  right_right.update(rx.getChannel(7), 2000);
  right_up.update(rx.getChannel(8), 2000);
  right_down.update(rx.getChannel(8), 1000);
  swa_back.update(rx.getChannel(6), 1000);
  swa_mid.update(rx.getChannel(6), 1500);
  swa_fwd.update(rx.getChannel(6), 2000);
  swd_back.update(rx.getChannel(5), 1000);
  swd_fwd.update(rx.getChannel(5), 2000);
  swb_back.update(rx.getChannel(12), 1000);
  swb_mid.update(rx.getChannel(12), 1500);
  swb_fwd.update(rx.getChannel(12), 2000);

  if (!rx.isLinkUp()) drive.mode = NO_CONNECTION;
  else if (swa_back.is_held()) drive.mode = STOP;
  else if (swa_mid.is_held()) drive.mode = ARCADE;
  else if (swa_fwd.is_held()) drive.mode = MELTY;

  if (drive.mode != MELTY) return;

  // Spin power: the average power both motors run at
  if (right_bumper.just_pressed()) drive.spin_power += 0.02;
  if (left_bumper.just_pressed()) drive.spin_power -= 0.02;
  if (right_up.is_held() || right_down.is_held()) drive.spin_power = 0;
  else if (right_left.is_held() || right_right.just_released()) drive.spin_power = 0.18;  // cruise
  else if (right_right.is_held()) drive.spin_power = 1;                                   // to get going, out of pins, or hit harder

  if (swd_back.is_held()) drive.reversed = false;
  else if (swd_fwd.is_held()) drive.reversed = true;

  // Left arrows trim the LED offset for whichever direction switch C selects
  float *offset = swb_mid.is_held() ? &drive.led_offset_cw : swb_fwd.is_held() ? &drive.led_offset_ccw : nullptr;
  if (offset) {
    if (left_left.just_pressed()) *offset += 0.1;   // rad, 5.7 degrees
    if (left_right.just_pressed()) *offset -= 0.1;
  }
}

// CRSF channel (1000..2000) to -1..1 with a dead zone
float axis(int channel_value) {
  float a = constrain((channel_value - 1500) / 500.0f, -1.0f, 1.0f);
  return fabsf(a) < 0.03f ? 0 : a;
}

void logTelemetry() {
  if (board.accel_fresh) {
    Telemetry.add("accel_x", board.accel_x);  // m/s^2
    Telemetry.add("accel_y", board.accel_y);
    Telemetry.add("accel_z", board.accel_z);
  }
  if (board.mag_fresh) {
    Telemetry.add("mag_x", board.mag_x);  // uT
    Telemetry.add("mag_y", board.mag_y);
    Telemetry.add("mag_z", board.mag_z);
  }

  Telemetry.add("drive_mode", drive.mode);
  Telemetry.add("heading", drive.angle * RAD_TO_DEG);  // what the LED arc and steering use
  Telemetry.add("motor_foo", drive.motor_foo);         // -1..1
  Telemetry.add("motor_bar", drive.motor_bar);
  Telemetry.add("foo_rpm", board.foo.telemetryValid() ? board.foo.rpm() : NAN);  // a gap while the ESC isn't answering
  Telemetry.add("bar_rpm", board.bar.telemetryValid() ? board.bar.rpm() : NAN);

  // Heading estimator internals: enough to replay the filter offline
  Telemetry.add("est_heading", heading.theta * RAD_TO_DEG);
  Telemetry.add("est_mag_heading", heading.thetaMag * RAD_TO_DEG);
  Telemetry.add("est_innov", heading.e * RAD_TO_DEG);  // compass minus estimate at the last fix
  Telemetry.add("est_omega", heading.w);               // rad/s
  Telemetry.add("est_r_mm", heading.r * 1000);         // learned radius
  Telemetry.add("est_cu", heading.cu);                 // learned circle center, uT
  Telemetry.add("est_cv", heading.cv);
  Telemetry.add("est_a_off", heading.aOff);            // learned accel Z offset, m/s^2
  Telemetry.add("est_mag_ok", heading.accepted);       // last fix passed gate 1

  static uint32_t last_slow_ms = 0;  // battery and ESC health, 10 times a second
  if (millis() - last_slow_ms >= 100) {
    last_slow_ms = millis();
    Telemetry.add("vin", board.vin);
    Telemetry.add("foo_temp_c", board.foo.temperatureC());  // NAN until the ESC sends EDT
    Telemetry.add("foo_volts", board.foo.voltage());
    Telemetry.add("foo_amps", board.foo.current());
    Telemetry.add("bar_temp_c", board.bar.temperatureC());
    Telemetry.add("bar_volts", board.bar.voltage());
    Telemetry.add("bar_amps", board.bar.current());
  }
  Telemetry.send();
}
