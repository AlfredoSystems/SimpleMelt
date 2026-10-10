/*
  Rotini V4 - meltybrain firmware for the Rotini V4 board.

  RotiniV4 (RotiniV4.h, next to this file) owns the hardware. The library
  does the rest: HeadingEstimator turns the sensors into a heading, MeltyDrive
  turns the sticks and that heading into motor powers. This sketch is the part
  that is yours to change: the controller mapping, the calibration numbers,
  and what gets logged.

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
#include "RotiniV4.h"

RotiniV4 board;
MeltyDrive drive;
HeadingEstimator heading;

// +1 if the compass heading increases while spinning with `reversed` false,
// -1 if it decreases. If the LED arc runs backwards after a direction change, flip this.
const int MAG_SPIN_SIGN = 1;

// ---- controller: each is a channel at 1000 / 1500 / 2000 = DOWN / MIDDLE / UP ----
CrsfSwitch mode_switch(6);        // switch B: DOWN stop, MIDDLE arcade, UP melty
CrsfSwitch direction_switch(5);   // switch F: UP spins reversed
CrsfSwitch trim_switch(12);       // switch C: which LED offset the left arrows trim
CrsfSwitch bumpers(11);           // DOWN left bumper, UP right bumper
CrsfSwitch left_lr(10);           // left arrows: DOWN left, UP right
CrsfSwitch left_ud(9);            // left arrows: DOWN down, UP up
CrsfSwitch right_lr(7);           // right arrows
CrsfSwitch right_ud(8);
CrsfSwitch *const switches[] = {&mode_switch, &direction_switch, &trim_switch, &bumpers,
                                &left_lr, &left_ud, &right_lr, &right_ud};
const CrsfSwitch::Position DOWN = CrsfSwitch::DOWN, MIDDLE = CrsfSwitch::MIDDLE, UP = CrsfSwitch::UP;

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
  drive.arcade_power = 0.2;     // top speed in arcade (tank) mode, 0..1
  drive.arcade_deadband = 0.1;  // stick travel ignored in arcade mode

  // The estimator learns the radius and the mag circle center while spinning
  // and the accel offset while parked, so the radius only shapes the first
  // second and the other two start from zero (the 10-09 log fit the center
  // near 11, 8 uT at zero current and the offset near 36 m/s2). B_NOM is the
  // local horizontal Earth field.
  heading.R0 = 0.117;    // m
  heading.B_NOM = 19.7;  // uT
  heading.begin();

  // Slip limit, live-tunable from the viewer page. The motors get the power that rolls the
  // wheels at the body's spin rate plus a push, until the body reaches the speed that spin
  // power holds (see MeltyDrive.h). Measured on this robot once it was balanced, 2026-10-09:
  // a steady spin takes 0.0405 motor volts per rad/s, and a push of 0.07 accelerates the
  // body at about 25 rad/s2. More push only slid the tread.
  //   volts_per_rad_s       steady spin: spin_power_sent x vin / est_omega. Sets the speed
  //                         each spin power means
  //   spin_push             power added above rolling. Past the knee the wheel speed leaves
  //                         the body speed for no gain
  //   spin_push_fade_rad_s  how far below the target speed the push starts to fade. Smaller
  //                         pushes harder to the end
  drive.volts_per_rad_s = 0.0405;
  drive.spin_push = 0.07;
  drive.spin_push_fade_rad_s = 5;
  Telemetry.tune("spin_push", &drive.spin_push);
  Telemetry.tune("volts_per_rad_s", &drive.volts_per_rad_s);
  Telemetry.tune("spin_push_fade_rad_s", &drive.spin_push_fade_rad_s);
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
  // The direction the body is really turning, not the switch: after a direction change the
  // drive brakes first and only then reports the new direction
  int spin_dir = (drive.turning_reversed ? -1 : 1) * MAG_SPIN_SIGN;
  bool motors_off = drive.motor_foo == 0 && drive.motor_bar == 0;
  // What the wheels are doing, from the ESCs' rpm replies (fresh within 20 ms). The estimator learns
  // the accel offset only with both wheels stopped, and the radius and compass center only with a
  // wheel turning. No replies (USB power, ESCs off) = unknown, and nothing can spin anyway.
  HeadingEstimator::Wheels wheels = HeadingEstimator::WHEELS_UNKNOWN;
  if (board.foo.ageUs() < 20000 && board.bar.ageUs() < 20000)
    wheels = (board.foo.rpm() > 0 || board.bar.rpm() > 0) ? HeadingEstimator::WHEELS_TURNING : HeadingEstimator::WHEELS_STOPPED;
  heading.update(dt, board.accel_z, board.mag_fresh, board.mag_x, board.mag_y, board.mag_z, spin_dir, motors_off, wheels);

  // Slip limit inputs: the body's spin rate and the battery voltage
  drive.spin_rate = heading.w;
  drive.battery = board.vin;

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
  for (CrsfSwitch *s : switches) s->update(rx);

  drive.rotation = rx.getAxis(1);  // right stick X
  drive.throttle = rx.getAxis(3);  // left stick Y

  if (!rx.isLinkUp()) drive.mode = NO_CONNECTION;
  else switch (mode_switch.position()) {
    case DOWN:   drive.mode = STOP;   break;
    case MIDDLE: drive.mode = ARCADE; break;
    case UP:     drive.mode = MELTY;  break;
  }
  if (drive.mode != MELTY) return;

  // Spin power: the average power both motors run at
  if (bumpers.movedTo(UP)) drive.spin_power += 0.02;
  if (bumpers.movedTo(DOWN)) drive.spin_power -= 0.02;
  if (right_ud.is(UP) || right_ud.is(DOWN)) drive.spin_power = 0;
  else if (right_lr.is(DOWN) || right_lr.movedFrom(UP)) drive.spin_power = 0.18;  // cruise
  else if (right_lr.is(UP)) drive.spin_power = 1;                                 // to get going, out of pins, or hit harder
  drive.spin_unlimited = right_lr.is(UP);  // and straight to the motors, no slip limit

  drive.reversed = direction_switch.is(UP);

  // Left arrows trim the LED offset for whichever direction switch C selects
  float *offset = trim_switch.is(MIDDLE) ? &drive.led_offset_cw : trim_switch.is(UP) ? &drive.led_offset_ccw : nullptr;
  if (offset) {
    if (left_lr.movedTo(DOWN)) *offset += 0.1;  // rad, 5.7 degrees
    if (left_lr.movedTo(UP)) *offset -= 0.1;
  }
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
  Telemetry.add("spin_power", drive.spin_power);       // 0..1, what the controller asks for
  Telemetry.add("spin_power_sent", drive.spin_power_sent);        // 0..1, what the motors get after the slip limit
  Telemetry.add("spin_power_rolling", drive.spin_power_rolling);  // 0..1, the power that just rolls the wheels at the body's spin rate
  Telemetry.add("spin_target_rad_s", drive.spin_target_rad_s);    // the spin rate this spin power holds: where the push ends
  Telemetry.add("spin_reversing", drive.spin_reversing);          // direction change: 0 none, 1 braking, 2 crossing zero
  Telemetry.add("throttle", drive.throttle);           // -1..1, the translation stick
  Telemetry.add("motor_foo", drive.motor_foo);         // -1..1
  Telemetry.add("motor_bar", drive.motor_bar);
  Telemetry.add("spin_rpm", heading.w * 60 / TWO_PI);  // the robot's spin, signed (est_omega is the same in rad/s)
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
  Telemetry.add("est_b_flat", heading.bFlat);          // spin-axis field when flat, uT (NAN until seen)
  Telemetry.add("est_parked", heading.parked);         // this loop counted as parked (offset learning on)
  Telemetry.add("est_mag_ok", heading.accepted);       // last fix passed gate 1

  static uint32_t last_slow_ms = 0;  // tunables, battery and ESC health, 10 times a second
  if (millis() - last_slow_ms >= 100) {
    last_slow_ms = millis();
    Telemetry.add("spin_push", drive.spin_push);  // the slip limit's tunables, so a recording
    Telemetry.add("volts_per_rad_s", drive.volts_per_rad_s);  // shows what was in force
    Telemetry.add("spin_push_fade_rad_s", drive.spin_push_fade_rad_s);
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
