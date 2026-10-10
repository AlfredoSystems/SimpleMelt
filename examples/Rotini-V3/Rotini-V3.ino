/*
  Rotini V3 - meltybrain firmware for the Rotini V3 board.

  RotiniV3 (RotiniV3.h, next to this file) owns the hardware. The library
  does the rest: HeadingEstimator turns the sensors into a heading, MeltyDrive
  turns the sticks and that heading into motor powers. This sketch is the part
  that is yours to change: the controller mapping, the calibration numbers,
  and what gets logged.

  The V3 runs OneShot125 ESCs (no telemetry back from them) and otherwise
  the same loop as the Rotini V4 example.

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
  The channel list is in logTelemetry().
*/

#include <SimpleMelt.h>
#include <AlfredoTelemetry.h>
#include "RotiniV3.h"

RotiniV3 board;
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

  // Telemetry first, so a hardware problem can't keep the robot off the viewer
  Telemetry.setMaxRate(200);
  Telemetry.setRadioRate(WIFI_PHY_RATE_6M);
  Telemetry.begin("Rotini");

  if (!board.begin()) Serial.println("board.begin(): something is missing, see the status lines");

  drive.led_offset_cw = 2.22;   // rad
  drive.led_offset_ccw = 4.02;  // rad
  drive.turn_speed = 1.1;       // rev/s
  drive.spin_lead = 1;          // no slip limit: it has not been measured for this robot's motors

  // The estimator learns the radius and the mag circle center while spinning
  // and the accel offset while parked. R0 is the V3's radius from the 1.x
  // firmware (0.090 m + 0.032 m trim); B_NOM is the local horizontal Earth
  // field (the V4's value; check est_mag_ok in a log of this robot).
  heading.R0 = 0.122;    // m
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
  Telemetry.add("motor_foo", drive.motor_foo);         // -1..1
  Telemetry.add("motor_bar", drive.motor_bar);

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

  static uint32_t last_slow_ms = 0;  // battery, 10 times a second
  if (millis() - last_slow_ms >= 100) {
    last_slow_ms = millis();
    Telemetry.add("vin", board.vin);
  }
  Telemetry.send();
}
