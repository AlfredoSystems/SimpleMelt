#include <math.h>
#include "MeltyDrive.h"

static float clampf(float x, float lo, float hi) { return x > hi ? hi : (x < lo ? lo : x); }
static float wrap2pi(float a) { return a - floorf(a / M_TWOPI) * M_TWOPI; }  // [0, 2pi)

void MeltyDrive::update() {
  uint32_t now = micros();
  float dt = (now - last_us) * 1e-6f;
  last_us = now;
  if (dt > 0.5f) dt = 0;  // first loop, or a stall: skip the step

  switch (mode) {
    case MELTY:  melty(dt);     break;
    case ARCADE: arcade();      break;
    case STOP:   stop();        break;
    default:     disconnected(); break;
  }
}

void MeltyDrive::melty(float dt) {
  spin_power = clampf(spin_power, 0, 1);

  // The heading is absolute, so the stick turns a trim rather than the heading
  // itself. Pushing right turns the robot's "forward" clockwise.
  heading_trim = wrap2pi(heading_trim - rotation * M_TWOPI * turn_speed * dt);
  angle = wrap2pi(heading + heading_trim);

  // Where the stick points and how far. Forward and back only, no omnidirectional movement.
  float stick_angle = throttle >= 0 ? M_PI_2 : -M_PI_2;
  float stick_magnitude = clampf(fabsf(throttle), 0, 1);
  float angle_diff = wrap2pi((stick_angle - angle) + M_PI) - M_PI;  // [-pi, pi)

  // The LED is lit for a quarter of each turn, so the arc shows which way is forward
  float led_offset = reversed ? led_offset_ccw : led_offset_cw;
  float led_angle = wrap2pi(angle - led_offset);
  melty_led = M_PI_4 < led_angle && led_angle < M_3PI_4;

  // Both motors run at spin_power. On the half of each turn that moves the robot
  // toward the stick, one motor is pushed harder and the other eased off.
  float deflection = spin_power * stick_magnitude * 0.5f;
  float dir = reversed ? 1 : -1;
  if (angle_diff < 0) {
    motor_foo = -(spin_power + deflection * 3) * dir;
    motor_bar =  (spin_power - deflection * 1) * dir;
  } else {
    motor_foo = -(spin_power - deflection * 1) * dir;
    motor_bar =  (spin_power + deflection * 3) * dir;
  }

  status_led = true;
}

void MeltyDrive::arcade() {
  float fwd = -throttle;
  float turn = -rotation;
  float max_input = (fwd > 0 ? 1 : -1) * fmaxf(fabsf(fwd), fabsf(turn));
  float left, right;
  if (fwd > 0) {
    if (turn > 0) { left = max_input;  right = fwd - turn; }
    else          { left = fwd + turn; right = max_input; }
  } else {
    if (turn > 0) { left = max_input;  right = fwd + turn; }
    else          { left = fwd - turn; right = max_input; }
  }
  motor_foo = left * arcade_power;
  motor_bar = right * arcade_power;

  melty_led = true;
  status_led = true;
}

void MeltyDrive::stop() {
  motor_foo = 0;
  motor_bar = 0;

  int t = millis() % 1000;  // melty LED blinks three times a second
  melty_led = t < 100 || (t > 200 && t < 300) || (t > 400 && t < 500);
  status_led = true;
}

void MeltyDrive::disconnected() {
  motor_foo = 0;
  motor_bar = 0;

  int t = millis() % 1000;  // melty LED blinks twice a second, status LED once
  melty_led = t < 100 || (t > 200 && t < 300);
  status_led = t >= 500;
}
