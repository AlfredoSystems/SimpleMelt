#ifndef MELTYDRIVE_H
#define MELTYDRIVE_H

#include <Arduino.h>
#include <math.h>

// The robot's drive modes. The sketch picks one from the controller.
enum DriveMode { NO_CONNECTION, STOP, ARCADE, MELTY };

// Turns the sticks and a heading into two motor powers and the LED states.
//
// Every loop: set the inputs, call update(), hand the outputs to the board.
// The heading comes from a HeadingEstimator, or anything else that gives
// radians in [0, 2pi). This class has no idea what board it runs on.
//
// Melty mode, in order:
//   1  slip limit    spin_power -> spin_power_sent, held to what the wheels can use at the body's spin rate
//   2  translation   the push / ease waveform, applied to spin_power_sent afterwards and not itself limited
class MeltyDrive {
  public:
    // ---- tuning ----
    float led_offset_cw = 0;   // rad, where the LED arc sits relative to the heading, spinning clockwise
    float led_offset_ccw = 0;  // rad, same, spinning counter-clockwise
    float turn_speed = 1;      // rev/s the heading trims at with the stick fully sideways (melty mode)
    float arcade_power = 0.2;     // scales the motors in arcade mode (0..1)
    float arcade_deadband = 0.1;  // stick travel (0..1) ignored in arcade mode, on top of the receiver's dead zone

    // ---- tuning: slip limit ----
    // A motor command is a voltage, so it sets a wheel SPEED, and a brushless wheel reaches that speed in
    // milliseconds while the body takes seconds. Asking for more wheel speed than the floor is passing
    // under the wheel only slides the tread. So the power sent to the motors is the power that rolls the
    // wheels at the body's measured spin rate, plus a small push:
    //
    //     spin_power_rolling = volts_per_rad_s * spin_rate / battery
    //     spin_power_sent    = spin_power_rolling + spin_push       while the body is below its target speed
    //
    // spin_power sets the TARGET SPEED, the spin rate that spin_power holds on its own:
    //
    //     spin_target_rad_s  = spin_power * battery / volts_per_rad_s
    //
    // The full push is kept until the body is within spin_push_fade_rad_s of the target, then fades to
    // nothing at the target, where spin_power_sent is spin_power again. So the push does not tail off on
    // the way up, and while the body is below target spin_power_sent sits ABOVE spin_power, by up to
    // spin_push. At or above the target spin_power_sent is spin_power. The limit acts the same way after
    // a hit, in a pin, and on a restart while still turning: the wheel is never asked to outrun the floor
    // by more than the push.
    //
    // The limit trusts the spin rate; there is no fallback. One that reads zero holds the robot at
    // spin_push. A robot with no spin rate to give sets spin_push to 1, which switches the limit off.
    float volts_per_rad_s = 0.0405;   // V per rad/s  motor volts (power x battery) that hold a steady spin: at
                                      //              cruise, spin_power_sent x battery / spin rate. It turns
                                      //              spin_power into the target speed, so measure it. Too high
                                      //              also adds to the push as speed rises, too low takes away
    float spin_push = 0.07;           // 0..1         how far above the rolling power the motors are driven. This
                                      //              is what accelerates the body: 0.07 gave about 25 rad/s2 on
                                      //              Rotini V4, and more only slid the tread. It is also the most
                                      //              the wheel can outrun the floor (0.07 = about 3 m/s). A
                                      //              spin_power at or below it is applied directly. 1 = no limit
    float spin_push_fade_rad_s = 5;   // rad/s        the push fades out over this much speed below the target.
                                      //              Smaller pushes harder to the end. About spin_push x battery
                                      //              / volts_per_rad_s (29) is the old behaviour, spin_power as a cap
    float rate_smooth = 0.04;         // s            smoothing on the spin rate the limit uses. The accelerometer
                                      //              reading is noisy at low speed and spikes in a hit
    float cross_rate = 10;            // rad/s        a direction change is complete once the body is slower than this
    float battery_nominal = 17;       // V            used when the battery input is missing

    // ---- inputs, set every loop ----
    DriveMode mode = STOP;
    float throttle = 0;        // -1..1, the left stick's Y axis
    float rotation = 0;        // -1..1, the right stick's X axis
    float spin_power = 0;      // 0..1, average motor power while spinning. Reset to 0 whenever the
                               // robot is not in melty mode, so melty always starts from a standstill
    bool spin_unlimited = false;  // skip the slip limit and send spin_power as it is: a "just go" button
                                  // for getting out of a pin or hitting harder, tread slip and all
    bool reversed = false;     // false spins clockwise. The direction ASKED FOR; see turning_reversed
    float heading = 0;         // rad, the robot's estimated heading
    float spin_rate = 0;       // rad/s, the body's measured spin rate (the sign is ignored)
    float battery = 0;         // V, battery voltage. Under 6 V counts as missing

    // ---- outputs ----
    float motor_foo = 0;       // -1..1
    float motor_bar = 0;
    bool melty_led = false;
    bool status_led = false;
    float angle = 0;           // rad, heading plus the stick trim: what the LED arc and steering use
    float spin_power_sent = 0;     // 0..1, spin power after the slip limit: what the translation waveform is applied to
    float spin_power_rolling = 0;  // 0..1, the power that rolls the wheels at the measured spin rate
    float spin_target_rad_s = 0;   // rad/s, the spin rate spin_power holds: where the push ends
    bool turning_reversed = false;  // the direction the body is actually turning. Follows `reversed` only
                                    // once the body has slowed, so give THIS to the heading estimator
    int spin_reversing = 0;        // direction change: 0 none, 1 braking, 2 crossing zero

    void update() {
      uint32_t now = micros();
      float dt = (now - last_us) * 1e-6f;
      last_us = now;
      if (dt > 0.5f) dt = 0;  // first loop, or a stall: skip the step

      // The spin rate the limit uses, smoothed. Kept up to date in every mode so a direction
      // change asked for while coasting in stop mode still waits for the body.
      float k = rate_smooth > 0 ? dt / rate_smooth : 1;
      rate_f += (fabsf(spin_rate) - rate_f) * (k > 1 ? 1 : k);
      float volts = battery > 6 ? battery : battery_nominal;
      spin_power_rolling = volts_per_rad_s * rate_f / volts;
      spin_target_rad_s = volts_per_rad_s > 0 ? spin_power * volts / volts_per_rad_s : 0;

      switch (mode) {
        case MELTY:  melty(dt);      break;
        case ARCADE: arcade();       break;
        case STOP:   stop();         break;
        default:     disconnected(); break;
      }
    }

  private:
    void melty(float dt) {
      spin_power = clamp(spin_power, 0, 1);

      // STEP 1 — slip limit. Sets spin_power_sent and which way the motors turn.
      bool motors_reversed = turning_reversed;
      if (reversed == turning_reversed) {
        // Normal running.
        spin_reversing = 0;
        float below = spin_target_rad_s - rate_f;   // rad/s the body still has to gain
        if (spin_unlimited || spin_push >= 1 || spin_power <= spin_push || below <= 0) {
          // No limit asked for, a spin power too small to slip by more than the lead, or the
          // body already at its target: spin_power as it is.
          spin_power_sent = spin_power;
        } else {
          // Below the target: rolling command plus the lead, full until the last spin_push_fade_rad_s.
          float fade = spin_push_fade_rad_s > 0 ? below / spin_push_fade_rad_s : 1;
          spin_power_sent = spin_power_rolling + spin_push * (fade > 1 ? 1 : fade);
        }
      } else {
        // Direction change while turning. The body cannot reverse at once, and the heading
        // estimator must keep the old direction until it really has.
        if (spin_reversing == 0) { spin_reversing = 1; phase_s = 0; }
        phase_s += dt;
        if (spin_reversing == 1) {
          // Braking: the old direction, a lead BELOW the rolling command, so the wheels hold the
          // body back with grip instead of skidding. Done when that command reaches zero. The
          // time limit covers an ESC that will not brake: then the reversal is forced.
          spin_power_sent = fminf(spin_power, spin_power_rolling - spin_push);
          if (spin_power_rolling <= spin_push || phase_s > BRAKE_LIMIT_S) { spin_reversing = 2; phase_s = 0; }
        }
        if (spin_reversing == 2) {
          // Crossing zero: the new direction at the lead alone, carrying the body through zero.
          // The spin rate has no sign, so the crossing is taken as done once the body is slow
          // (cross_rate) or after CROSS_LIMIT_S. Until then the heading keeps the old direction.
          motors_reversed = reversed;
          spin_power_sent = fminf(spin_power, spin_push);
          if (rate_f < cross_rate || phase_s > CROSS_LIMIT_S) { turning_reversed = reversed; spin_reversing = 0; }
        }
      }
      spin_power_sent = clamp(spin_power_sent, 0, 1);

      // The heading is absolute, so the stick turns a trim rather than the heading
      // itself. Pushing right turns the robot's "forward" clockwise.
      heading_trim = wrap2pi(heading_trim - rotation * M_TWOPI * turn_speed * dt);
      angle = wrap2pi(heading + heading_trim);

      // Where the stick points and how far. Forward and back only, no omnidirectional movement.
      float stick_angle = throttle >= 0 ? M_PI_2 : -M_PI_2;
      float stick_magnitude = clamp(fabsf(throttle), 0, 1);
      float angle_diff = wrap2pi((stick_angle - angle) + M_PI) - M_PI;  // [-pi, pi)

      // The LED is lit for a quarter of each turn, so the arc shows which way is forward
      float led_offset = turning_reversed ? led_offset_ccw : led_offset_cw;
      float led_angle = wrap2pi(angle - led_offset);
      melty_led = M_PI_4 < led_angle && led_angle < M_3PI_4;

      // STEP 2 — translation waveform, applied to the limited spin power. On the half of each
      // turn that moves the robot toward the stick, one motor is pushed harder and the other
      // eased off. The push side goes above spin_power_sent on purpose: the slip limit sets the
      // level the waveform works around, it does not clip the waveform.
      float deflection = spin_power_sent * stick_magnitude * 0.5f;
      float dir = motors_reversed ? 1 : -1;
      if (angle_diff < 0) {
        motor_foo = -(spin_power_sent + deflection * 3) * dir;
        motor_bar =  (spin_power_sent - deflection * 1) * dir;
      } else {
        motor_foo = -(spin_power_sent - deflection * 1) * dir;
        motor_bar =  (spin_power_sent + deflection * 3) * dir;
      }

      status_led = true;
    }

    void arcade() {
      idle();
      float fwd = -deadband(throttle, arcade_deadband);
      float turn = -deadband(rotation, arcade_deadband);
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

    void stop() {
      idle();
      motor_foo = 0;
      motor_bar = 0;

      int t = millis() % 1000;  // melty LED blinks three times a second
      melty_led = t < 100 || (t > 200 && t < 300) || (t > 400 && t < 500);
      status_led = true;
    }

    void disconnected() {
      idle();
      motor_foo = 0;
      motor_bar = 0;

      int t = millis() % 1000;  // melty LED blinks twice a second, status LED once
      melty_led = t < 100 || (t > 200 && t < 300);
      status_led = t >= 500;
    }

    // Not spinning under power. The spin command starts over (melty mode always begins at zero
    // spin power), and a direction asked for here is taken up as soon as the body is slow.
    void idle() {
      spin_power = 0;
      spin_power_sent = 0;
      spin_reversing = 0;
      if (reversed != turning_reversed && rate_f < cross_rate) turning_reversed = reversed;
    }

    static float clamp(float x, float lo, float hi) { return x > hi ? hi : (x < lo ? lo : x); }
    // Zero inside the band, then rising from 0 at its edge so there is no jump
    static float deadband(float x, float band) {
      if (fabsf(x) <= band) return 0;
      return (x > 0 ? x - band : x + band) / (1 - band);
    }
    static float wrap2pi(float a) { return a - floorf(a / M_TWOPI) * M_TWOPI; }  // [0, 2pi)

    static constexpr float BRAKE_LIMIT_S = 8;  // s, longest a direction change brakes before reversing anyway
    static constexpr float CROSS_LIMIT_S = 1;  // s, longest it waits for the body to cross zero

    float heading_trim = 0;    // rad, accumulated from the rotation stick
    float rate_f = 0;          // rad/s, smoothed spin rate
    float phase_s = 0;         // s, time in the current direction-change phase
    uint32_t last_us = 0;
};

#endif
