#ifndef MELTYDRIVE_H
#define MELTYDRIVE_H

#include <Arduino.h>

// The robot's drive modes. The sketch picks one from the controller.
enum DriveMode { NO_CONNECTION, STOP, ARCADE, MELTY };

// Turns the sticks and a heading into two motor powers and the LED states.
//
// Every loop: set the inputs, call update(), hand the outputs to the board.
// The heading comes from a HeadingEstimator, or anything else that gives
// radians in [0, 2pi). This class has no idea what board it runs on.
class MeltyDrive {
  public:
    // ---- tuning ----
    float led_offset_cw = 0;   // rad, where the LED arc sits relative to the heading, spinning clockwise
    float led_offset_ccw = 0;  // rad, same, spinning counter-clockwise
    float turn_speed = 1;      // rev/s the heading trims at with the stick fully sideways (melty mode)
    float arcade_power = 0.1;  // scales the motors in arcade mode (0..1)

    // ---- inputs, set every loop ----
    DriveMode mode = STOP;
    float throttle = 0;        // -1..1, the left stick's Y axis
    float rotation = 0;        // -1..1, the right stick's X axis
    float spin_power = 0;      // 0..1, average motor power while spinning
    bool reversed = false;     // false spins clockwise
    float heading = 0;         // rad, the robot's estimated heading

    // ---- outputs ----
    float motor_foo = 0;       // -1..1
    float motor_bar = 0;
    bool melty_led = false;
    bool status_led = false;
    float angle = 0;           // rad, heading plus the stick trim: what the LED arc and steering use

    void update();

  private:
    void melty(float dt);
    void arcade();
    void stop();
    void disconnected();

    float heading_trim = 0;    // rad, accumulated from the rotation stick
    uint32_t last_us = 0;
};

#endif
