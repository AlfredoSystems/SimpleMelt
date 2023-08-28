#include "SimpleMelt.h"
#include <math.h>

float lerp(float val, float in_min, float in_max, float out_min, float out_max) { return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; };
float constrain(float x, float min, float max) { return x > max ? max : (x < min ? min : x); };
float magnitude(float x, float y) { return sqrt(x * x + y * y); };
float modulo(float x, float n) { return x - floor(x / n) * n; }; // Mathematical modulo


void SimpleMelt::meltyStateUpdate() {

   float stick_angle = atan2(axis_left_y, 0); // No omnidirectional movement
   float stick_magnitude = constrain(magnitude(0, axis_left_y, 0, 1);

   // TODO: This is the reverse of what expect. Is that cause I'm applying it to the stick instead of robot angle?
   // Motor acceleration lag causes the robot to move too far past forward. Shift the angle back to compensate.
   stick_angle += MOTOR_LAG_ANGLE * (reversed ? -1 : 1);

   // Make sure time step is valid
   // TODO: This was once useful because it allowed an accidental switch to the STOP state to be
   // reset within half a second while keeping the old melty settings. That doesn't happen anymore,
   // so should we put it back in or remove it?
   uint64_t time_step;
   if (micros() < previous_melty_frame_us) time_step = 0;
   else time_step = micros() - previous_melty_frame_us;
   previous_melty_frame_us = micros();
   if (time_step > 500000) time_step = 0; // Just switched to melty mode, bad frame

   float cen_accel = magnitude(acceleration_x, acceleration_y);
   float angular_vel = (reversed ? -1 : 1) * sqrt(fabs(cen_accel / (ACCELEROMETER_RADIUS + radius_trim)));

   // Trapezoidal Riemann sum to calculate change in angle
   float delta_angle = (angular_vel + prev_ang_vel) * 0.5 * time_step * 0.000001;
   angle = fmod(angle + delta_angle, M_TWOPI);
   previous_ang_vel = angular_vel;

   // Positive (pushed right) stick gives negative clockwise heading rotation
   angle -= gamepad->get_right_x() * M_TWOPI * TURN_SPEED * time_step * 0.000001;

   // Angle from the robot's angle to the joystick's angle, from [-PI, PI)
   float angle_diff = modulo((stick_angle - angle) + M_PI, M_TWOPI) - M_PI;

   // Draw arc
   float melty_led_angle = modulo(angle - MELTY_LED_OFFSET, M_TWOPI);
   melty_led = M_PI_4 < melty_led_angle && melty_led_angle < M_3PI_4;

   // Set motor speed
   float deflection = spin_power * lerp(stick_magnitude, 0, 1, 0, 0.5);
   if (angle_diff < 0) {
     motor_power_foo = (spin_power + deflection * 3) * (reversed ? 1 : -1);
     motor_power_bar = (spin_power - deflection * 1) * (reversed ? 1 : -1);
   } else {
     motor_power_foo = (spin_power - deflection * 1) * (reversed ? 1 : -1);
     motor_power_bar = (spin_power + deflection * 3) * (reversed ? 1 : -1);
   }
   
   // green status LED solid when connected
   status_led = true;
}

// TODO: snake_case-ify
// TODO: Clean up
// Not using curvature drive because with the quick turn threshold far below the point that
// the motors overcome friction and move, the robot will turn in place when the stick is
// below the threshold but not just above. Could bump up the threshold but just avoid the
// issue with arcade.
void SimpleMelt::arcadeStateUpdate() {
    float throttle = -axis_left_y;
    float rotation = -axis_right_x;
    float leftPower = 0;
    float rightPower = 0;
    float maxInput = (throttle > 0 ? 1 : -1) * max(fabs(throttle), fabs(rotation));
    if (throttle > 0) {
        if (rotation > 0) {
            leftPower = maxInput;
            rightPower = throttle - rotation;
        }
        else {
            leftPower = throttle + rotation;
            rightPower = maxInput;
        }
    } else {
        if (rotation > 0) {
            leftPower = maxInput;
            rightPower = throttle + rotation;
        }
        else {
            leftPower = throttle - rotation;
            rightPower = maxInput;
        }
    }
    
   melty_led = 1;
    
   motor_power_foo = leftPower *= 0.1;
   motor_power_bar = rightPower *= 0.1;
   
   // green status LED solid when connected
   status_led = true;
}

void SimpleMelt::stopStateUpdate() {
   // Blink three times when connected
   int t = millis() % 1000;
   if (t < 100 || (t > 200 && t < 300) || (t > 400 && t < 500)) melty_led = true;
   else melty_led = false;
    
   //Turn off motors
   motor_power_foo = 0;
   motor_power_bar = 0;
   
   // green status LED solid when connected
   status_led = true;
}

void SimpleMelt::disconnectedStateUpdate() {
   // Blink melty LED twice when disconnected
   int t = millis() % 1000;
   if (t < 100 || (t > 200 && t < 300) ) melty_led = true;
   else melty_led = false;

   //Turn off motors
   motor_power_foo = 0;
   motor_power_bar = 0;
   
   // Blink green status LED when disconnected
   status_led = (millis() % 1000 < 500) ? false : true;
}

bool Button::just_pressed() {
    return !prev_held && held;
}

bool Button::just_released() {
    return prev_held && !held;
}

void Button::update(int channel_value) {
    prev_held = held;
    held = abs(channel_value - target_value) < 15;
}
