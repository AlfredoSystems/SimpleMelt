#include <Arduino.h>
#include <math.h>
#include "SimpleMelt.h"
#include "SimpleMeltUtility.h"


void SimpleMelt::meltyStateUpdate() {

   float stick_angle = atan2(throttle, 0); // No omnidirectional movement
   float stick_magnitude = fconstrain(magnitude(0, throttle), 0, 1);
   spin_power = fconstrain(spin_power, 0, 1);
   
   //stick_angle += motor_lag_angle * (reversed ? -1 : 1);

   uint32_t now_us = micros();
   uint32_t time_step = now_us - (uint32_t)previous_melty_frame_us;
   previous_melty_frame_us = now_us;
   if (time_step > 500000) time_step = 0; // Just switched to melty mode, bad frame

   float cen_accel = sqrt(accelerometer_x * accelerometer_x + accelerometer_y * accelerometer_y + accelerometer_z * accelerometer_z);
   float angular_vel = (reversed ? -1 : 1) * sqrt(fabs(cen_accel / (accelerometer_radius + radius_trim)));

   // Trapezoidal Riemann sum to calculate change in angle
   float delta_angle = (angular_vel + previous_ang_vel) * 0.5 * time_step * 0.000001;
   angle = modulo(angle + delta_angle, M_TWOPI);
   previous_ang_vel = angular_vel;

   // Positive (pushed right) stick gives positive clockwise heading rotation
   angle -= rotation * M_TWOPI * turn_speed * time_step * 0.000001;

   // Angle from the robot's angle to the joystick's angle, from [-PI, PI)
   float angle_diff = modulo((stick_angle - angle) + M_PI, M_TWOPI) - M_PI;

   // Draw arc
   float melty_led_offset = (!reversed) ? melty_led_offset_CW : melty_led_offset_CCW;
   float melty_led_angle = modulo(angle - melty_led_offset, M_TWOPI);
   melty_led = M_PI_4 < melty_led_angle && melty_led_angle < M_3PI_4;
   
   // Set motor speed
   float deflection = spin_power * lerp(stick_magnitude, 0, 1, 0, 0.5);
   if (angle_diff < 0) {
     motor_power_foo = -1 * (spin_power + deflection * 3) * (reversed ? 1 : -1);
     motor_power_bar =      (spin_power - deflection * 1) * (reversed ? 1 : -1);
   } else {
     motor_power_foo = -1 * (spin_power - deflection * 1) * (reversed ? 1 : -1);
     motor_power_bar =      (spin_power + deflection * 3) * (reversed ? 1 : -1);
   }
     
   // green status LED solid when connected
   status_led = true;
}

void SimpleMelt::meltyMagStateUpdate() {

   // Stick has no omnidirectional movement, so heading is straight up or down
   float stick_angle = (throttle >= 0) ? M_PI_2 : -M_PI_2;
   float stick_magnitude = fconstrain(fabs(throttle), 0, 1);
   spin_power = fconstrain(spin_power, 0, 1);

   //stick_angle += motor_lag_angle * (reversed ? -1 : 1);

   uint32_t now_us = micros();
   uint32_t time_step = now_us - (uint32_t)previous_melty_frame_us;
   previous_melty_frame_us = now_us;
   if (time_step > 500000) time_step = 0; // Just switched to melty mode, bad frame

   //-----------------mag-stuff------------------------//

        float magnetometer_xy = 0.7071 * (magnetometer_x + magnetometer_y);
		
        comp_xy = (comp_xy * alpha_xy) + (magnetometer_xy * (1-alpha_xy));
        float filtered_xy = magnetometer_xy - comp_xy;

        comp_z = (comp_z * alpha_z) + (magnetometer_z * (1-alpha_z));
        float filtered_z = magnetometer_z - comp_z;

        angle = atan2(filtered_xy, filtered_z);


   //-----------------mag-stuff------------------------//
   // Positive (pushed right) stick gives positive clockwise heading rotation
   angle -= rotation * M_TWOPI * turn_speed * time_step * 0.000001;

   // Angle from the robot's angle to the joystick's angle, from [-PI, PI)
   float angle_diff = modulo((stick_angle - angle) + M_PI, M_TWOPI) - M_PI;

   // Draw arc
   float melty_led_offset = (!reversed) ? melty_led_offset_CW : melty_led_offset_CCW;
   float melty_led_angle = modulo(angle - melty_led_offset, M_TWOPI);
   melty_led = M_PI_4 < melty_led_angle && melty_led_angle < M_3PI_4;
   
   // Set motor speed
   float deflection = spin_power * lerp(stick_magnitude, 0, 1, 0, 0.5);
   if (angle_diff < 0) {
     motor_power_foo = -1 * (spin_power + deflection * 3) * (reversed ? 1 : -1);
     motor_power_bar =      (spin_power - deflection * 1) * (reversed ? 1 : -1);
   } else {
     motor_power_foo = -1 * (spin_power - deflection * 1) * (reversed ? 1 : -1);
     motor_power_bar =      (spin_power + deflection * 3) * (reversed ? 1 : -1);
   }
     
   // green status LED solid when connected
   status_led = true;
}

void SimpleMelt::arcadeStateUpdate() {
    float fwd = -throttle;
    float turn = -rotation;
    float leftPower = 0;
    float rightPower = 0;
    float maxInput = (fwd > 0 ? 1 : -1) * max(fabs(fwd), fabs(turn));
    if (fwd > 0) {
        if (turn > 0) {
            leftPower = maxInput;
            rightPower = fwd - turn;
        }
        else {
            leftPower = fwd + turn;
            rightPower = maxInput;
        }
    } else {
        if (turn > 0) {
            leftPower = maxInput;
            rightPower = fwd + turn;
        }
        else {
            leftPower = fwd - turn;
            rightPower = maxInput;
        }
    }

   melty_led = true;

   motor_power_foo = leftPower * arcade_power;
   motor_power_bar = rightPower * arcade_power;

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

