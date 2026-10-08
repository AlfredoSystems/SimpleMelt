#ifndef SIMPLEMELT_H
#define SIMPLEMELT_H

enum DriveMode { NO_CONNECTION, STOP, ARCADE, MELTY };

class SimpleMelt {
   public:

      // Melty drive, heading dead-reckoned from the accelerometer alone (the original method).
      void meltyStateUpdate();
      // Melty drive from a heading computed elsewhere, e.g. HeadingEstimator (accel + mag).
      // That heading is absolute, so the stick's turn input accumulates into heading_trim
      // instead of nudging the heading itself.
      void meltyHeadingStateUpdate(float heading);
      void arcadeStateUpdate();
      void stopStateUpdate();
      void disconnectedStateUpdate();

      float melty_led_offset_CW = 0; // radians (CCW is positive)
      float melty_led_offset_CCW = 0; // radians (CCW is positive)
      float turn_speed = 1; // rotations per second
      float motor_lag_angle = 0; // radians
      float accelerometer_radius = 0.1; // meters (meltyStateUpdate only; HeadingEstimator learns its own)
      float radius_trim = 0; //meters
      float arcade_power = 0.1; // scales arcade-mode motor output (0...1)
      float heading_trim = 0; // radians, added to the external heading (meltyHeadingStateUpdate)

      DriveMode drive_mode = STOP;

      float throttle = 0;
      float rotation = 0;

      float accelerometer_x;
      float accelerometer_y;
      float accelerometer_z;

      float spin_power = 0;
      float angle = 0;
      bool reversed = false; // False is clockwise

      float motor_power_foo;
      float motor_power_bar;
      bool melty_led;
      bool status_led;

      float previous_ang_vel = 0;
      unsigned long long previous_melty_frame_us = 0;

   private:
      // Shared tail of the melty modes: LED arc and motor powers from `angle` and the stick.
      void meltyDrive(uint32_t time_step_us);
      uint32_t meltyTimeStep();
};

#endif
