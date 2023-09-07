#ifndef SIMPLEMELT_H
#define SIMPLEMELT_H
      
typedef enum DriveMode { NO_CONNECTION, STOP, ARCADE, MELTY};

float lerp(float val, float in_min, float in_max, float out_min, float out_max);
float fconstrain(float x, float min, float max);
float magnitude(float x, float y);
float modulo(float x, float n);
    
class SimpleMelt {
   public:

      void meltyStateUpdate();
      void arcadeStateUpdate();
      void stopStateUpdate();
      void disconnectedStateUpdate();
      
      float melty_led_offset_CW = 0; // radians (CCW is positive)
      float melty_led_offset_CCW = 0; // radians (CCW is positive)
      float turn_speed = 1; // rotations per second
      float motor_lag_angle = 0; // radians
      float accelerometer_radius = 0.1; // meters
      float radius_trim = 0; //meters
      
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
};

   

#endif
