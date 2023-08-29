#ifndef SIMPLEMELT_H
#define SIMPLEMELT_H
      
typedef enum DriveMode { NO_CONNECTION, STOP, ARCADE, MELTY};

float lerp(float val, float in_min, float in_max, float out_min, float out_max);
float fconstrain(float x, float min, float max);
float magnitude(float x, float y);
float modulo(float x, float n);

class Button {
   public:
      bool is_held() { return held; };
      bool just_released();
      bool just_pressed();
      void update(int channel_value);
      
      int channel;
      int target_value; // Value read from channel that signals the button is held
      bool held = false;
      bool prev_held = false;
      
      Button(int ch, int tv) : channel(ch), target_value(tv) {}
};
    
class SimpleMelt {
   public:
      float MELTY_LED_OFFSET = 0.698132; // rad (CCW is positive)
      float TURN_SPEED = 1.1; // rps
      float MOTOR_LAG_ANGLE = -2.35619; // rad
      float ACCELEROMETER_RADIUS = 0.090; // rad
              
      void meltyStateUpdate();
      void arcadeStateUpdate();
      void stopStateUpdate();
      void disconnectedStateUpdate();

      DriveMode drive_mode = STOP;
      
      Button left_bumper{11,1000};
      Button right_bumper{11,2000};
      Button left_left_arrow{10,1000};
      Button left_right_arrow{10, 2000}; // Rightward-pointing arrow on the left-hand side
      Button left_up_arrow{9, 2000}; // Upward-pointing arrow on the left-hand side
      Button left_down_arrow{9, 1000};
      Button right_left_arrow{7, 1000};
      Button right_right_arrow{7, 2000};
      Button right_up_arrow{8, 2000};
      Button right_down_arrow{8, 1000};
      
      // SWA
      Button front_left_switch_backward{6, 1000}; // On top of controller, front left switch is pushed away from driver
      Button front_left_switch_neutral{6, 1500};
      Button front_left_switch_forward{6, 2000};
      
      // SWD
      Button back_right_switch_backward{5, 1000};
      Button back_right_switch_forward{5, 2000};
      
      float axis_left_x = 0;
      float axis_left_y = 0;
      float axis_right_x = 0;
      float axis_right_y = 0;

      float accelerometer_x;
      float accelerometer_y;
      float accelerometer_z;

      float spin_power = 0;
      float angle = 0;
      float radius_trim = 0;
      bool reversed = false; // False is clockwise

      float motor_power_foo;
      float motor_power_bar;
      bool melty_led;
      bool status_led;

      float previous_ang_vel = 0;
      unsigned long long previous_melty_frame_us = 0;
};

   

#endif
