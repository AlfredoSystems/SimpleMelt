#ifndef ROBOT_H
#define ROBOT_H

typedef enum DriveMode { NO_CONNECTION, STOP, ARCADE, MELTY};
     
class SimpleMelt {
   public:
              
      void meltyStateUpdate();
      void arcadeStateUpdate();
      void stopStateUpdate();
      void disconnectedStateUpdate();

      static constexpr float GOOD_POWER = 0.18;
      static constexpr float MELTY_LED_OFFSET = 0.698132; // rad (CCW is positive)
      static constexpr float TURN_SPEED = 1.1; // rps
      static constexpr float MOTOR_LAG_ANGLE = -2.35619; // rad
      static constexpr float ACCELEROMETER_RADIUS = 0.090; // rad

      DriveMode drive_mode = STOP;
      
      Button right_bumper{.channel = 11, .target_value = 2000};
      Button left_left_arrow{.channel = 10, .target_value = 1000};
      Button left_right_arrow{.channel = 10, .target_value = 2000}; // Rightward-pointing arrow on the left-hand side
      Button left_up_arrow{.channel = 9, .target_value = 2000}; // Upward-pointing arrow on the left-hand side
      Button left_down_arrow{.channel = 9, .target_value = 1000};
      Button right_left_arrow{.channel = 7, .target_value = 1000};
      Button right_right_arrow{.channel = 7, .target_value = 2000};
      Button right_up_arrow{.channel = 8, .target_value = 2000};
      Button right_down_arrow{.channel = 8, .target_value = 1000};
      
      // SWA
      Button front_left_switch_backward{.channel = 6, .target_value = 1000}; // On top of controller, front left switch is pushed away from driver
      Button front_left_switch_neutral{.channel = 6, .target_value = 1500};
      Button front_left_switch_forward{.channel = 6, .target_value = 2000};
      
      // SWD
      Button back_right_switch_backward{.channel = 5, .target_value = 1000};
      Button back_right_switch_forward{.channel = 5, .target_value = 2000};
      
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
      uint64_t previous_melty_frame_us = 0;
};



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
};

   

#endif
