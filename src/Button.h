#ifndef BUTTON_H
#define BUTTON_H

#include <stdlib.h>

// One button, or one position of a switch, on a CRSF channel. It is held while
// the channel sits within 15 of target_value (1000, 1500 or 2000).
//
//   Button arm;
//   arm.update(crsf.getChannel(6), 2000);   // every loop
//   if (arm.just_pressed()) ...
class Button {
  public:
    void update(int channel_value, int target_value) {
      prev = held;
      held = abs(channel_value - target_value) < 15;
    }
    bool is_held() const { return held; }
    bool just_pressed() const { return held && !prev; }
    bool just_released() const { return prev && !held; }

  private:
    bool held = false;
    bool prev = false;
};

#endif
