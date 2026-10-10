#ifndef ONESHOT125_H
#define ONESHOT125_H

#include <Arduino.h>

// One ESC on the OneShot125 protocol: a 125..250 us pulse at 400 Hz from the
// ESP32's LEDC peripheral. Bidirectional: 187.5 us is stop, 125 is full
// reverse, 250 is full forward.
//
//   OneShot125 foo;
//   foo.begin(PIN);            // in setup()
//   foo.set(0.5);              // -1..1, every loop
class OneShot125 {
  public:
    // Attaches the pin, sweeps the throttle once (so the ESC sees the full
    // range at power-up) and leaves it at stop. False if no LEDC channel is free.
    bool begin(uint8_t pin) {
      this->pin = pin;
      if (!ledcAttach(pin, PWM_FREQUENCY, PWM_RESOLUTION)) return false;
      for (float power = 0; power <= 1; power += 0.001) set(power);
      for (float power = 1; power >= 0; power -= 0.001) set(power);
      set(0);
      return true;
    }

    // -1..1; the sign is flipped when reversed
    void set(float power) {
      if (power > 1) power = 1;
      if (power < -1) power = -1;
      if (reversed) power = -power;
      float duty = power / 40.0f + 0.075f;  // -1..1 -> 125..250 us of the 2500 us period
      ledcWrite(pin, duty * (1 << PWM_RESOLUTION));
    }

    void setReversed(bool r) { reversed = r; }

  private:
    static const int PWM_FREQUENCY = 400;  // Hz
    static const int PWM_RESOLUTION = 12;  // bits

    uint8_t pin = 0;
    bool reversed = false;
};

#endif
