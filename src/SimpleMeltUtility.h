#ifndef SIMPLEMELTUTILITY_H
#define SIMPLEMELTUTILITY_H

float lerp(float val, float in_min, float in_max, float out_min, float out_max);
float fconstrain(float x, float min, float max);
float magnitude(float x, float y);
float modulo(float x, float n);

class Button {
   public:
      bool is_held() { return held; };
      bool just_released();
      bool just_pressed();
      void update(int channel_value, int target_value);
    private:
      bool held = false;
      bool prev_held = false;
};

class OneShot125 {
    public:
        bool begin(uint8_t pin, uint8_t channel); // Channels: 0 thru 7

        // From -1...1
        void set_percent(float);
        void stop();
        void setReversed(bool isReversed) {reversed = isReversed; };
    private:
        uint8_t channel;
        uint16_t pin;
        bool reversed;
        static const int PWM_FREQUENCY = 400; // Hz
        static const int PWM_RESOLUTION = 12; // bits
};

#endif
