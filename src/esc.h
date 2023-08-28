#ifndef ESC_H
#define ESC_H

#include <inttypes.h>

class ESC {
    public:
        bool begin(uint8_t pin, uint8_t channel); // Channels: 0 thru 7

        // From -1...1
        void set_percent(float);
        void stop();
    private:
        uint8_t channel;
        uint16_t pin;
        static const int PWM_FREQUENCY = 400; // Hz
        static const int PWM_RESOLUTION = 12; // bits
};

#endif
