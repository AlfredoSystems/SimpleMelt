#include "esc.h"
#include <Arduino.h>

bool ESC::begin(uint8_t pin, uint8_t channel) {
    this->pin = pin;
    this->channel = channel;
    
    ledcSetup(channel, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(pin, channel);

    for (float power = 0; power <= 1; power += 0.001) set_percent(power);
    for (float power = 1; power >= 0; power -= 0.001) set_percent(power);
    stop();
    return true;
}

void ESC::set_percent(float percent) {
    percent = -percent; // TODO: Temporary for NHRL finals while motors are inverted
    // TODO: The old ESCs used a 50-100% duty cycle, but these use 5-10%. Why?
    percent = percent > 1 ? 1 : percent < -1 ? -1 : percent; // Constrain -1...1
    percent = (percent / 40.0f) + 0.075f; // Map to 0.05...0.1
    ledcWrite(channel, percent * (1 << PWM_RESOLUTION));
}

void ESC::stop() {
    set_percent(0);
}
