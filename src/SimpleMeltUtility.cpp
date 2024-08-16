#include <Arduino.h>
#include "SimpleMeltUtility.h"

float lerp(float val, float in_min, float in_max, float out_min, float out_max) {
   return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float fconstrain(float x, float min, float max) {
   return x > max ? max : (x < min ? min : x);
}

float magnitude(float x, float y) {
   return sqrt(x * x + y * y);
}

float modulo(float x, float n) {
   return x - floor(x / n) * n;
} // Mathematical modulo

bool Button::just_pressed() {
    return !prev_held && held;
}

bool Button::just_released() {
    return prev_held && !held;
}

void Button::update(int channel_value, int target_value) {
    prev_held = held;
    held = abs(channel_value - target_value) < 15;
}


bool OneShot125::begin(uint8_t pin, uint8_t channel) {
    this->pin = pin;
    this->channel = channel;
    
    ledcSetup(channel, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(pin, channel);

    for (float power = 0; power <= 1; power += 0.001) set_percent(power);
    for (float power = 1; power >= 0; power -= 0.001) set_percent(power);
    stop();
    return true;
}

void OneShot125::set_percent(float percent) {
    percent = fconstrain(percent, -1, 1); // Constrain -1...1
    percent *= (reversed ? -1 : 1);
    percent = (percent / 40.0f) + 0.075f; // Map -1...1 to 0.05...0.1
    ledcWrite(channel, percent * (1 << PWM_RESOLUTION));
}

void OneShot125::stop() {
    set_percent(0);
}