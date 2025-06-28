#pragma once
#include <Arduino.h>

class Brake {
public:
    Brake(uint8_t relayPin);
    void begin();
    void setRelay(bool state);
private:
    uint8_t pin;
};
