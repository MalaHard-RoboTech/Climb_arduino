#include "Brake.h"

Brake::Brake(uint8_t relayPin) : pin(relayPin) {}

void Brake::begin() {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void Brake::setRelay(bool state) {
    digitalWrite(pin, state ? HIGH : LOW);
}
