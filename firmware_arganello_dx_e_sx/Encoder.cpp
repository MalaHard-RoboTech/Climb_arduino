#include "Encoder.h"

Encoder* Encoder::instance = nullptr;

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : pinA(pinA), pinB(pinB), count(0), lastEncoded(0) {
    instance = this;
}

void Encoder::begin() {
    
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    
    attachInterrupt(digitalPinToInterrupt(pinA), isrWrapperA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), isrWrapperB, CHANGE);
}

long Encoder::getCount() {
    noInterrupts();
    long temp = count;
    interrupts();
    return temp;
}

void Encoder::reset() {
    noInterrupts();
    count = 0;
    interrupts();
}

void Encoder::update() {
    int MSB = digitalRead(pinA);
    int LSB = digitalRead(pinB);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        count--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        count++;

    lastEncoded = encoded;
}

void Encoder::isrWrapperA() {
    if (instance) instance->update();
}

void Encoder::isrWrapperB() {
    if (instance) instance->update();
}
