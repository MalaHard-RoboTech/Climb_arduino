#pragma once
#include <Arduino.h>

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB);
    void begin();
    long getCount();
    void reset();

private:
    uint8_t pinA, pinB;
    volatile long count;
    int lastEncoded;

    static Encoder* instance;
    static void isrWrapperA();
    static void isrWrapperB();
    void update();
};
