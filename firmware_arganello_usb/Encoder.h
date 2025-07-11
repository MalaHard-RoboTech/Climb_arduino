#pragma once
#include <Arduino.h>
#include <stdint.h>

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB);
    void begin();
    int32_t getCount();
    void reset();

private:
    uint8_t pinA, pinB;
    volatile int32_t count;      // ✅ Fixed: was 'long', now 'int32_t'
    int32_t lastEncoded;

    static Encoder* instance;
    static void isrWrapperA();
    static void isrWrapperB();
    void update();
};
