#pragma once
#include <Arduino.h>
#include <stdint.h>

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB);
    void begin();
    int32_t getCount();
    void reset();
    float getVelocity();  // velocity based on tick interval

private:
    uint8_t pinA, pinB;
    volatile int32_t count;
    int32_t lastEncoded;

    static constexpr float DRUM_RADIUS_M = 0.02f;  // 2 cm
    static constexpr int COUNTS_PER_REV = 400;
    static constexpr unsigned long VELOCITY_TIMEOUT_MICROS = 10000; // 10 ms


    // For tick-to-tick velocity
    volatile unsigned long lastTickMicros = 0;
    volatile unsigned long currentTickMicros = 0;

    static Encoder* instance;
    static void isrWrapperA();
    static void isrWrapperB();
    void update();
};
