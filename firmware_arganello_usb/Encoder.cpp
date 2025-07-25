#include "Encoder.h"

Encoder* Encoder::instance = nullptr;

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : pinA(pinA), pinB(pinB), count(0), lastEncoded(0),
      lastTickMicros(0), currentTickMicros(0) {
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

int32_t Encoder::getCount() {
    noInterrupts();
    int32_t value = count;
    interrupts();
    return value;
}

void Encoder::reset() {
    noInterrupts();
    count = 0;
    interrupts();
}

float Encoder::getVelocity() {
    // Temporarily disable interrupts while accessing shared tick timestamps
    noInterrupts();
    unsigned long prev = lastTickMicros;     // Previous tick time (in microseconds)
    unsigned long now = currentTickMicros;   // Most recent tick time (in microseconds)
    interrupts();  // Re-enable interrupts

    // Calculate how long it's been since the last encoder tick
    unsigned long lastTickAge = micros() - now;

    // If no tick has occurred recently (e.g., in over 10 ms), assume the motor is stopped
    if (lastTickAge > VELOCITY_TIMEOUT_MICROS) {
        return 0.0f;
    }

    // Compute time between the last two ticks
    unsigned long delta = now - prev;

    // If ticks happened at the same time (shouldn't happen), avoid division by zero
    if (delta == 0) return 0.0f;

    // Each tick represents 1 / COUNTS_PER_REV of a full revolution
    float revs = 1.0f / COUNTS_PER_REV;

    // Convert delta time from microseconds to seconds
    float dt_s = delta / 1e6f;

    // Compute linear distance per tick: revolutions * circumference
    float distance = revs * (2.0f * PI * DRUM_RADIUS_M);

    // Velocity = distance / time → meters per second
    float velocity = distance / dt_s;

    return velocity;
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

    // Save tick times
    lastTickMicros = currentTickMicros;
    currentTickMicros = micros();
}

void Encoder::isrWrapperA() {
    if (instance) instance->update();
}

void Encoder::isrWrapperB() {
    if (instance) instance->update();
}
