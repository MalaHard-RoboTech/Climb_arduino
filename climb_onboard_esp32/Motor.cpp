#include "Motor.h"

Motor::Motor(uint8_t rpwmPin, uint8_t lpwmPin, uint32_t pwmHz)
: rpwm_(rpwmPin),
  lpwm_(lpwmPin),
  pwmHz_(pwmHz ? pwmHz : 1000),
  periodUs_(0),
  cmd_(0.0f),
  lastHighR_(false),
  lastHighL_(false),
  epochUs_(0) {}

void Motor::begin() {
  pinMode(rpwm_, OUTPUT);
  pinMode(lpwm_, OUTPUT);
  digitalWrite(rpwm_, LOW);
  digitalWrite(lpwm_, LOW);
  lastHighR_ = false;
  lastHighL_ = false;
  recalcPeriod();
  epochUs_ = micros();
}

void Motor::set(float s) {
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  cmd_ = s;
}

void Motor::setFrequency(uint32_t hz) {
  if (hz < 100) hz = 100;        // simple sanity floor
  if (hz > 5000) hz = 5000;      // simple sanity ceiling
  pwmHz_ = hz;
  recalcPeriod();
}

void Motor::recalcPeriod() {
  periodUs_ = (uint32_t)(1000000UL / pwmHz_);
  if (periodUs_ == 0) periodUs_ = 1;
}

void Motor::update() {
  // Software PWM using phase within the current period
  uint32_t now = micros();
  uint32_t dt  = now - epochUs_;
  if (dt >= periodUs_) {
    // start a new period
    epochUs_ = now - (dt % periodUs_);
    dt = now - epochUs_;
  }

  float mag = fabsf(cmd_);              // 0..1
  uint32_t highUs = (uint32_t)(mag * (float)periodUs_);

  bool phaseHigh = dt < highUs;         // within HIGH window?
  if (cmd_ > 0.0f) {
    // Forward: RPWM PWM, LPWM LOW
    writeL(LOW);
    writeR(phaseHigh ? HIGH : LOW);
  } else if (cmd_ < 0.0f) {
    // Reverse: LPWM PWM, RPWM LOW
    writeR(LOW);
    writeL(phaseHigh ? HIGH : LOW);
  } else {
    // Stop: both LOW
    writeR(LOW);
    writeL(LOW);
  }
}
