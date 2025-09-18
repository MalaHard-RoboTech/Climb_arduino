#pragma once
#include <Arduino.h>

class Motor {
public:
  // rpwmPin / lpwmPin: BTS7960 inputs
  // pwmHz: software-PWM frequency (default 1000 Hz)
  Motor(uint8_t rpwmPin, uint8_t lpwmPin, uint32_t pwmHz = 1000);

  void begin();

  // Set target command in [-1..1]
  void set(float s);
  void stop() { set(0.0f); }

  // Change PWM frequency (e.g., 200..2000)
  void setFrequency(uint32_t hz);

  // Call frequently from loop(); generates PWM
  void update();

  float lastCommand() const { return cmd_; }

private:
  uint8_t  rpwm_, lpwm_;
  uint32_t pwmHz_;
  uint32_t periodUs_;
  float    cmd_;        // [-1..1]
  bool     lastHighR_;  // last HIGH state for RPWM
  bool     lastHighL_;  // last HIGH state for LPWM
  uint32_t epochUs_;    // start of current PWM period

  inline void writeR(bool level) { if (level != lastHighR_) { digitalWrite(rpwm_, level); lastHighR_ = level; } }
  inline void writeL(bool level) { if (level != lastHighL_) { digitalWrite(lpwm_, level); lastHighL_ = level; } }
  void       recalcPeriod();
};
