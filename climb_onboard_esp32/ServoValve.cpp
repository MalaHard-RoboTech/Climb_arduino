#include "ServoValve.h"

ServoValve::ServoValve(int pin, int min_us, int max_us, int frame_us)
  : pin_(pin),
    min_us_(min_us),
    max_us_(max_us),
    frame_us_(frame_us),
    angle_deg_(0.0f)
{}

void ServoValve::begin() {
  pinMode(pin_, OUTPUT);
  digitalWrite(pin_, LOW);
}

// Clamp angle to 0–90 and store it
void ServoValve::setAngle(float deg) {
  if (deg < 0)   deg = 0;
  if (deg > 90)  deg = 90;
  angle_deg_ = deg;
}

// Convert degrees to pulse width (µs) scaled over full 500–2500 µs range
int ServoValve::angleToUs(float deg) const {
  return (int)(min_us_ + (deg / 90.0f) * (max_us_ - min_us_));
}

// Bit-bang one 20 ms frame for the current angle
void ServoValve::sendFrame() {
  int high_us = angleToUs(angle_deg_);
  unsigned long start = micros();
  digitalWrite(pin_, HIGH);

  while ((micros() - start) < frame_us_) {
    if ((micros() - start) >= (unsigned long)high_us) {
      digitalWrite(pin_, LOW);
      break; // pin LOW, remaining time just waits
    }
  }
}
