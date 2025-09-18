#pragma once
#include <Arduino.h>

class ServoValve
 {
public:
  // Constructor: pin = GPIO, frame_us = servo frame period (20 ms)
  ServoValve
(int pin, int min_us = 500, int max_us = 2500, int frame_us = 20000);

  // Call in setup()
  void begin();

  // Set target angle in degrees (0–90). Clamped if out of range.
  void setAngle(float deg);

  // Send one 20 ms pulse frame (call every loop or with a timer)
  void sendFrame();

private:
  int   pin_;
  int   min_us_;
  int   max_us_;
  int   frame_us_;
  float angle_deg_;

  int angleToUs(float deg) const;
};
