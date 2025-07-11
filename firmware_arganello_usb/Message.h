#pragma once
#include <Arduino.h>

// Direction: PC → Arganello (commands)
struct __attribute__((packed)) MessageIn {
  bool brake_command;   // true = engage brake
  bool motor_mode;      // false = IDLE, true = CLOSED_LOOP
  float target_torque;  // in Nm
  float target_velocity;
  float target_position;
};

// Direction: Arganello → PC (status/telemetry)
struct __attribute__((packed)) MessageOut {
  bool brake_status;       // true = brake is engaged
  bool motor_mode_status;  // 0 = IDLE, 1 = CLOSED_LOOP
  int32_t encoder_count;
  float iq_current;         // in Amps
  float vbus_voltage;       // in Volts
  float motor_temperature;  // in °C
};
