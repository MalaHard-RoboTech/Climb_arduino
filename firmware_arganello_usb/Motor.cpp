#include "Motor.h"
#include <Arduino.h>

Motor::Motor(HardwareSerial& serial, int rxPin, int txPin, uint32_t baudrate)
    : serial(serial), odrive(serial), rxPin(rxPin), txPin(txPin), baudrate(baudrate) {}

void Motor::begin() {
    serial.begin(baudrate, SERIAL_8N1, rxPin, txPin);
    delay(100);
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);

    // Init values to zero
    odrive.setTorque(0.0f);
    odrive.setVelocity(0.0f);
    odrive.setPosition(0.0f);
}

void Motor::setIdle() {
    odrive.setState(AXIS_STATE_IDLE);
}

void Motor::setClosedLoop() {
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
}

void Motor::setTorque(float val) {
    if (val != lastTorque) {
        odrive.setTorque(val);
        lastTorque = val;
    }
}

void Motor::setVelocity(float val) {
    if (val != lastVelocity) {
        odrive.setVelocity(val);
        lastVelocity = val;
    }
}

void Motor::setPosition(float val) {
    if (val != lastPosition) {
        odrive.setPosition(val);
        lastPosition = val;
    }
}

float Motor::getVbusVoltage() {
    return odrive.getParameterAsFloat("vbus_voltage");
}

float Motor::getIqCurrent() {
    return odrive.getParameterAsFloat("motor.current_control.Iq_measured");
}

float Motor::getMotorTemp() {
    return odrive.getParameterAsFloat("motor.temp_motor");
}
