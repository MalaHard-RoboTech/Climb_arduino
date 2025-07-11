#pragma once
#include <ODriveUART.h>
#include <HardwareSerial.h>

class Motor {
public:
    Motor(HardwareSerial& serial, int rxPin, int txPin, uint32_t baudrate = 921600);

    void begin();

    void setIdle();
    void setClosedLoop();

    void setTorque(float val);
    void setVelocity(float val);
    void setPosition(float val);

    float getVbusVoltage();
    float getIqCurrent();
    float getMotorTemp();

private:
    HardwareSerial& serial;
    ODriveUART odrive;
    int rxPin, txPin;
    uint32_t baudrate;

    float lastTorque;
    float lastVelocity;
    float lastPosition;
};
