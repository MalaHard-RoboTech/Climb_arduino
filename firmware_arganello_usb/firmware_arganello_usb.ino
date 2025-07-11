#include "Encoder.h"
#include "Brake.h"
#include "Motor.h"
#include "Message.h"

#define ODRIVE_RX 17
#define ODRIVE_TX 16
#define RELAY_PIN 4
#define ENCODER_A 5
#define ENCODER_B 6

Encoder encoder(ENCODER_A, ENCODER_B);
Brake brake(RELAY_PIN);

HardwareSerial odrive_serial(1); // Use UART1
Motor motor(odrive_serial, 17, 16); // RX = 16 (ODrive TX), TX = 17 (ODrive RX)

MessageIn messageIn;
MessageOut messageOut;

void setup() {
    Serial.begin(115200);
    brake.begin();
    encoder.begin();
    motor.begin();

    Serial.println("✅ Arganello ready");
}

void loop() {
    // ─────────────────────────────────────────────────────
    // 1. Read command from PC
    // ─────────────────────────────────────────────────────
    if (Serial.available() >= sizeof(MessageIn)) {
        Serial.readBytes((uint8_t*)&messageIn, sizeof(MessageIn));

        brake.setBrakeEngaged(messageIn.brake_command);

        switch (messageIn.motor_mode) {
            case 0: motor.setIdle(); break;
            case 1: motor.setClosedLoop(); break;
        }

        //only called in when value actually changes
        motor.setTorque(messageIn.target_torque);
        motor.setVelocity(messageIn.target_velocity);
        motor.setPosition(messageIn.target_position);
    }

    // ─────────────────────────────────────────────────────
    // 2. Send telemetry to PC
    // ─────────────────────────────────────────────────────
    static unsigned long lastSend = 0;
    if (millis() - lastSend >= 5) {
        lastSend = millis();

        messageOut.brake_status = brake.isBrakeEngaged();
        messageOut.motor_mode_status = messageIn.motor_mode;
        messageOut.encoder_count = encoder.getCount();
        messageOut.iq_current = motor.getIqCurrent();
        messageOut.vbus_voltage = motor.getVbusVoltage();
        messageOut.motor_temperature = motor.getMotorTemp();

        // printTelemetry(messageOut);  // Uncomment for debug
        Serial.write((uint8_t*)&messageOut, sizeof(MessageOut));
    }
}

void printTelemetry(const MessageOut& msg) {
    Serial.println("📤 MessageOut:");
    Serial.print("  Brake Status: ");       Serial.println(msg.brake_status ? "ENGAGED" : "RELEASED");
    Serial.print("  Motor Mode: ");         Serial.println(msg.motor_mode_status == 1 ? "CLOSED_LOOP" : "IDLE");
    Serial.print("  Encoder Count: ");      Serial.println(msg.encoder_count);
    Serial.print("  Iq Current: ");         Serial.println(msg.iq_current, 3);
    Serial.print("  VBus Voltage: ");       Serial.println(msg.vbus_voltage, 2);
    Serial.print("  Motor Temp: ");         Serial.println(msg.motor_temperature, 1);
}
