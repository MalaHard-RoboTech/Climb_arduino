#include "Encoder.h"
#include "Brake.h"
#include "Esp32.h"
#include <ODriveUART.h>
#include <HardwareSerial.h>

#define ODRIVE_RX 17
#define ODRIVE_TX 16
#define BAUDRATE 115200

HardwareSerial odrive_serial(1);
ODriveUART odrive(odrive_serial);

// ──────── PUBLISH RATES ────────
#define RT_INTERVAL_MS     5     // Real-time high-priority data: 200 Hz (every 5 ms)
#define LOWPRIO_INTERVAL_MS 1000 // Low-priority data: 1 Hz (every 1000 ms)


#define RELAY_PIN 4
#define ENCODER_A 5
#define ENCODER_B 6



uint8_t peerMac[] = {0xCC, 0xBA, 0x97, 0x14, 0x0A, 0x14};

Encoder encoder(ENCODER_A, ENCODER_B);
Brake brake(RELAY_PIN);
Esp32 espnow(peerMac);

long lastCount = 0;
unsigned long lastSend = 0;
unsigned long lastPrint = 0;

void handleCommand(const char* cmd) {
    Serial.printf("📨 Received: %s\n", cmd);

    // Relay control
    if (strcmp(cmd, "on") == 0) {
        brake.setRelay(false);
        Serial.println("🔴 Relay OFF");
        return;
    } else if (strcmp(cmd, "off") == 0) {
        brake.setRelay(true);
        Serial.println("🟢 Relay ON");
        return;
    }

    // Stop all motion
    if (strcmp(cmd, "stop") == 0) {
        odrive.setVelocity(0);
        odrive.setTorque(0);
        Serial.println("🛑 All ODrive commands stopped");
        return;
    }

    // Position control
    if (strncmp(cmd, "pos:", 4) == 0) {
        float pos = atof(cmd + 4);
        odrive.setPosition(pos);
        Serial.printf("📍 Position set: %.2f turns\n", pos);
        return;
    }

    // Velocity control
    if (strncmp(cmd, "vel:", 4) == 0) {
        float vel = atof(cmd + 4);
        odrive.setVelocity(vel);
        Serial.printf("🚀 Velocity set: %.2f turns/s\n", vel);
        return;
    }

    // Torque control
    if (strncmp(cmd, "torque:", 7) == 0) {
        float torque = atof(cmd + 7);
        odrive.setTorque(torque);
        Serial.printf("💪 Torque set: %.2f Nm (or equivalent)\n", torque);
        return;
    }

    // Unknown command
    Serial.println("❓ Unknown command");
}



void setup() {
    Serial.begin(115200);
    delay(100);
    
    brake.begin();
    encoder.begin();
    espnow.begin();
    espnow.onReceive(handleCommand);


    odrive_serial.begin(BAUDRATE, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX);

    Serial.println("Waiting for ODrive...");
    while (odrive.getState() == AXIS_STATE_UNDEFINED) {
        delay(100);
    }

    
    Serial.println("Found ODrive");
    Serial.print("DC voltage: ");
    Serial.println(odrive.getParameterAsFloat("vbus_voltage"));

    Serial.println("Enabling closed loop control...");
    while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrive.clearErrors();
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        delay(10);
    }

    Serial.println("ODrive running!");



    Serial.printf("🔧 MAC: %s\n", WiFi.macAddress().c_str());
    Serial.println("🟢 Arganello ready");
}

void loop() {
    unsigned long now = millis();
    long count = encoder.getCount();

    // Real-time data (200 Hz)
    if (now - lastSend >= RT_INTERVAL_MS) {
        lastSend = now;
        if (count != lastCount) {
            char msg[64];
            snprintf(msg, sizeof(msg), "ENC: %ld", count);
            espnow.sendMessage(msg);
            lastCount = count;
        }
    }

    // Low-priority data (1 Hz)
    if (now - lastPrint >= LOWPRIO_INTERVAL_MS) {
        lastPrint = now;

        // Fetch ODrive feedback
        ODriveFeedback feedback = odrive.getFeedback();
        float voltage = odrive.getParameterAsFloat("vbus_voltage");
        float temperature = odrive.getParameterAsFloat("axis0.fet_thermistor.temperature");
        float current = odrive.getParameterAsFloat("axis0.motor.current_control.Iq_measured");

        Serial.println("🔽 Low-priority stats:");
        Serial.printf("🔄 Count: %ld\n", count);
        Serial.printf("📍 Position: %.2f turns\n", feedback.pos);
        Serial.printf("🚀 Velocity: %.2f turns/s\n", feedback.vel);
        Serial.printf("🔋 Voltage: %.2f V\n", voltage);
        Serial.printf("🌡️ Temp: %.2f °C\n", temperature);
        Serial.printf("⚡ Current: %.2f A\n", current);
        Serial.println();
    }
}

