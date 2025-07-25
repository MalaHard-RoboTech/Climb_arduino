#include "Encoder.h"
#include "Brake.h"
#include <ODriveUART.h>

#define ODRIVE_RX 17
#define ODRIVE_TX 16
#define RELAY_PIN 4
#define ENCODER_A 5
#define ENCODER_B 6

Encoder encoder(ENCODER_A, ENCODER_B);
Brake brake(RELAY_PIN);

HardwareSerial& odrive_serial = Serial1;  // UART1
ODriveUART odrive(odrive_serial);

String inputBuffer;

void setup() {
    Serial.begin(115200);
    odrive_serial.begin(921600, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX);

    brake.begin();
    encoder.begin();

    Serial.println("✅ Arganello ready");
}

void loop() {
    // For testing: continuously show encoder velocity every 100ms
    static unsigned long lastPrint = 0;
    unsigned long now = millis();

    if (now - lastPrint >= 2) {
        lastPrint = now;
        handleCommand("get_encoder_velocity");
    }

    // Still allow manual commands via Serial if needed
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            handleCommand(inputBuffer);
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }
}


void handleCommand(const String& cmd) {
    if (cmd == "get_encoder_counts") {
        Serial.println(encoder.getCount());
    }
    else if (cmd == "get_encoder_velocity") {
        Serial.println(encoder.getVelocity(), 3);  // Print with 3 decimal places
    }
    else if (cmd == "get_brake_status") {
        Serial.println(brake.isBrakeEngaged() ? "1" : "0");
    }
    else if (cmd.startsWith("set_brake ")) {
        bool engage = cmd.endsWith("1");
        brake.setBrakeEngaged(engage);
        Serial.println("OK");
    }
    else if (cmd.startsWith("send_odrive ")) {
        String line = cmd.substring(strlen("send_odrive "));  // Remove prefix

        // Send raw passthrough command
        if (line.startsWith("p ")) {
            odrive_serial.println(line.substring(2));  // Directly print to UART
            Serial.println("OK");
            return;
        }

        // Parse "w axis0.param value"
        int firstSpace = line.indexOf(' ');
        int secondSpace = line.indexOf(' ', firstSpace + 1);

        if (firstSpace < 0 || secondSpace < 0) {
            Serial.println("ERR Invalid send_odrive format");
            return;
        }

        String op = line.substring(0, firstSpace);
        String path = line.substring(firstSpace + 1, secondSpace);
        String value = line.substring(secondSpace + 1);

        if (op == "w") {
            odrive.setParameter(path, value);
            Serial.println("OK");
        } else {
            Serial.println("ERR Unsupported op");
        }
    }
    else if (cmd.startsWith("read_odrive ")) {
        String line = cmd.substring(strlen("read_odrive "));  // Ex: "r axis0.encoder.vel_estimate"

        int spacePos = line.indexOf(' ');
        if (spacePos < 0) {
            Serial.println("ERR Invalid read_odrive format");
            return;
        }

        String op = line.substring(0, spacePos);
        String path = line.substring(spacePos + 1);

        if (op == "r") {
            String result = odrive.getParameterAsString(path);
            Serial.println(result);
        } else {
            Serial.println("ERR Unsupported op");
        }
    }
    else {
        Serial.println("ERR Unknown command");
    }
}
