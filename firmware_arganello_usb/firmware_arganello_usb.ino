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
    Serial.begin(1000000);
    odrive_serial.begin(921600, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX);

    brake.begin();
    encoder.begin();

    Serial.println("✅ Arganello ready");
}

void loop() {


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
    String trimmed = cmd;
    trimmed.trim();  // Remove \r, \n, spaces

    if (trimmed == "get_encoder_counts") {
        Serial.println((int)encoder.getCount());  // Ensure integer
    }
    else if (trimmed == "get_encoder_velocity") {
        Serial.println(encoder.getVelocity(), 3);
    }
    else if (trimmed == "get_brake_status") {
        Serial.println(brake.isBrakeEngaged() ? "1" : "0");
    }
    else if (trimmed.startsWith("set_brake ")) {
        char lastChar = trimmed.charAt(trimmed.length() - 1);
        if (lastChar == '0' || lastChar == '1') {
            brake.setBrakeEngaged(lastChar == '1');
            Serial.println("OK");
        } else {
            Serial.println("ERR Invalid brake value");
        }
    }
    else if (trimmed.startsWith("send_odrive ")) {
        String rest = trimmed.substring(strlen("send_odrive "));

        if (rest.startsWith("p ")) {
            odrive_serial.println(rest.substring(2));  // Passthrough
            Serial.println("OK");
        }
        else if (rest.startsWith("w ")) {
            int firstSpace = rest.indexOf(' ');
            int secondSpace = rest.indexOf(' ', firstSpace + 1);

            if (firstSpace < 0 || secondSpace < 0) {
                Serial.println("ERR Invalid send_odrive format");
                return;
            }

            String path = rest.substring(firstSpace + 1, secondSpace);
            String value = rest.substring(secondSpace + 1);

            odrive.setParameter(path, value);
            Serial.println("OK");
        }
        else {
            Serial.println("ERR Unknown send_odrive op");
        }
    }
    else if (trimmed.startsWith("read_odrive ")) {
        String rest = trimmed.substring(strlen("read_odrive "));
        if (rest.startsWith("r ")) {
            String path = rest.substring(2);
            String result = odrive.getParameterAsString(path);
            Serial.println(result);
        } else {
            Serial.println("ERR Invalid read_odrive format");
        }
    }
    else {
        Serial.println("ERR Unknown command");
    }
}
