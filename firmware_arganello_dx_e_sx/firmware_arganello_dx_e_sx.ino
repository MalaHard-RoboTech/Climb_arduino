#include "Encoder.h"
#include "Brake.h"
#include "Esp32.h"

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
    if (strcmp(cmd, "on") == 0) {
        brake.setRelay(false);
        Serial.println("🔴 Relay OFF");
    } else if (strcmp(cmd, "off") == 0) {
        brake.setRelay(true);
        Serial.println("🟢 Relay ON");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("🟢 Brake Ready");

    brake.begin();
    encoder.begin();
    espnow.begin();
    espnow.onReceive(handleCommand);

    Serial.printf("🔧 MAC: %s\n", WiFi.macAddress().c_str());
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
        Serial.printf("🔄 Count: %ld\n", count);
    }
}
