#include "Esp32.h"
#include "Encoder.h"
#include "Brake.h"
#include <ODriveUART.h>
#include <cstring>  // for memcpy, strlen

extern Encoder encoder;
extern ODriveUART odrive;

Esp32* Esp32::instance = nullptr;

const uint8_t Esp32::NUM_PRIMARY = 1;
const uint8_t Esp32::NUM_SECONDARY = 3;

void (*Esp32::primaryTelemetry[])() = { Esp32::sendEncoder };
void (*Esp32::secondaryTelemetry[])() = {
    Esp32::sendVbusVoltage,
    Esp32::sendMotorTemp,
    Esp32::sendIqCurrent
};

Esp32::Esp32(const uint8_t* peerMac) {
    memcpy(mac, peerMac, 6);
    instance = this;
    userCallback = nullptr;
    primaryCounter = 0;
    secondaryIndex = 0;
    lastProcessedCommand[0] = '\0';
    sendMsg.text[0] = '\0';
}

void Esp32::begin() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW init failed");
        return;
    }

    esp_now_register_send_cb(onDataSentStatic);
    esp_now_register_recv_cb(onDataRecvStatic);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("❌ Failed to add peer");
    }
}

void Esp32::sendMessage(const char* msg) {
    esp_now_send(mac, (const uint8_t*)msg, strlen(msg) + 1);
}

void Esp32::onReceive(RecvCallback cb) {
    userCallback = cb;
}

void Esp32::onDataRecvStatic(const uint8_t *mac, const uint8_t *data, int len) {
    if (instance && instance->userCallback) {
        instance->userCallback((const char*)data);
    }
}

void Esp32::onDataSentStatic(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Optional debug
}

void Esp32::sendEncoder() {
    snprintf(instance->sendMsg.text, MESSAGE_SIZE, "ENC: %ld", encoder.getCount());
}

void Esp32::sendVbusVoltage() {
    snprintf(instance->sendMsg.text, MESSAGE_SIZE, "VBUS: %.1f", odrive.getParameterAsFloat("vbus_voltage"));
}

void Esp32::sendMotorTemp() {
    snprintf(instance->sendMsg.text, MESSAGE_SIZE, "TEMP: %.1f", odrive.getParameterAsFloat("motor.temp_motor"));
}

void Esp32::sendIqCurrent() {
    snprintf(instance->sendMsg.text, MESSAGE_SIZE, "CUR: %.1f", odrive.getParameterAsFloat("motor.current_control.Iq_measured"));
}

void Esp32::sendTelemetryBack(uint8_t primaryLimit) {
    if (primaryCounter < primaryLimit) {
        Serial.println("🔄 [Telemetry] Sending primary data...");
        primaryTelemetry[0]();
        Serial.printf("🧮 Encoder count: %ld\n", encoder.getCount());
        Serial.printf("📤 Prepared: '%s'\n", sendMsg.text);
        primaryCounter++;
    } else {
        Serial.print("📡 [Telemetry] Sending secondary data: ");
        switch (secondaryIndex) {
            case 0: Serial.print("VBUS → "); break;
            case 1: Serial.print("TEMP → "); break;
            case 2: Serial.print("CUR → "); break;
        }

        secondaryTelemetry[secondaryIndex]();
        Serial.printf("📤 Prepared: '%s'\n", sendMsg.text);
        secondaryIndex = (secondaryIndex + 1) % NUM_SECONDARY;
        primaryCounter = 0;
    }

    esp_err_t result = esp_now_send(mac, (uint8_t*)&sendMsg, sizeof(Message));
    Serial.printf("📤 Sent: '%s' | Status: %s\n", sendMsg.text, result == ESP_OK ? "✅ OK" : "❌ FAIL");
}
