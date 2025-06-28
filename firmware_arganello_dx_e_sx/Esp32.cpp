#include "Esp32.h"

Esp32* Esp32::instance = nullptr;

Esp32::Esp32(const uint8_t* peerMac) {
    memcpy(mac, peerMac, 6);
    instance = this;
    userCallback = nullptr;
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
