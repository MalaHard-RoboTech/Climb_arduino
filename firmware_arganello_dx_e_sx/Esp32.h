#pragma once
#include <esp_now.h>
#include <WiFi.h>

class Esp32 {
public:
    typedef void (*RecvCallback)(const char*);

    Esp32(const uint8_t* peerMac);
    void begin();
    void sendMessage(const char* msg);
    void onReceive(RecvCallback cb);

private:
    uint8_t mac[6];
    RecvCallback userCallback;

    static Esp32* instance;
    static void onDataRecvStatic(const uint8_t *mac, const uint8_t *data, int len);
    static void onDataSentStatic(const uint8_t *mac_addr, esp_now_send_status_t status);
};
