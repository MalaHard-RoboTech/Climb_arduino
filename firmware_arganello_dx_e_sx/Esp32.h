#pragma once
#include <esp_now.h>
#include <WiFi.h>

#define MESSAGE_SIZE 20

typedef struct {
  char text[MESSAGE_SIZE];
} Message;

class Esp32 {
public:
    typedef void (*RecvCallback)(const char*);

    Esp32(const uint8_t* peerMac);
    void begin();
    void sendMessage(const char* msg);
    void onReceive(RecvCallback cb);
    void sendTelemetryBack(uint8_t primaryLimit);

    Message sendMsg;

    static constexpr uint8_t PRIMARY_VS_SECONDARY_FEEDBACK_RATIO = 10;

private:
    uint8_t mac[6];
    RecvCallback userCallback;
    uint8_t primaryCounter;
    uint8_t secondaryIndex;

    static Esp32* instance;

    static void onDataRecvStatic(const uint8_t *mac, const uint8_t *data, int len);
    static void onDataSentStatic(const uint8_t *mac_addr, esp_now_send_status_t status);

    static void sendEncoder();
    static void sendVbusVoltage();
    static void sendMotorTemp();
    static void sendIqCurrent();

    static void (*primaryTelemetry[])();
    static void (*secondaryTelemetry[])();
    static const uint8_t NUM_PRIMARY;
    static const uint8_t NUM_SECONDARY;
    char lastProcessedCommand[MESSAGE_SIZE];

};
