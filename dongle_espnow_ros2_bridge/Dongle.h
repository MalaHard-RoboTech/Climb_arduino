#pragma once
#include <esp_now.h>
#include <WiFi.h>

#define MAX_TEXT_LENGTH 200

typedef struct struct_message {
  char text[MAX_TEXT_LENGTH];
} struct_message;

class DongleController {
public:
  DongleController(const uint8_t* peer_mac);
  void begin();
  void handleSerialInput();

private:
  struct_message sendMsg;
  struct_message recvMsg;
  esp_now_peer_info_t peerInfo;
  uint8_t peerMac[6];

  static DongleController* instance;
  static void onDataSentStatic(const uint8_t *mac_addr, esp_now_send_status_t status);
  static void onDataRecvStatic(const uint8_t *mac, const uint8_t *incomingData, int len);

  void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
  void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
  void sendCommand(const char* cmd);
};
