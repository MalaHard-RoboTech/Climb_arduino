#pragma once
#include <esp_now.h>
#include <WiFi.h>

#define MAX_TEXT_LENGTH 200

typedef struct struct_message {
  char text[MAX_TEXT_LENGTH];
} struct_message;

class DongleController {
public:
  DongleController(const uint8_t* mac_dx, const uint8_t* mac_sx);
  void begin();
  void handleSerialInput();

private:
  struct_message sendMsg;
  struct_message recvMsg;
  esp_now_peer_info_t peerInfoDx;
  esp_now_peer_info_t peerInfoSx;

  uint8_t macDx[6];
  uint8_t macSx[6];

  static DongleController* instance;
  static void onDataSentStatic(const uint8_t *mac_addr, esp_now_send_status_t status);
  static void onDataRecvStatic(const uint8_t *mac, const uint8_t *incomingData, int len);

  void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
  void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
  void sendCommandToDX(const char* cmd);
  void sendCommandToSX(const char* cmd);
};
