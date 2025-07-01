#include "Dongle.h"

DongleController* DongleController::instance = nullptr;

DongleController::DongleController(const uint8_t* mac_dx, const uint8_t* mac_sx) {
  memcpy(macDx, mac_dx, 6);
  memcpy(macSx, mac_sx, 6);
  instance = this;
}

void DongleController::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onDataSentStatic);
  esp_now_register_recv_cb(onDataRecvStatic);

  // Add DX peer
  memcpy(peerInfoDx.peer_addr, macDx, 6);
  peerInfoDx.channel = 0;
  peerInfoDx.encrypt = false;
  esp_now_add_peer(&peerInfoDx);

  // Add SX peer
  memcpy(peerInfoSx.peer_addr, macSx, 6);
  peerInfoSx.channel = 0;
  peerInfoSx.encrypt = false;
  esp_now_add_peer(&peerInfoSx);

  Serial.print("🔧 Dongle MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("✅ Type 'dx:<cmd>' or 'sx:<cmd>' to send commands.");
}

void DongleController::handleSerialInput() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.startsWith("dx:")) {
    String cmd = input.substring(3);
    sendCommandToDX(cmd.c_str());
  } else if (input.startsWith("sx:")) {
    String cmd = input.substring(3);
    sendCommandToSX(cmd.c_str());
  } else {
    Serial.println("⚠️ Invalid. Use 'dx:<cmd>' or 'sx:<cmd>' only.");
  }
}

void DongleController::sendCommandToDX(const char* cmd) {
  strncpy(sendMsg.text, cmd, MAX_TEXT_LENGTH);
  sendMsg.text[MAX_TEXT_LENGTH - 1] = '\0';

  Serial.print("📤 → DX: ");
  Serial.println(sendMsg.text);

  esp_err_t result = esp_now_send(macDx, (uint8_t*)&sendMsg, sizeof(sendMsg));
  if (result != ESP_OK) {
    Serial.println("❌ Failed to send to DX");
  }
}

void DongleController::sendCommandToSX(const char* cmd) {
  strncpy(sendMsg.text, cmd, MAX_TEXT_LENGTH);
  sendMsg.text[MAX_TEXT_LENGTH - 1] = '\0';

  Serial.print("📤 → SX: ");
  Serial.println(sendMsg.text);

  esp_err_t result = esp_now_send(macSx, (uint8_t*)&sendMsg, sizeof(sendMsg));
  if (result != ESP_OK) {
    Serial.println("❌ Failed to send to SX");
  }
}

void DongleController::onDataSentStatic(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (instance) instance->onDataSent(mac_addr, status);
}

void DongleController::onDataRecvStatic(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (instance) instance->onDataRecv(mac, incomingData, len);
}

void DongleController::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("📤 Stato invio: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "✅ OK" : "❌ Failed");
}

void DongleController::onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&recvMsg, incomingData, sizeof(recvMsg));

  const char* sender = "❓ Unknown";
  if (memcmp(mac, macDx, 6) == 0) {
    sender = "📟 Arganello DX";
  } else if (memcmp(mac, macSx, 6) == 0) {
    sender = "📟 Arganello SX";
  }

  Serial.print("📨 From ");
  Serial.print(sender);
  Serial.print(" [");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print("]: ");
  Serial.println(recvMsg.text);
}
