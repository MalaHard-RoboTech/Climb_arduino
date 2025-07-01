#include "Dongle.h"

DongleController* DongleController::instance = nullptr;

DongleController::DongleController(const uint8_t* peer_mac) {
  memcpy(peerMac, peer_mac, 6);
  instance = this;
}

void DongleController::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Errore inizializzazione ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSentStatic);
  esp_now_register_recv_cb(onDataRecvStatic);

  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Errore nell'aggiunta peer");
  }

  Serial.print("🔧 MAC dongle: ");
  Serial.println(WiFi.macAddress());
  Serial.println("✅ Pronto. Digita un comando nella seriale.");
}

void DongleController::handleSerialInput() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (
    input == "on" || input == "off" || input == "stop" ||
    input == "idle" || input == "closed_loop" ||
    input.startsWith("torque:") ||
    input.startsWith("vel:") ||
    input.startsWith("pos:")
  ) {
    sendCommand(input.c_str());
  } else {
    Serial.println("⚠️ Comando non valido. Usa 'on', 'off', 'stop', 'idle', 'closed_loop', 'torque:x', 'vel:x', 'pos:x'.");
  }
}

void DongleController::sendCommand(const char* cmd) {
  strncpy(sendMsg.text, cmd, MAX_TEXT_LENGTH);
  sendMsg.text[MAX_TEXT_LENGTH - 1] = '\0';

  Serial.print("📤 Inviando comando all'Arganello: ");
  Serial.println(sendMsg.text);

  esp_err_t result = esp_now_send(peerMac, (uint8_t*)&sendMsg, sizeof(sendMsg));
  if (result != ESP_OK) {
    Serial.println("❌ Errore durante l'invio");
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
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "✅ Riuscito" : "❌ Fallito");
}

void DongleController::onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&recvMsg, incomingData, sizeof(recvMsg));
  Serial.print("📨 Risposta da Arganello [");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print("]: ");
  Serial.println(recvMsg.text);
}
