#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

// ──────── CONFIG ────────
uint8_t arganelloMac[] = {0xCC, 0xBA, 0x97, 0x14, 0x2C, 0x14};

#define MAX_TEXT_LENGTH 200

// ──────── TYPES ────────
typedef struct struct_message {
  char text[MAX_TEXT_LENGTH];
} struct_message;

struct_message myDataSend;
struct_message myDataRecv;
esp_now_peer_info_t peerInfo;

// ──────── CALLBACKS ────────
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("📤 Stato invio: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "✅ Riuscito" : "❌ Fallito");
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myDataRecv, incomingData, sizeof(myDataRecv));
  Serial.print("📨 Risposta da Arganello [");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print("]: ");
  Serial.println(myDataRecv.text);
}

// ──────── INIT FUNCTIONS ────────
void initWiFiEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Errore inizializzazione ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  memcpy(peerInfo.peer_addr, arganelloMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Errore nell'aggiunta peer");
    return;
  }
}

// ──────── INPUT HANDLER ────────
void handleSerialInput() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input == "on" || input == "off") {
    sendCommand(input.c_str());
  } else {
    Serial.println("⚠️ Comando non valido. Usa solo 'on' o 'off'.");
  }
}

// ──────── SEND FUNCTION ────────
void sendCommand(const char* cmd) {
  strncpy(myDataSend.text, cmd, MAX_TEXT_LENGTH);
  myDataSend.text[MAX_TEXT_LENGTH - 1] = '\0';

  Serial.print("📤 Inviando comando all'Arganello: ");
  Serial.println(myDataSend.text);

  esp_err_t result = esp_now_send(arganelloMac, (uint8_t*)&myDataSend, sizeof(myDataSend));
  if (result != ESP_OK) {
    Serial.println("❌ Errore durante l'invio");
  }
}

// ──────── SETUP & LOOP ────────
void setup() {
  Serial.begin(115200);
  Serial.println("🟢 ESP-NOW Dongle - Comandi ON/OFF");

  initWiFiEspNow();

  Serial.print("🔧 MAC dongle: ");
  Serial.println(WiFi.macAddress());
  Serial.println("✅ Pronto. Digita 'on' o 'off' nella seriale.");
}

void loop() {
  handleSerialInput();
}
