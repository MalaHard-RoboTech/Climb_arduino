#include <esp_now.h>
#include <WiFi.h>

// ──────── CONFIG ────────
#define RELAY_PIN 4
#define ENCODER_PIN_A 5
#define ENCODER_PIN_B 6

#define ENCODER_SEND_INTERVAL 5
#define DEBUG_PRINT_INTERVAL 1000

uint8_t dongleMac[] = {0xCC, 0xBA, 0x97, 0x14, 0x0A, 0x14};

// ──────── TYPES ────────
typedef struct struct_message {
  char text[200];
} struct_message;

struct_message myDataSend;
struct_message myDataRecv;
esp_now_peer_info_t peerInfo;

// ──────── ENCODER ────────
volatile long encoderCount = 0;
int lastEncoded = 0;
long lastSentCount = 0;

void IRAM_ATTR updateEncoder() {
  int MSB = digitalRead(ENCODER_PIN_A);
  int LSB = digitalRead(ENCODER_PIN_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  // Inverted direction
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderCount--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderCount++;

  lastEncoded = encoded;
}

void initEncoder() {
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);
}

// ──────── RELAY ────────
void initRelay() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Default OFF
}

void setRelay(bool state) {
  digitalWrite(RELAY_PIN, state ? HIGH : LOW);  // Inverted logic
}

// ──────── ESP-NOW ────────
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional debug
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myDataRecv, incomingData, sizeof(myDataRecv));
  Serial.print("📨 Ricevuto comando: ");
  Serial.println(myDataRecv.text);

  if (strcmp(myDataRecv.text, "on") == 0) {
    setRelay(false);  // Inverted: "on" -> relay OFF
    Serial.println("🔴 Relay DISATTIVATO");
  } else if (strcmp(myDataRecv.text, "off") == 0) {
    setRelay(true);   // Inverted: "off" -> relay ON
    Serial.println("🟢 Relay ATTIVATO");
  }
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init fallita");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, dongleMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Errore nell'aggiungere il peer");
    return;
  }
}

// ──────── MAIN ────────
void setup() {
  Serial.begin(115200);
  Serial.println("🟢 Arganello - Pronto");

  initRelay();
  initEncoder();
  initEspNow();

  Serial.print("🔧 MAC: ");
  Serial.println(WiFi.macAddress());
}

// ──────── LOOP ────────
void loop() {
  static unsigned long lastSend = 0;
  static unsigned long lastPrint = 0;

  // Send encoder count if changed
  if (millis() - lastSend >= ENCODER_SEND_INTERVAL) {
    lastSend = millis();
    long currentCount;
    noInterrupts();
    currentCount = encoderCount;
    interrupts();

    if (currentCount != lastSentCount) {
      snprintf(myDataSend.text, sizeof(myDataSend.text), "ENC: %ld", currentCount);
      esp_now_send(dongleMac, (uint8_t *)&myDataSend, sizeof(myDataSend));
      lastSentCount = currentCount;
    }
  }

  // Periodic debug print
  if (millis() - lastPrint >= DEBUG_PRINT_INTERVAL) {
    lastPrint = millis();
    Serial.print("🔄 Encoder count: ");
    Serial.println(encoderCount);
  }
}
