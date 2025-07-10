#include "Encoder.h"
#include "Brake.h"
#include "Esp32.h"
#include "Arganello.h"
#include <ODriveUART.h>
#include <HardwareSerial.h>

#define ODRIVE_RX 17
#define ODRIVE_TX 16
#define BAUDRATE 115200
#define RELAY_PIN 4
#define ENCODER_A 5
#define ENCODER_B 6

uint8_t dongleMac[] = { 0xCC, 0xBA, 0x97, 0x14, 0x0A, 0x14 };

HardwareSerial odrive_serial(1);
ODriveUART odrive(odrive_serial);
Encoder encoder(ENCODER_A, ENCODER_B);
Brake brake(RELAY_PIN);
Esp32 espnow(dongleMac);
Arganello arganello(espnow);

void setup() {
  Serial.begin(115200);
  delay(100);


  brake.begin();
  encoder.begin();
  espnow.begin();
  arganello.begin("sx:");

  espnow.onReceive([](const char* raw) {
    arganello.handleCommand(raw);
  });

  odrive_serial.begin(BAUDRATE, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX);
  Serial.println("⏳ Waiting for ODrive...");
  while (odrive.getState() == AXIS_STATE_UNDEFINED) delay(100);
  Serial.println("✅ Found ODrive");

  odrive.clearErrors();
  odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);

  Serial.printf("🔗 MAC: %s\n", WiFi.macAddress().c_str());
  Serial.println("✅ Arganello ready");
}

void loop() {
  // Everything handled by ESP-NOW callback
}
