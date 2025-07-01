#include "Dongle.h"

uint8_t arganelloDxMac[] = {0xCC, 0xBA, 0x97, 0x14, 0x2C, 0x14};
uint8_t arganelloSxMac[] = {0xCC, 0xBA, 0x97, 0x14, 0x0F, 0x7C};

DongleController dongle(arganelloDxMac, arganelloSxMac);

void setup() {
  Serial.begin(115200);
  Serial.println("🟢 ESP-NOW Dongle in modalità OOP");
  dongle.begin();
}

void loop() {
  dongle.handleSerialInput();
}
