#include "Dongle.h"

uint8_t arganelloMac[] = {0xCC, 0xBA, 0x97, 0x14, 0x2C, 0x14};
DongleController dongle(arganelloMac);

void setup() {
  Serial.begin(115200);
  Serial.println("🟢 ESP-NOW Dongle in modalità OOP");
  dongle.begin();
}

void loop() {
  dongle.handleSerialInput();
}
