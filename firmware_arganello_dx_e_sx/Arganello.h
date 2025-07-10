#pragma once
#include "Esp32.h"

class Arganello {
public:
    Arganello(Esp32& esp);
    void begin(const char* prefix);
    void handleCommand(const char* data);

private:
    Esp32& espnow;
    char lastProcessedCommand[MESSAGE_SIZE];
    char nodePrefix[5];  // Enough to hold "dx:" or "sx:" plus null terminator
};
