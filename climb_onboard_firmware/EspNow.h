#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_idf_version.h>

// === Public API ===
bool EspNow_init(const uint8_t peer_mac[6]);   // setup WiFi STA, esp_now, register callbacks
bool EspNow_send(const String& line);          // send a String to peer
void EspNow_loop();                            // poll internal RX queue and dispatch to callback

// Command callback that your main will set (e.g., handleCommandLine)
typedef void (*EspNowCommandCallback)(const String& cmd);
void EspNow_setCommandCallback(EspNowCommandCallback cb);

// Stats
uint32_t EspNow_txCount();
