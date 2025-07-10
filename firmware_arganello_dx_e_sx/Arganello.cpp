#include "Arganello.h"
#include "Brake.h"
#include "Encoder.h"
#include <ODriveUART.h>
#include <Arduino.h>
#include <cstring>

extern Encoder encoder;
extern ODriveUART odrive;
extern Brake brake;

Arganello::Arganello(Esp32& esp) : espnow(esp) {
    lastProcessedCommand[0] = '\0';
    nodePrefix[0] = '\0';
}

void Arganello::begin(const char* prefix) {
    strncpy(nodePrefix, prefix, sizeof(nodePrefix) - 1);
    nodePrefix[sizeof(nodePrefix) - 1] = '\0';
}

void Arganello::handleCommand(const char* data) {
    Message recvMsg;
    memcpy(&recvMsg, data, sizeof(Message));
    recvMsg.text[MESSAGE_SIZE - 1] = '\0';

    Serial.printf("📨 Received: '%s'\n", recvMsg.text);

    // Ignore if prefix doesn't match
    size_t prefixLen = strlen(nodePrefix);
    if (strncmp(recvMsg.text, nodePrefix, prefixLen) != 0) return;

    // Extract command body
    const char* rawBody = recvMsg.text + prefixLen;
    char cleanBody[18];
    strncpy(cleanBody, rawBody, sizeof(cleanBody) - 1);
    cleanBody[sizeof(cleanBody) - 1] = '\0';

    // Trim trailing spaces
    for (int i = strlen(cleanBody) - 1; i >= 0 && cleanBody[i] == ' '; --i)
        cleanBody[i] = '\0';

    // Compose full command string
    char fullCommand[MESSAGE_SIZE];
    snprintf(fullCommand, sizeof(fullCommand), "%s%s", nodePrefix, cleanBody);

    // Check if it's a new command
    bool isNewCommand = strcmp(fullCommand, lastProcessedCommand) != 0;

    if (isNewCommand) {
        strncpy(lastProcessedCommand, fullCommand, sizeof(lastProcessedCommand));
        Serial.printf("✅ Parsed: '%s'\n", cleanBody);

        // Execute command
        if (strcmp(cleanBody, "brake:on") == 0) {
            brake.setRelay(false);
            Serial.println("🟢 Relay OFF");
        } else if (strcmp(cleanBody, "brake:off") == 0) {
            brake.setRelay(true);
            Serial.println("🔴 Relay ON");
        } else if (strcmp(cleanBody, "idle") == 0) {
            odrive.setState(AXIS_STATE_IDLE);
            Serial.println("⚙️ ODrive IDLE");
        } else if (strcmp(cleanBody, "closed_loop") == 0) {
            odrive.clearErrors();
            odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
            Serial.println("⚙️ ODrive CLOSED_LOOP");
        } else if (strncmp(cleanBody, "pos:", 4) == 0) {
            odrive.setPosition(atof(cleanBody + 4));
        } else if (strncmp(cleanBody, "vel:", 4) == 0) {
            odrive.setVelocity(atof(cleanBody + 4));
        } else if (strncmp(cleanBody, "torque:", 7) == 0) {
            odrive.setTorque(atof(cleanBody + 7));
        }
    }

    // Always send telemetry
    espnow.sendTelemetryBack(Esp32::PRIMARY_VS_SECONDARY_FEEDBACK_RATIO);
}

