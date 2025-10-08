/*
README — ESP32-S3 Servo + BTS7960 Motor Tester (Software-PWM Motor)
===================================================================

What this does
--------------
- Drives TWO servo valves on GPIO 35 and 36 using your ServoValve class (bit-banged 50 Hz).
- Drives ONE DC motor via a BTS7960 (RPWM=GPIO 37, LPWM=GPIO 38) using your *software-PWM* Motor class.
- Serial + ESP-NOW command console to set angles and motor duty.
- 100 Hz ESP-NOW telemetry sender: epoch_ms,<imu1_csv_wo_nl>,<imu2_csv_wo_nl>

Requirements
------------
- Arduino IDE with "esp32 by Espressif Systems" installed.
  * File → Preferences → Additional Boards Manager URLs:
      https://dl.espressif.com/dl/package_esp32_index.json
  * Tools → Board → ESP32 Arduino → **ESP32S3 Dev Module**
- Files in your project:
  * ServoValve.h / ServoValve.cpp  (0–90° mapping, .begin(), .setAngle(), .sendFrame())
  * Motor.h / Motor.cpp            (begin(), setFrequency(hz), set(val), stop(), update())
  * Movella.h / Movella.cpp        (imu.begin(...), .update(), .printCSV(Print&))
  * EspNow.h / EspNow.cpp          (from our previous step)

Wiring (default pins)
---------------------
- ServoValve1 signal → GPIO 35
- ServoValve2 signal → GPIO 36
- BTS7960 RPWM → GPIO 37
- BTS7960 LPWM → GPIO 38
- Common GND between ESP32-S3, servos, and BTS7960 power.

Serial Commands
---------------
- s1 <deg>     → set valve1 angle in degrees (0..90)
- s2 <deg>     → set valve2 angle in degrees (0..90)
- m<val>       → motor command in [-1..1], e.g. m-1, m0, m0.25, m1
- mf <hz>      → set motor PWM to arbitrary frequency (e.g., "mf 200")
- mstop        → stop motor (0 duty)
- status       → print current angles, motor command, espnow tx count
- help         → reprint help
*/

#include <Arduino.h>
#include <WiFi.h>
#include "ServoValve.h"
#include "Motor.h"
#include "Movella.h"
#include "EspNow.h"
#include <esp_mac.h>  // at top, with other includes

// ── Peer (of dongle) MAC address — CHANGE THIS ───────────────────────────────────
uint8_t DONGLE_MAC[6] = { 0x50, 0x78, 0x7D, 0x16, 0xA9, 0x0C };

// ── Pins (adjust if needed) ───────────────────────────────────────────────────
static constexpr uint8_t SERVO1_PIN = 35;
static constexpr uint8_t SERVO2_PIN = 36;
static constexpr uint8_t RPWM_PIN   = 37;   // BTS7960 RPWM
static constexpr uint8_t LPWM_PIN   = 38;   // BTS7960 LPWM

// ── Objects ───────────────────────────────────────────────────────────────────
ServoValve ServoValve1(SERVO1_PIN);
ServoValve ServoValve2(SERVO2_PIN);

// Software-PWM Motor (keep low-moderate due to blocking servo frames)
Motor motor(RPWM_PIN, LPWM_PIN, /*pwmHz*/1000);

// Movella IMUs on two UARTs
HardwareSerial Xsens1(2);  // UART2: RX=16, TX=17
HardwareSerial Xsens2(1);  // UART1: RX=5,  TX=4
Movella imu1(Xsens1, 1);
Movella imu2(Xsens2, 2);

// ── Simple line reader for Serial ─────────────────────────────────────────────
String line;
bool readLine(String& out) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { out.trim(); return true; }
    out += c;
  }
  return false;
}

void printHelp() {
  Serial.println(F(
    "Commands:\n"
    "  s1 <deg>   - set valve1 angle (0..90)\n"
    "  s2 <deg>   - set valve2 angle (0..90)\n"
    "  m<val>     - motor in [-1..1], e.g. m-1, m0, m0.25, m1\n"
    "  mf <hz>    - set motor PWM to <hz>, e.g. 'mf 200'\n"
    "  mstop      - stop motor\n"
    "  status     - print current state\n"
    "  help       - show this help\n"
  ));
}

// ── Helpers & forward declarations ────────────────────────────────────────────
// Sink that satisfies Stream& (required by Movella::printCSV)
struct StringStreamSink : public Stream {
  String s;

  // ---- Stream required methods (not used for input) ----
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}

  // ---- Write (used by printCSV) ----
  size_t write(uint8_t c) override { s += char(c); return 1; }
  size_t write(const uint8_t* b, size_t n) override {
    s.reserve(s.length() + n);
    for (size_t i = 0; i < n; ++i) s += char(b[i]);
    return n;
  }

  void clear() { s.remove(0); }
};


static inline void rstrip_nl(String& s) {
  while (s.endsWith("\n") || s.endsWith("\r")) s.remove(s.length() - 1);
}

// Shared command parser (Serial + ESP-NOW)
void handleCommandLine(const String& cmd);

// Build one combined CSV line:
// epoch_ms,<imu1_csv_wo_nl>,<imu2_csv_wo_nl>\n
String buildDualImuCsv();

// 100 Hz TX task pinned to APP CPU (keeps timing despite blocking in loop)
void EspNowTxTask(void* arg);

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1500);

  // --- Valves ---
  ServoValve1.begin();
  ServoValve2.begin();

  ServoValve1.setAngle(0);
  ServoValve2.setAngle(0);
  for (int i = 0; i < 5; ++i) {
    ServoValve1.sendFrame();
    ServoValve2.sendFrame();
    delay(20);
  }

  // --- Motor ---
  motor.begin();
  motor.set(0.0f);

  // --- IMUs ---
  imu1.begin(115200, 16, 17);
  imu2.begin(115200, 5, 4);

  // --- ESP-NOW ---
  WiFi.mode(WIFI_STA);                 // required
  uint8_t localMac[6] = {0};
  esp_read_mac(localMac, ESP_MAC_WIFI_STA);

  // ⚠️ Initialize ESP-NOW and set the command callback BEFORE starting the TX task
  bool espnow_ok = EspNow_init(DONGLE_MAC);
  Serial.printf(
    "[ESP-NOW] local=%02X:%02X:%02X:%02X:%02X:%02X  "
    "peer=%02X:%02X:%02X:%02X:%02X:%02X  init_ok=%s\n",
    localMac[0],localMac[1],localMac[2],localMac[3],localMac[4],localMac[5],
    DONGLE_MAC[0],DONGLE_MAC[1],DONGLE_MAC[2],DONGLE_MAC[3],DONGLE_MAC[4],DONGLE_MAC[5],
    espnow_ok ? "true" : "false"
  );

  if (espnow_ok) {
    EspNow_setCommandCallback(handleCommandLine);

    // Launch 100 Hz telemetry TX task ONLY if init succeeded
    BaseType_t ok = xTaskCreatePinnedToCore(
        EspNowTxTask, "espnow_tx", 4096, nullptr, 2, nullptr, APP_CPU_NUM);
    if (ok != pdPASS) Serial.println("[ESP-NOW] ERROR: TX task create failed!");
  } else {
    Serial.println("[ESP-NOW] init failed — TX task not started.");
  }

  Serial.println("Ready.");
  printHelp();
  Serial.print("> ");
}
// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // Parse Serial commands
  if (readLine(line)) {
    String cmd = line; cmd.trim();
    handleCommandLine(cmd);  // shared parser
    line = "";
    Serial.print("> ");
  }

  // Pump ESP-NOW RX queue → dispatches to handleCommandLine()
  EspNow_loop();

  // Run software PWM for motor as often as possible
  motor.update();

  // Refresh valves (bit-bang one ~20 ms frame each)
  // NOTE: These calls block; the 100 Hz TX task keeps running anyway.
  ServoValve1.sendFrame();
  ServoValve2.sendFrame();

  // One extra motor.update() after long blocking frames (optional)
  motor.update();

  // IMU CSV is handled by the 100 Hz TX task; avoid double-printing here.
  // if (imu1.update()) imu1.printCSV(Serial);
  // if (imu2.update()) imu2.printCSV(Serial);
}

// ── Command parser used by Serial *and* ESP-NOW ───────────────────────────────
void handleCommandLine(const String& in) {
  String cmd = in; cmd.trim();
  String low = cmd; low.toLowerCase();

  if (low.startsWith("s1")) {
    float v = cmd.substring(2).toFloat();
    ServoValve1.setAngle(v);
    Serial.printf("Valve1 -> %.1f deg\n", v);

  } else if (low.startsWith("s2")) {
    float v = cmd.substring(2).toFloat();
    ServoValve2.setAngle(v);
    Serial.printf("Valve2 -> %.1f deg\n", v);

  } else if (low == "mstop") {
    motor.stop();
    Serial.println("Motor stopped.");

  } else if (low.startsWith("mf ")) {
    String arg = cmd.substring(2);
    arg.trim();
    uint32_t hz = (uint32_t)arg.toInt();
    if (hz < 100) hz = 100;       // sane floor for software PWM here
    if (hz > 2000) hz = 2000;     // keep moderate due to blocking
    motor.setFrequency(hz);
    Serial.printf("Motor PWM set to %u Hz.\n", hz);

  } else if (low.startsWith("m")) {
    String arg = cmd.substring(1);
    arg.replace(" ", "");
    if (arg.length() == 0) {
      Serial.println("Usage: m<val>  where val in [-1..1]");
    } else {
      float v = arg.toFloat();
      motor.set(v);
      Serial.printf("Motor -> %.3f\n", v);
    }

  } else if (low == "status") {
    Serial.println("--- STATUS ---");
    Serial.println("(Angles are whatever you last set; ServoValve stores them internally.)");
    Serial.printf("Motor duty cmd: %.3f\n", motor.lastCommand());
    Serial.printf("ESP-NOW tx_count: %lu\n", (unsigned long)EspNow_txCount());

  } else if (low == "help" || low == "?") {
    printHelp();

  } else if (low.length() > 0) {
    Serial.println("Unknown. Type 'help'.");
  }
}

// ── Build dual-IMU CSV line ───────────────────────────────────────────────────
String buildDualImuCsv() {
  // Update both IMUs; collect their CSV into strings without trailing newline.
  imu1.update();
  imu2.update();

  StringStreamSink p1, p2;
  imu1.printCSV(p1);
  imu2.printCSV(p2);
  rstrip_nl(p1.s);
  rstrip_nl(p2.s);

  // epoch_ms is millis() here (not absolute Unix time)
  uint32_t ms = millis();

  String out;
  out.reserve(16 + p1.s.length() + p2.s.length() + 4);
  out += String(ms);
  out += ",";
  out += p1.s;
  out += ",";
  out += p2.s;
  out += "\n";
  return out;
}

// ── 100 Hz ESP-NOW TX task ───────────────────────────────────────────────────
void EspNowTxTask(void* arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
  TickType_t next = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&next, period);

    String csv = buildDualImuCsv();
    // Optional: echo to USB for debug
    // Serial.print(csv);

    // Send over ESP-NOW (silently ignore if not initialized)
    EspNow_send(csv);
  }
}
