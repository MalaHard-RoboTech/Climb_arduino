/*
README — ESP32-S3 Servo + BTS7960 Motor Tester (Software-PWM Motor)
===================================================================

What this does
--------------
- Drives TWO servo valves on GPIO 35 and 36 using your ServoValve class (bit-banged 50 Hz).
- Drives ONE DC motor via a BTS7960 (RPWM=GPIO 37, LPWM=GPIO 38) using your *software-PWM* Motor class.
- Serial command console to set angles and motor duty.

Requirements
------------
- Arduino IDE with "esp32 by Espressif Systems" installed.
  * File → Preferences → Additional Boards Manager URLs:
      https://dl.espressif.com/dl/package_esp32_index.json
  * Tools → Board → ESP32 Arduino → **ESP32S3 Dev Module**
- Files in your project:
  * ServoValve.h / ServoValve.cpp  (0–90° mapping, .begin(), .setAngle(), .sendFrame())
  * Motor.h / Motor.cpp            (begin(), setFrequency(hz), set(val), stop(), update())

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
- status       → print current angles and motor command
- help         → reprint help

Notes
-----
- The two servo frames are sent sequentially; each frame blocks ~20 ms (bit-bang). Because of this,
  the software PWM for the motor should be kept relatively LOW frequency (e.g., 200–500 Hz) so the
  average duty remains reasonable despite blocking.
- If you later convert ServoValve to LEDC (non-blocking), you can raise motor PWM frequency.
*/

#include <Arduino.h>
#include "ServoValve.h"
#include "Motor.h"

// ======== Pins (adjust if needed) ========
static constexpr uint8_t SERVO1_PIN = 35;
static constexpr uint8_t SERVO2_PIN = 36;
static constexpr uint8_t RPWM_PIN   = 37;   // BTS7960 RPWM
static constexpr uint8_t LPWM_PIN   = 38;   // BTS7960 LPWM

// ======== Objects ========
ServoValve ServoValve1(SERVO1_PIN);
ServoValve ServoValve2(SERVO2_PIN);

// Software-PWM Motor: default to 200 Hz to tolerate blocking servo frames
Motor motor(RPWM_PIN, LPWM_PIN, /*pwmHz*/1000);

// ======== Simple line reader ========
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

void setup() {
  Serial.begin(115200);
  delay(150);

  // --- Valves ---
  ServoValve1.begin();
  ServoValve2.begin();

  // Move both valves to 0° at startup (send a few frames so they actually move)
  ServoValve1.setAngle(0);
  ServoValve2.setAngle(0);
  for (int i = 0; i < 5; ++i) {
    ServoValve1.sendFrame();
    ServoValve2.sendFrame();
    delay(20);
  }

  // --- Motor (software PWM) ---
  motor.begin();
  motor.set(0.0f); // ensure stopped

  Serial.println("Ready.");
  printHelp();
  Serial.print("> ");
}

void loop() {
  // ===== Parse commands =====
  if (readLine(line)) {
    String cmd = line; cmd.trim();
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
      // Generic frequency: "mf 200"
      String arg = cmd.substring(2);
      arg.trim();
      uint32_t hz = (uint32_t)arg.toInt();
      if (hz < 100) hz = 100;       // sane floor for software PWM here
      if (hz > 2000) hz = 2000;     // keep moderate due to blocking
      motor.setFrequency(hz);
      Serial.printf("Motor PWM set to %u Hz.\n", hz);

    } else if (low.startsWith("m")) {
      // Accept: m-1, m0, m1, m0.25, etc.
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

    } else if (low == "help" || low == "?") {
      printHelp();

    } else if (low.length() > 0) {
      Serial.println("Unknown. Type 'help'.");
    }

    line = "";
    Serial.print("> ");
  }

  // ===== Run software PWM for motor as often as possible =====
  motor.update();

  // ===== Refresh valves (bit-bang one 20 ms frame each) =====
  // Note: Each sendFrame() busy-waits ~20 ms. This blocks loop(), so keep motor PWM freq moderate.
  ServoValve1.sendFrame();
  ServoValve2.sendFrame();

  // One extra update call after long blocking frames (optional)
  motor.update();
}
