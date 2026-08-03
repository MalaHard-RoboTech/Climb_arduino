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
- THR,f1,...,f6 / t1..t6 → command per-EDF thrust in newtons (datasheet map)
- PROP_COMMAND,f1,f2,f3,f4 → ROS/high-level per-EDF thrust in newtons (datasheet map)
- umax/yumax <N>            → pitch/yaw maximum force per active EDF
- status                    → print force, throttle, PWM and controller state
- help                      → reprint help
*/

#include <Arduino.h>
#include <WiFi.h>
#include "ServoValve.h"
#include "Motor.h"
#include "Movella.h"
#include "EspNow.h"
#include <esp_mac.h>  // at top, with other includes

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ── Peer (of dongle) MAC address — CHANGE THIS ───────────────────────────────────
uint8_t DONGLE_MAC[6] = { 0x50, 0x78, 0x7D, 0x16, 0xA9, 0x0C };

// Set to -1 if positive IMU pitch means nose-up.
// Set to +1 if positive IMU pitch means nose-down.
static constexpr float PITCH_PID_OUTPUT_SIGN = -1.0f;

static constexpr float YAW_PID_OUTPUT_SIGN = 1.0f;

static bool pitchHoldActiveBand = false;
static bool yawHoldActiveBand = false;
static bool  yawHoldInit       = false;
static float yawLastRad        = 0.0f;
static float yawRateFilt       = 0.0f;

// Soglie yaw robuste
volatile float yawDeadOutDeg      = 8.0f;  // parte solo sopra questa soglia
volatile float yawDriftRateDegS   = 1.5f;   // sotto questa velocità considero drift lento
volatile float yawDriftBandDeg    = 30.0f;  // compenso drift solo entro questa banda
volatile float yawDriftAlpha      = 0.0015f;


// Runtime serial mode.
// Default = Arduino Serial Monitor / manual bench mode.
// ROS will switch this at runtime by sending: "ros on".
volatile bool rosSerialBridgeMode = false;

// Default off, otherwise Arduino Serial Monitor gets flooded.
// ROS enables it at runtime.
volatile bool usbTelemetryEnabled = false;

// 100 Hz / 5 = 20 Hz when enabled.
static constexpr uint8_t USB_TELEMETRY_DIV_DEFAULT = 5;
volatile uint8_t usbTelemetryDiv = USB_TELEMETRY_DIV_DEFAULT;
static uint8_t usbTelemCounter = 0;

// Keep false for the USB-serial test.
// You can still turn this into a runtime command later if needed.
volatile bool espnowTelemetryEnabled = false;

// ── Pins (adjust if needed) ───────────────────────────────────────────────────
static constexpr uint8_t SERVO1_PIN = 35;
static constexpr uint8_t SERVO2_PIN = 36;
static constexpr uint8_t RPWM_PIN   = 37;   // BTS7960 RPWM
static constexpr uint8_t LPWM_PIN   = 38;   // BTS7960 LPWM

// Thruster pins: questi sono quelli verificati nel tester LEDC su ESP32-S3.
static constexpr uint8_t PIN_T1 = 12;
static constexpr uint8_t PIN_T2 = 13;
static constexpr uint8_t PIN_T3 = 14;
static constexpr uint8_t PIN_T4 = 15;
static constexpr uint8_t PIN_T5 = 39;   // top pitch thruster
static constexpr uint8_t PIN_T6 = 21;   // bottom pitch thruster

// LEDC channels: devono essere espliciti e unici, altrimenti i PWM si copiano tra pin.
static constexpr uint8_t CH_T1 = 0;
static constexpr uint8_t CH_T2 = 1;
static constexpr uint8_t CH_T3 = 2;
static constexpr uint8_t CH_T4 = 3;
static constexpr uint8_t CH_T5 = 4;
static constexpr uint8_t CH_T6 = 5;

// ESC pulse values
static constexpr int ESC_MIN_US  = 1000;
static constexpr int ESC_MAX_US  = 2000;
static constexpr int ESC_STOP_US = 1000;

// PWM settings for ESCs
static constexpr uint32_t ESC_PWM_FREQ_HZ   = 50;
static constexpr uint8_t  ESC_PWM_RES_BITS  = 14;
static constexpr uint32_t ESC_PWM_PERIOD_US = 1000000UL / ESC_PWM_FREQ_HZ;

static inline float clampf(float x, float a, float b) {
  return x < a ? a : (x > b ? b : x);
}

// ── QX-MOTOR QF2611PRO 3100KV datasheet thrust map ──────────────────────────
// Per-EDF catalogue map at 24 V. The 51 entries are obtained by piecewise-linear
// interpolation between the manufacturer operating points:
//   0.50 ->  4.810 N
//   0.60 ->  6.180 N
//   0.70 ->  7.350 N
//   0.80 ->  9.610 N
//   0.90 -> 11.870 N
//   1.00 -> 14.030 N
//
// The table covers every command from 0.50 to 1.00 with a 0.01 increment.
// 0 N means ESC stop (1000 us). A positive force request below 4.810 N is
// raised to the first mapped point because the lower range is not characterised.
static constexpr size_t THRUST_MAP_SIZE = 51;

static constexpr float THRUST_COMMAND_MAP[THRUST_MAP_SIZE] = {
  0.50f, 0.51f, 0.52f, 0.53f, 0.54f, 0.55f,
  0.56f, 0.57f, 0.58f, 0.59f, 0.60f, 0.61f,
  0.62f, 0.63f, 0.64f, 0.65f, 0.66f, 0.67f,
  0.68f, 0.69f, 0.70f, 0.71f, 0.72f, 0.73f,
  0.74f, 0.75f, 0.76f, 0.77f, 0.78f, 0.79f,
  0.80f, 0.81f, 0.82f, 0.83f, 0.84f, 0.85f,
  0.86f, 0.87f, 0.88f, 0.89f, 0.90f, 0.91f,
  0.92f, 0.93f, 0.94f, 0.95f, 0.96f, 0.97f,
  0.98f, 0.99f, 1.00f
};

static constexpr float THRUST_FORCE_MAP_N[THRUST_MAP_SIZE] = {
  4.810f, 4.947f, 5.084f, 5.221f, 5.358f, 5.495f,
  5.632f, 5.769f, 5.906f, 6.043f, 6.180f, 6.297f,
  6.414f, 6.531f, 6.648f, 6.765f, 6.882f, 6.999f,
  7.116f, 7.233f, 7.350f, 7.576f, 7.802f, 8.028f,
  8.254f, 8.480f, 8.706f, 8.932f, 9.158f, 9.384f,
  9.610f, 9.836f, 10.062f, 10.288f, 10.514f, 10.740f,
  10.966f, 11.192f, 11.418f, 11.644f, 11.870f, 12.086f,
  12.302f, 12.518f, 12.734f, 12.950f, 13.166f, 13.382f,
  13.598f, 13.814f, 14.030f
};

static constexpr float THRUST_MIN_CHARACTERISED_N = 4.810f;
static constexpr float THRUST_MAX_FORCE_N         = 14.030f;

static_assert(THRUST_MAP_SIZE == 51, "Unexpected thrust-map size");

static float forceToThrottle(float forceN) {
  if (!isfinite(forceN) || forceN <= 0.0f) {
    return 0.0f;
  }

  if (forceN <= THRUST_FORCE_MAP_N[0]) {
    return THRUST_COMMAND_MAP[0];
  }

  if (forceN >= THRUST_FORCE_MAP_N[THRUST_MAP_SIZE - 1]) {
    return THRUST_COMMAND_MAP[THRUST_MAP_SIZE - 1];
  }

  for (size_t i = 0; i + 1 < THRUST_MAP_SIZE; ++i) {
    const float f0 = THRUST_FORCE_MAP_N[i];
    const float f1 = THRUST_FORCE_MAP_N[i + 1];

    if (forceN <= f1) {
      const float d0 = THRUST_COMMAND_MAP[i];
      const float d1 = THRUST_COMMAND_MAP[i + 1];
      const float alpha = (forceN - f0) / (f1 - f0);
      return d0 + alpha * (d1 - d0);
    }
  }

  return 1.0f;
}

static float throttleToForce(float throttle) {
  if (!isfinite(throttle) || throttle <= 0.0f) {
    return 0.0f;
  }

  if (throttle < THRUST_COMMAND_MAP[0]) {
    // Commands below 0.50 are outside the datasheet map.
    // Return zero rather than inventing a calibrated force value.
    return 0.0f;
  }

  if (throttle == THRUST_COMMAND_MAP[0]) {
    return THRUST_FORCE_MAP_N[0];
  }

  if (throttle >= THRUST_COMMAND_MAP[THRUST_MAP_SIZE - 1]) {
    return THRUST_FORCE_MAP_N[THRUST_MAP_SIZE - 1];
  }

  for (size_t i = 0; i + 1 < THRUST_MAP_SIZE; ++i) {
    const float d0 = THRUST_COMMAND_MAP[i];
    const float d1 = THRUST_COMMAND_MAP[i + 1];

    if (throttle <= d1) {
      const float f0 = THRUST_FORCE_MAP_N[i];
      const float f1 = THRUST_FORCE_MAP_N[i + 1];
      const float alpha = (throttle - d0) / (d1 - d0);
      return f0 + alpha * (f1 - f0);
    }
  }

  return THRUST_MAX_FORCE_N;
}

static inline uint32_t escUsToDuty(int us) {
  if (us < 0) us = 0;
  if (us > (int)ESC_PWM_PERIOD_US) us = ESC_PWM_PERIOD_US;

  const uint32_t maxDuty = (1UL << ESC_PWM_RES_BITS) - 1;
  return (uint32_t)(((uint64_t)us * maxDuty + ESC_PWM_PERIOD_US / 2) / ESC_PWM_PERIOD_US);
}

static inline bool escAttachPwm(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttachChannel(pin, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS, channel);
#else
  ledcSetup(channel, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
  ledcAttachPin(pin, channel);
  return true;
#endif
}

static inline bool escWritePwm(uint8_t channel, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcWriteChannel(channel, duty);
#else
  ledcWrite(channel, duty);
  return true;
#endif
}

// Sostituisce il vecchio EscThruster basato su ESP32Servo.
// Nome diverso per evitare conflitti se EscThruster.h/.cpp restano nella cartella del progetto.
// Mantiene la stessa API usata dal resto del programma: begin(), setThrottle(), stop(), lastThrottle().
class LedcThruster {
public:
  LedcThruster(uint8_t pin,
              uint8_t channel,
              int min_us = ESC_MIN_US,
              int max_us = ESC_MAX_US,
              int stop_us = ESC_STOP_US)
  : pin_(pin),
    channel_(channel),
    minUs_(min_us),
    maxUs_(max_us),
    stopUs_(stop_us),
    currentUs_(stop_us),
    throttle_(0.0f),
    attached_(false) {}

  bool begin() {
    pinMode(pin_, OUTPUT);
    attached_ = escAttachPwm(pin_, channel_);
    stop();
    return attached_;
  }

  // Low-level ESC command. Kept for the onboard PID, which still works
  // internally with its original normalised output.
  void setThrottle(float x) {
    if (!isfinite(x)) x = 0.0f;
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;

    throttle_ = x;
    const int us = minUs_ + static_cast<int>((maxUs_ - minUs_) * throttle_);
    writeUs(us);
  }

  // Physical interface used by ROS/high-level and manual force commands.
  // The lookup converts per-EDF thrust [N] to the normalised ESC command.
  void setForceN(float forceN) {
    setThrottle(forceToThrottle(forceN));
  }

  void stop() {
    throttle_ = 0.0f;
    writeUs(stopUs_);
  }

  void refresh() {
    writeUs(currentUs_);
  }

  void arm(uint32_t arm_ms = 3000, uint32_t period_ms = 20) {
    const uint32_t t0 = millis();
    while ((millis() - t0) < arm_ms) {
      stop();
      delay(period_ms);
    }
  }

  float lastThrottle() const { return throttle_; }
  float lastForceN() const { return throttle_ <= 0.0f ? 0.0f : throttleToForce(throttle_); }
  int lastPulseUs() const { return currentUs_; }
  uint8_t pin() const { return pin_; }
  uint8_t channel() const { return channel_; }
  bool attached() const { return attached_; }

private:
  void writeUs(int us) {
    currentUs_ = us;
    const uint32_t duty = escUsToDuty(currentUs_);
    const bool ok = escWritePwm(channel_, duty);
    if (!ok) {
      Serial.print("ERROR ledcWrite on thruster pin=");
      Serial.print(pin_);
      Serial.print(" ch=");
      Serial.println(channel_);
    }
  }

  uint8_t pin_;
  uint8_t channel_;
  int minUs_;
  int maxUs_;
  int stopUs_;
  int currentUs_;
  float throttle_;
  bool attached_;
};

// ── Objects ───────────────────────────────────────────────────────────────────
ServoValve ServoValve1(SERVO1_PIN);
ServoValve ServoValve2(SERVO2_PIN);

LedcThruster thr1(PIN_T1, CH_T1);
LedcThruster thr2(PIN_T2, CH_T2);
LedcThruster thr3(PIN_T3, CH_T3);
LedcThruster thr4(PIN_T4, CH_T4);
LedcThruster thr5(PIN_T5, CH_T5);   // upper pitch
LedcThruster thr6(PIN_T6, CH_T6);   // lower pitch

// Software-PWM Motor (keep low-moderate due to blocking servo frames)
Motor motor(RPWM_PIN, LPWM_PIN, 1000);

// Movella IMUs on two UARTs
HardwareSerial Xsens1(2);  // UART2: RX=16, TX=17
HardwareSerial Xsens2(1);  // UART1: RX=5,  TX=4
Movella imu1(Xsens1, 1);
Movella imu2(Xsens2, 2);

// ── Thruster timeout ─────────────────
volatile uint32_t lastThrCmdMs = 0;
static constexpr uint32_t THR_TIMEOUT_MS = 500;         // external cyclic commands, e.g. WRC / ROS / ESP-NOW
static constexpr uint32_t MANUAL_THR_TIMEOUT_MS = 5000; // bench commands from Serial: t1..t6, THR, pth
static constexpr uint32_t SERIAL_IDLE_COMMAND_MS = 80;  // accepts Serial Monitor set to "No line ending"

// ── ESP-NOW telemetry health / auto-recovery ─────────────────
volatile bool g_espnowHealthy = false;
volatile uint32_t g_espnowConsecutiveSendFails = 0;
volatile uint32_t g_espnowLastOkTxMs = 0;
volatile uint32_t g_espnowLastReinitMs = 0;
static constexpr uint32_t ESPNOW_REINIT_COOLDOWN_MS = 2000;
static constexpr uint32_t ESPNOW_STALE_MS = 1000;
static constexpr uint32_t ESPNOW_FAIL_THRESHOLD = 5;


// Temporary force-bias convention used by WRC:
//   cmdFx = per-active-EDF normal-axis force bias [N]
//   cmdFy = per-active-EDF lateral force bias [N]
//   cmdMz = per-active-EDF yaw-pair force bias [N], NOT a calibrated N*m moment.
// For a true physical body wrench, use a geometric allocator before sending
// individual EDF forces through PROP_COMMAND.
volatile float cmdFx = 0.0f;
volatile float cmdFy = 0.0f;
volatile float cmdMz = 0.0f;

volatile bool propCtrlEnabled = false;  // WRC force-bias control enabled
volatile bool yawCtlEnabled   = false;  // IMU-based yaw hold enabled
volatile bool manualThrusterMode = false; // true for THR / tN / pth bench commands until timeout

// Direct T1..T4 force mode driven by PROP_COMMAND at cyclic ROS rate.
// It overrides WRC allocation on lateral thrusters but can still coexist with
// the dedicated T5/T6 pitch hold.
volatile bool directLateralForceMode = false;
volatile float directT1ForceN = 0.0f;
volatile float directT2ForceN = 0.0f;
volatile float directT3ForceN = 0.0f;
volatile float directT4ForceN = 0.0f;

// Final thruster configuration (already agreed with wiring):
// T1 = Y1 = CW   , Pack A
// T2 = Y2 = CCW  , Pack A
// T3 = Y3 = CCW  , Pack B
// T4 = Y4 = CW   , Pack B
// T5 = P1 = CW   , Pack A  (pitch +)
// T6 = P2 = CCW  , Pack B  (pitch -)
// Yaw +  => T1 + T3
// Yaw -  => T2 + T4

static constexpr float FX_GAIN = 1.0f;
static constexpr float FY_GAIN = 1.0f;
static constexpr float MZ_GAIN = 1.0f;

// Internal PID thresholds remain normalised to preserve the existing tuning.
// External/manual/high-level commands are expressed in newtons and converted
// through forceToThrottle().
static constexpr float PITCH_MIN_ACTIVE = 0.80f;  // 9.610 N on datasheet map
static constexpr float YAW_MIN_ACTIVE   = 0.55f;  // 6.180 N on datasheet map

static constexpr float PITCH_RAMP_STEP   = 0.10f;
static constexpr float LATERAL_RAMP_STEP = 0.08f;
// assist mode ----------------------------------------------------
volatile bool assistEnabled = false;
volatile uint32_t assistEndMs = 0;
volatile float assistUmax = 1.0f;
volatile float assistKp = 3.0f;
volatile float assistKd = 0.25f;

// ── Pitch hold (onboard, time-critical) ─────────────────────────
volatile bool  pitchCtlEnabled = false;
volatile float pitchRefRad     = 0.0f;
volatile float pitchKp         = 1.2f;
volatile float pitchKi         = 0.0f;
volatile float pitchKd         = 0.05f;
volatile float pitchUmax       = 1.0f;  // 14.030 N on datasheet map
volatile float pitchDeadDeg    = 2.0f;   // "dritto" band
volatile float pitchSafeDeg    = 65.0f;
static float pitchPidI         = 0.0f;
static float pitchPrevE        = 0.0f;

// ── Yaw hold (onboard, time-critical) ───────────────────────────
volatile float yawRefRad       = 0.0f;
volatile float yawKp           = 0.9f;
volatile float yawKd           = 0.04f;
volatile float yawUmax         = 0.70f; // 6.180 N on datasheet map
volatile float yawDeadDeg      = 3.0f;

// Yaw robusto contro drift lento IMU
volatile float yawDeadOutExtraDeg = 8.0f;   // con ydb=10 parte sopra circa 18 deg


static float yawPrevE          = 0.0f;

static float lastPitchCmd      = 0.0f;
static float lastYawCmd        = 0.0f;

static inline float applyMinActiveSigned(float u, float minActive, float maxActive) {
  maxActive = clampf(maxActive, 0.0f, 1.0f);
  minActive = clampf(minActive, 0.0f, maxActive);

  u = clampf(u, -maxActive, maxActive);

  if (fabsf(u) < 1e-5f) {
    return 0.0f;
  }

  float a = fabsf(u);
  if (a < minActive) {
    a = minActive;
  }

  return (u > 0.0f) ? a : -a;
}
// Attitude for propeller stabilization must come from the BODY IMU.
// In alpine_odometry_node.py the mapping is:
//   IMU1 = rope IMU
//   IMU2 = body IMU
// If your physical wiring is the opposite, swap this helper accordingly.
static inline void getAttitudeQuat(float q[4]) {
  imu2.getQuaternion(q);   // body IMU
}

// ── Robust line reader for Serial ─────────────────────────────────────────────
// Works with Arduino Serial Monitor set to Newline, Both NL & CR, or No line ending.
String line;
static uint32_t lastSerialCharMs = 0;
static constexpr size_t SERIAL_MAX_LINE_LEN = 180;

bool readLine(String& out) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    lastSerialCharMs = millis();

    if (c == '\r' || c == '\n') {
      out.trim();

      if (out.length() == 0) {
        out = "";
        return false;
      }

      return true;
    }

    out += c;

    if (out.length() > SERIAL_MAX_LINE_LEN) {
      out = "";

      if (!rosSerialBridgeMode) {
        Serial.println("ERR: serial command too long, buffer cleared.");
      }

      return false;
    }
  }

  // Solo per Arduino Serial Monitor / test manuali.
  // In ROS mode NO: il nodo ROS manda già newline.
  if (!rosSerialBridgeMode &&
      out.length() > 0 &&
      (millis() - lastSerialCharMs) >= SERIAL_IDLE_COMMAND_MS) {
    out.trim();

    if (out.length() == 0) {
      out = "";
      return false;
    }

    return true;
  }

  return false;
}

void printHelp() {
  Serial.println(F(
    "Commands:\n"
    "  s1 <deg>                         - set valve1 angle (0..90)\n"
    "  s2 <deg>                         - set valve2 angle (0..90)\n"
    "  THR,f1,f2,f3,f4,f5,f6           - set all EDF forces [N]\n"
    "  t1 <N> ... t6 <N>               - set one EDF force [N]\n"
    "  pth <signed_N>                   - manual pitch force: +T5 / -T6\n"
    "  PROP_COMMAND,f1,f2,f3,f4        - set T1..T4 forces [N] from ROS\n"
    "  WRC,fxN,fyN,yawPairN            - temporary per-EDF force-bias mixer [N]\n"
    "  pitch / yaw                     - print current attitude angle\n"
    "  apon / apoff                    - enable/disable onboard pitch hold\n"
    "  ayon / ayoff                    - enable/disable onboard yaw hold\n"
    "  atton / attoff                  - enable/disable both pitch+yaw hold\n"
    "  apzero / ayzero / attzero       - set attitude references\n"
    "  pid <Kp> <Ki> <Kd>              - set pitch PID gains (internal normalised PID)\n"
    "  ypid <Kp> <Kd>                  - set yaw PID gains (internal normalised PID)\n"
    "  umax <N>                        - set maximum pitch force per active EDF\n"
    "  yumax <N>                       - set maximum yaw force per active EDF\n"
    "  pdb <deg> / ydb <deg>           - set pitch/yaw deadbands\n"
    "  m<val>                          - motor command in [-1..1]\n"
    "  mf <hz>                         - set motor PWM frequency\n"
    "  mstop                           - stop motor\n"
    "  pron / proff                    - enable/disable lateral force-bias control\n"
    "  ext / ret                       - test motor extend/retract\n"
    "  status                          - print state, force, throttle and PWM\n"
    "  arm / disarm                    - arm / force minimum throttle\n"
    "  stop                            - stop all thrusters\n"
    "  help                            - show this help\n"
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
bool ensureEspNowHealthy();





static inline float wrapPi(float a) {
  while (a >  3.1415926f) a -= 2.0f * 3.1415926f;
  while (a < -3.1415926f) a += 2.0f * 3.1415926f;
  return a;
}

// Euler extraction from quaternion (assume q0=w, q1=x, q2=y, q3=z)
static float pitchFromQuatWXYZ(float w, float x, float y, float z) {
  float sinp = 2.0f * (w*y - z*x);
  sinp = clampf(sinp, -1.0f, 1.0f);
  return asinf(sinp);
}

static float yawFromQuatWXYZ(float w, float x, float y, float z) {
  float siny = 2.0f * (w*z + x*y);
  float cosy = 1.0f - 2.0f * (y*y + z*z);
  return atan2f(siny, cosy);
}
static float pitchPidStep(float pitchRad, float dt) {
  float e = pitchRefRad - pitchRad;

  const float deadIn  = pitchDeadDeg * (3.1415926f / 180.0f);
  const float deadOut = (pitchDeadDeg + 3.0f) * (3.1415926f / 180.0f);

  float ae = fabsf(e);

  if (ae < deadIn) {
    pitchHoldActiveBand = false;
  } else if (ae > deadOut) {
    pitchHoldActiveBand = true;
  }

  if (!pitchHoldActiveBand) {
    pitchPidI = 0.0f;
    pitchPrevE = 0.0f;
    return 0.0f;
  }

  float P = pitchKp * e;
  pitchPidI += pitchKi * e * dt;
  pitchPidI = clampf(pitchPidI, -0.5f, 0.5f);

  float D = pitchKd * (e - pitchPrevE) / dt;
  pitchPrevE = e;

  float u = P + pitchPidI + D;
float uSat = clampf(u, -pitchUmax, pitchUmax);

if (u != uSat) pitchPidI *= 0.95f;

return applyMinActiveSigned(uSat, PITCH_MIN_ACTIVE, pitchUmax);
}

static float yawPidStep(float yawRad, float dt) {
  if (dt <= 1e-5f) {
    return 0.0f;
  }

  if (!yawHoldInit) {
    yawHoldInit = true;
    yawLastRad = yawRad;
    yawRateFilt = 0.0f;
    yawPrevE = 0.0f;
    yawHoldActiveBand = false;
    return 0.0f;
  }

  // Velocità yaw misurata
  float dy = wrapPi(yawRad - yawLastRad);
  yawLastRad = yawRad;

  float yawRate = dy / dt;
  yawRateFilt = 0.92f * yawRateFilt + 0.08f * yawRate;

  float e = wrapPi(yawRefRad - yawRad);

  const float deadIn    = yawDeadDeg * (3.1415926f / 180.0f);
  const float deadOut   = (yawDeadDeg + yawDeadOutExtraDeg) * (3.1415926f / 180.0f);
  const float driftRate = yawDriftRateDegS * (3.1415926f / 180.0f);
  const float driftBand = yawDriftBandDeg * (3.1415926f / 180.0f);

  // Se lo yaw si sposta lentamente e siamo in una banda ragionevole,
  // lo considero drift IMU, non rotazione reale.
  if (!yawHoldActiveBand &&
      fabsf(e) < driftBand &&
      fabsf(yawRateFilt) < driftRate) {

    yawRefRad = wrapPi(yawRefRad - yawDriftAlpha * e);
    e = wrapPi(yawRefRad - yawRad);
  }

  // Deadband con isteresi
  if (fabsf(e) < deadIn) {
    yawHoldActiveBand = false;
    yawPrevE = 0.0f;
    return 0.0f;
  }

  if (fabsf(e) > deadOut) {
    yawHoldActiveBand = true;
  }

  if (!yawHoldActiveBand) {
    yawPrevE = 0.0f;

    // Smorzamento solo se ruota davvero veloce anche nella zona intermedia
    if (fabsf(yawRateFilt) > 3.0f * driftRate) {
      float uDamp = -yawKd * yawRateFilt;
     return applyMinActiveSigned(uDamp, YAW_MIN_ACTIVE, yawUmax);
    }

    return 0.0f;
  }

  // P sulla posizione + D sulla velocità reale
  float u = yawKp * e - yawKd * yawRateFilt;
  u = clampf(u, -yawUmax, yawUmax);

  return applyMinActiveSigned(u, YAW_MIN_ACTIVE, yawUmax);
}

// opzionale: rampetta per non dare step bruschi agli ESC
static float pitchTopCmd = 0.0f;
static float pitchBotCmd = 0.0f;
static float latCmd1 = 0.0f;
static float latCmd2 = 0.0f;
static float latCmd3 = 0.0f;
static float latCmd4 = 0.0f;

static inline float rampTowards(float current, float target, float step) {
  if (target > current + step) return current + step;
  if (target < current - step) return current - step;
  return target;
}

static inline bool isManualThrusterActive(uint32_t now = millis()) {
  return manualThrusterMode && ((now - lastThrCmdMs) <= MANUAL_THR_TIMEOUT_MS);
}

static inline void enterManualThrusterMode() {
  manualThrusterMode = true;
  directLateralForceMode = false;
  directT1ForceN = directT2ForceN = directT3ForceN = directT4ForceN = 0.0f;
  propCtrlEnabled = false;
  yawCtlEnabled = false;
  pitchCtlEnabled = false;
  assistEnabled = false;
  cmdFx = 0.0f;
  cmdFy = 0.0f;
  cmdMz = 0.0f;
  lastThrCmdMs = millis();
}

static inline void setPitchThrustersDirect(float signedForceN) {
  signedForceN = clampf(signedForceN, -THRUST_MAX_FORCE_N, THRUST_MAX_FORCE_N);

  const float topForceN = signedForceN > 0.0f ? signedForceN : 0.0f;
  const float botForceN = signedForceN < 0.0f ? -signedForceN : 0.0f;

  thr5.setForceN(topForceN);
  thr6.setForceN(botForceN);

  pitchTopCmd = thr5.lastThrottle();
  pitchBotCmd = thr6.lastThrottle();
}

static inline bool setSingleThrusterManual(uint8_t number, float forceN) {
  forceN = clampf(forceN, 0.0f, THRUST_MAX_FORCE_N);

  switch (number) {
    case 1:
      thr1.setForceN(forceN);
      latCmd1 = thr1.lastThrottle();
      break;
    case 2:
      thr2.setForceN(forceN);
      latCmd2 = thr2.lastThrottle();
      break;
    case 3:
      thr3.setForceN(forceN);
      latCmd3 = thr3.lastThrottle();
      break;
    case 4:
      thr4.setForceN(forceN);
      latCmd4 = thr4.lastThrottle();
      break;
    case 5:
      thr5.setForceN(forceN);
      pitchTopCmd = thr5.lastThrottle();
      break;
    case 6:
      thr6.setForceN(forceN);
      pitchBotCmd = thr6.lastThrottle();
      break;
    default:
      return false;
  }

  return true;
}

static inline void printThrusterAttachStatus(const char* name, uint8_t pin, uint8_t channel, bool attached) {
  Serial.print("Attach ");
  Serial.print(name);
  Serial.print(" pin=");
  Serial.print(pin);
  Serial.print(" ch=");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.println(attached ? "OK" : "FAIL");
}



static inline float pitchActiveTarget(float x) {
  x = clampf(x, 0.0f, 1.0f);

  const float maxPitch = clampf(pitchUmax, 0.0f, 1.0f);
  const float minPitch = clampf(PITCH_MIN_ACTIVE, 0.0f, maxPitch);

  if (x <= 1e-5f) {
    return 0.0f;
  }

  if (x < minPitch) {
    x = minPitch;
  }

  return clampf(x, 0.0f, maxPitch);
}

static inline void setPitchThrusters(float pitchCmd) {
  pitchCmd = clampf(pitchCmd, -1.0f, 1.0f);

  const float maxPitch = clampf(pitchUmax, 0.0f, 1.0f);
  const float minPitch = clampf(PITCH_MIN_ACTIVE, 0.0f, maxPitch);

  float t5Target = 0.0f;
  float t6Target = 0.0f;

  if (pitchCmd > 0.0f) {
    t5Target = pitchActiveTarget(pitchCmd);
  } else if (pitchCmd < 0.0f) {
    t6Target = pitchActiveTarget(-pitchCmd);
  }

  if (t5Target > 0.0f) {
    pitchBotCmd = 0.0f;

    if (pitchTopCmd < minPitch) {
      pitchTopCmd = minPitch;
    } else {
      pitchTopCmd = rampTowards(pitchTopCmd, t5Target, PITCH_RAMP_STEP);
    }

    pitchTopCmd = clampf(pitchTopCmd, minPitch, maxPitch);

  } else if (t6Target > 0.0f) {
    pitchTopCmd = 0.0f;

    if (pitchBotCmd < minPitch) {
      pitchBotCmd = minPitch;
    } else {
      pitchBotCmd = rampTowards(pitchBotCmd, t6Target, PITCH_RAMP_STEP);
    }

    pitchBotCmd = clampf(pitchBotCmd, minPitch, maxPitch);

  } else {
    pitchTopCmd = 0.0f;
    pitchBotCmd = 0.0f;
  }

  thr5.setThrottle(pitchTopCmd);
  thr6.setThrottle(pitchBotCmd);
}

static inline void setLateralThrustersFromWrench(
    float fxForceN,
    float fyForceN,
    float yawPairForceN) {

  fxForceN = clampf(fxForceN, -THRUST_MAX_FORCE_N, THRUST_MAX_FORCE_N);
  fyForceN = clampf(fyForceN, -THRUST_MAX_FORCE_N, THRUST_MAX_FORCE_N);
  yawPairForceN = clampf(yawPairForceN, -THRUST_MAX_FORCE_N, THRUST_MAX_FORCE_N);

  float f1 = 0.0f;
  float f2 = 0.0f;
  float f3 = 0.0f;
  float f4 = 0.0f;

  // Temporary force allocation. Inputs are per-active-EDF thrust requests [N].
  // This is not yet a calibrated body-wrench allocator.
  if (yawPairForceN > 0.0f) {
    f1 += MZ_GAIN * yawPairForceN;
    f3 += MZ_GAIN * yawPairForceN;
  } else if (yawPairForceN < 0.0f) {
    f2 += MZ_GAIN * (-yawPairForceN);
    f4 += MZ_GAIN * (-yawPairForceN);
  }

  // +Fy => T1 + T2, -Fy => T3 + T4.
  if (fyForceN > 0.0f) {
    f1 += FY_GAIN * fyForceN;
    f2 += FY_GAIN * fyForceN;
  } else if (fyForceN < 0.0f) {
    f3 += FY_GAIN * (-fyForceN);
    f4 += FY_GAIN * (-fyForceN);
  }

  // +Fx => T1 + T4, -Fx => T2 + T3.
  if (fxForceN > 0.0f) {
    f1 += FX_GAIN * fxForceN;
    f4 += FX_GAIN * fxForceN;
  } else if (fxForceN < 0.0f) {
    f2 += FX_GAIN * (-fxForceN);
    f3 += FX_GAIN * (-fxForceN);
  }

  f1 = clampf(f1, 0.0f, THRUST_MAX_FORCE_N);
  f2 = clampf(f2, 0.0f, THRUST_MAX_FORCE_N);
  f3 = clampf(f3, 0.0f, THRUST_MAX_FORCE_N);
  f4 = clampf(f4, 0.0f, THRUST_MAX_FORCE_N);

  // Convert the force targets to the original internal throttle representation
  // so the existing smooth ramp remains unchanged.
  const float t1Target = forceToThrottle(f1);
  const float t2Target = forceToThrottle(f2);
  const float t3Target = forceToThrottle(f3);
  const float t4Target = forceToThrottle(f4);

  latCmd1 = rampTowards(latCmd1, t1Target, LATERAL_RAMP_STEP);
  latCmd2 = rampTowards(latCmd2, t2Target, LATERAL_RAMP_STEP);
  latCmd3 = rampTowards(latCmd3, t3Target, LATERAL_RAMP_STEP);
  latCmd4 = rampTowards(latCmd4, t4Target, LATERAL_RAMP_STEP);

  thr1.setThrottle(latCmd1);
  thr2.setThrottle(latCmd2);
  thr3.setThrottle(latCmd3);
  thr4.setThrottle(latCmd4);
}

static inline void stopLateralThrusters() {
  thr1.stop();
  thr2.stop();
  thr3.stop();
  thr4.stop();
  latCmd1 = latCmd2 = latCmd3 = latCmd4 = 0.0f;
}

static inline void stopPitchThrusters() {
  thr5.stop();
  thr6.stop();
  pitchTopCmd = 0.0f;
  pitchBotCmd = 0.0f;
}

static inline void stopAllThrusters() {
  stopLateralThrusters();
  stopPitchThrusters();
}

static inline void armAllThrusters(uint32_t armMs = 3000, uint32_t periodMs = 20) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < armMs) {
    stopAllThrusters();
    delay(periodMs);
  }
}


// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1500);
  
  lastThrCmdMs = millis();

  // --- Valves ---
  ServoValve1.begin();
  ServoValve2.begin();
  
  thr1.begin(); printThrusterAttachStatus("T1", PIN_T1, CH_T1, thr1.attached());
  thr2.begin(); printThrusterAttachStatus("T2", PIN_T2, CH_T2, thr2.attached());
  thr3.begin(); printThrusterAttachStatus("T3", PIN_T3, CH_T3, thr3.attached());
  thr4.begin(); printThrusterAttachStatus("T4", PIN_T4, CH_T4, thr4.attached());
  thr5.begin(); printThrusterAttachStatus("T5", PIN_T5, CH_T5, thr5.attached());
  thr6.begin(); printThrusterAttachStatus("T6", PIN_T6, CH_T6, thr6.attached());

  // Robust ESC arming: send min-throttle for a few seconds at 50 Hz.
  // This avoids the periodic beep / micro-movement state typical of "not armed" ESCs.
  Serial.println("Arming ESCs at minimum throttle...");
  armAllThrusters(3000, 20);
  lastThrCmdMs = millis();
  Serial.println("ESCs armed.");

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

  // Initialize ESP-NOW and start telemetry task even if first init fails.
  g_espnowHealthy = EspNow_init(DONGLE_MAC);
  g_espnowConsecutiveSendFails = 0;
  g_espnowLastOkTxMs = 0;
  g_espnowLastReinitMs = millis();

  Serial.printf(
    "[ESP-NOW] local=%02X:%02X:%02X:%02X:%02X:%02X  "
    "peer=%02X:%02X:%02X:%02X:%02X:%02X  init_ok=%s\n",
    localMac[0],localMac[1],localMac[2],localMac[3],localMac[4],localMac[5],
    DONGLE_MAC[0],DONGLE_MAC[1],DONGLE_MAC[2],DONGLE_MAC[3],DONGLE_MAC[4],DONGLE_MAC[5],
    g_espnowHealthy ? "true" : "false"
  );

  EspNow_setCommandCallback(handleCommandLine);

  // Always launch telemetry TX task; it will self-heal/reinit if ESP-NOW is down.
  BaseType_t ok = xTaskCreatePinnedToCore(
      EspNowTxTask, "espnow_tx", 4096, nullptr, 1, nullptr, APP_CPU_NUM);
  if (ok != pdPASS) Serial.println("[ESP-NOW] ERROR: TX task create failed!");

 
  Serial.println("Ready.");
printHelp();
Serial.print("> ");
}
// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // Parse Serial commands
  //Serial.println("hello!");
 if (readLine(line)) {
  String cmd = line;
  cmd.trim();
  line = "";

  if (cmd.length() > 0) {
    if (!rosSerialBridgeMode) {
      Serial.print("RX: ");
      Serial.println(cmd);
    }

    handleCommandLine(cmd);
  }

  if (!rosSerialBridgeMode) {
    Serial.print("> ");
  }
}

  // Pump ESP-NOW RX queue → dispatches to handleCommandLine()
  EspNow_loop();

  // Safety timeout for externally requested propeller commands.
  // Manual bench commands use a longer timeout so they do not disappear immediately.
  const uint32_t now = millis();
  const bool manualActive = isManualThrusterActive(now);

  if (manualThrusterMode && !manualActive) {
    manualThrusterMode = false;
    stopAllThrusters();
  }

  if (!manualActive && (now - lastThrCmdMs > THR_TIMEOUT_MS)) {
    cmdFx = 0.0f;
    cmdFy = 0.0f;
    cmdMz = 0.0f;

    if (directLateralForceMode) {
      directLateralForceMode = false;
      directT1ForceN = directT2ForceN = directT3ForceN = directT4ForceN = 0.0f;
    }

    if (!yawCtlEnabled && !propCtrlEnabled && !directLateralForceMode) {
      stopLateralThrusters();
    }
    if (!pitchCtlEnabled) {
      stopPitchThrusters();
    }
  }

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
  String cmd = in;
  cmd.trim();
  String low = cmd;
  low.toLowerCase();

  if (low == "ros on" || low == "roson" || low == "bridge on") {
    rosSerialBridgeMode = true;
    usbTelemetryEnabled = true;
    usbTelemetryDiv = USB_TELEMETRY_DIV_DEFAULT;
    usbTelemCounter = 0;

    Serial.printf("OK ros on usbtelem_div=%u\n", usbTelemetryDiv);
    return;

  } else if (low == "ros off" || low == "rosoff" || low == "console") {
    usbTelemetryEnabled = false;
    rosSerialBridgeMode = false;
    usbTelemCounter = 0;

    Serial.println("OK console mode");
    printHelp();
    Serial.print("> ");
    return;

  } else if (low == "usbtelem off" || low == "usbtelem 0" || low == "telem off") {
    usbTelemetryEnabled = false;
    usbTelemCounter = 0;

    Serial.println("OK usbtelem off");
    return;

  } else if (low.startsWith("usbtelem ")) {
    String arg = low.substring(String("usbtelem ").length());
    arg.trim();

    int div = arg.toInt();
    if (div < 1) div = USB_TELEMETRY_DIV_DEFAULT;
    if (div > 100) div = 100;

    usbTelemetryDiv = (uint8_t)div;
    usbTelemetryEnabled = true;
    usbTelemCounter = 0;

    Serial.printf("OK usbtelem div=%u\n", usbTelemetryDiv);
    return;

  } else if (low == "telem on") {
    usbTelemetryEnabled = true;
    usbTelemetryDiv = USB_TELEMETRY_DIV_DEFAULT;
    usbTelemCounter = 0;

    Serial.printf("OK telem on div=%u\n", usbTelemetryDiv);
    return;
  }

  if (low == "ping") {
    Serial.println("pong");

  } else if (low.startsWith("s1")) {
    float v = cmd.substring(2).toFloat();
    ServoValve1.setAngle(v);
    Serial.printf("Valve1 -> %.1f deg\n", v);

  } else if (low.startsWith("s2")) {
    float v = cmd.substring(2).toFloat();
    ServoValve2.setAngle(v);
    Serial.printf("Valve2 -> %.1f deg\n", v);

  } else if (low.startsWith("thr,")) {
    float f1, f2, f3, f4, f5, f6;
    int matched = sscanf(
      cmd.c_str(),
      "THR,%f,%f,%f,%f,%f,%f",
      &f1, &f2, &f3, &f4, &f5, &f6
    );
    if (matched != 6) {
      matched = sscanf(
        cmd.c_str(),
        "thr,%f,%f,%f,%f,%f,%f",
        &f1, &f2, &f3, &f4, &f5, &f6
      );
    }

    if (matched == 6) {
      enterManualThrusterMode();

      f1 = clampf(f1, 0.0f, THRUST_MAX_FORCE_N);
      f2 = clampf(f2, 0.0f, THRUST_MAX_FORCE_N);
      f3 = clampf(f3, 0.0f, THRUST_MAX_FORCE_N);
      f4 = clampf(f4, 0.0f, THRUST_MAX_FORCE_N);
      f5 = clampf(f5, 0.0f, THRUST_MAX_FORCE_N);
      f6 = clampf(f6, 0.0f, THRUST_MAX_FORCE_N);

      thr1.setForceN(f1);
      thr2.setForceN(f2);
      thr3.setForceN(f3);
      thr4.setForceN(f4);
      thr5.setForceN(f5);
      thr6.setForceN(f6);

      latCmd1 = thr1.lastThrottle();
      latCmd2 = thr2.lastThrottle();
      latCmd3 = thr3.lastThrottle();
      latCmd4 = thr4.lastThrottle();
      pitchTopCmd = thr5.lastThrottle();
      pitchBotCmd = thr6.lastThrottle();

      Serial.printf(
        "THR FORCE -> %.3f %.3f %.3f %.3f %.3f %.3f N, manual timeout=%lu ms\n",
        f1, f2, f3, f4, f5, f6,
        (unsigned long)MANUAL_THR_TIMEOUT_MS
      );
    } else {
      Serial.println("ERR: usage THR,f1_n,f2_n,f3_n,f4_n,f5_n,f6_n");
    }

  } else if (low.startsWith("t")) {
    char commandLetter = 0;
    int thrusterNumber = 0;
    float forceN = 0.0f;

    int itemsRead = sscanf(cmd.c_str(), "%c%d %f", &commandLetter, &thrusterNumber, &forceN);
    if (itemsRead != 3) {
      itemsRead = sscanf(cmd.c_str(), "%c%d,%f", &commandLetter, &thrusterNumber, &forceN);
    }

    if (itemsRead == 3 && (commandLetter == 't' || commandLetter == 'T')) {
      enterManualThrusterMode();
      if (setSingleThrusterManual((uint8_t)thrusterNumber, forceN)) {
        lastThrCmdMs = millis();
        Serial.printf(
          "T%d -> %.3f N (throttle %.3f), manual timeout=%lu ms\n",
          thrusterNumber,
          clampf(forceN, 0.0f, THRUST_MAX_FORCE_N),
          forceToThrottle(forceN),
          (unsigned long)MANUAL_THR_TIMEOUT_MS
        );
      } else {
        Serial.println("ERR: use t1..t6, example: t1 6.0");
      }
    } else {
      Serial.println("ERR: usage t1 6.0");
    }

  } else if (low.startsWith("pth ")) {
  float forceN = 0.0f;
  int n = sscanf(cmd.c_str(), "pth %f", &forceN);
  if (n == 1) {
    enterManualThrusterMode();
    stopLateralThrusters();
    setPitchThrustersDirect(forceN);
    lastThrCmdMs = millis();
    Serial.printf(
      "Pitch force -> %.3f N (+T5 / -T6), manual timeout=%lu ms\n",
      clampf(forceN, -THRUST_MAX_FORCE_N, THRUST_MAX_FORCE_N),
      (unsigned long)MANUAL_THR_TIMEOUT_MS
    );
  } else {
    Serial.println("Usage: pth <signed force N>");
  }

  } else if (low.startsWith("prop_command,")) {
  float f1, f2, f3, f4;
  int matched = sscanf(
    cmd.c_str(),
    "PROP_COMMAND,%f,%f,%f,%f",
    &f1, &f2, &f3, &f4
  );
  if (matched != 4) {
    matched = sscanf(
      cmd.c_str(),
      "prop_command,%f,%f,%f,%f",
      &f1, &f2, &f3, &f4
    );
  }

  if (matched == 4) {
    manualThrusterMode = false;
    directLateralForceMode = true;
    propCtrlEnabled = false;
    lastThrCmdMs = millis();

    directT1ForceN = clampf(f1, 0.0f, THRUST_MAX_FORCE_N);
    directT2ForceN = clampf(f2, 0.0f, THRUST_MAX_FORCE_N);
    directT3ForceN = clampf(f3, 0.0f, THRUST_MAX_FORCE_N);
    directT4ForceN = clampf(f4, 0.0f, THRUST_MAX_FORCE_N);

    // Apply immediately; the 100 Hz task will keep refreshing these values.
    thr1.setForceN(directT1ForceN);
    thr2.setForceN(directT2ForceN);
    thr3.setForceN(directT3ForceN);
    thr4.setForceN(directT4ForceN);

    latCmd1 = thr1.lastThrottle();
    latCmd2 = thr2.lastThrottle();
    latCmd3 = thr3.lastThrottle();
    latCmd4 = thr4.lastThrottle();

    Serial.printf(
      "PROP FORCE -> %.3f %.3f %.3f %.3f N\n",
      directT1ForceN,
      directT2ForceN,
      directT3ForceN,
      directT4ForceN
    );
  } else {
    Serial.println("ERR: usage PROP_COMMAND,f1_n,f2_n,f3_n,f4_n");
  }

  } else if (low.startsWith("wrc,")) {
  float fxForceN, fyForceN, yawPairForceN;
  int matched = sscanf(
    cmd.c_str(),
    "WRC,%f,%f,%f",
    &fxForceN, &fyForceN, &yawPairForceN
  );
  if (matched != 3) {
    matched = sscanf(
      cmd.c_str(),
      "wrc,%f,%f,%f",
      &fxForceN, &fyForceN, &yawPairForceN
    );
  }

  if (matched == 3) {
    manualThrusterMode = false;
    directLateralForceMode = false;
    directT1ForceN = directT2ForceN = directT3ForceN = directT4ForceN = 0.0f;
    propCtrlEnabled = true;
    cmdFx = clampf(fxForceN, -THRUST_MAX_FORCE_N, THRUST_MAX_FORCE_N);
    cmdFy = clampf(fyForceN, -THRUST_MAX_FORCE_N, THRUST_MAX_FORCE_N);
    cmdMz = clampf(yawPairForceN, -THRUST_MAX_FORCE_N, THRUST_MAX_FORCE_N);
    lastThrCmdMs = millis();

    setLateralThrustersFromWrench(cmdFx, cmdFy, cmdMz);

    Serial.printf(
      "WRC FORCE BIAS -> Fx=%.3f N Fy=%.3f N yawPair=%.3f N\n",
      cmdFx, cmdFy, cmdMz
    );
  } else {
    Serial.println("ERR: usage WRC,fx_n,fy_n,yaw_pair_force_n");
  }

} else if (low == "stop") {
    manualThrusterMode = false;
    directLateralForceMode = false;
    directT1ForceN = directT2ForceN = directT3ForceN = directT4ForceN = 0.0f;
    pitchCtlEnabled = false;
    yawCtlEnabled = false;
    propCtrlEnabled = false;
    assistEnabled = false;
    cmdFx = 0.0f;
    cmdFy = 0.0f;
    cmdMz = 0.0f;
    stopAllThrusters();
    Serial.println("All thrusters stopped.");

} else if (low == "mstop") {
    motor.stop();
    Serial.println("Motor stopped.");

  } else if (low.startsWith("mf ")) {
    String arg = cmd.substring(2);
    arg.trim();
    uint32_t hz = (uint32_t)arg.toInt();
    if (hz < 100) hz = 100;
    if (hz > 2000) hz = 2000;
    motor.setFrequency(hz);
    Serial.printf("Motor PWM set to %u Hz.\n", hz);

  } else if (low == "ext") {
    pitchCtlEnabled = false;
    motor.set(+0.5f);
    Serial.println("EXT test: motor +0.5 (should extend toward the black ball)");

  } else if (low == "ret") {
    pitchCtlEnabled = false;
    motor.set(-0.5f);
    Serial.println("RET test: motor -0.5 (should retract)");

  } else if (low == "pitch") {
    float q[4];
    getAttitudeQuat(q);
    float pitchRad = pitchFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    float pitchDeg = pitchRad * 180.0f / 3.1415926f;
    Serial.printf("Pitch = %.2f deg\n", pitchDeg);

  } else if (low == "yaw") {
    float q[4];
    getAttitudeQuat(q);
    float yawRad = yawFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    float yawDeg = yawRad * 180.0f / 3.1415926f;
    Serial.printf("Yaw = %.2f deg\n", yawDeg);

  } else if (low == "apon") {
    manualThrusterMode = false;
    pitchCtlEnabled = true;
    pitchPidI = 0.0f;
    pitchPrevE = 0.0f;
    Serial.println("AutoPitch ON (pitch thrusters)");

  } else if (low == "apoff") {
  manualThrusterMode = false;
  pitchCtlEnabled = false;
  assistEnabled = false;
  stopPitchThrusters();
  motor.stop();
  Serial.println("AutoPitch OFF");

  } else if (low == "ayon") {
    manualThrusterMode = false;
    yawCtlEnabled = true;
    yawPrevE = 0.0f;
    Serial.println("AutoYaw ON (lateral thrusters)");

  } else if (low == "ayoff") {
    manualThrusterMode = false;
    yawCtlEnabled = false;
    if (!propCtrlEnabled) stopLateralThrusters();
    Serial.println("AutoYaw OFF");

  } else if (low == "atton") {
    manualThrusterMode = false;
    pitchCtlEnabled = true;
    yawCtlEnabled = true;
    pitchPidI = 0.0f;
    pitchPrevE = 0.0f;
    yawPrevE = 0.0f;
    Serial.println("Attitude hold ON (pitch + yaw)");

  } else if (low == "attoff") {
    manualThrusterMode = false;
    directLateralForceMode = false;
    directT1ForceN = directT2ForceN = directT3ForceN = directT4ForceN = 0.0f;
    pitchCtlEnabled = false;
    yawCtlEnabled = false;
    assistEnabled = false;
    stopAllThrusters();
    Serial.println("Attitude hold OFF");

  } else if (low == "pron") {
  manualThrusterMode = false;
  directLateralForceMode = false;
  directT1ForceN = directT2ForceN = directT3ForceN = directT4ForceN = 0.0f;
  propCtrlEnabled = true;
  Serial.println("Lateral WRC force-bias control ON");
} else if (low == "proff") {
  manualThrusterMode = false;
  directLateralForceMode = false;
  directT1ForceN = directT2ForceN = directT3ForceN = directT4ForceN = 0.0f;
  propCtrlEnabled = false;
  cmdFx = 0.0f;
  cmdFy = 0.0f;
  cmdMz = 0.0f;
  stopLateralThrusters();
  Serial.println("Lateral prop control OFF");
  }else if (low == "apzero") {
    float q[4];
    getAttitudeQuat(q);
    pitchRefRad = pitchFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    pitchPidI = 0.0f;
    pitchPrevE = 0.0f;
    Serial.println("AutoPitch reference set (apzero)");

  } else if (low == "ayzero") {
    float q[4];
    getAttitudeQuat(q);
    yawRefRad = yawFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    yawPrevE = 0.0f;
    yawHoldInit = false;
    yawHoldActiveBand = false;
    yawRateFilt = 0.0f;
    Serial.println("AutoYaw reference set (ayzero)");

  } else if (low == "attzero") {
    float q[4];
    getAttitudeQuat(q);
    pitchRefRad = pitchFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    yawRefRad = yawFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    pitchPidI = 0.0f;
    pitchPrevE = 0.0f;
    yawPrevE = 0.0f;
    yawHoldInit = false;
    yawHoldActiveBand = false;
    yawRateFilt = 0.0f;
    Serial.println("Pitch+Yaw references set from current attitude");

  } else if (low.startsWith("pid ")) {
    float p, i, d;
    int n = sscanf(cmd.c_str(), "pid %f %f %f", &p, &i, &d);
    if (n == 3) {
      pitchKp = p;
      pitchKi = i;
      pitchKd = d;
      Serial.printf("PID set: Kp=%.3f Ki=%.3f Kd=%.3f\n", pitchKp, pitchKi, pitchKd);
    } else {
      Serial.println("Usage: pid <Kp> <Ki> <Kd>");
    }

  } else if (low.startsWith("umax ")) {
    float forceN;
    int n = sscanf(cmd.c_str(), "umax %f", &forceN);
    if (n == 1) {
      forceN = clampf(forceN, 0.0f, THRUST_MAX_FORCE_N);
      pitchUmax = forceToThrottle(forceN);
      Serial.printf(
        "Pitch max force=%.3f N -> uMax=%.3f\n",
        forceN, pitchUmax
      );
    } else {
      Serial.println("Usage: umax <force N>");
    }

  } else if (low.startsWith("ypid ")) {
    float p, d;
    int n = sscanf(cmd.c_str(), "ypid %f %f", &p, &d);
    if (n == 2) {
      yawKp = p;
      yawKd = d;
      Serial.printf("Yaw PID set: Kp=%.3f Kd=%.3f\n", yawKp, yawKd);
    } else {
      Serial.println("Usage: ypid <Kp> <Kd>");
    }

  } else if (low.startsWith("yumax ")) {
    float forceN;
    int n = sscanf(cmd.c_str(), "yumax %f", &forceN);
    if (n == 1) {
      forceN = clampf(forceN, 0.0f, THRUST_MAX_FORCE_N);
      yawUmax = forceToThrottle(forceN);
      Serial.printf(
        "Yaw max force=%.3f N -> uMax=%.3f\n",
        forceN, yawUmax
      );
    } else {
      Serial.println("Usage: yumax <force N>");
    }

  } else if (low.startsWith("pdb ")) {
    float x;
    int n = sscanf(cmd.c_str(), "pdb %f", &x);
    if (n == 1) {
      pitchDeadDeg = clampf(x, 0.0f, 30.0f);
      Serial.printf("Pitch deadband=%.2f deg\n", pitchDeadDeg);
    } else {
      Serial.println("Usage: pdb <deg>");
    }

  } else if (low.startsWith("ydb ")) {
    float x;
    int n = sscanf(cmd.c_str(), "ydb %f", &x);
    if (n == 1) {
      yawDeadDeg = clampf(x, 0.0f, 45.0f);
      Serial.printf("Yaw deadband=%.2f deg\n", yawDeadDeg);
    } else {
      Serial.println("Usage: ydb <deg>");
    }

  } else if (low.startsWith("assist ")) {
    manualThrusterMode = false;
    int ms = 0;
    int n = sscanf(cmd.c_str(), "assist %d", &ms);
    if (n == 1) {
      if (ms < 50) ms = 50;
      if (ms > 2000) ms = 2000;
      assistEnabled = true;
      assistEndMs = millis() + (uint32_t)ms;
      pitchPidI = 0.0f;
      pitchPrevE = 0.0f;
      Serial.printf("Assist ON for %d ms\n", ms);
    } else {
      Serial.println("Usage: assist <ms 50..2000>");
    }

  } else if (low.startsWith("m")) {
    if (pitchCtlEnabled) {
      Serial.println("AutoPitch ON: usa 'apoff' per controllo manuale motore.");
      return;
    }

    String arg = cmd.substring(1);
    arg.replace(" ", "");
    if (arg.length() == 0) {
      Serial.println("Usage: m<val> where val in [-1..1]");
    } else {
      float v = arg.toFloat();
      motor.set(v);
      Serial.printf("Motor -> %.3f\n", v);
    }

  } else if (low == "status") {
  Serial.println("--- STATUS ---");
  Serial.printf("Motor duty cmd: %.3f\n", motor.lastCommand());
  Serial.printf(
    "cmdFx=%.3fN cmdFy=%.3fN cmdYawPair=%.3fN manualThr=%d manualActive=%d directForce=%d\n",
    cmdFx, cmdFy, cmdMz,
    (int)manualThrusterMode,
    (int)isManualThrusterActive(),
    (int)directLateralForceMode
  );
  Serial.printf(
    "direct_T1..T4_N: %.3f %.3f %.3f %.3f\n",
    directT1ForceN,
    directT2ForceN,
    directT3ForceN,
    directT4ForceN
  );
  Serial.printf(
    "THR_TIMEOUT_MS=%lu MANUAL_THR_TIMEOUT_MS=%lu\n",
    (unsigned long)THR_TIMEOUT_MS,
    (unsigned long)MANUAL_THR_TIMEOUT_MS
  );
  Serial.printf(
    "pitchCtl=%d yawCtl=%d propCtrl=%d pitchRef=%.2fdeg yawRef=%.2fdeg pDead=%.2f yDead=%.2f\n",
    (int)pitchCtlEnabled,
    (int)yawCtlEnabled,
    (int)propCtrlEnabled,
    pitchRefRad * 180.0f / 3.1415926f,
    yawRefRad * 180.0f / 3.1415926f,
    pitchDeadDeg,
    yawDeadDeg
  );
  Serial.printf(
    "pitchMax=%.3fN yawMax=%.3fN lastPitchCmd=%.3f lastYawCmd=%.3f\n",
    pitchUmax <= 0.0f ? 0.0f : throttleToForce(pitchUmax),
    yawUmax <= 0.0f ? 0.0f : throttleToForce(yawUmax),
    lastPitchCmd,
    lastYawCmd
  );
  Serial.printf(
    "T_force_N: %.3f %.3f %.3f %.3f %.3f %.3f\n",
    thr1.lastForceN(),
    thr2.lastForceN(),
    thr3.lastForceN(),
    thr4.lastForceN(),
    thr5.lastForceN(),
    thr6.lastForceN()
  );
  Serial.printf(
    "T_throttle: %.3f %.3f %.3f %.3f %.3f %.3f\n",
    thr1.lastThrottle(),
    thr2.lastThrottle(),
    thr3.lastThrottle(),
    thr4.lastThrottle(),
    thr5.lastThrottle(),
    thr6.lastThrottle()
  );
  Serial.printf(
    "T_us: %d %d %d %d %d %d\n",
    thr1.lastPulseUs(),
    thr2.lastPulseUs(),
    thr3.lastPulseUs(),
    thr4.lastPulseUs(),
    thr5.lastPulseUs(),
    thr6.lastPulseUs()
  );
  Serial.printf("ESP-NOW tx_count: %lu\n", (unsigned long)EspNow_txCount());

  } else if (low == "arm") {
    manualThrusterMode = false;
    Serial.println("Arming ESCs...");
    armAllThrusters(3000, 20);
    lastThrCmdMs = millis();
    Serial.println("ESCs armed.");

  } else if (low == "disarm") {
    manualThrusterMode = false;
    directLateralForceMode = false;
    directT1ForceN = directT2ForceN = directT3ForceN = directT4ForceN = 0.0f;
    pitchCtlEnabled = false;
    yawCtlEnabled = false;
    propCtrlEnabled = false;
    assistEnabled = false;
    cmdFx = 0.0f;
    cmdFy = 0.0f;
    cmdMz = 0.0f;
    stopAllThrusters();
    Serial.println("ESCs forced to minimum throttle.");

  } else if (low == "help" || low == "?") {
    printHelp();

  } else if (low.length() > 0) {
    Serial.println("Unknown. Type 'help'.");
  }
}

// ── Build dual-IMU CSV line ───────────────────────────────────────────────────
bool ensureEspNowHealthy() {
  const uint32_t now = millis();

  const bool stale = (g_espnowLastOkTxMs > 0) && ((now - g_espnowLastOkTxMs) > ESPNOW_STALE_MS);
  const bool tooManyFails = (g_espnowConsecutiveSendFails >= ESPNOW_FAIL_THRESHOLD);

  if (g_espnowHealthy && !stale && !tooManyFails) return true;
  if ((now - g_espnowLastReinitMs) < ESPNOW_REINIT_COOLDOWN_MS) return g_espnowHealthy;

  g_espnowLastReinitMs = now;
  Serial.printf("[ESP-NOW] reinit requested (healthy=%s stale=%s fails=%lu)\n",
                g_espnowHealthy ? "true" : "false",
                stale ? "true" : "false",
                (unsigned long)g_espnowConsecutiveSendFails);

  g_espnowHealthy = EspNow_init(DONGLE_MAC);
  EspNow_setCommandCallback(handleCommandLine);

  if (g_espnowHealthy) {
    g_espnowConsecutiveSendFails = 0;
    Serial.println("[ESP-NOW] reinit OK");
  } else {
    Serial.println("[ESP-NOW] reinit FAILED");
  }

  return g_espnowHealthy;
}

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

// ── 100 Hz body control + telemetry task ─────────────────────────────────────
void EspNowTxTask(void* arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
  TickType_t next = xTaskGetTickCount();
  const float dt = 0.01f;

  for (;;) {
    vTaskDelayUntil(&next, period);

    String csv = buildDualImuCsv();

    // Telemetry verso ROS su USB seriale, decimata.
    // 100 Hz / 5 = 20 Hz con USB a 115200.
    if (usbTelemetryEnabled) {
  uint8_t div = usbTelemetryDiv;
  if (div < 1) div = 1;

  usbTelemCounter++;
  if (usbTelemCounter >= div) {
    usbTelemCounter = 0;
    Serial.print(csv);
  }
}

    // Onboard attitude stabilization should live here (time-critical),
    // while high-level ROS only sends bias wrench commands.
    float q[4];
    getAttitudeQuat(q);
    const float pitchRad = pitchFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    const float yawRad   = yawFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    const float pitchDeg = pitchRad * 180.0f / 3.1415926f;

    bool assistActive = assistEnabled && ((int32_t)(millis() - assistEndMs) < 0);
    if (assistEnabled && !assistActive) assistEnabled = false;

    const bool manualActive = isManualThrusterActive();

    if (manualThrusterMode && !manualActive) {
      manualThrusterMode = false;
      stopAllThrusters();
    }

    if (manualActive) {
      // Manual bench commands THR / t1..t6 / pth own the PWM outputs until timeout.
      lastPitchCmd = 0.0f;
      lastYawCmd = 0.0f;

    } else {
      // Pitch hold on dedicated pair T5/T6
      if (pitchCtlEnabled || assistActive) {
        if (fabsf(pitchDeg) > pitchSafeDeg) {
          lastPitchCmd = 0.0f;
          stopPitchThrusters();

        } else if (assistActive) {
          const float e = pitchRefRad - pitchRad;
          const float D = assistKd * (e - pitchPrevE) / dt;
          pitchPrevE = e;
          lastPitchCmd = clampf(assistKp * e + D, -assistUmax, assistUmax);
          setPitchThrusters(lastPitchCmd);

        } else {
          lastPitchCmd = pitchPidStep(pitchRad, dt);
          setPitchThrusters(PITCH_PID_OUTPUT_SIGN * lastPitchCmd);
        }

      } else {
        lastPitchCmd = 0.0f;
        stopPitchThrusters();
      }

      // Yaw hold PID remains internally normalised. Convert its output to the
      // datasheet per-EDF force map before adding it to high-level force bias.
      float yawHoldCmd = 0.0f;
      if (yawCtlEnabled) {
        yawHoldCmd = YAW_PID_OUTPUT_SIGN * yawPidStep(yawRad, dt);
      }

      lastYawCmd = yawHoldCmd;

      float yawHoldForceN = 0.0f;
      if (yawHoldCmd > 0.0f) {
        yawHoldForceN = throttleToForce(yawHoldCmd);
      } else if (yawHoldCmd < 0.0f) {
        yawHoldForceN = -throttleToForce(-yawHoldCmd);
      }

      if (directLateralForceMode) {
        // Direct T1..T4 forces from ROS. The yaw hold, when enabled, is added
        // as a per-EDF pair force and then clamped.
        float f1 = directT1ForceN;
        float f2 = directT2ForceN;
        float f3 = directT3ForceN;
        float f4 = directT4ForceN;

        if (yawHoldForceN > 0.0f) {
          f1 += yawHoldForceN;
          f3 += yawHoldForceN;
        } else if (yawHoldForceN < 0.0f) {
          f2 += -yawHoldForceN;
          f4 += -yawHoldForceN;
        }

        thr1.setForceN(clampf(f1, 0.0f, THRUST_MAX_FORCE_N));
        thr2.setForceN(clampf(f2, 0.0f, THRUST_MAX_FORCE_N));
        thr3.setForceN(clampf(f3, 0.0f, THRUST_MAX_FORCE_N));
        thr4.setForceN(clampf(f4, 0.0f, THRUST_MAX_FORCE_N));

        latCmd1 = thr1.lastThrottle();
        latCmd2 = thr2.lastThrottle();
        latCmd3 = thr3.lastThrottle();
        latCmd4 = thr4.lastThrottle();

      } else {
        const bool lateralActive = propCtrlEnabled || yawCtlEnabled;
        if (lateralActive) {
          setLateralThrustersFromWrench(
            cmdFx,
            cmdFy,
            cmdMz + yawHoldForceN
          );
        } else {
          stopLateralThrusters();
        }
      }
    }

    // ESP-NOW spento per test ROS via USB seriale.
    if (espnowTelemetryEnabled) {
      EspNow_send(csv);
     }
  }
}