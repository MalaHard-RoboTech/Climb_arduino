#include <Arduino.h>
#include <ArduinoJson.h>     // v7+
#include <ODriveUART.h>
#include <stdlib.h>          // strtoull
#include "Brake.h"

// ──────────────────────────────────────────────────────────────────────────────
// Hardware / Serial
// ──────────────────────────────────────────────────────────────────────────────
#define ODRIVE_RX 17
#define ODRIVE_TX 16
#define RELAY_PIN 4

static const uint32_t USB_BAUD    = 1000000;  // USB CDC to PC
static const uint32_t ODRIVE_BAUD = 1000000;  // UART to ODrive

HardwareSerial& odrive_serial = Serial1;
ODriveUART odrive(odrive_serial);
Brake brake(RELAY_PIN);

// ──────────────────────────────────────────────────────────────────────────────
// Field schema
// ──────────────────────────────────────────────────────────────────────────────
enum class Src : uint8_t { ODRIVE, BRAKE };
enum class Typ : uint8_t { FLOAT_, INT_, BOOL_ };

struct Field {
  String name;   // CSV header name
  Src    source; // ODRIVE | BRAKE
  Typ    type;   // float | int | bool
  String path;   // ODrive path when source==ODRIVE
};

// Schema store
static Field  fields[64];
static size_t n_fields = 0;

// Cache of latest values (one per field)
static String field_cache[64];

// ──────────────────────────────────────────────────────────────────────────────
// CONFIG state
// ──────────────────────────────────────────────────────────────────────────────
static String   last_config_json = "";
static bool     config_received  = false;
static uint32_t rate_hz          = 200;
static bool     include_timestamp  = true;
static bool     include_last_reply = true;
static bool     print_header_once  = true;

static uint32_t loop_interval_us = 1000000UL / 200;
static uint32_t last_tick_us     = 0;

// Last command reply (optional CSV column)
static String last_reply = "";

// ──────────────────────────────────────────────────────────────────────────────
// SYNC state (PC sends: "sync <epoch>")
// ──────────────────────────────────────────────────────────────────────────────
static bool     sync_active     = false;
static uint64_t sync_epoch_ms   = 0;
static uint32_t sync_millis0    = 0;

// ──────────────────────────────────────────────────────────────────────────────
// Helpers
// ──────────────────────────────────────────────────────────────────────────────
static void appendUint64(String& out, uint64_t v) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%llu", (unsigned long long)v);
  out += buf;
}

static String trim_copy(const String& s) { String t = s; t.trim(); return t; }

static Src parseSrc(const char* s) {
  if (!s) return Src::ODRIVE;
  return (strcmp(s, "brake") == 0) ? Src::BRAKE : Src::ODRIVE;
}
static Typ parseTyp(const char* s) {
  if (!s) return Typ::FLOAT_;
  if (strcmp(s, "int")  == 0) return Typ::INT_;
  if (strcmp(s, "bool") == 0) return Typ::BOOL_;
  return Typ::FLOAT_;
}

static void clearSchema() { n_fields = 0; }
static bool addField(const Field& f) {
  if (n_fields >= (sizeof(fields) / sizeof(fields[0]))) return false;
  fields[n_fields++] = f;
  return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// CONFIG ingestion
// ──────────────────────────────────────────────────────────────────────────────
static bool applyConfigJSON(const String& jsonLine) {
  StaticJsonDocument<4096> doc;
  DeserializationError err = deserializeJson(doc, jsonLine);
  if (err) {
    last_reply = String("ERR CONFIG: ") + err.c_str();
    Serial.println(last_reply);
    return false;
  }

  rate_hz            = doc["rate_hz"]            | 200;
  include_timestamp  = doc["include_timestamp"]  | true;
  include_last_reply = doc["include_last_reply"] | true;
  loop_interval_us   = (rate_hz > 0) ? (1000000UL / rate_hz) : 5000UL;

  clearSchema();
  JsonArray arr = doc["fields"];
  if (arr.isNull()) {
    last_reply = "ERR CONFIG: fields missing";
    Serial.println(last_reply);
    return false;
  }

  for (JsonObject f : arr) {
    Field x;
    const char* name = f["name"]   | "";
    const char* src  = f["source"] | "odrive";
    const char* typ  = f["type"]   | "float";
    const char* pth  = f["path"]   | "";

    x.name   = name;
    x.source = parseSrc(src);
    x.type   = parseTyp(typ);

    if (x.source == Src::ODRIVE) {
      if (pth[0] == '\0') {
        last_reply = "ERR CONFIG: odrive field missing path";
        Serial.println(last_reply);
        return false;
      }
      x.path = pth;
    }
    if (!addField(x)) {
      last_reply = "ERR CONFIG: too many fields";
      Serial.println(last_reply);
      return false;
    }
  }

  last_config_json  = jsonLine;
  config_received   = true;
  print_header_once = true;
  last_reply = "OK CONFIG";
  Serial.println(last_reply);
  return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// SYNC ingestion
// ──────────────────────────────────────────────────────────────────────────────
static String applySyncCommand(const String& arg, bool is_ns_hint, bool is_ms_hint) {
  String s = arg; s.trim();
  if (s.endsWith("ms")) { s.remove(s.length()-2); is_ms_hint = true; is_ns_hint = false; }
  else if (s.endsWith("ns")) { s.remove(s.length()-2); is_ns_hint = true; is_ms_hint = false; }

  uint64_t val = strtoull(s.c_str(), nullptr, 10);
  bool treat_as_ns = is_ns_hint;
  if (!is_ns_hint && !is_ms_hint) {
    treat_as_ns = (s.length() >= 16);
  }

  uint64_t val_ms = treat_as_ns ? (val / 1000000ULL) : val;
  sync_epoch_ms = val_ms;
  sync_millis0  = millis();
  sync_active   = true;

  String msg = "OK SYNC ";
  msg += String((unsigned long long)val_ms);
  Serial.println(msg);
  return msg;
}

// ──────────────────────────────────────────────────────────────────────────────
// Command handler
// ──────────────────────────────────────────────────────────────────────────────
static String handleCommand(const String& raw) {
  String line = trim_copy(raw);
  if (!line.length()) return "";

  // SYNC commands
  if (line.startsWith("sync_ns ")) return applySyncCommand(line.substring(8), true, false);
  if (line.startsWith("sync_ms ")) return applySyncCommand(line.substring(8), false, true);
  if (line.startsWith("sync "))    return applySyncCommand(line.substring(5), false, false);

  // CONFIG
  if (line.startsWith("CONFIG ")) return applyConfigJSON(line.substring(7)) ? "OK CONFIG" : last_reply;
  if (line == "GET_CONFIG") { Serial.println(last_config_json); return "OK"; }
  if (line == "PING")       { Serial.println("PONG"); return "PONG"; }

  // Brake control
  if (line.startsWith("set_brake ")) {
    char c = line.charAt(line.length() - 1);
    if (c == '0' || c == '1') { brake.setBrakeEngaged(c == '1'); return "OK"; }
    return "ERR brake value";
  }

  // ODrive writes
  if (line.startsWith("send_odrive w ")) {
    String rest = line.substring(strlen("send_odrive w "));
    int i = rest.indexOf(' ');
    if (i < 0) return "ERR send_odrive w format";
    String path  = trim_copy(rest.substring(0, i));
    String value = trim_copy(rest.substring(i + 1));
    if (!path.length() || !value.length()) return "ERR send_odrive w format";
    odrive.setParameter(path, value);
    return "OK";
  }

  // ODrive passthrough (p/v/c/t/f)
  if (line.startsWith("send_odrive ")) {
    String rest = line.substring(strlen("send_odrive "));
    if (rest.length() >= 2 && rest.charAt(1) == ' ') {
      char cmd = rest.charAt(0);
      String payload = rest.substring(2);
      if (cmd=='p'||cmd=='v'||cmd=='c'||cmd=='t'||cmd=='f') {
        odrive_serial.println(String(cmd) + " " + payload);
        return "OK";
      }
    }
    return "ERR Unknown send_odrive op";
  }

  // ODrive read
  if (line.startsWith("read_odrive r ")) {
    String path = trim_copy(line.substring(strlen("read_odrive r ")));
    if (!path.length()) return "ERR read_odrive format";
    String res = odrive.getParameterAsString(path);
    Serial.println(res);
    return res.length() ? res : "ERR empty";
  }

  return "ERR Unknown";
}

// ──────────────────────────────────────────────────────────────────────────────
// Continuous polling (background updates)
// ──────────────────────────────────────────────────────────────────────────────
static void pollOneField() {
  static size_t poll_index = 0;
  if (n_fields == 0) return;

  Field &f = fields[poll_index];
  String val;

  switch (f.source) {
    case Src::ODRIVE:
      if (f.type == Typ::FLOAT_)
        val = String(odrive.getParameterAsFloat(f.path.c_str()), 6);
      else if (f.type == Typ::INT_)
        val = String(odrive.getParameterAsInt(f.path.c_str()));
      else
        val = odrive.getParameterAsInt(f.path.c_str()) ? "1" : "0";
      break;

    case Src::BRAKE:
      {
        bool b = brake.isBrakeEngaged();
        if (f.type == Typ::FLOAT_) val = b ? "1.0" : "0.0";
        else                       val = b ? "1"   : "0";
      }
      break;
  }

  field_cache[poll_index] = val;
  poll_index = (poll_index + 1) % n_fields;
}

// ──────────────────────────────────────────────────────────────────────────────
// Telemetry output
// ──────────────────────────────────────────────────────────────────────────────
static void printCSVHeaderIfNeeded() {
  if (!print_header_once) return;
  print_header_once = false;

  String hdr; bool first = true;
  if (include_timestamp) { hdr += (sync_active ? "epoch_ms" : "micros"); first = false; }
  for (size_t i = 0; i < n_fields; ++i) {
    hdr += (first ? "" : ",");
    hdr += fields[i].name.length() ? fields[i].name : String("f") + i;
    first = false;
  }
  if (include_last_reply) hdr += ",last_reply";
  Serial.println(hdr);
}

static void printTelemetry(uint32_t now_us) {
  printCSVHeaderIfNeeded();

  String out; out.reserve(256);
  bool first = true;

  // Timestamp
  if (include_timestamp) {
    if (sync_active) {
      uint32_t dt_ms = (uint32_t)(millis() - sync_millis0);
      uint64_t ts_ms = sync_epoch_ms + (uint64_t)dt_ms;
      appendUint64(out, ts_ms);
    } else {
      out += String(now_us);
    }
    first = false;
  }

  // Cached fields
  for (size_t i = 0; i < n_fields; ++i) {
    out += (first ? "" : ",");
    out += field_cache[i];
    first = false;
  }

  // Last reply (optional)
  if (include_last_reply) {
    out += ",\""; out += last_reply; out += "\"";
    last_reply = "";
  }

  Serial.println(out);
}

// ──────────────────────────────────────────────────────────────────────────────
// Main
// ──────────────────────────────────────────────────────────────────────────────
static String rx_buf;

void setup() {
  Serial.begin(USB_BAUD);
  odrive_serial.begin(ODRIVE_BAUD, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX);
  brake.begin();

  Serial.println("READY");
  Serial.println("WAITING_CONFIG");
}

void loop() {
  // 1) Read line-buffered commands from USB
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx_buf.length()) {
        last_reply = handleCommand(rx_buf);
        rx_buf = "";
      }
    } else {
      rx_buf += c;
      if (rx_buf.length() > 1024) rx_buf = ""; // safety
    }
  }

  // 2) Background polling (update one field per loop iteration)
  if (config_received) {
    pollOneField();

    // 3) Telemetry tick
    uint32_t now_us = micros();
    if ((uint32_t)(now_us - last_tick_us) >= loop_interval_us) {
      last_tick_us += loop_interval_us;
      printTelemetry(now_us);
    }
  }
}
