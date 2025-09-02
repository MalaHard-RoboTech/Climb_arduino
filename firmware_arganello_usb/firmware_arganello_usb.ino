#include "Brake.h"
#include <ODriveUART.h>

#define ODRIVE_RX 17
#define ODRIVE_TX 16
#define RELAY_PIN 4

HardwareSerial& odrive_serial = Serial1;
ODriveUART odrive(odrive_serial);

Brake brake(RELAY_PIN);

String inputBuffer;
String lastOdriveResponse = "";


struct Telemetry {

    unsigned long micros;
    bool brake_status;

    float vbus_voltage;
    float motor_temp;

    float input_id;
    float input_iq;

    float input_position;
    float input_velocity;
    float input_torque;

    float motor_torque;
    float motor_pos;
    float motor_vel;

    int32_t sync_roller_raw;
    float sync_roller_pos;
    float sync_roller_vel;
};

Telemetry getAllTelemetry(bool debug_overhead = false) {
    Telemetry t;

    unsigned long t_start = micros();

    // Read all values from ODrive and brake in order
    t.micros = micros();
    t.brake_status     = brake.isBrakeEngaged();

    t.vbus_voltage     = odrive.getParameterAsFloat("vbus_voltage");
    t.motor_temp    = odrive.getParameterAsFloat("axis0.motor.fet_thermistor.temperature");

    t.input_id         = odrive.getParameterAsFloat("axis0.motor.input_id");
    t.input_iq         = odrive.getParameterAsFloat("axis0.motor.input_iq");

    t.input_position   = odrive.getParameterAsFloat("axis0.controller.input_pos");
    t.input_velocity   = odrive.getParameterAsFloat("axis0.controller.input_vel");
    t.input_torque     = odrive.getParameterAsFloat("axis0.controller.input_torque");

    t.motor_torque     = odrive.getParameterAsFloat("axis0.motor.torque_estimate");

    t.motor_pos        = odrive.getParameterAsFloat("encoder_estimator0.pos_estimate");
    t.motor_vel        = odrive.getParameterAsFloat("encoder_estimator0.vel_estimate");


    t.sync_roller_pos   = odrive.getParameterAsFloat("encoder_estimator1.pos_estimate");
    t.sync_roller_vel   = odrive.getParameterAsFloat("encoder_estimator1.vel_estimate");
    t.sync_roller_raw   = odrive.getParameterAsInt("inc_encoder0.raw");

     

    if (debug_overhead) {
        unsigned long t_end = micros();
        unsigned long elapsed = t_end - t_start;

        static unsigned long t_prev = 0;
        float frequency = 0.0;

        if (t_prev > 0 && (t_end > t_prev)) {
            unsigned long delta = t_end - t_prev;
            frequency = 1e6 / float(delta);  // Hz
        }

        t_prev = t_end;

        Serial.print("⚙️ Telemetry fetch time: ");
        Serial.print(elapsed);
        Serial.print(" µs, rate: ");
        Serial.print(frequency, 1);
        Serial.println(" Hz");
    }


    return t;
}


unsigned long lastLoopMicros = 0;
const unsigned long LOOP_INTERVAL_MICROS = 5000;  // 200 Hz

void setup() {
    Serial.begin(1000000);
    odrive_serial.begin(2000000, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX);

    brake.begin();

    Serial.println("✅ Arganello ready");
}

void loop() {
    // 1. Handle incoming USB commands
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            lastOdriveResponse = handleCommand(inputBuffer);
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }

    // 2. Read ODrive + brake telemetry
    Telemetry t = getAllTelemetry(false);

    // 3. Output telemetry at 200 Hz
    unsigned long now = micros();
    if (now - lastLoopMicros >= LOOP_INTERVAL_MICROS) {
        lastLoopMicros += LOOP_INTERVAL_MICROS;

        Serial.print(t.micros);                        Serial.print(",");
        Serial.print(t.brake_status ? "1" : "0");      Serial.print(",");
        Serial.print(t.vbus_voltage, 2);               Serial.print(",");
        Serial.print(t.motor_temp, 1);                 Serial.print(",");
        Serial.print(t.input_id, 3);                   Serial.print(",");
        Serial.print(t.input_iq, 3);                   Serial.print(",");
        Serial.print(t.input_position, 3);             Serial.print(",");
        Serial.print(t.input_velocity, 3);             Serial.print(",");
        Serial.print(t.input_torque, 3);               Serial.print(",");
        Serial.print(t.motor_torque, 3);               Serial.print(",");
        Serial.print(t.motor_pos, 3);                  Serial.print(",");
        Serial.print(t.motor_vel, 3);                  Serial.print(",");
        Serial.print(t.sync_roller_pos, 3);            Serial.print(",");
        Serial.print(t.sync_roller_vel, 3);            Serial.print(",");
        Serial.print(t.sync_roller_raw);               Serial.print(",\"");

        Serial.print(lastOdriveResponse);              Serial.println("\"");

        lastOdriveResponse = "";

    }
}


String handleCommand(const String& cmd) {
    String trimmed = cmd;
    trimmed.trim();

    if (trimmed.startsWith("set_brake ")) {
        char lastChar = trimmed.charAt(trimmed.length() - 1);
        if (lastChar == '0' || lastChar == '1') {
            brake.setBrakeEngaged(lastChar == '1');
            return "OK";
        } else {
            return "ERR Invalid brake value";
        }
    }
    else if (trimmed.startsWith("send_odrive ")) {
        String rest = trimmed.substring(strlen("send_odrive "));

        if (rest.startsWith("p ")) {
            odrive_serial.println(rest.substring(2));
            return "OK";
        }
        else if (rest.startsWith("w ")) {
            int firstSpace = rest.indexOf(' ');
            int secondSpace = rest.indexOf(' ', firstSpace + 1);

            if (firstSpace < 0 || secondSpace < 0) {
                return "ERR Invalid send_odrive format";
            }

            String path = rest.substring(firstSpace + 1, secondSpace);
            String value = rest.substring(secondSpace + 1);

            odrive.setParameter(path, value);
            return "OK";
        }
        else {
            return "ERR Unknown send_odrive op";
        }
    }
    else if (trimmed.startsWith("read_odrive ")) {
        String rest = trimmed.substring(strlen("read_odrive "));
        if (rest.startsWith("r ")) {
            String path = rest.substring(2);
            String result = odrive.getParameterAsString(path);
            return result;
        } else {
            return "ERR Invalid read_odrive format";
        }
    }
    else {
        return "ERR Unknown command";
    }
}
