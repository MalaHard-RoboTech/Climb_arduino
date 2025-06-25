#include <ODriveUART.h>
#include <HardwareSerial.h>
//Change to HW serial in order to communicate with ESP32
////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// ESP32 has 3 UARTs: Serial (USB), Serial1, Serial2
// We'll use Serial1 (UART1) on custom pins

#define ODRIVE_RX 17 // ESP32 RX (connect to ODrive TX)
#define ODRIVE_TX 16 // ESP32 TX (connect to ODrive RX)
#define BAUDRATE 115200 // Must match the ODrive config

HardwareSerial odrive_serial(1); // Use UART1

ODriveUART odrive(odrive_serial);

void setup() {
  // Start communication with ODrive
  odrive_serial.begin(BAUDRATE, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX);

  // Debug serial
  Serial.begin(115200);
  delay(10);

  Serial.println("Waiting for ODrive...");
  while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    delay(100);
  }

  Serial.println("Found ODrive");
  
  Serial.print("DC voltage: ");
  Serial.println(odrive.getParameterAsFloat("vbus_voltage"));
  
  Serial.println("Enabling closed loop control...");
  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }

  Serial.println("ODrive running!");
}

void loop() {
  float SINE_PERIOD = 2.0f; // seconds
  float t = 0.001 * millis();
  float phase = t * (TWO_PI / SINE_PERIOD);

  odrive.setPosition(
    sin(phase),                      // position
    cos(phase) * (TWO_PI / SINE_PERIOD) // velocity feedforward
  );

  ODriveFeedback feedback = odrive.getFeedback();
  Serial.print("pos:");
  Serial.print(feedback.pos);
  Serial.print(", ");
  Serial.print("vel:");
  Serial.print(feedback.vel);
  Serial.println();

}
