# climb_onboard_firmware

Firmware for the ESP32-S3 onboard controller.  
It drives two servo valves, one BTS7960 motor (software-PWM), and streams dual-IMU telemetry over ESP-NOW.  
Control is possible both via USB Serial and via ESP-NOW commands from the dongle.

---

## Inputs (Commands)

All commands can be sent via Serial console or ESP-NOW.

Command     | Example   | Description
------------|-----------|-----------------------------------------------
s1 <deg>    | s1 45     | Set ServoValve1 angle in degrees (0–90).
s2 <deg>    | s2 30     | Set ServoValve2 angle in degrees (0–90).
m<val>      | m0.5      | Set motor duty cycle in range [-1…1].
mf <hz>     | mf 500    | Set motor PWM frequency (100–2000 Hz).
mstop       | mstop     | Stop motor (duty = 0).
status      | status    | Print current servo angles, motor cmd, tx cnt.
help / ?    | help      | Show command list.

---

## Outputs (Responses)

The firmware prints responses to Serial.  
Examples:

- s1 45 → Valve1 -> 45.0 deg
- s2 30 → Valve2 -> 30.0 deg
- m0.5 → Motor -> 0.500
- m-1 → Motor -> -1.000
- mf 500 → Motor PWM set to 500 Hz.
- mstop → Motor stopped.

Status output:
--- STATUS ---
(Angles are whatever you last set; ServoValve stores them internally.)
Motor duty cmd: 0.500
ESP-NOW tx_count: 1234

Help output:
Commands:
  s1 <deg>   - set valve1 angle (0..90)
  s2 <deg>   - set valve2 angle (0..90)
  m<val>     - motor in [-1..1], e.g. m-1, m0, m0.25, m1
  mf <hz>    - set motor PWM to <hz>, e.g. 'mf 200'
  mstop      - stop motor
  status     - print current state
  help       - show this help

Unknown command:
Unknown. Type 'help'.

---

## Telemetry

Telemetry is sent at **100 Hz** via ESP-NOW.  
Each line is a CSV record with **23 fields**:
epoch_ms,q0,q1,q2,q3,ax,ay,az,gx,gy,gz,id,q0,q1,q2,q3,ax,ay,az,gx,gy,gz,id

Where:

- **epoch_ms** = milliseconds since ESP32 boot.  
- **q0–q3** = IMU orientation quaternion (unitless, normalized).  
- **ax, ay, az** = Linear acceleration in m/s².  
- **gx, gy, gz** = Angular velocity in rad/s.  
- **id** = IMU identifier (e.g., 1 = IMU1, 2 = IMU2).  

Thus:
- `<imu1_csv>` = `q0,q1,q2,q3,ax,ay,az,gx,gy,gz,id` (11 values).  
- `<imu2_csv>` = `q0,q1,q2,q3,ax,ay,az,gx,gy,gz,id` (11 values).  

---
---

## Example Session

> help
Commands:
  s1 <deg>   - set valve1 angle (0..90)
  s2 <deg>   - set valve2 angle (0..90)
  m<val>     - motor in [-1..1], e.g. m-1, m0, m0.25, m1
  mf <hz>    - set motor PWM to <hz>, e.g. 'mf 200'
  mstop      - stop motor
  status     - print current state
  help       - show this help

> s1 30
Valve1 -> 30.0 deg

> m0.5
Motor -> 0.500

> status
--- STATUS ---
(Angles are whatever you last set; ServoValve stores them internally.)
Motor duty cmd: 0.500
ESP-NOW tx_count: 42

---

## Notes

- Servo control (50 Hz) and software PWM are blocking; telemetry runs in a separate FreeRTOS task at a stable 100 Hz.  
- ESP-NOW peer MAC address (DONGLE_MAC) must be set in the code.  
- Ensure common ground between ESP32-S3, BTS7960, servos, and IMUs.


# dongle_espnow_ros2_bridge

Firmware for the ESP32-S3 dongle that acts as a **wireless Serial bridge** between your PC (ROS 2) and the onboard ESP32.  

- Reads commands from **Serial (USB)** at 1,000,000 baud.  
- Sends them immediately to the **onboard ESP32** via **ESP-NOW**.  
- Prints any received telemetry (CSV lines) from the onboard back to **Serial**.  
- If the onboard stops sending telemetry for >50 ms, the dongle automatically **re-sends the last command at 100 Hz** (keep-alive).  

---

## Usage

1. Connect dongle via USB to your PC.  
2. Open Serial Monitor (or ROS 2 node) at **1,000,000 baud**.  
3. Type commands like:

m0.25
s1 45
s2 30
mf 200
mstop

4. Telemetry from the onboard will appear prefixed with `[RX …]`:
[RX CC:BA:97:14:0A:14] 123456,0.9987,0.0123,...,2 


---

## Data Flow

**PC (ROS 2 / Serial) ↔ Dongle (ESP-NOW) ↔ Onboard ESP32**

- **Commands:** PC → Dongle → Onboard  
- **Telemetry:** Onboard → Dongle → PC  

The dongle therefore mirrors the onboard Serial port wirelessly, with an added **100 Hz keep-alive resend** mechanism.







