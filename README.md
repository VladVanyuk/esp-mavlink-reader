# ESP32-S3 MAVLink Reader

An Arduino/PlatformIO firmware project for reading flight-controller telemetry over MAVLink with an ESP32-S3. The firmware forwards no telemetry to a network service yet; it decodes the incoming MAVLink messages and prints a human-readable report over the ESP32-S3 native USB serial port.

## Features

- ESP32-S3 DevKitC-1 target using the Arduino framework.
- MAVLink UART connection to an ArduPilot, PX4, or other MAVLink flight controller.
- GCS heartbeat transmitted once per second.
- Telemetry stream setup using `MAV_CMD_SET_MESSAGE_INTERVAL`, with legacy data-stream requests as a fallback.
- Stream refresh every 10 seconds.
- Once-per-second serial output for:
  - Flight-controller identity, autopilot, armed state, system state, and custom mode.
  - Attitude.
  - Global position, altitude, velocity, and heading.
  - GPS fix, satellite count, and HDOP.
  - VFR HUD airspeed, groundspeed, heading, throttle, altitude, and climb rate.
  - Battery voltage, current, remaining percentage, and communication drop rate.
  - Flight-controller `STATUSTEXT` messages as they arrive.
- A stale-connection marker after three seconds without a flight-controller heartbeat.

## Hardware and wiring

Use a 3.3 V UART and connect the grounds:

| ESP32-S3 | Flight controller |
| --- | --- |
| GPIO17 (TX) | TELEM RX |
| GPIO18 (RX) | TELEM TX |
| GND | GND |

The default flight-controller UART speed is `57600` baud. Configure the selected ArduPilot `SERIALx` port or PX4 TELEM port to use MAVLink at the same speed. Do not connect a 5 V UART signal directly to the ESP32-S3.

## Requirements

- VS Code with PlatformIO, or the PlatformIO CLI.
- ESP32-S3 DevKitC-1.
- A MAVLink-compatible flight controller and a 3.3 V serial connection.
- The ESP32-S3 USB port labeled **USB** for native USB serial. The firmware is configured with USB CDC enabled at boot.

## Build and upload

From the project directory:

```sh
pio run
pio run --target upload
pio device monitor
```

The monitor uses `115200` baud. The firmware upload speed is configured as `921600` in `platformio.ini`.

The MAVLink library dependency is installed automatically by PlatformIO:

```text
okalachev/MAVLink@^2.0.33
```

## Expected startup output

After opening the serial monitor, the board reports its UART configuration and waits for a valid flight-controller heartbeat:

```text
ESP32-S3 MAVLink2 telemetry bridge
FC UART: RX=18 TX=17 baud=57600
Debug output: USB Serial @ 115200
Waiting for flight controller...
```

Once a heartbeat is received, the firmware identifies the flight controller and requests telemetry. Reports then appear once per second under a `MAVLink telemetry (1 Hz)` header.

## Runtime behavior

The ESP32-S3 identifies itself as GCS system ID `255` and component ID `MAV_COMP_ID_MISSIONPLANNER`. It accepts the first valid flight-controller heartbeat as the active telemetry source and uses that message's system and component IDs for subsequent requests.

The following MAVLink messages are decoded:

| Message | Data used |
| --- | --- |
| `HEARTBEAT` | Identity, autopilot, mode, armed flag, and system state |
| `SYS_STATUS` | Battery and communication status |
| `ATTITUDE` | Roll, pitch, and yaw |
| `GLOBAL_POSITION_INT` | Position, altitude, velocity, and heading |
| `GPS_RAW_INT` | Fix type, satellites, and HDOP |
| `VFR_HUD` | Airspeed, groundspeed, heading, throttle, altitude, and climb |
| `BATTERY_STATUS` | Battery voltage, current, and remaining percentage |
| `STATUSTEXT` | Flight-controller diagnostic messages |

## Configuration

The main connection settings are constants near the top of [`src/main.cpp`](src/main.cpp):

- `MAV_RX_PIN`: ESP32 receive pin, default `18`.
- `MAV_TX_PIN`: ESP32 transmit pin, default `17`.
- `MAV_BAUD`: flight-controller UART speed, default `57600`.
- `DEBUG_BAUD`: USB serial monitor speed, default `115200`.

The PlatformIO target, library dependency, USB CDC settings, and monitor configuration are in [`platformio.ini`](platformio.ini).

## Troubleshooting

- **Only the waiting message appears:** verify TX/RX are crossed, GND is shared, the flight controller is powered, and its UART is configured for MAVLink at `57600` baud.
- **The monitor is blank:** use the native USB connector labeled **USB**, select the correct serial device, and reopen the monitor after reset.
- **Telemetry is marked stale:** check the UART connection and flight-controller output rate. The marker appears after more than three seconds without a heartbeat.
- **Battery values are missing:** the firmware prints each section only after receiving its corresponding MAVLink message, and some flight controllers do not provide every battery field.

## Current limitations

- No Wi-Fi, HTTP server, or web dashboard is implemented yet. [`data/index.htm`](data/index.htm) is currently empty and is not served by the firmware.
- Only one flight-controller system/component pair is tracked at a time: the first valid heartbeat received after boot.
- MAVLink messages other than the explicitly handled types are ignored.