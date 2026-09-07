# AWD Digital Water-Level Devices — Firmware

Firmware for the ESP32 devices that measure water level inside AWD
(Alternate Wetting and Drying) field pipes and push readings to Google
Sheets. Three device families share one status rule and one sensor, but
differ in how they reach the internet.

| Family | Folder | Radio / uplink | Readme |
|---|---|---|---|
| LoRa gateway | `1_LoRa_System/Gateway/AWD_LoRa_Gateway_V11/` | E220-900T30D LoRa in, Quectel EC200U 4G out | [gateway](1_LoRa_System/Gateway/README.md) |
| LoRa nodes (5) | `1_LoRa_System/Nodes/AWD_LoRa_Node1_V6/` … `Node5` | E220-900T22D LoRa to the gateway | [nodes](1_LoRa_System/Nodes/README.md) |
| LoRa Apps Script | `1_LoRa_System/AppsScript/` | Google Sheets backend for the gateway | [script](1_LoRa_System/AppsScript/README.md) |
| Standalone GSM | `2_Standalone_GSM/AWD_GSM_Monitor_V3/` | Quectel EC200U 4G, WiFi fallback, SMS | [gsm](2_Standalone_GSM/README.md) |
| Standalone WiFi | `3_Standalone_WiFi/AWD_WiFi_Monitor_V8/` | WiFi only (saved SSID or open hotspot) | [wifi](3_Standalone_WiFi/README.md) |

Release notes for every build: `CHANGELOG.md`.

```
AWD_Digital_Firmware/
├── README.md                  this file
├── CHANGELOG.md               what changed in each build and why
├── 1_LoRa_System/
│   ├── Gateway/AWD_LoRa_Gateway_V11/    one sketch, flash to the gateway
│   ├── Nodes/AWD_LoRa_Node1_V6 … Node5  one sketch per field node
│   └── AppsScript/Code.gs               Google Sheets backend
├── 2_Standalone_GSM/AWD_GSM_Monitor_V3/
└── 3_Standalone_WiFi/AWD_WiFi_Monitor_V8/
```

Each numbered folder has its own README.md. Open a sketch folder in the
Arduino IDE (File, Open, pick the `.ino`); the folder and file names
already match as the IDE requires.

## Common rules

**Water status** (absolute centimetres of water above the pipe bottom,
identical in every family since this release):

| Level | Status |
|---|---|
| sensor failed | `SensorError` |
| below 7 cm | `Low` |
| 7 to below 15 cm | `Good` |
| 15 to below 20 cm | `Excess` |
| 20 cm and above | `Flood Alert` |

**Sensor.** HC-SR04 ultrasonic mounted at the top of the pipe looking
down. `level = pipe_height − measured_distance`. Pipe height is set in
each device's config portal. A reading needs at least three valid echoes
out of five, otherwise it is a `SensorError`, never a "0 cm".

**Config portal.** Every device: short GPIO15 to GND, press reset, join
the device's WiFi access point, open `http://192.168.4.1`. Portals close
themselves after 10 minutes.

**Time.** All devices carry a DS3231 RTC (nodes excepted). Time is set
from the modem network (GSM/gateway) or NTP (WiFi). A device never fakes
a timestamp: if the clock is not valid the record says so.

## Build environment

- Arduino IDE 2.x, board package **esp32 by Espressif 3.3.x**
- Board: **ESP32 Dev Module**, Partition: **Default 4MB with spiffs**
- Libraries: ArduinoJson 7.x, RTClib (Adafruit), NewPing, TelnetStream
  (WiFi only)
- Serial monitor: 115200 baud

Every sketch prints a `[SYS]` banner with its version at boot. If you do
not see it, check the USB cable and the board selection.

## Hardware notes that apply everywhere

- Power the ESP32 from a clean 5 V; brownouts are the number-one cause
  of "random" behaviour. A 470 µF to 1000 µF capacitor at the board's
  5 V input is cheap insurance.
- Modems (EC200U) need their own 3 A regulator with a large capacitor.
  Never share a DevKit 3.3 V pin between a modem and a LoRa module.
- Keep antennas above the crop canopy. Rice at 1 m absorbs 868 MHz.
