# Apps Script V7.0 for the LoRa gateway

`Code.gs` is the Google Apps Script web app that the gateway posts to.
It keeps three tabs in the bound spreadsheet.

| Tab | Written by | Content |
|---|---|---|
| `AWD_Gateway_Data` | data uploads | one row per node reading |
| `Heartbeat` | heartbeat uploads | one row per gateway cycle |
| `AppSettings` | the dashboard | per-device settings (crop, plot polygon) |

## Deploy or update

1. Open the spreadsheet, Extensions, Apps Script.
2. Replace the contents of `Code.gs` with this file.
3. Deploy, Manage deployments, Edit (pencil), Version: New version, Deploy.
4. The URL does not change. The gateway keeps working; new columns
   appear on the right of each tab the first time a V11 gateway posts.

Migration is append-only. Existing rows and column order are never
touched.

## Data tab columns

1 Gateway Received Time, 2 Device ID, 3 Transmitter Data (distance cm),
4 Water Level (cm), 5 Status, 6 Network, 7 Batch Upload Time,
8 SIM Operator, 9 WiFi Strength, 10 GSM Strength (CSQ),
11 LittleFS Free (KB), 12 Source, **13 RSSI (dBm)** (new).

Status colours: Low pink, Good green, Excess yellow, Flood Alert purple,
SensorError grey. For SensorError the level and distance cells are
blank, not zero.

## Heartbeat tab columns

1 to 18 as before (upload time, gateway, GSM, operator, CSQ, modem
temperature, last LoRa RX, four overflow/recovery counters, LoRa
restarts, Lora1 to Lora5 silence minutes, LittleFS free), then new:
19 Noise (dBm), 20 to 24 Lora1 to Lora5 RSSI (dBm), 25 LoRa Cfg OK,
26 Reset Reason, 27 Uptime (min), 28 Free Heap, 29 Channel, 30 Firmware.

## HTTP interface

| Request | Returns |
|---|---|
| `GET ?days=7` | data rows, newest first, last 7 days (`days=all` for everything) |
| `GET ?days=7&limit=500` | at most 500 rows |
| `GET ?heartbeat=1&days=7` | heartbeat rows, same filters (new) |
| `GET ?action=getSettings` | `{deviceId: {key: value}}` from AppSettings |
| `POST {"action":"saveSetting","deviceId":..,"key":..,"value":..}` | upsert one setting |
| `POST {"type":"heartbeat", ...}` | append a heartbeat row |
| `POST {"readings":[...], ...}` | append data rows |

The dashboard at lora-awd-fsmm.vercel.app uses the first, fourth and
fifth of these.
