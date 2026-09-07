# Standalone WiFi Water Monitor V8

Ultrasonic pipe monitor that uploads over WiFi: the saved network, or
any open hotspot it can find. Meant for sites with a phone hotspot or a
village access point. Stores every reading first and syncs when it can.

## Hardware

| Signal | GPIO |
|---|---|
| HC-SR04 TRIG | 13 |
| HC-SR04 ECHO | 27 (via divider) |
| DS3231 SDA / SCL | 21 / 22 |
| SD CS | 5 (SCK 18, MISO 19, MOSI 23) |
| Config jumper | 15 to GND |

The DS3231 is optional; without it, timestamps come from NTP once WiFi
has worked at least once per boot, otherwise the record carries
`clockValid:false` and an empty timestamp. The SD card is optional and
only used as overflow when LittleFS is 75 % full.

## Wake cycle (V8 order)

1. Mount LittleFS (format only after 3 failed mounts), RTC, SD. Merge
   any temp file left by a crash. Load config.
2. **Read the sensor and append the record to `/data.txt`.**
3. Decide whether to try WiFi. Backoff: after 3 failed attempts the
   next 5 boots skip WiFi; an attempt is forced every 8 boots, when
   LittleFS is over 60 % full, or after a real power-on/reset button.
4. WiFi: saved SSID for 15 s (gives up early when the SSID is absent),
   else one scan and the two strongest open networks for 10 s each,
   each checked with a connectivity probe.
5. Online: NTP (10 s), then `syncAllToSheets()` posts `/data.txt` and
   the SD backup in batches of 10 through the raw TLS client that
   follows the Apps Script 302 redirect. Failed batches are kept.
6. Deep sleep for the configured interval, or 60 s when the level moved
   more than 5 cm since the last reading (smart sleep).

## Record format

```json
{"timestamp":"2026-09-07 14:05:11","clockValid":true,"deviceId":"1a2b3c",
 "network":"pending","wifiStrength":0,"lfsUsedPct":3,
 "waterLevel":12.0,"status":"Good","dataType":"Current"}
```
Stored records are re-labelled `Backup` when uploaded later. A failed
reading (fewer than 3 valid echoes) is stored as
`"dataType":"SensorError","status":"SensorFailure","waterLevel":null`.

## Config portal

Short GPIO15 to GND, reset. Join **wifi - AWD PIPE** (password
`12345678`), open `http://192.168.4.1`, login `digitalawd` / `password`.

| Field | Meaning |
|---|---|
| WiFi SSID / password | the network to use first; leave blank to rely on open hotspots |
| Pipe height (cm) | sensor face to pipe bottom, 5 to 300 |
| Update frequency | preset list, or a custom value 10 to 86400 s |

`/download` returns the unsent data, `/clear-backup` deletes it,
`/live-data` shows a live reading. The portal restarts the device after
10 minutes if nobody saves.

## Flashing

Board ESP32 Dev Module, partition Default 4MB with spiffs, ArduinoJson
7, NewPing, RTClib, TelnetStream. The sketch uses 96 % of the app
partition; if a future change no longer fits, remove TelnetStream (it
is never started) or switch to the Minimal SPIFFS partition.

Expect on serial:
```
[System] === AWD PIPE Water Monitor ===
[LFS] Ready. ...
[Sensor] Distance=43cm  Level=12.0cm  Status=Good
[SAVE] Stored in LittleFS. Usage now 0.3%
[WiFi] Internet OK via: MyHotspot
[NTP] Time synced: 2026-09-07 14:05:20
[FLUSH-LFS] OK (1/1)
[Sleep] Going to sleep for 600s. Bye.
```

## Troubleshooting

- Rows appear twice: the upload succeeded but the redirect body was not
  read. V8 fixed the 15 ms timeout that caused this; if it persists,
  the hotspot is dropping the second TLS connection. Nothing is lost.
- Nothing uploads, `[HTTP] Connect failed`: DNS or TLS blocked on that
  hotspot. Try another network; records stay in `/data.txt`.
- Device stuck in AP mode: GPIO15 is a strapping pin. Remove the jumper
  and keep moisture off the header. V8 exits the portal after 10 min.
- Battery drains fast: check the serial line `[WiFi] Skipping WiFi
  (backoff)`. If WiFi runs every boot, the network is present but has
  no internet; the connectivity probe then fails every time. Configure
  a working SSID or move the device.
