# Standalone GSM Water Monitor V3

One self-contained unit: ultrasonic sensor, ESP32, Quectel EC200U 4G
modem, DS3231 RTC, LittleFS buffer, optional SD archive. Wakes on a
timer, reads the pipe, stores the reading, uploads it and any backlog
over 4G, answers SMS queries, and deep sleeps.

## Hardware

| Signal | GPIO | Signal | GPIO |
|---|---|---|---|
| HC-SR04 TRIG | 13 | EC200U RX (ESP TX) | 26 |
| HC-SR04 ECHO | 12 (via divider) | EC200U TX (ESP RX) | 27 |
| DS3231 SDA / SCL | 21 / 22 | SD CS / SCK / MISO / MOSI | 5 / 18 / 19 / 23 |
| Config jumper | 15 to GND | | |

Modem UART is 115200 baud; the firmware also recovers a module stuck at
9600. There is no PWRKEY line, so the modem is never fully powered
down: between wakes it sits in `CFUN=0` at roughly 20 mA. If battery
life matters, wire PWRKEY (or a MOSFET on the modem supply) and add
`AT+QPOWD` handling.

ECHO on GPIO12 works only because the divider holds it low at boot. A
free GPIO such as 14 would be a safer choice on the next board.

## Wake cycle (V3 order)

1. Mount LittleFS, RTC, SD (optional). Load config.
2. **Read the sensor and write the record to `/backup.txt` first.**
   Everything after this point can fail without losing the reading.
3. Modem: `AT`, `CFUN=1`, `CTZU=1`, wait for `CEREG` 1 or 5 (60 s max),
   read SIM info and CSQ, sync the RTC from `CCLK`, detect the APN from
   the IMSI (Jio `jionet`, Airtel `airtelgprs.com`, Vi `www`, BSNL
   `bsnlnet`, else the configured APN), `QIACT`.
4. If 4G failed and a WiFi SSID is configured, try WiFi for 10 s.
5. Check incoming SMS. A message containing `hi` marks the sender for a
   reply with the current reading; the reply is sent on this cycle.
6. `flushOfflineData()` posts every line in `/backup.txt` one by one
   (`AT+QHTTPPOST`, 65 s response wait). Lines that fail are kept.
   After three consecutive failures the rest is kept for next time.
7. NTP over WiFi if that path was used. Modem to `CFUN=0`. Deep sleep
   for `normal_sleep_s` (default 3600).

## Record format

```json
{"timestamp":"2026-09-07 14:05:11","network":"pending","sim":"N/A",
 "simOperator":"N/A","wifiStrength":0,"gsmStrength":-1,
 "waterLevel":12.0,"status":"Good","device":"AWD ONLINE",
 "deviceId":"GSM-1A2B3C","dataType":"Live","smsStatus":"None"}
```
`waterLevel` is `null` and `status` is `SensorError` when fewer than
three of five pings are valid. `device` stays `AWD ONLINE` so the
existing dashboard card keeps working; `deviceId` is unique per board.

## Config portal

Short GPIO15 to GND, reset. Join **WaterMonitor-Setup**; the password
is the chip ID printed on serial as `[Portal] AP: ... | Pass: XXXXXXXX`.
Open `http://192.168.4.1`, login `admin` / `password`.

| Field | Meaning |
|---|---|
| WiFi SSID / password | fallback network, leave blank to disable |
| APN | used only when the SIM's IMSI is not recognised |
| Pipe height (cm) | sensor face to pipe bottom |
| Sleep (s) | 60 to 86400, default 3600 |
| Timezone offset (s) | 19800 for IST, used only if the network gives no offset |

`/download` returns the unsent backup, `/erase` clears it, `/data` shows
a live reading.

## Storage

`/backup.txt` in LittleFS holds unsent records. When it passes 75 % of
the partition the oldest lines are streamed to the SD card (`/archive.txt`)
or, without a card, discarded with a log line, down to 40 %. Temp files
left by a crash are merged back at the next boot. LittleFS is formatted
only after three consecutive failed mounts.

## Flashing

Board ESP32 Dev Module, partition Default 4MB with spiffs, ArduinoJson 7.
Expect on serial:
```
[SYS] === Water Monitor V3 Booting ===
[LFS] LittleFS ready ...
[Sensor] Distance=43cm Level=12.0cm Status=Good
[LFS] Backup saved to LittleFS.
[GSM] Registered on LTE network.
[GSM] APN: jionet
[GSM] IP obtained.
[Flush] Done. Sent: 1 | Failed: 0
[Sleep] Sleeping for 3600 seconds.
```

## Troubleshooting

- `Module not responding`: modem has no power or is still booting. It
  needs 10 to 15 s after power-up; the first cycle after a battery swap
  may fail and succeed on the next.
- `PDP activation failed` with Jio: the SIM may need `jionet` with no
  authentication. Set the APN in the portal; the IMSI rule is then the
  only thing that overrides it, so also check the serial APN line.
- Timestamps 5 h 30 min off: the RTC was set by an older build. One
  successful connect with V3 corrects it.
- Rows with `SensorError`: sensor unpowered, or mounted too close
  (under 2 cm) or too far (over 200 cm) from the water.
