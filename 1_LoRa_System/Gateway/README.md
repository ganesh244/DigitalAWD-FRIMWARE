# LoRa Gateway V11

Receives LoRa frames from up to five field nodes, buffers them in the
ESP32's internal flash, and uploads them in batches over 4G to a Google
Apps Script. Sends a heartbeat with radio and modem diagnostics every
upload cycle.

## Hardware

| Part | Notes |
|---|---|
| ESP32 Dev Module (WROOM-32) | 4 MB flash, "Default 4MB with spiffs" partition |
| Ebyte E220-900T30D | LoRa receiver, UART, factory settings are **not** relied on |
| Quectel EC200U | 4G modem, Jio or Airtel SIM, PWRKEY and RESET wired |
| DS3231 | RTC, synced from the network time every connect |
| Solar + battery | The gateway is always on; budget about 60 mA average |

### Pins

| Signal | GPIO | Signal | GPIO |
|---|---|---|---|
| E220 RX (ESP TX) | 17 | EC200U RX (ESP TX) | 26 |
| E220 TX (ESP RX) | 16 | EC200U TX (ESP RX) | 27 |
| E220 M0 | 21 | EC200U PWRKEY | 4 |
| E220 M1 | 19 | EC200U RESET | 32 |
| E220 AUX | 18 | DS3231 SDA / SCL | 22 / 23 |
| Config jumper | 15 to GND | E220 power switch (optional) | 5, see below |

**Power the E220-900T30D from its own regulator**, not from the DevKit
3.3 V pin the modem also loads. The field data shows the receiver
hanging for days after modem bursts. Put 470 µF or more at the module.

**Optional power switch.** A P-channel MOSFET high-side switch on the
E220 VCC lets the firmware hard-reset a hung module. Gate through 10 kΩ
to GPIO5, 100 kΩ from gate to VCC. GPIO LOW = module on. Then set
`#define LORA_PWR_PIN 5` in the sketch. Without it, the recovery ladder
can only re-configure and finally restart the ESP32.

## Radio settings

At every boot the gateway writes the full E220 register block and reads
it back. It does not depend on the module's saved state.

| Register | Value | Meaning |
|---|---|---|
| ADDH ADDL | 00 00 | broadcast address |
| REG0 | 0x62 | 9600 baud, 8N1, 2.4 kbps air rate |
| REG1 | 0x20 | 200-byte sub-packet, ambient-RSSI reads on, max power |
| REG2 | 0x0F | channel 15 = 865.125 MHz |
| REG3 | 0x83 | RSSI byte after every packet, transparent mode, LBT off |

Change the channel only via `LORA_CHANNEL` and change it on every node
too. 865 to 867 MHz is India's licence-free band; the Ebyte factory
channel 23 (873.125 MHz) is inside Jio's LTE downlink and must not be
used.

## How it works

1. Every node frame `T22D(LoraN)|distance|level` is received with a
   trailing RSSI byte, timestamped, and queued.
2. The queue drains to `/lfs_buffer.txt` in LittleFS every 60 s or 5
   frames. About 1.5 MB is available, roughly three weeks of data.
3. Every `Upload Interval` minutes (default 60) the modem connects,
   uploads the buffer in batches of 20 readings, then sends a heartbeat.
4. If no frame arrives for 45 minutes the recovery ladder starts:
   re-configure and verify the E220; then power-cycle it (if wired);
   after 3 hours save the queue and restart the ESP32, once.

### Heartbeat fields

`upload_ts`, `gateway`, `gsm`, `simOperator`, `gsmStrength` (CSQ),
`temp` (modem °C), `last_lora_rx`, overflow counters, `lora_recovery`,
`lora_restarts`, `lfsFreeKB`, `dev_silence_min[5]` (minutes since each
node was heard, 9999 = never since boot), `noise` (ambient RSSI dBm),
`dev_rssi[5]` (last packet RSSI per node), `lora_cfg_ok` (1 = the E220
answered the register read-back), `reset`, `uptime_min`, `heap`,
`channel`, `fw`.

Read the heartbeat first when something looks wrong:

| Symptom in Heartbeat sheet | Meaning |
|---|---|
| No rows at all | Gateway has no power, no SIM, or no signal |
| Rows arrive, `lora_cfg_ok` = 0 | E220 not answering: wiring, rail, or dead module |
| `lora_cfg_ok` = 1, all silence rising, noise around −100 | Nodes are not transmitting or out of range |
| `lora_cfg_ok` = 1, noise above −85 dBm | Interference on the channel; try channel 16 or 17 |
| One node's RSSI below −115 dBm | That node is at the edge of range |

## Config portal

Short GPIO15 to GND, reset. Join **LoRaGateway-Setup** (password
`12345678`), open `http://192.168.4.1`, login `admin` / `password`.

Fields: WiFi SSID and password (unused for upload, kept for future use),
Google Script URL, Upload Interval in minutes, and a nickname for each of
the five nodes. `View Logs` and `Download` show the unsent buffer;
`Clear Log Buffer` deletes it.

## Flashing

1. Board ESP32 Dev Module, partition Default 4MB with spiffs, 115200.
2. Upload `AWD_LoRa_Gateway_V11.ino`.
3. Expect on serial:
   ```
   [SYS] Booting LoRa Gateway V11 ...
   [LORA] Configuring E220: ch=15 (865.125 MHz) ...
   [LORA] write response (9 bytes): C1 00 06 00 00 62 20 0F 83
   [LORA] read-back: 00 00 62 20 0F 83 → MATCH
   [LORA] E220 configured and verified.
   [GSM] ... IP obtained.
   ```
4. Power a node nearby. Expect `[LORA] RX: T22D(Lora1)|... (RSSI -45 dBm)`.

## Troubleshooting

- `C1 read failed` at boot: the module is not in config mode or not
  powered. M0 and M1 must be on GPIO21/19 and go HIGH together; check
  the 3.3 V at the module during modem activity.
- Frames print with `RSSI 0`: the REG3 write did not take. Look at the
  read-back line; the last byte must be 83.
- Frames arrive but nothing reaches the sheet: read the `[MAINT]`
  block on serial. `+QHTTPPOST: 0,302` is success; anything else names
  the failing AT command.
- Modem `SEARCH` for minutes: weak signal or wrong band. CSQ under 10 is
  unusable; move the antenna.
