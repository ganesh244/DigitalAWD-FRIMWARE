# Changelog

## LoRa system — Gateway V11, Nodes V6, Apps Script V7 (2026-09-07)

    =======================================================
      LoRa Water Level Monitoring System — V11 Release
      Gateway V11 | Transmitter V6 | AppScript V7.0
      Prepared: 2026-09-07 from the V10.1 field/bench release.
      Compiled clean against esp32 core 3.3.10 (ESP32 Dev Module,
      partition "Default 4MB with spiffs"). NOT yet hardware tested —
      bench-verify with the checklist at the bottom before deploying.
    =======================================================
    
    WHY THIS RELEASE EXISTS — WHAT THE FIELD DATA SHOWED
    -----------------------------------------------------
    Analysis of AWD_Gateway_Data (24,655 rows, Nov 2025 – Sep 2026) and
    the Heartbeat tab (456 rows, Feb – Aug 2026):
    
      1. Every multi-day outage hit all five nodes in the same minute.
         Five independent solar nodes with 15 Ah LiFePO4 batteries cannot
         die together. The common element is the gateway receiver.
    
      2. Feb 28 – Mar 2 2026: the gateway uploaded 24 heartbeats a day,
         CSQ 30, modem 22–39 °C, but "Last LoRa RX" stayed frozen at
         Feb 27 19:05 for four days and then recovered WITHOUT a reboot.
         That is a hung E220 receiver. The E220-900T30D shares the DevKit
         3.3 V pin with the EC200U (2 A bursts) — supply dips are the most
         likely trigger. V10.1 could neither detect nor reset that state.
    
      3. Even on good field days only ~45 % of packets arrived, and
         reception between 09:00 and 15:00 IST was a third lower than at
         night. On the bench in June reception was 98 %. The loss is radio
         path (300 m – 1 km, whip antennas, interference), not code.
         Factory channel 23 = 873.125 MHz sits inside the LTE band-5
         downlink (869–894 MHz) used by Jio, and outside India's
         865–867 MHz licence-free band. V11/V6 move to channel 15
         (865.125 MHz).
    
      4. Since June every row shows "Transmitter Data 60 / Water Level 0 /
         Low". That is the node's "all 5 pings failed → assume empty pipe"
         path. Sensor failure and a dry pipe were indistinguishable.
    
      5. The modem 60 °C upload gate never triggered (max 44 °C).
    
    FOLDER STRUCTURE
    ----------------
    1_LoRa_System/Gateway/AWD_LoRa_Gateway_V11        — Flash to gateway ESP32
    1_LoRa_System/Nodes/AWD_LoRa_NodeN_V6       — One folder per node, N = 1..5
    1_LoRa_System/AppsScript/Code.gs                            — Paste into the Apps Script project
    
    =======================================================
    GATEWAY V10.1 → V11
    =======================================================
    FIX-V11-1  resetE220ToFactory() removed. "C4 C4 C4" is an E32 command;
               the E220 (LLCC68) only understands C0 (write, saved),
               C2 (write, volatile) and C1 (read). configureE220() now
               writes the whole register block with C0 at boot and reads
               it back with C1. A failed read-back = "module hung".
                 ADDH ADDL REG0 REG1 REG2 REG3
                 00   00   62   20   0F   83
                 REG0 62 : 9600 baud, 8N1, 2.4 kbps air
                 REG1 20 : 200-byte sub-packet, ambient-RSSI ON, max power
                 REG2 0F : channel 15 = 865.125 MHz
                 REG3 83 : RSSI byte ON, transparent, LBT off, WOR 2000 ms
    FIX-V11-2  LORA_CHANNEL 15. One define per file; gateway and all nodes
               must match.
    FIX-V11-3  RSSI logging. Every frame is stored as
                 ts|T22D(LoraN)|distance|level|rssi
               and uploaded with "rssi". Ambient noise floor is read before
               every heartbeat ("noise"). Old buffered lines without the
               rssi field are still parsed.
    FIX-V11-4  Deaf-receiver recovery ladder (checkLoRaHealth):
                 45 min silent  → volatile re-config (C2) + verify
                 90 min silent  → power-cycle if LORA_PWR_PIN wired,
                                  otherwise persistent re-config (C0)
                 every 45 min   → re-config again
                 3 h silent     → save queue, restart ESP32 — at most once
                                  until a frame is heard again (no loop)
    FIX-V11-5  Heartbeat sent EVERY cycle after the data flush (V10.1 only
               sent it when the buffer was empty, so diagnostics vanished
               exactly when data was flowing). New fields: noise, dev_rssi[5],
               lora_cfg_ok, reset, uptime_min, heap, channel, fw.
    FIX-V11-6  Crash-safe upload. A reset during the flush used to leave
               /lfs_buffer.txt.tmp, which the next flush DELETED — every
               unsent reading lost. It is now merged back at boot and before
               each flush.
    FIX-V11-7  getFormattedTime() wrote 34 bytes ("... (RTC_INVALID)") into
               32-byte stack buffers when the DS3231 returned a bad year.
               Buffers are TS_LEN (40) and bounded.
    FIX-V11-8  gsmSendCommand() stops on ERROR / +CME ERROR;
               AT+QHTTPURL / AT+QHTTPPOST wait for CONNECT instead of a
               fixed 15 s each. ~30 s saved per batch.
    FIX-V11-9  Modem over-temperature gate 60 → 70 °C (EC200U rated +75 °C).
    FIX-V11-10 Water status "SensorError" when a node reports level < 0;
               uploaded as waterLevel null.
    FIX-V11-11 Last LoRa RX timestamp kept in RTC memory across ESP.restart().
    
    =======================================================
    TRANSMITTER V5.1 → V6
    =======================================================
    FIX-V6-1   Volatile register write (C2) on EVERY wake while the module is
               in mode 3: 9600 8N1, 2.4 kbps, DEVICE_CHANNEL 15, 22 dBm,
               RSSI byte OFF. Never again depends on factory defaults or on
               whatever the E220 GUI left behind.
    FIX-V6-2   Waits for AUX LOW→HIGH after handing over the payload, so the
               node no longer sleeps while the packet is still on the air.
    FIX-V6-3   E220 parked in mode 3 (~3 µA) with M0/M1 held HIGH through
               deep sleep (gpio_hold). TRIG held LOW. V5.1 left the module in
               receive mode (~12 mA) between wakes.
    FIX-V6-4   Sensor failure sent as distance -1 / level -1 → "SensorError"
               on the sheet instead of "Low 0 cm".
    Unchanged: DEVICE_ID, stagger (0/4/8/12/16 min), 20–25 min sleep,
               config portal (GPIO15 → GND at boot, SSID LoRaTX-LoraN-Setup,
               192.168.4.1, admin / password).
    
    =======================================================
    APPSCRIPT V6.1 → V7.0
    =======================================================
    - Data sheet: column 13 "RSSI (dBm)" appended automatically.
    - Status "SensorError" gets a grey background; Water Level cell blank.
    - Heartbeat sheet: columns 19–30 appended automatically
      (Noise, Lora1-5 RSSI, LoRa Cfg OK, Reset Reason, Uptime, Free Heap,
      Channel, Firmware).
    - GET ?heartbeat=1 (with ?days= / ?limit=) returns the Heartbeat tab as
      JSON for the dashboard.
    - Re-deploy: Deploy > Manage deployments > Edit > New version.
    
    =======================================================
    PIN CONNECTIONS
    =======================================================
    GATEWAY (unchanged from V10.1):
      LoRa E220-900T30D:  RX->17  TX->16  M0->21  M1->19  AUX->18
      GSM EC200U:         RX->26  TX->27  PWR->4  RST->32
      RTC DS3231:         SDA->22 SCL->23
      Config:             GPIO15 -> GND at boot = config portal
      Optional (FIX-V11-4): LORA_PWR_PIN — a high-side P-MOSFET on the E220
      VCC. Gate via 10 kΩ to GPIO5, 100 kΩ gate pull-up to VCC. GPIO LOW =
      module powered. Set "#define LORA_PWR_PIN 5" when wired; default -1.
    
    NODES (unchanged from V5.1):
      LoRa E220-900T22D:  RX->17  TX->16  M0->21  M1->19  AUX->18 (required)
      HC-SR04:            TRIG->13, ECHO -> 1k -> GPIO12 -> 2k -> GND
      Config:             GPIO15 -> GND at boot = config portal
      M0/M1 MUST stay on GPIO21/19 (not GND) — V6 drives them HIGH to put the
      module to sleep.
    
    =======================================================
    HARDWARE CHANGES STRONGLY RECOMMENDED (not required to flash)
    =======================================================
    1. Gateway: give the E220-900T30D its own regulator (3.3 V LDO from the
       battery, or 5 V — the T30D accepts 3.3–5.5 V) and ≥470 µF at its VCC.
       Do NOT feed it from the DevKit 3.3 V pin that also sees the EC200U.
    2. Gateway: EC200U on its own 3.8–4.2 V / 3 A buck with ≥1000 µF.
    3. Gateway: fit the LORA_PWR_PIN MOSFET so the recovery ladder can
       actually reset a hung module.
    4. Both: move antennas above the crop canopy and away from the modem
       antenna; a 3 dBi 868 MHz dipole on 1 m of coax beats the stock whip.
    5. Nodes: ECHO on GPIO12 is a strapping pin. It works with the 1k/2k
       divider holding it low at boot; do not remove that divider.
    
    =======================================================
    FLASH ORDER + BENCH CHECKLIST
    =======================================================
    1. Paste 1_LoRa_System/AppsScript/Code.gs, re-deploy a new version. Existing rows are
       untouched; new headers appear on the right.
    2. Flash AWD_LoRa_Gateway_V11 (Board: ESP32 Dev Module, Partition: Default
       4MB with spiffs). Serial 115200 must show:
         [LORA] Configuring E220: ch=15 (865.125 MHz) ...
         [LORA] write response (9 bytes): C1 00 06 00 00 62 20 0F 83
         [LORA] read-back: 00 00 62 20 0F 83 → MATCH
         [LORA] E220 configured and verified.
       If you see "C1 read failed" the module is not answering: check M0/M1
       wiring and the 3.3 V rail before going further.
    3. Flash each Nodes/AWD_LoRa_NodeN_V6. Serial must show:
         [LORA] C2 config OK (9 bytes: C1 00 06 00 00 62 00 0F 03)
         [TX] Payload: T22D(LoraN)|xx.xx|yy.yy
       and the gateway must print "[LORA] RX: T22D(LoraN)|... (RSSI -xx dBm)".
       A gateway RSSI of 0 means the RSSI byte is not arriving — the gateway
       REG3 write did not stick.
    4. Point the sensor at nothing: node must send "-1.00|-1.00" and the
       sheet must show status SensorError with a blank level.
    5. Measure node sleep current at the battery: expect the DevKit floor
       (~10 mA) — if it is ~22 mA the E220 is not sleeping (M0/M1 on GND?).
    6. Leave the gateway running one full hour: the Heartbeat row must show
       LoRa Cfg OK = 1, a Noise value around -100 dBm, and per-node RSSI.

## Standalone GSM V3 and Standalone WiFi V8 (2026-09-07)

    AWD firmware fixes — 2026-09-07
    All four sketches compile clean on esp32 core 3.3.10 (ESP32 Dev Module,
    partition "Default 4MB with spiffs", ArduinoJson 7.4.2, RTClib, NewPing,
    TelnetStream). Nothing has been flashed to hardware yet — bench test first.
    
    1_LoRa_System/         Gateway V11, nodes V6, Apps Script V7.
                                 Full notes + bench checklist in its README.md.
    
    2_AWD_GSM_Monitor_V3/        Standalone GSM (EC200U, Jio). Changes (V3-n in code):
      V3-1  Reading saved to LittleFS BEFORE the modem runs (write-ahead). A
            brownout during LTE no longer loses the cycle. One upload path.
      V3-2  Modem always set to CFUN=0 when it answered AT, even after a
            failed registration (V2 left it searching all night, 50-150 mA).
      V3-3  Sensor failure (fewer than 3 valid pings) → "SensorError", null
            level. V2 turned a missing echo into "Low, 0 cm".
      V3-4  Status rule = fleet standard: Low <7, Good <15, Excess <20, else
            "Flood Alert" (was percent-of-pipe).
      V3-5  drainLfsToSd() streams instead of loading the file into RAM
            (was a guaranteed crash at ~1 MB). Works without an SD card.
      V3-6  AT+CLTS=1 (SIMCom) → AT+CTZU=1 (Quectel); CCLK ±zz parsed; tz
            offset applied only when the network gives none.
      V3-7  300 s task watchdog; ERROR breaks AT waits; QHTTPPOST wait 65 s.
      V3-8  Config loaded before the portal; sleep clamped 60..86400 s.
      V3-9  LittleFS formats only after 3 consecutive mount failures.
      V3-10 WiFi fallback only when an SSID is configured.
      V3-11 "deviceId" (efuse MAC) in every record; flush aborts after 3
            consecutive failures; activeConnection set only after PDP is up.
      V3-12 Leftover /flushing.tmp and /drain.tmp merged back at boot.
      Not fixed in software (hardware): ECHO on GPIO12 strapping pin; the
      modem has no PWRKEY wired so it is never fully powered down (~20 mA).
    
    3_AWD_WiFi_Monitor_V8/  Standalone WiFi. Changes (V8-n in code):
      V8-1  Reading saved to LittleFS before WiFi (write-ahead); direct-send
            path removed, syncAllToSheets() sends everything.
      V8-2  Batch JSON overflow: the 4 KB doc dropped ~1/3 of every 20-record
            batch while counting them as sent. Now 10 per 8 KB, add() checked.
      V8-3  client.setTimeout(15) was 15 ms on this core → header reads
            returned empty, redirect missed, duplicates. Now 15 s / 10 s.
      V8-4  Backoff "force every 8 boots" counted boots since SUCCESS, so
            after 8 failures WiFi ran every boot. Now boots since attempt.
      V8-5  Only a real power-on/EN reset bypasses backoff; brownout, panic
            and watchdog resets count as failures.
      V8-6  Brownout register restored to its saved value (was written "1").
      V8-7  rename() without remove(); leftover temp file merged at boot.
      V8-8  WiFi budget: fail fast on no-SSID/connect-failed, one scan pass,
            at most 2 open networks × 10 s.
      V8-9  RTC no longer set from compile time; clockValid:false + empty
            timestamp until NTP.
      V8-10 LittleFS formats only after 3 consecutive mount failures.
      V8-11 Config portal exits after 10 minutes.
      V8-12 Sleep 10..86400 s and pipe height 5..300 cm clamped.
      V8-13 "Flooding" → "Flood Alert"; median needs ≥3 pings; deviceId field.
      V8-14 NTP only when the clock is invalid or on the first upload of the
            day. A field log from 2026-09-09 showed an iPhone hotspot blocking
            NTP while the DS3231 held correct time, so the old unconditional
            10 s wait spent radio power on every boot for a sync that could
            not succeed, then wrongly logged the timestamps as invalid.
      V8-15 Watchdog 30 s → 60 s. With the TLS timeouts now bounded, one
            connect + handshake can block 20 s without feeding the watchdog,
            which left too little margin before a panic reset mid-upload.
      V8-16 Clock is corrected from the HTTP "Date:" header of every upload.
            The device reaches the internet through phone hotspots, which
            block NTP's UDP port 123, so NTP alone could never fix a wrong
            RTC in the field. The header is already being read, so this
            costs no extra radio time and is accurate to a second or two.
      V8-17 WiFi acquisition reworked around the real operating model: there
            is no WiFi in the field at all, and a sync happens when a farmer
            switches on a phone hotspot and presses the reset button, once a
            month or two or after harvest. The reset button is therefore
            treated as the sync command and always tries. Timer wakes take a
            cheap look about once a day, or every 6 wakes once the buffer
            passes 60 % full, and skip the radio otherwise. When a look does
            happen it scans first and connects only to a network the scan
            saw, instead of blind-dialling the saved SSID for 15 s in an
            empty field, so each check costs roughly 3 s instead of 20 s.
      V8-18 Upload batch 10 -> 40 records. A month or two of hourly readings
            is 700-1400 buffered rows, which at 10 per upload meant up to
            144 TLS round trips and close to ten minutes of the farmer
            holding the hotspot open. At 40 it is about three minutes.
      V8-19 Reset detection made fail-safe. Any boot that is not the ordinary
            deep-sleep timer wake now counts as a sync request, rather than
            having to match a list of reset codes: boards differ in what they
            report for a button press, and an unrecognised code must never
            silently disable the only sync path the device has. Repeated
            fault resets (brownout, panic, watchdog) still stop counting, but
            only after the first couple, since a tired battery can brown out
            exactly when the radio starts.
      V8-20 A sync request now retries for about two minutes instead of
            giving up after a single pass, so the hotspot can be switched on
            after the reset button is pressed. Routine timer checks still
            make one pass.
      V8-21 A sonar echo landing more than 8 cm past the bottom of the pipe
            is now a SensorError instead of being clamped to "Low, 0 cm".
            The 2026-09-09 bench log read 58 cm against a 55 cm pipe and
            called it dry; in the field that same reading can mean the head
            has slipped or the pipe has been pulled out, and a false "dry"
            tells a farmer to irrigate a field that may be flooded. The raw
            distance is included in the record so the cause is visible.
      V8-22 The HTTP-Date clock check reports once per boot even when no
            correction was needed, so it can be verified before the day the
            RTC is actually wrong.
      V8-23 An open network that fails its internet check is not retried for
            the rest of that boot. A bench log showed the same dead open AP
            being associated with on three consecutive retry passes, roughly
            50 s of the two-minute sync window, while the farmer's hotspot
            was still coming up.
      Note: 95 % of the 1.2 MB app partition. Use "Minimal SPIFFS" partition
      if you add more code, or drop TelnetStream (never started anyway).
    
    Apps Script for the GSM and WiFi devices: not changed. Both now send
    "deviceId", "clockValid" (WiFi) and null waterLevel on SensorError; the
    scripts ignore unknown fields, so nothing breaks, but you may want a
    column for deviceId once more than one unit shares a sheet.
