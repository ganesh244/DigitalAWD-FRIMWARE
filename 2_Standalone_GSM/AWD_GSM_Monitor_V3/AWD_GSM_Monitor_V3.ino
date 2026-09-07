#include <WiFi.h>
#include <HTTPClient.h>
#include <NewPing.h>
#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <algorithm>
#include "esp_sleep.h"
#include "rom/rtc.h"

// ======================================================
// V3 (2026-09-07) — audit fixes. Search "V3:" for each change.
//  V3-1  Reading is taken and written to LittleFS BEFORE any modem work
//        (write-ahead). A brownout or hang during the 1–4 min LTE phase no
//        longer loses the cycle. The live POST path is gone — everything
//        goes through flushOfflineData(), one code path.
//  V3-2  Modem is always put back to CFUN=0 when it answered AT, even if
//        registration or PDP failed (V2 left it searching all night).
//  V3-3  Sensor failure (fewer than 3 valid pings) → status "SensorError",
//        waterLevel null. V2 mapped "no echo" to 200 cm → 0 cm → "Low".
//  V3-4  Status rule = fleet standard, absolute cm:
//        Low <7, Good <15, Excess <20, else "Flood Alert".
//  V3-5  drainLfsToSd() streams line by line through a temp file instead
//        of loading the whole backup into RAM (guaranteed OOM at ~1 MB).
//        Works with or without an SD card (without: oldest lines dropped).
//  V3-6  AT+CLTS=1 (SIMCom) replaced by AT+CTZU=1 (Quectel). CCLK's ±zz
//        quarter-hour field is parsed; tz offset is applied only when the
//        network gave none. Fixes the 5 h 30 min double offset.
//  V3-7  Task watchdog 300 s. gsmSendCommand() breaks on ERROR even with
//        waitFor. QHTTPPOST waited for 65 s (declared 60 s) not 20 s.
//  V3-8  Config loaded BEFORE the portal starts (portal used to show and
//        re-save compile-time defaults). Sleep clamped 60..86400 s.
//  V3-9  LittleFS.begin(false); formats only after 3 consecutive mount
//        failures (counter in NVS) instead of on the first hiccup.
//  V3-10 WiFi fallback only when an SSID is configured.
//  V3-11 "deviceId" (efuse MAC tail) added to every record; "device"
//        name unchanged so the dashboard keeps working. Flush aborts after
//        3 consecutive failures instead of grinding through the backlog.
//  V3-12 A leftover /flushing.tmp or /drain.tmp is merged back at boot.
// ======================================================
#define WDT_TIMEOUT_S        300
#define FLUSH_MAX_FAILS      3
#define STATUS_LOW_CM        7.0f
#define STATUS_GOOD_CM       15.0f
#define STATUS_EXCESS_CM     20.0f

// ======================================================
// ⚙ GLOBAL CONFIGURATION & PINS
// ======================================================

// --- HARDWARE PINS ---
#define CONFIG_JUMPER_PIN   15
#define TRIG_PIN            13
#define ECHO_PIN            12
#define MAX_DISTANCE_CM     200

// --- SD CARD PINS (archive/overflow only) ---
#define SD_CS               5
#define SD_SCK              18
#define SD_MISO             19
#define SD_MOSI             23

// --- I2C PINS (RTC) ---
#define SDA_PIN             21
#define SCL_PIN             22

// --- GSM MODULE PINS (Serial1) ---
#define GSM_RX_PIN          26
#define GSM_TX_PIN          27
#define GSM_BAUD            115200

// --- SYSTEM SETTINGS ---
#define DEVICE_NAME         "AWD ONLINE"

// --- WEB SERVER CREDENTIALS ---
const char* CONFIG_USERNAME = "admin";
const char* CONFIG_PASSWORD = "password";

// --- LITTLEFS STORAGE THRESHOLDS ---
// When LittleFS backup file exceeds this % of total LittleFS size, drain oldest
// entries to SD card (if available). Keeps LittleFS as hot buffer only.
#define LFS_SPILL_PERCENT   75   // spill to SD when backup.txt > 75% of LittleFS
#define LFS_TARGET_PERCENT  40   // after draining, aim to be at ~40% usage

// ======================================================
// ☁ CLOUD & CONNECTIVITY
// ======================================================
const char* GSCRIPT_URL = "https://script.google.com/macros/s/AKfycby61hthQVULKFW_1--hI0V2t-gjxOVSnUzZ6iHK-Q-RT2cpUbvgvmM7BfFt5rSOuR0MFw/exec";

// ======================================================
// 📋 CONFIGURATION VARIABLES (loaded from LittleFS)
// ======================================================
String wifi_ssid      = "";
String wifi_pass      = "";
String gsm_apn        = "airtelgprs.com";
float  pipeHeightCm   = 55.0;
uint32_t normalSleepS = 3600;   // safe fallback — 60 minutes in internal flash
int32_t  tzOffsetS    = 19800;  // UTC+5:30 (IST) — configurable

uint32_t deepSleepSeconds = normalSleepS;

// ======================================================
// 📊 DATA STRUCTURE
// ======================================================
struct SensorData {
  String timestamp   = "";
  String network     = "N/A";
  String sim         = "N/A";
  String simOperator = "N/A";
  long   wifiStrength  = 0;
  int    gsmStrength   = -1;
  float  waterLevel    = 0.0;
  String status        = "Unknown";
  String device        = DEVICE_NAME;
  String dataType      = "Reading";
  String smsStatus     = "None";
  // Storage info (populated at runtime for web UI only — not sent to cloud)
  size_t lfsTotal      = 0;
  size_t lfsUsed       = 0;
  float  sdSizeMB      = 0.0;
  float  sdFreeMB      = 0.0;
};

// ======================================================
// 🌍 GLOBAL OBJECTS & STATE FLAGS
// ======================================================
HardwareSerial gsmSerial(1);
WebServer      server(80);
NewPing        sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE_CM);
RTC_DS3231     rtc;
SensorData     currentData;

bool lfsReady  = false;   // LittleFS (primary)
bool modemAlive = false;  // V3-2: modem answered AT this boot
String deviceId = "";     // V3-11
bool sdReady   = false;   // SD card  (archive/overflow)
bool rtcReady  = false;
bool shouldSaveConfig = false;

enum ConnectionType { NONE, WIFI, GSM };
ConnectionType activeConnection = NONE;

// ======================================================
// 📝 FORWARD DECLARATIONS
// ======================================================
String gsmSendCommand(const String& cmd, uint32_t timeout = 2000, const String& waitFor = "");
void handleRoot();
void handleSave();
void handleLiveData();
void handleDownload();
void handleErase();
void drainLfsToSd();

// ======================================================
// 🛠 UTILITY FUNCTIONS
// ======================================================
void log_message(const String& message) {
  Serial.println(message);
}

void print_reset_reason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   log_message("[SYS] Reset: Power on");          break;
    case ESP_RST_EXT:       log_message("[SYS] Reset: External pin");      break;
    case ESP_RST_SW:        log_message("[SYS] Reset: Software");          break;
    case ESP_RST_PANIC:     log_message("[SYS] Reset: Panic/exception");   break;
    case ESP_RST_DEEPSLEEP: log_message("[SYS] Reset: Woke from deep sleep"); break;
    case ESP_RST_BROWNOUT:  log_message("[SYS] Reset: Brownout — check power supply!"); break;
    default:                log_message("[SYS] Reset: Other/Unknown");
  }
}

String getTimestamp() {
  if (!rtcReady) return "Time Not Set";
  DateTime now = rtc.now();
  if (now.year() < 2024) return "Time Not Set";
  char buf[25];
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
  return String(buf);
}

// V3-3: median of VALID pings only; returns 0 when fewer than 3 are valid.
unsigned int getMedianDistanceCm() {
  unsigned int readings[5]; int valid = 0;
  for (int i = 0; i < 5; i++) {
    unsigned int d = sonar.ping_cm();
    if (d > 0 && d <= MAX_DISTANCE_CM) readings[valid++] = d;
    delay(50);
  }
  if (valid < 3) return 0;
  std::sort(readings, readings + valid);
  return readings[valid / 2];
}

// V3-4: fleet-standard absolute thresholds
String statusForLevel(float lvl) {
  if (lvl < 0)                 return "SensorError";
  if (lvl < STATUS_LOW_CM)     return "Low";
  if (lvl < STATUS_GOOD_CM)    return "Good";
  if (lvl < STATUS_EXCESS_CM)  return "Excess";
  return "Flood Alert";
}

String deviceIdFromMac() {
  char buf[16];
  snprintf(buf, sizeof(buf), "GSM-%06X", (uint32_t)(ESP.getEfuseMac() >> 24) & 0xFFFFFF);
  return String(buf);
}

// V3-12: merge a leftover temp file back into backup.txt (never delete data)
void recoverTempFile(const char* tmpPath) {
  if (!lfsReady || !LittleFS.exists(tmpPath)) return;
  File src = LittleFS.open(tmpPath, "r");
  File dst = LittleFS.open("/backup.txt", "a");
  int n = 0;
  if (src && dst) {
    while (src.available()) {
      esp_task_wdt_reset();
      String line = src.readStringUntil('\n'); line.trim();
      if (line.length() > 5) { dst.println(line); n++; }
    }
  }
  bool ok = (src && dst);
  if (src) src.close();
  if (dst) dst.close();
  if (ok) { LittleFS.remove(tmpPath); log_message("[LFS] Recovered " + String(n) + " lines from " + String(tmpPath)); }
}

// ======================================================
// 💾 LITTLEFS — PRIMARY STORAGE
// ======================================================

// Returns how full backup.txt is relative to total LittleFS space, as a %.
// Used to decide when to drain to SD.
int lfsBackupUsagePercent() {
  if (!lfsReady) return 0;
  size_t total = LittleFS.totalBytes();
  if (total == 0) return 0;
  File f = LittleFS.open("/backup.txt", "r");
  size_t backupSize = f ? f.size() : 0;
  if (f) f.close();
  // We measure backup.txt size against total LittleFS partition (conservative)
  return (int)((backupSize * 100UL) / total);
}

bool lfsLoadConfig() {
  if (!lfsReady) return false;
  File f = LittleFS.open("/config.json", "r");
  if (!f) {
    log_message("[Config] config.json not found in LittleFS.");
    return false;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    log_message("[Config] Failed to parse config.json: " + String(err.c_str()));
    return false;
  }
  wifi_ssid    = doc["wifi_ssid"]    | wifi_ssid;
  wifi_pass    = doc["wifi_pass"]    | wifi_pass;
  gsm_apn      = doc["gsm_apn"]      | gsm_apn;
  pipeHeightCm = doc["pipe_height"]  | pipeHeightCm;
  normalSleepS = doc["normal_sleep_s"]| normalSleepS;
  tzOffsetS    = doc["tz_offset_s"]  | tzOffsetS;
  if (normalSleepS < 60)    normalSleepS = 60;       // V3-8 clamp
  if (normalSleepS > 86400) normalSleepS = 86400;
  if (pipeHeightCm < 5.0f || pipeHeightCm > 300.0f) pipeHeightCm = 55.0f;
  log_message("[Config] Loaded from LittleFS.");
  return true;
}

void lfsSaveConfig() {
  if (!lfsReady) return;
  LittleFS.remove("/config.json");
  File f = LittleFS.open("/config.json", "w");
  if (!f) { log_message("[Config] Failed to write config.json"); return; }
  JsonDocument doc;
  doc["wifi_ssid"]      = wifi_ssid;
  doc["wifi_pass"]      = wifi_pass;
  doc["gsm_apn"]        = gsm_apn;
  doc["pipe_height"]    = pipeHeightCm;
  doc["normal_sleep_s"] = normalSleepS;
  doc["tz_offset_s"]    = tzOffsetS;
  if (serializeJson(doc, f) == 0) log_message("[Config] Write failed.");
  else                             log_message("[Config] Saved to LittleFS.");
  f.close();
}

// Append one JSON line to LittleFS backup.
// After writing, check if we should drain to SD.
void lfsSaveBackup(const String& jsonLine) {
  if (!lfsReady) return;
  File f = LittleFS.open("/backup.txt", "a");
  if (!f) { log_message("[LFS] Cannot open backup.txt for append."); return; }
  f.println(jsonLine);
  f.close();
  log_message("[LFS] Backup saved to LittleFS.");

  // Check if we need to drain oldest entries to SD
  int pct = lfsBackupUsagePercent();
  log_message("[LFS] Backup usage: " + String(pct) + "% of LittleFS partition.");
  if (pct >= LFS_SPILL_PERCENT) {
    log_message("[LFS] Threshold reached — draining oldest entries to SD card.");
    drainLfsToSd();
  }
}

// ======================================================
// 💿 SD CARD — ARCHIVE / OVERFLOW ONLY
// ======================================================

bool initSd() {
  for (int attempt = 0; attempt < 3; attempt++) {
    if (SD.begin(SD_CS, SPI)) {
      sdReady = true;
      currentData.sdSizeMB = SD.cardSize() / (1024.0f * 1024.0f);
      currentData.sdFreeMB = (SD.totalBytes() - SD.usedBytes()) / (1024.0f * 1024.0f);
      log_message("[SD] Initialized. Size: " + String(currentData.sdSizeMB, 1) + " MB");
      return true;
    }
    log_message("[SD] Init attempt " + String(attempt + 1) + " failed. Retrying...");
    delay(300);
  }
  log_message("[SD] SD card not available — archive features disabled.");
  return false;
}

void sdAppendLine(const String& line) {
  if (!sdReady) return;
  File f = SD.open("/archive.txt", FILE_APPEND);
  if (!f) { log_message("[SD] Cannot open archive.txt for append."); return; }
  f.println(line);
  f.close();
}

// V3-5: Drain the OLDEST entries from LittleFS backup.txt to SD archive.txt,
// streaming through a temp file. Never holds more than one line in RAM.
// Without an SD card the oldest lines are discarded (logged).
void drainLfsToSd() {
  if (!lfsReady) return;
  if (!sdReady) initSd();

  File src = LittleFS.open("/backup.txt", "r");
  if (!src || src.size() == 0) { if (src) src.close(); return; }
  size_t totalBytes = src.size();
  src.close();

  size_t keepBytes = (LittleFS.totalBytes() * LFS_TARGET_PERCENT) / 100;
  if (totalBytes <= keepBytes) return;
  size_t dropBytes = totalBytes - keepBytes;

  if (!LittleFS.rename("/backup.txt", "/drain.tmp")) {
    log_message("[LFS] drain: rename failed."); return;
  }
  File in  = LittleFS.open("/drain.tmp", "r");
  File out = LittleFS.open("/backup.txt", "w");
  if (!in || !out) {
    if (in) in.close(); if (out) out.close();
    recoverTempFile("/drain.tmp");
    return;
  }
  size_t consumed = 0; int drained = 0, kept = 0;
  while (in.available()) {
    esp_task_wdt_reset();
    String line = in.readStringUntil('\n'); line.trim();
    size_t cost = line.length() + 1;
    if (line.length() <= 5) { consumed += cost; continue; }
    if (consumed < dropBytes) {
      if (sdReady) sdAppendLine(line);
      drained++;
    } else {
      out.println(line); kept++;
    }
    consumed += cost;
  }
  in.close(); out.close();
  LittleFS.remove("/drain.tmp");
  log_message("[LFS->SD] " + String(drained) + (sdReady ? " archived to SD, " : " DISCARDED (no SD), ") +
              String(kept) + " kept. Usage now " + String(lfsBackupUsagePercent()) + "%");
}

// ======================================================
// 📶 GSM FUNCTIONS (Quectel EC200U)
// ======================================================

// Improved: breaks early on known terminators instead of always waiting full timeout.
String gsmSendCommand(const String& cmd, uint32_t timeout, const String& waitFor) {
  while (gsmSerial.available()) gsmSerial.read(); // flush stale data
  gsmSerial.println(cmd);

  String response = "";
  response.reserve(256);
  unsigned long startTime = millis();

  while (millis() - startTime < timeout) {
    esp_task_wdt_reset();
    while (gsmSerial.available()) {
      response += (char)gsmSerial.read();
    }
    // Break early if we have a definitive terminator
    if (waitFor.length() > 0) {
      if (response.indexOf(waitFor) != -1) break;
      if (response.indexOf("\r\nERROR") != -1 || response.indexOf("+CME ERROR") != -1) break; // V3-7
    } else {
      if (response.indexOf("OK\r\n")    != -1 ||
          response.indexOf("ERROR")     != -1 ||
          response.indexOf("CONNECT")   != -1 ||
          response.indexOf("+CME ERROR")!= -1) break;
    }
    delay(5);
  }
  return response;
}

bool syncRtcWithGsmTime() {
  log_message("[RTC] Syncing with GSM network time...");
  String response = gsmSendCommand("AT+CCLK?", 5000, "+CCLK:");

  int startIndex = response.indexOf('"');
  if (startIndex == -1) return false;

  int year, month, day, hour, minute, second, tzQ = 0;
  // AT+CCLK? format: "yy/MM/dd,hh:mm:ss±zz"  (zz = quarter hours, local time)
  const char* p = response.c_str() + startIndex + 1;
  if (sscanf(p, "%d/%d/%d,%d:%d:%d", &year, &month, &day, &hour, &minute, &second) != 6) return false;
  bool haveTz = false;
  { const char* q = strchr(p, '"'); String head = q ? String(p).substring(0, q - p) : String(p);
    int sp = head.lastIndexOf('+'); int sm = head.lastIndexOf('-');
    int at = max(sp, sm);
    if (at > 10) { tzQ = head.substring(at + 1).toInt(); if (sm > sp) tzQ = -tzQ; haveTz = true; } }

  // Sanity check on 2-digit year: accept 20-99 (year 2020–2099)
  if (year < 20 || year > 99) return false;
  if (month < 1 || month > 12 || day < 1 || day > 31) return false;
  if (hour > 23 || minute > 59 || second > 59)         return false;

  // GSM CCLK can return local or UTC depending on CLTS and network.
  // We set CLTS=1 earlier, so most networks return local time.
  // If your network returns UTC, set tzOffsetS to 0 and handle NTP offset separately.
  DateTime networkTime(year + 2000, month, day, hour, minute, second);
  // V3-6: with CTZU=1 the modem reports LOCAL time plus the ±zz offset.
  // Only when the network gave no offset (zz absent or 0 → UTC) do we add
  // the configured tz offset ourselves.
  if (haveTz && tzQ != 0) rtc.adjust(networkTime);
  else                    rtc.adjust(networkTime + TimeSpan(tzOffsetS));

  log_message("[RTC] Synced via GSM. Time: " + getTimestamp());
  return true;
}

void getAndShowSimInfo() {
  log_message("[GSM] Getting SIM info...");

  // --- Phone number via AT+CNUM ---
  // Response: +CNUM: "","<number>",<type>
  // More robust: find the number between the 2nd pair of quotes on the +CNUM line
  String cnumResp = gsmSendCommand("AT+CNUM", 3000, "OK");
  int lineStart = cnumResp.indexOf("+CNUM:");
  if (lineStart != -1) {
    int lineEnd = cnumResp.indexOf('\n', lineStart);
    String cnumLine = cnumResp.substring(lineStart, lineEnd);
    // Fields: +CNUM: "<alpha>","<number>",<type>
    int q1 = cnumLine.indexOf('"');
    int q2 = q1 != -1 ? cnumLine.indexOf('"', q1 + 1) : -1; // end of alpha
    int q3 = q2 != -1 ? cnumLine.indexOf('"', q2 + 1) : -1; // start of number
    int q4 = q3 != -1 ? cnumLine.indexOf('"', q3 + 1) : -1; // end of number
    if (q3 != -1 && q4 != -1) {
      String candidate = cnumLine.substring(q3 + 1, q4);
      // Validate: must start with + or digit and be at least 7 chars
      if (candidate.length() >= 7 &&
          (candidate.charAt(0) == '+' || isDigit(candidate.charAt(0)))) {
        currentData.sim = candidate;
      }
    }
  }

  // --- Signal strength via AT+CSQ ---
  // Response: +CSQ: <rssi>,<ber>   rssi 0–31, 99=unknown
  String csqResp = gsmSendCommand("AT+CSQ", 2000, "OK");
  int csqColon = csqResp.indexOf('+');
  int csqComma = csqResp.indexOf(',');
  if (csqColon != -1 && csqComma != -1) {
    int rssi = csqResp.substring(csqColon + 6, csqComma).toInt();
    currentData.gsmStrength = (rssi == 99) ? -1 : rssi;
  }
}

String autoDetectApn() {
  log_message("[GSM] Auto-detecting APN via IMSI...");
  String imsiResp = gsmSendCommand("AT+CIMI", 3000, "OK");
  String imsi = "";
  for (unsigned int i = 0; i < imsiResp.length(); i++) {
    if (isDigit(imsiResp.charAt(i))) imsi += imsiResp.charAt(i);
  }

  currentData.simOperator = "Unknown";
  if (imsi.length() < 6) {
    log_message("[GSM] IMSI too short, using config APN: " + gsm_apn);
    return gsm_apn;
  }

  // India MCC 405 — Jio (all MNCs)
  if (imsi.startsWith("405")) {
    currentData.simOperator = "Jio";
    return "jionet";
  }

  // India MCC 404
  if (imsi.startsWith("404")) {
    int mnc = imsi.substring(3, 5).toInt();
    // Airtel MNCs under 404
    if (mnc==10||mnc==31||mnc==45||mnc==49||mnc==70||(mnc>=92&&mnc<=98)) {
      currentData.simOperator = "Airtel"; return "airtelgprs.com";
    }
    // Vodafone Idea MNCs
    if (mnc==1||mnc==5||mnc==11||mnc==20||mnc==27||mnc==30||mnc==46||mnc==84||mnc==86) {
      currentData.simOperator = "Vodafone Idea"; return "www";
    }
    // BSNL
    if (mnc==34||(mnc>=51&&mnc<=55)||(mnc>=57&&mnc<=59)) {
      currentData.simOperator = "BSNL"; return "bsnlnet";
    }
  }

  // India MCC 406 (BSNL circles)
  if (imsi.startsWith("406")) {
    currentData.simOperator = "BSNL"; return "bsnlnet";
  }

  log_message("[GSM] IMSI prefix '" + imsi.substring(0, 5) + "' not matched — using config APN: " + gsm_apn);
  return gsm_apn;
}

bool gsmConnect() {
  log_message("[GSM] Establishing 4G/LTE connection...");

  // Baud rate check — try 115200 first, fall back to 9600 auto-baud recovery
  if (gsmSendCommand("AT", 1000, "OK").indexOf("OK") == -1) {
    log_message("[GSM] No response at 115200. Trying 9600 baud recovery...");
    gsmSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    delay(500);
    if (gsmSendCommand("AT", 1000, "OK").indexOf("OK") != -1) {
      gsmSendCommand("AT+IPR=115200", 1000);
      delay(200);
    }
    gsmSerial.begin(GSM_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    delay(500);
    if (gsmSendCommand("AT", 1000, "OK").indexOf("OK") == -1) {
      log_message("[GSM] Module not responding. Check power and wiring.");
      return false;
    }
  }
  modemAlive = true;   // V3-2: from here on we always shut the modem down

  // Wake module from sleep / ensure full functionality
  log_message("[GSM] Setting full functionality...");
  gsmSendCommand("AT+CFUN=1", 8000, "OK");
  delay(2000);
  gsmSendCommand("ATE0",    1000, "OK");  // echo off
  gsmSendCommand("AT+CTZU=1", 1000, "OK"); // V3-6: Quectel automatic time-zone update

  log_message("[GSM] Waiting for network registration (up to 60s)...");
  unsigned long startTime = millis();
  while (millis() - startTime < 60000) {
    String ceregResp = gsmSendCommand("AT+CEREG?", 1000, "OK");
    if (ceregResp.indexOf(",1") != -1 || ceregResp.indexOf(",5") != -1) {
      log_message("[GSM] Registered on LTE network.");
      currentData.network = "GSM/LTE";

      getAndShowSimInfo();
      if (rtcReady) syncRtcWithGsmTime();

      // Deactivate any existing PDP context before reconfiguring
      gsmSendCommand("AT+QIDEACT=1", 10000);
      delay(1000);

      String detectedApn = autoDetectApn();
      log_message("[GSM] APN: " + detectedApn);
      gsmSendCommand("AT+QICSGP=1,1,\"" + detectedApn + "\",\"\",\"\",1", 5000, "OK");
      delay(1000);

      log_message("[GSM] Activating PDP context...");
      String actResp = gsmSendCommand("AT+QIACT=1", 15000, "OK");
      if (actResp.indexOf("OK") == -1) {
        log_message("[GSM] PDP activation failed.");
        return false;
      }

      String ipResp = gsmSendCommand("AT+QIACT?", 5000, "OK");
      if (ipResp.indexOf('.') == -1) {
        log_message("[GSM] No IP address assigned.");
        return false;
      }
      log_message("[GSM] IP obtained.");
      activeConnection = GSM;   // V3-11: only once data is really usable
      return true;
    }
    for (int i = 0; i < 20; i++) { delay(100); esp_task_wdt_reset(); }
  }
  log_message("[GSM] Network registration timed out.");
  return false;
}

bool gsmPOST(const String& jsonData, bool setUrl) {
  if (setUrl) {
    String urlCmd = "AT+QHTTPURL=" + String(strlen(GSCRIPT_URL)) + ",80";
    if (gsmSendCommand(urlCmd, 5000, "CONNECT").indexOf("CONNECT") == -1) {
      log_message("[GSM-POST] URL set failed.");
      return false;
    }
    delay(200);
    gsmSendCommand(GSCRIPT_URL, 10000);
    delay(200);
  }

  String postCmd = "AT+QHTTPPOST=" + String(jsonData.length()) + ",60,60";
  if (gsmSendCommand(postCmd, 5000, "CONNECT").indexOf("CONNECT") == -1) {
    log_message("[GSM-POST] POST initiation failed.");
    return false;
  }

  delay(200);
  gsmSerial.print(jsonData);

  String finalResponse = "";
  finalResponse.reserve(128);
  unsigned long startTime = millis();
  while (millis() - startTime < 65000) {   // V3-7: rsptime is 60 s
    esp_task_wdt_reset();
    while (gsmSerial.available()) finalResponse += (char)gsmSerial.read();
    if (finalResponse.indexOf("+QHTTPPOST:") != -1) break;
    delay(20);
  }

  // Accept HTTP 200 or 302 (Google Script redirect)
  if (finalResponse.indexOf("+QHTTPPOST: 0,200") != -1 ||
      finalResponse.indexOf("+QHTTPPOST: 0,302") != -1) {
    log_message("[GSM-POST] Success.");
    gsmSendCommand("AT+QHTTPREAD=80", 10000);
    return true;
  }
  log_message("[GSM-POST] Failed. Response: " + finalResponse);
  return false;
}

// ======================================================
// 💬 SMS HANDLING
// ======================================================
bool sendSms(const String& number, const String& message) {
  log_message("[SMS] Sending to: " + number);
  gsmSendCommand("AT+CMGF=1", 1000, "OK");

  String cmd = "AT+CMGS=\"" + number + "\"";
  String response = gsmSendCommand(cmd, 3000, ">");

  if (response.indexOf('>') == -1) {
    log_message("[SMS] No prompt received.");
    currentData.smsStatus = "Failed (no prompt)";
    return false;
  }

  gsmSerial.print(message);
  gsmSerial.write(26); // Ctrl+Z to send

  String sendResp = "";
  unsigned long startTime = millis();
  while (millis() - startTime < 30000) {
    while (gsmSerial.available()) sendResp += (char)gsmSerial.read();
    if (sendResp.indexOf("OK")    != -1) { currentData.smsStatus = "Replied to " + number; return true; }
    if (sendResp.indexOf("ERROR") != -1) break;
    delay(50);
  }
  currentData.smsStatus = "Send failed";
  return false;
}

void checkAndProcessSms() {
  log_message("[SMS] Checking inbox...");
  gsmSendCommand("AT+CMGF=1", 1000, "OK");
  String response = gsmSendCommand("AT+CMGL=\"ALL\"", 8000, "OK");

  if (response.indexOf("+CMGL:") == -1) {
    log_message("[SMS] Inbox empty or no response.");
    return;
  }

  int searchPos = 0;
  while (true) {
    int headerIndex = response.indexOf("+CMGL:", searchPos);
    if (headerIndex == -1) break;

    int headerEnd = response.indexOf('\n', headerIndex);
    if (headerEnd == -1) break;

    String headerLine = response.substring(headerIndex, headerEnd);

    // Parse: +CMGL: <index>,"<status>","<sender>",<date>
    // Use indexOf/substring — safer than sscanf for AT response strings
    int msgIndex = -1;
    String senderNumber = "";

    int firstComma = headerLine.indexOf(',');
    if (firstComma != -1) {
      msgIndex = headerLine.substring(7, firstComma).toInt(); // after "+CMGL: "
    }

    // Find sender — it is the 3rd field (after index and status)
    // Fields separated by commas; status is quoted so we find 3rd comma
    int c1 = headerLine.indexOf(',');
    int c2 = c1 != -1 ? headerLine.indexOf(',', c1 + 1) : -1;
    if (c2 != -1) {
      // Now extract quoted value right after c2
      int q1 = headerLine.indexOf('"', c2);
      int q2 = q1 != -1 ? headerLine.indexOf('"', q1 + 1) : -1;
      if (q1 != -1 && q2 != -1) {
        String candidate = headerLine.substring(q1 + 1, q2);
        if (candidate.length() >= 7 &&
            (candidate.charAt(0) == '+' || isDigit(candidate.charAt(0)))) {
          senderNumber = candidate;
        }
      }
    }

    // Message body is on the next line
    int bodyStart = headerEnd + 1;
    int nextHeader = response.indexOf("+CMGL:", bodyStart);
    int bodyEnd = (nextHeader == -1) ? response.length() : nextHeader;

    String body = response.substring(bodyStart, bodyEnd);
    body.trim();
    body.toLowerCase();

    log_message("[SMS] From: " + senderNumber + " | Body: " + body);

    if (body.indexOf("hi") != -1 && senderNumber.length() >= 7) {
      currentData.smsStatus = "Received 'hi' from " + senderNumber;
      // Store reply target in LittleFS (not SD)
      if (lfsReady) {
        File replyFile = LittleFS.open("/sms_reply.txt", "w");
        if (replyFile) {
          replyFile.print(senderNumber);
          replyFile.close();
          log_message("[SMS] Reply target saved to LittleFS.");
        }
      }
    }

    // Delete message from SIM regardless
    if (msgIndex >= 0) {
      gsmSendCommand("AT+CMGD=" + String(msgIndex), 5000, "OK");
    }

    searchPos = bodyEnd;
  }
}

// ======================================================
// 🔄 OFFLINE DATA FLUSH (LittleFS → Cloud)
// ======================================================
void flushOfflineData() {
  if (activeConnection == NONE) return;
  if (!lfsReady) return;

  File src = LittleFS.open("/backup.txt", "r");
  if (!src || src.size() < 5) {
    if (src) src.close();
    return;
  }

  log_message("[Flush] Flushing LittleFS backup to cloud...");

  // Rename to a temp file to mark flush-in-progress
  // (survives a crash mid-flush — on next boot we recover the .tmp)
  src.close();
  LittleFS.rename("/backup.txt", "/flushing.tmp");

  File flushFile = LittleFS.open("/flushing.tmp", "r");
  if (!flushFile) return;

  File failFile;       // will hold lines that failed to send
  bool failFileOpen  = false;
  bool firstGsmPost  = true;
  int  sentCount     = 0;
  int  failCount     = 0;
  int  consecutiveFails = 0;

  while (flushFile.available()) {
    esp_task_wdt_reset();
    String line = flushFile.readStringUntil('\n');
    line.trim();
    if (line.length() < 5) continue;

    bool sent = false;
    // V3-11: give up after FLUSH_MAX_FAILS in a row — keep everything else
    if (consecutiveFails >= FLUSH_MAX_FAILS) {
      if (!failFileOpen) { failFile = LittleFS.open("/backup.txt", "a"); failFileOpen = true; }
      if (failFileOpen) failFile.println(line);
      failCount++;
      continue;
    }

    if (activeConnection == GSM) {
      sent = gsmPOST(line, firstGsmPost);
      if (sent)  firstGsmPost = false;
      else       firstGsmPost = true; // force URL re-send on next attempt
    } else if (activeConnection == WIFI) {
      HTTPClient http;
      http.begin(GSCRIPT_URL);
      http.addHeader("Content-Type", "application/json");
      int code = http.POST(line);
      http.end();
      sent = (code >= 200 && code < 400);
    }

    if (sent) {
      sentCount++; consecutiveFails = 0;
    } else {
      failCount++; consecutiveFails++;
      if (!failFileOpen) {
        failFile = LittleFS.open("/backup.txt", "a");
        failFileOpen = true;
      }
      if (failFileOpen) failFile.println(line);
    }
  }

  flushFile.close();
  if (failFileOpen) failFile.close();
  LittleFS.remove("/flushing.tmp");

  log_message("[Flush] Done. Sent: " + String(sentCount) + " | Failed: " + String(failCount));
}

// ======================================================
// 🖥 WEB SERVER — CONFIG PORTAL
// ======================================================
void startConfigPortal() {
  log_message("[Portal] Starting configuration portal...");

  // Unique AP password from chip ID
  char apPass[9];
  snprintf(apPass, sizeof(apPass), "%08X", (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFF));
  WiFi.softAP("WaterMonitor-Setup", apPass);
  log_message("[Portal] AP: WaterMonitor-Setup | Pass: " + String(apPass));
  log_message("[Portal] IP: " + WiFi.softAPIP().toString());

  server.on("/",        HTTP_GET,  handleRoot);
  server.on("/save",    HTTP_POST, handleSave);
  server.on("/data",    HTTP_GET,  handleLiveData);
  server.on("/download",HTTP_GET,  handleDownload);
  server.on("/erase",   HTTP_POST, handleErase);
  server.begin();

  while (!shouldSaveConfig) {
    server.handleClient();
    delay(10);
  }

  log_message("[Portal] Config saved. Restarting...");
  delay(1000);
  ESP.restart();
}

// ======================================================
// 🚀 MAIN SETUP
// ======================================================
void setup() {
  Serial.begin(115200);
  gsmSerial.begin(GSM_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(100);

  log_message("\n[SYS] === Water Monitor V3 Booting ===");
  print_reset_reason();

  // V3-7: task watchdog covers the whole wake
  esp_task_wdt_deinit();
  esp_task_wdt_config_t wdt = { .timeout_ms = WDT_TIMEOUT_S * 1000, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_init(&wdt);
  esp_task_wdt_add(NULL);
  deviceId = deviceIdFromMac();

  // --- LittleFS (PRIMARY storage — must init first) ---
  // V3-9: never format on the first failed mount
  bool lfsOk = LittleFS.begin(false);
  {
    Preferences prefs; prefs.begin("sys", false);
    uint32_t fails = prefs.getUInt("lfsfail", 0);
    if (!lfsOk) {
      fails++;
      log_message("[LFS] Mount failed (" + String(fails) + "/3).");
      if (fails >= 3) { log_message("[LFS] Formatting LittleFS."); lfsOk = LittleFS.begin(true); fails = 0; }
    } else fails = 0;
    prefs.putUInt("lfsfail", fails); prefs.end();
  }
  if (lfsOk) {
    lfsReady = true;
    log_message("[LFS] LittleFS ready. Total: " + String(LittleFS.totalBytes()) +
                " B | Used: " + String(LittleFS.usedBytes()) + " B");

    // V3-12: recover any interrupted flush / drain from the previous session
    recoverTempFile("/flushing.tmp");
    recoverTempFile("/drain.tmp");
  } else {
    log_message("[LFS] LittleFS FAILED — this is a critical error. Check partition scheme.");
    // Continue anyway — device will fall back to online-only mode
  }

  // --- RTC ---
  Wire.begin(SDA_PIN, SCL_PIN);
  if (rtc.begin()) {
    rtcReady = true;
    log_message("[RTC] Initialized.");
    if (rtc.lostPower()) {
      // Set compile-time as a placeholder; will be corrected via GSM/NTP
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      log_message("[RTC] Lost power — set to compile time as placeholder.");
    }
  } else {
    log_message("[RTC] Not found — timestamps will be unavailable.");
  }

  // --- SD Card (optional archive — init here to check availability) ---
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  initSd(); // non-fatal — sdReady will be false if unavailable

  // V3-8: load config BEFORE the portal so it shows real values
  if (!lfsLoadConfig()) {
    log_message("[Config] Using hardcoded defaults (normalSleepS=" + String(normalSleepS) + "s).");
  }

  // --- Config portal check ---
  pinMode(CONFIG_JUMPER_PIN, INPUT_PULLUP);
  delay(100);
  if (digitalRead(CONFIG_JUMPER_PIN) == LOW) startConfigPortal();

  // ======================================================
  // V3-1: SENSOR FIRST, WRITE-AHEAD TO FLASH, THEN NETWORK
  // ======================================================
  unsigned int distanceCm = getMedianDistanceCm();
  if (distanceCm == 0) {
    currentData.waterLevel = -1.0f;
    log_message("[Sensor] FAILURE — fewer than 3 valid pings.");
  } else {
    currentData.waterLevel = constrain(pipeHeightCm - (float)distanceCm, 0.0f, pipeHeightCm);
  }
  currentData.status    = statusForLevel(currentData.waterLevel);
  currentData.timestamp = getTimestamp();
  log_message("[Sensor] Distance=" + String(distanceCm) + "cm Level=" + String(currentData.waterLevel, 1) +
              "cm Status=" + currentData.status);
  {
    JsonDocument doc;
    doc["timestamp"]   = currentData.timestamp;
    doc["network"]     = "pending";
    doc["sim"]         = "N/A";
    doc["simOperator"] = "N/A";
    doc["wifiStrength"]= 0;
    doc["gsmStrength"] = -1;
    if (currentData.waterLevel < 0) doc["waterLevel"] = nullptr;
    else                            doc["waterLevel"] = currentData.waterLevel;
    doc["status"]      = currentData.status;
    doc["device"]      = currentData.device;
    doc["deviceId"]    = deviceId;
    doc["dataType"]    = "Live";
    doc["smsStatus"]   = "None";
    String line; serializeJson(doc, line);
    lfsSaveBackup(line);            // write-ahead: safe before the modem runs
  }

  // --- Network ---
  bool gsmOk = gsmConnect();

  if (!gsmOk && wifi_ssid.length() > 0) {   // V3-10
    log_message("[SYS] GSM failed — trying WiFi fallback...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
    for (int i = 20; i > 0 && WiFi.status() != WL_CONNECTED; i--) delay(500);

    if (WiFi.status() == WL_CONNECTED) {
      activeConnection = WIFI;
      currentData.network = "WiFi";
      currentData.wifiStrength = WiFi.RSSI();
      log_message("[WiFi] Connected. IP: " + WiFi.localIP().toString());
    } else {
      log_message("[SYS] No network available — data will be saved to LittleFS.");
    }
  } else {
    // Only check SMS when on GSM
    checkAndProcessSms();
  }

  // --- SMS reply (target stored in LittleFS) ---
  if (activeConnection == GSM && lfsReady && LittleFS.exists("/sms_reply.txt")) {
    File replyFile = LittleFS.open("/sms_reply.txt", "r");
    String replyNumber = "";
    if (replyFile && replyFile.size() > 5) { replyNumber = replyFile.readString(); replyNumber.trim(); }
    if (replyFile) replyFile.close();
    if (replyNumber.length() > 5) {
      String msg = "Water level: " + (currentData.waterLevel < 0 ? String("sensor error") : String(currentData.waterLevel, 2) + " cm") +
                   " | Status: " + currentData.status + " | " + currentData.timestamp;
      if (sendSms(replyNumber, msg)) LittleFS.remove("/sms_reply.txt");   // V3: delete only after success
    } else {
      LittleFS.remove("/sms_reply.txt");
    }
  }

  // --- Upload everything (this cycle's line + any backlog) ---
  bool dataSent = false;
  if (activeConnection != NONE) {
    flushOfflineData();
    dataSent = true;
  }

  // --- NTP time sync over WiFi (more accurate than GSM CLTS) ---
  if (dataSent && rtcReady && activeConnection == WIFI) {
    configTime(tzOffsetS, 0, "pool.ntp.org", "time.cloudflare.com");
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 6000)) {
      rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1,
                          timeinfo.tm_mday, timeinfo.tm_hour,
                          timeinfo.tm_min,  timeinfo.tm_sec));
      log_message("[NTP] RTC updated via NTP. Time: " + getTimestamp());
    }
  }

  // --- Shut down connections before sleep ---
  if (activeConnection == WIFI) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    log_message("[WiFi] Disconnected.");
  }
  if (modemAlive) {   // V3-2: even when registration/PDP failed
    if (gsmOk) gsmSendCommand("AT+QIDEACT=1", 10000, "OK");
    gsmSendCommand("AT+CFUN=0", 5000, "OK"); // low power mode
    log_message("[GSM] Deactivated.");
  }

  // --- Deep sleep ---
  deepSleepSeconds = normalSleepS;
  log_message("[Sleep] Sleeping for " + String(deepSleepSeconds) + " seconds. Goodbye.");
  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)deepSleepSeconds * 1000000ULL); // cast prevents overflow
  esp_deep_sleep_start();
}

void loop() { /* Empty — deep sleep architecture means loop never runs */ }

// ======================================================
// 🖥 WEB SERVER ENDPOINT HANDLERS
// ======================================================
void handleRoot() {
  if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD))
    return server.requestAuthentication();

  // Storage info for display
  String lfsInfo = "N/A";
  if (lfsReady) {
    size_t total = LittleFS.totalBytes();
    size_t used  = LittleFS.usedBytes();
    int pct = (total > 0) ? (int)((used * 100UL) / total) : 0;
    lfsInfo = String(used / 1024) + " KB used / " + String(total / 1024) + " KB total (" + String(pct) + "%)";
  }
  String sdInfo = sdReady
    ? (String(currentData.sdSizeMB, 0) + " MB total, " + String(currentData.sdFreeMB, 0) + " MB free")
    : "Not detected";

  String html = R"rawliteral(
<!DOCTYPE html><html><head><title>Water Monitor Config</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body{font-family:sans-serif;background:#f4f4f4;margin:0;padding:20px;}
  .card{padding:20px;max-width:500px;margin:0 auto 20px;background:#fff;border-radius:8px;box-shadow:0 2px 5px rgba(0,0,0,.1);}
  input[type=text],input[type=password],input[type=number]{width:100%;padding:10px;margin:6px 0 12px;border:1px solid #ccc;border-radius:4px;box-sizing:border-box;}
  button,.btn{background:#4CAF50;color:#fff;padding:12px 20px;margin:6px 0;border:none;cursor:pointer;width:100%;border-radius:4px;text-align:center;text-decoration:none;display:inline-block;font-size:15px;}
  button:hover,.btn:hover{background:#45a049;}
  .btn-danger{background:#f44336;}.btn-danger:hover{background:#d32f2f;}
  h2,h3{text-align:center;color:#333;margin-top:0;}
  label{font-weight:bold;display:block;margin-top:8px;}
  .info{color:#666;font-size:0.85em;margin:2px 0 8px;}
  #liveDist{font-size:2.4em;font-weight:bold;color:#2c3e50;text-align:center;display:block;margin:8px 0;}
  .storage-row{display:flex;justify-content:space-between;font-size:0.85em;color:#555;padding:4px 0;border-bottom:1px solid #eee;}
  .storage-row:last-child{border-bottom:none;}
</style>
</head><body>

<div class="card">
  <h2>Live Calibration</h2>
  <span id="liveDist">--</span>
  <p class="info" style="text-align:center">Distance reading from sensor (cm)</p>
</div>

<div class="card">
  <h3>Storage Status</h3>
  <div class="storage-row"><span>LittleFS (primary)</span><span>)rawliteral";
  html += lfsInfo;
  html += R"rawliteral(</span></div>
  <div class="storage-row"><span>SD Card (archive)</span><span>)rawliteral";
  html += sdInfo;
  html += R"rawliteral(</span></div>
</div>

<div class="card">
  <h2>Configuration</h2>
  <form action="/save" method="POST">
    <h3>Connectivity</h3>
    <label>WiFi SSID</label>
    <input type="text" name="ssid" value=")rawliteral";
  html += wifi_ssid;
  html += R"rawliteral(">
    <label>WiFi Password</label>
    <input type="password" name="pass" placeholder="Leave blank to keep existing">
    <label>GSM APN <span style="font-weight:normal;color:#888">(blank = auto-detect)</span></label>
    <input type="text" name="apn" value=")rawliteral";
  html += gsm_apn;
  html += R"rawliteral(">

    <h3>Sensor & Timing</h3>
    <label>Pipe Height (cm)</label>
    <input type="number" step="0.1" min="1" name="pipe_height" value=")rawliteral";
  html += String(pipeHeightCm);
  html += R"rawliteral(">
    <label>Sleep Interval (seconds, min 60)</label>
    <input type="number" min="60" name="normal_sleep_s" value=")rawliteral";
  html += String(normalSleepS);
  html += R"rawliteral(">
    <label>Timezone Offset (seconds, IST=19800)</label>
    <input type="number" name="tz_offset_s" value=")rawliteral";
  html += String(tzOffsetS);
  html += R"rawliteral(">

    <button type="submit">Save &amp; Restart</button>
  </form>
</div>

<div class="card">
  <h3>Data Management</h3>
  <a href="/download?src=lfs"  class="btn" download="lfs_backup.txt">Download LittleFS Backup</a>
  <a href="/download?src=sd"   class="btn" download="sd_archive.txt">Download SD Archive</a>
  <button class="btn btn-danger" onclick="confirmErase('lfs')">Erase LittleFS Backup</button>
  <button class="btn btn-danger" onclick="confirmErase('sd')">Erase SD Archive</button>
</div>

<script>
setInterval(() => {
  fetch('/data').then(r => r.text()).then(d => {
    document.getElementById('liveDist').textContent = d + ' cm';
  }).catch(() => {});
}, 1000);

function confirmErase(src) {
  const label = src === 'lfs' ? 'LittleFS backup' : 'SD archive';
  if (confirm('Permanently erase ' + label + '? This cannot be undone.')) {
    fetch('/erase?src=' + src, {method:'POST'})
      .then(r => r.text()).then(d => alert(d));
  }
}
</script>
</body></html>)rawliteral";

  server.send(200, "text/html", html);
}

void handleSave() {
  if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD))
    return server.requestAuthentication();

  // Only update if non-empty (prevents accidental blank overwrite)
  if (server.arg("ssid").length() > 0) wifi_ssid = server.arg("ssid");
  if (server.arg("pass").length() > 0) wifi_pass = server.arg("pass");
  if (server.hasArg("apn"))            gsm_apn   = server.arg("apn");

  // Validated numerics — enforce safe minimums
  float newHeight = server.arg("pipe_height").toFloat();
  if (newHeight > 0.5f) pipeHeightCm = newHeight;

  uint32_t newSleep = (uint32_t)server.arg("normal_sleep_s").toInt();
  if (newSleep >= 60) normalSleepS = newSleep;

  int32_t newTz = server.arg("tz_offset_s").toInt();
  if (newTz >= -43200 && newTz <= 50400) tzOffsetS = newTz; // ±12h range

  lfsSaveConfig();
  server.send(200, "text/plain", "Settings saved. Restarting in 1 second...");
  delay(1000);
  shouldSaveConfig = true;
}

void handleLiveData() {
  unsigned int d = sonar.ping_cm();
  if (d == 0) d = MAX_DISTANCE_CM;
  server.send(200, "text/plain", String(d));
}

void handleDownload() {
  String src = server.arg("src");

  if (src == "sd") {
    if (!sdReady) { server.send(503, "text/plain", "SD card not available."); return; }
    File f = SD.open("/archive.txt", FILE_READ);
    if (!f) { server.send(404, "text/plain", "SD archive.txt not found."); return; }
    server.sendHeader("Content-Disposition", "attachment; filename=sd_archive.txt");
    server.streamFile(f, "text/plain");
    f.close();
  } else {
    // Default: LittleFS backup
    if (!lfsReady) { server.send(503, "text/plain", "LittleFS not available."); return; }
    File f = LittleFS.open("/backup.txt", "r");
    if (!f) { server.send(404, "text/plain", "LittleFS backup.txt not found."); return; }
    server.sendHeader("Content-Disposition", "attachment; filename=lfs_backup.txt");
    server.streamFile(f, "text/plain");
    f.close();
  }
}

void handleErase() {
  if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD))
    return server.requestAuthentication();

  String src = server.arg("src");

  if (src == "sd") {
    if (!sdReady) { server.send(503, "text/plain", "SD card not available."); return; }
    SD.remove("/archive.txt");
    server.send(200, "text/plain", "SD archive.txt erased.");
  } else {
    if (!lfsReady) { server.send(503, "text/plain", "LittleFS not available."); return; }
    LittleFS.remove("/backup.txt");
    server.send(200, "text/plain", "LittleFS backup.txt erased.");
  }
}
