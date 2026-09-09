// ============================================================
//  ESP32 Water Level Monitor  —  Field Deployment Build  V8
//  Config  : LittleFS only (SD card never stores config)
//  Storage : LittleFS primary log → SD overflow backup only
//  Sync    : LittleFS + SD → Google Sheets when WiFi up
//
//  V8 (2026-09-07) — audit fixes. Search "V8:" for each change.
//   V8-1  Reading taken and written to LittleFS BEFORE WiFi (write-ahead).
//         A watchdog/brownout during TLS no longer loses the cycle. The
//         direct-send path is gone; syncAllToSheets() sends everything.
//   V8-2  Batch JSON overflow fixed: the 4096-byte doc silently dropped
//         ~1/3 of every 20-record batch and still counted them as sent.
//         Now 10 records per 8 KB doc and arr.add() is checked.
//   V8-3  TLS client timeouts: setTimeout(15) was 15 ms on this core, so
//         header reads could return empty and the redirect was missed
//         (→ duplicate rows). Now 15000 ms + 10 s handshake + 10 s connect.
//   V8-4  WiFi backoff counter fixed (superseded by V8-17 below, which
//         removes boot-skipping altogether).
//   V8-5  Physical-reset detection = real power-on/reset only. Brownouts,
//         panics and watchdog resets no longer count as "farmer pressed
//         reset → force WiFi" (this was a battery-draining loop).
//   V8-6  Brownout detector register restored to its saved value, not
//         written with 1 (which set garbage thresholds).
//   V8-7  Temp-file swaps use rename() alone (LittleFS overwrites); a
//         leftover temp file is merged back at boot instead of truncated.
//   V8-8  WiFi time budget: stop waiting on WL_NO_SSID_AVAIL /
//         WL_CONNECT_FAILED; at most 2 open networks × 10 s.
//   V8-9  RTC no longer set from compile time; a record without a valid
//         clock carries clockValid:false and an empty timestamp.
//   V8-10 LittleFS.begin(false); formats only after 3 consecutive mount
//         failures (counter in NVS).
//   V8-11 Config portal exits after 10 min (GPIO15 is a strapping pin —
//         a stuck jumper used to park the device in AP mode until dead).
//   V8-12 Sleep and pipe height clamped; negative custom seconds no
//         longer become a 136-year sleep.
//   V8-13 Status "Flooding" → "Flood Alert" (fleet standard); median
//         needs ≥3 valid pings; "deviceId" (efuse MAC tail) in every record.
//   V8-14 NTP runs only when the clock is invalid or it is the first upload
//         of the day. Field logs show phone hotspots blocking UDP 123, so
//         the old unconditional 10 s wait burned radio time every boot for
//         a sync that could not succeed, and then wrongly reported the
//         timestamps as invalid while the DS3231 was keeping good time.
//   V8-15 Watchdog 30 s → 60 s: a bounded TLS connect+handshake can block
//         20 s without feeding it, which left too little margin.
//   V8-16 The clock is now corrected from the HTTP "Date:" header returned
//         by every upload. This device reaches the internet through phone
//         hotspots, which block NTP, so NTP alone could never fix a wrong
//         RTC in the field. The header costs nothing: it is already read.
//   V8-17 WiFi acquisition reworked around how the device is really used:
//         there is no WiFi in the field, and a sync happens when a farmer
//         switches on a hotspot and presses reset, once a month or two.
//         The reset button is therefore the sync command and always tries.
//         Timer wakes only take a cheap look about once a day (more often
//         if the buffer is filling). When we do look we scan first and
//         connect only to a network the scan saw, instead of blind-dialling
//         the saved SSID for 15 s in an empty field.
//   V8-18 Upload batch 10 -> 40 records. A month or two of hourly readings
//         is 700-1400 records; at 10 per upload the farmer had to hold the
//         hotspot open for the best part of ten minutes.
//   V8-19 Reset detection made fail-safe. Any boot that is not the ordinary
//         deep-sleep timer wake now counts as a sync request, instead of
//         having to match a list of reset codes. Boards differ in what they
//         report for a button press, and an unrecognised code must never
//         silently disable the only sync path the device has. A fault reset
//         (brownout, panic, watchdog) still counts for the first couple of
//         occurrences, because a tired battery can brown out exactly when
//         the radio starts, and only stops counting once it is clearly a
//         loop.
//   V8-20 A sync request retries for a couple of minutes instead of giving
//         up after one pass. The farmer may press the button before
//         switching the hotspot on, or the phone may take a few seconds to
//         bring it up. Routine timer checks still make a single pass.
// ============================================================
#define PORTAL_TIMEOUT_MS   600000UL

// ---------- Core Libraries ----------
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <NewPing.h>
#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <LittleFS.h>
#include <TelnetStream.h>
#include <Preferences.h>
#include <sys/time.h>   // V8-16: settimeofday() for the HTTP-Date clock
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <rom/rtc.h>
#include <algorithm>
#include <vector>
// Brownout detector control
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/gpio.h"

// ============================================================
//  WATCHDOG
// ============================================================
// V8-15: 30 s was too tight once the TLS timeouts were bounded — a single
// client.connect() can block for up to 10 s (connect) + 10 s (handshake)
// with no chance to feed the watchdog, leaving only 10 s of margin before a
// panic reset mid-upload. 60 s still catches a genuine hang; the device
// deep-sleeps between readings, so a longer ceiling costs nothing.
#define WDT_TIMEOUT_S 60

// ============================================================
//  SMART SLEEP
// ============================================================
#define NORMAL_SLEEP_SECONDS            600    // 10 min default
#define SHORT_SLEEP_SECONDS              60    // 1 min on rapid water change
#define WATER_LEVEL_CHANGE_THRESHOLD_CM 5.0f

// ============================================================
//  LITTLEFS SPILL THRESHOLDS
// ============================================================
#define LFS_SPILL_THRESHOLD  0.75f   // spill to SD when LittleFS >= 75%
#define LFS_SPILL_FRACTION   0.10f   // spill oldest 10% of records per cycle

// ============================================================
//  HARDWARE PINS
// ============================================================
#define CONFIG_JUMPER_PIN  15
#define TRIG_PIN           13
#define ECHO_PIN           27
#define MAX_DISTANCE_CM    200
#define SD_CS               5
#define SD_SCK             18
#define SD_MISO            19
#define SD_MOSI            23
#define SDA_PIN            21
#define SCL_PIN            22

// ============================================================
//  STORAGE FILE PATHS
// ============================================================
#define LFS_DATA_FILE    "/data.txt"       // LittleFS primary log
#define LFS_CONFIG_FILE  "/config.json"    // LittleFS config — NEVER on SD
#define LFS_TEMP_FILE    "/data_tmp.txt"   // LittleFS atomic swap temp
#define SD_BACKUP_FILE   "/backup.txt"     // SD overflow backup
#define SD_TEMP_FILE     "/temp.txt"       // SD atomic swap temp

// ============================================================
//  GOOGLE APPS SCRIPT — update these two if you redeploy
// ============================================================
#define GAS_HOST "script.google.com"
#define GAS_PATH "/macros/s/AKfycbxnK0YDb1hYhx5YPrSOtIgSDY0ctRokhGP_OmMcmzxlM9neGbGBbz5pxFwh-8H00ibgoQ/exec"

// ============================================================
//  WEB PORTAL CREDENTIALS
// ============================================================
const char* CONFIG_USERNAME = "digitalawd";
const char* CONFIG_PASSWORD = "password";

// ============================================================
//  GLOBALS
// ============================================================
WebServer        server(80);
NewPing          sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE_CM);
RTC_DS3231       rtc;

String   ssid_config;
String   pass_config;
float    pipeHeightCm        = 55.0f;
uint32_t configuredSleepSecs = NORMAL_SLEEP_SECONDS;

bool sdReady       = false;
bool rtcReady      = false;
bool ntpReady      = false;   // true when NTP time was successfully fetched this boot

// RTC_DATA_ATTR — these survive deep sleep in RTC fast memory
RTC_DATA_ATTR float    lastWaterLevel    = -1.0f;
static uint32_t bodSaved = 0;   // V8-6
RTC_DATA_ATTR uint32_t deepSleepSeconds  = NORMAL_SLEEP_SECONDS;

// ── WiFi state (survives deep sleep in RTC memory) ──
// There is no WiFi in the field. A sync happens when the farmer switches on
// a phone hotspot and presses reset, roughly every month or two. So a timer
// wake almost never has a network to find, and looking on every wake would
// waste power across the ~1400 wakes between two visits.
//
// The reset button is the real sync trigger and is never skipped. These two
// numbers only govern the safety-net checks on timer wakes, in units of wake
// cycles (one hour each by default).
#define WIFI_IDLE_CHECK_BOOTS    24   // routine look, about once a day
#define WIFI_URGENT_CHECK_BOOTS   6   // when the flash buffer is filling up
// A reset is the farmer asking to sync, so the device keeps looking for a
// few minutes rather than giving up after one pass: the hotspot may be
// switched on a moment after the button, or take time to come up.
#define SYNC_REQUEST_PASSES       6   // acquisition passes after a reset
#define SYNC_REQUEST_GAP_MS   15000   // wait between them
// Brownouts can be caused by the radio switching on with a tired battery,
// and the farmer's own reset can trigger one. Allow this many before we
// stop treating a fault reset as a possible sync request.
#define FAULT_RESET_GRACE         2
// After this many consecutive failures the device stops retrying unknown
// OPEN networks, so a captive portal near the field cannot drain it. The
// saved hotspot is always still tried.
#define WIFI_OPEN_SKIP_AFTER_FAILURES  5
RTC_DATA_ATTR uint32_t wifiFailStreak    = 0;   // consecutive offline boots
RTC_DATA_ATTR uint32_t bootsSinceSync    = 0;   // boots since last successful sync
RTC_DATA_ATTR uint32_t bootsSinceAttempt = 0;   // boots since last WiFi attempt
RTC_DATA_ATTR uint32_t faultResetStreak  = 0;   // consecutive brownout/panic/WDT resets
RTC_DATA_ATTR uint32_t lastNtpDay        = 0;   // V8-14: day number of the last NTP sync
String deviceId = "";                            // V8-13

// ============================================================
//  LOGGING — serial + telnet mirror
// ============================================================
void log_msg(const String& msg, bool nl = true) {
  if (nl) { Serial.println(msg); TelnetStream.println(msg); }
  else     { Serial.print(msg);  TelnetStream.print(msg);  }
}

// ============================================================
//  WATCHDOG FEED
// ============================================================
inline void wdt_feed() { esp_task_wdt_reset(); }

// ============================================================
//  SENSOR
//  Returns median of valid readings, or -1 on total failure.
//  FIX: original code returned 0 on failure and logged it as
//       a valid water level reading.
// ============================================================
int getDistanceCm() {
  // FIX: Fire a "dummy" ping first to clear any stale echoes or ringing
  // caused by the pin state transitioning during wakeup.
  sonar.ping_cm();
  delay(100);

  // FIX: 50ms inter-ping was too short for 200cm max range.
  // Round-trip at 200cm = ~11.8ms but echo ringing in a pipe needs
  // more settling time. 100ms gap is safe and still fast enough.
  unsigned int readings[5];
  int valid = 0;
  for (int i = 0; i < 5; i++) {
    wdt_feed();
    unsigned int d = sonar.ping_cm();
    if (d > 0 && d <= (unsigned int)MAX_DISTANCE_CM) {
      readings[valid++] = d;
    }
    delay(100);
  }
  if (valid < 3) return -1;   // V8-13: one or two stray echoes are not a reading
  std::sort(readings, readings + valid);
  return (int)readings[valid / 2];
}

// ============================================================
//  WiFi HELPERS
// ============================================================
String getWiFiSSID() {
  return (WiFi.status() == WL_CONNECTED) ? WiFi.SSID() : "NoWiFi";
}

int getWiFiStrength() {
  if (WiFi.status() != WL_CONNECTED) return 0;
  // FIX: map() returns long, constrain() expects matching types — cast explicitly
  long mapped = map((long)WiFi.RSSI(), -90L, -30L, 0L, 10L);
  return (int)constrain(mapped, 0L, 10L);
}

// FIX: original used plain HTTPClient with no timeout — could block indefinitely.
// Added 5 s timeout. Also added WiFi guard at top.
bool isInternetAvailable() {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  http.setTimeout(5000);
  http.begin("http://connectivitycheck.gstatic.com/generate_204");
  int code = http.GET();
  http.end();
  return (code == HTTP_CODE_NO_CONTENT);
}

// ============================================================
//  STATUS LABEL
// ============================================================
String getStatus(float lvl) {
  if      (lvl <  7.0f)  return "Low";
  else if (lvl < 15.0f)  return "Good";
  else if (lvl <  20.0f) return "Excess";
  else                    return "Flood Alert";   // V8-13 fleet standard
}

// ============================================================
//  RTC HELPERS
// ============================================================
// V8-9: never fake the clock. An invalid RTC stays invalid until NTP sets it.
bool clockValid() {
  if (rtcReady) { int y = rtc.now().year(); if (y >= 2025 && y <= 2099) return true; }
  return ntpReady;
}
void setInitialRtcTime() {
  if (rtcReady && !clockValid()) log_msg("[RTC] Clock invalid — waiting for NTP.");
}

// ============================================================
//  TIMESTAMP
//  Priority: hardware RTC → NTP system clock → invalid marker
//  FIX: previously returned "0000-00-00 00:00:00" whenever the
//  DS3231 module was absent, even when NTP had already synced the
//  ESP32's internal clock. Now reads struct tm from the system
//  clock (set by configTime+getLocalTime) as a direct fallback.
// ============================================================
String getTimestamp() {
  char buf[25];
  // 1. Hardware RTC — most reliable, works offline after first sync
  if (rtcReady) {
    DateTime now = rtc.now();
    if (now.year() >= 2025) {
      sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
              now.year(), now.month(), now.day(),
              now.hour(), now.minute(), now.second());
      return String(buf);
    }
  }
  // 2. NTP system clock — works when WiFi is up, no hardware needed
  if (ntpReady) {
    struct tm ti;
    if (getLocalTime(&ti, 100)) {   // 100 ms — already synced, just reading
      strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ti);
      return String(buf);
    }
  }
  // 3. No time source available — empty, with clockValid:false in the record
  return "";
}

// ============================================================
//  BOOT REASON
// ============================================================
void printResetReason(esp_sleep_wakeup_cause_t wakeup) {
  log_msg("[System] Boot reason:");
  if (wakeup == ESP_SLEEP_WAKEUP_TIMER) {
    log_msg("  -> Timer wakeup from deep sleep (normal).");
    return;
  }
  // FIX: calling rtc_get_reset_reason(0) twice (once in switch, once in
  // default) is wasteful and fragile — call once and store.
  RESET_REASON reason = rtc_get_reset_reason(0);
  switch (reason) {
    case POWERON_RESET:          log_msg("  -> Power-on reset.");    break;
    case SW_RESET:
    case SW_CPU_RESET:           log_msg("  -> Software reset.");     break;
    case RTCWDT_BROWN_OUT_RESET: log_msg("  -> Brownout reset.");     break;
    default:
      log_msg("  -> Other reset, code: " + String((int)reason));     break;
  }
}

// ============================================================
//  CONFIG — LittleFS ONLY. SD is never read or written for config.
// ============================================================
bool loadConfig() {
  File f = LittleFS.open(LFS_CONFIG_FILE, "r");
  if (!f) {
    log_msg("[CFG] No config in LittleFS — using defaults.");
    return false;
  }
  DynamicJsonDocument doc(256);
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    log_msg("[CFG] Parse error: " + String(err.c_str()) + " — using defaults.");
    return false;
  }
  ssid_config         = doc["ssid"]          | "";
  pass_config         = doc["pass"]          | "";
  pipeHeightCm        = doc["pipe_height"]   | 55.0f;
  configuredSleepSecs = doc["sleep_seconds"] | (uint32_t)NORMAL_SLEEP_SECONDS;
  // V8-12 clamps
  if (configuredSleepSecs < 10 || configuredSleepSecs > 86400) configuredSleepSecs = NORMAL_SLEEP_SECONDS;
  if (pipeHeightCm < 5.0f || pipeHeightCm > 300.0f) pipeHeightCm = 55.0f;
  log_msg("[CFG] Loaded. SSID=" + ssid_config +
          " PipeH=" + String(pipeHeightCm) +
          " Sleep=" + String(configuredSleepSecs) + "s");
  return true;
}

void saveConfig() {
  // Atomic write: write to temp file first, then rename.
  // If power is lost mid-write, the original config survives.
  LittleFS.remove("/cfg_tmp.json");
  File f = LittleFS.open("/cfg_tmp.json", "w");
  if (!f) {
    log_msg("[CFG] ERROR: Cannot open temp config for write!");
    return;
  }
  DynamicJsonDocument doc(256);
  doc["ssid"]          = ssid_config;
  doc["pass"]          = pass_config;
  doc["pipe_height"]   = pipeHeightCm;
  doc["sleep_seconds"] = configuredSleepSecs;
  serializeJson(doc, f);
  f.close();
  LittleFS.remove(LFS_CONFIG_FILE);
  LittleFS.rename("/cfg_tmp.json", LFS_CONFIG_FILE);
  log_msg("[CFG] Saved to LittleFS. SD untouched.");
}

// ============================================================
//  LITTLEFS USAGE
// ============================================================
float lfsUsageFraction() {
  size_t total = LittleFS.totalBytes();
  if (total == 0) return 1.0f;   // treat unknown as full — safe fallback
  return (float)LittleFS.usedBytes() / (float)total;
}

// Count non-empty lines in a LittleFS file
int countLinesLfs(const char* path) {
  File f = LittleFS.open(path, "r");
  if (!f) return 0;
  int n = 0;
  while (f.available()) {
    String l = f.readStringUntil('\n');
    l.trim();
    if (l.length() > 5) n++;
  }
  f.close();
  return n;
}

// Count non-empty lines in an SD file
int countLinesSd(const char* path) {
  if (!sdReady || !SD.exists(path)) return 0;
  File f = SD.open(path, FILE_READ);
  if (!f) return 0;
  int n = 0;
  while (f.available()) {
    String l = f.readStringUntil('\n');
    l.trim();
    if (l.length() > 5) n++;
  }
  f.close();
  return n;
}

// ============================================================
//  V8-16: CLOCK FROM THE HTTP "Date:" HEADER
//
//  This device usually reaches the internet through a farmer's phone
//  hotspot, and phone hotspots routinely block NTP (UDP port 123). A
//  field log from 2026-09-09 shows exactly that: the upload succeeded
//  while NTP timed out. So NTP alone cannot be trusted to correct the
//  clock here.
//
//  Every HTTP response carries a "Date:" header in GMT, including the
//  302 redirect that Apps Script returns. It costs nothing extra: the
//  bytes are already being read. Accuracy is a second or two, which is
//  far better than a drifting or unset RTC.
// ============================================================
#define TZ_OFFSET_SECONDS 19800   // IST, UTC+5:30

void applyHttpDate(String v) {
  v.trim();
  // RFC 7231 preferred form: "Tue, 09 Sep 2026 07:50:33 GMT"
  int comma = v.indexOf(',');
  if (comma >= 0) v = v.substring(comma + 1);
  v.trim();

  int  d = 0, y = 0, hh = 0, mi = 0, ss = 0;
  char mon[8] = {0};
  if (sscanf(v.c_str(), "%d %7s %d %d:%d:%d", &d, mon, &y, &hh, &mi, &ss) != 6) return;

  static const char* MONTHS = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char* hit = strstr(MONTHS, mon);
  if (!hit) return;
  int mo = (int)((hit - MONTHS) / 3) + 1;

  // Reject anything implausible rather than corrupting a good clock.
  if (y < 2025 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31 ||
      hh > 23 || mi > 59 || ss > 59) return;

  DateTime utc(y, mo, d, hh, mi, ss);
  DateTime local = utc + TimeSpan(TZ_OFFSET_SECONDS);

  // Give the ESP32's own clock a valid time too, so a unit whose DS3231
  // is missing or dead still produces usable timestamps.
  setenv("TZ", "IST-5:30", 1);
  tzset();
  struct timeval tv = { (time_t)utc.unixtime(), 0 };
  settimeofday(&tv, nullptr);
  ntpReady = true;

  if (rtcReady) {
    long drift = (long)local.unixtime() - (long)rtc.now().unixtime();
    if (!clockValid() || labs(drift) > 30) {
      rtc.adjust(local);
      lastNtpDay = (uint32_t)(local.unixtime() / 86400UL);
      log_msg("[CLOCK] Corrected from HTTP Date header (was off by " +
              String(drift) + " s).");
    }
  } else {
    log_msg("[CLOCK] Set from HTTP Date header (no DS3231 fitted).");
  }
}

// ============================================================
//  APPEND helpers
// ============================================================
void appendToLfs(const String& jsonLine) {
  File f = LittleFS.open(LFS_DATA_FILE, "a");
  if (!f) { log_msg("[LFS] ERROR: Cannot append to data file!"); return; }
  f.println(jsonLine);
  f.close();
}

// Note: SD is written only via spillLfsToSd() — no direct append path needed.

// ============================================================
//  HTTP REQUEST to Google Sheets — raw WiFiClientSecure approach
//
//  FIX: GAS always redirects POST: script.google.com → 302 → script.googleusercontent.com
//  HTTPClient cannot follow this cross-domain redirect with the same SSL context.
//  Furthermore, standard HTTP behavior dictates that a 302 redirect from a POST
//  must be followed up using a GET request. Sending a second POST to the 
//  redirected URL will result in Google throwing a 405 Method Not Allowed error.
// ============================================================

// Helper: send raw HTTP/1.1 request (POST or GET) and return response body
String rawHttpRequest(const String& method, const String& host, const String& path,
                    const String& body, String& redirectUrl) {
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(15000);          // V8-3: Stream timeout is in ms on this core
  client.setHandshakeTimeout(10);    // seconds

  redirectUrl = "";
  wdt_feed();
  if (!client.connect(host.c_str(), 443, 10000)) {
    log_msg("[HTTP] Connect failed: " + host);
    return "";
  }

  // Send request — build it all at once to avoid TCP fragmentation
  String req;
  req.reserve(256 + body.length());
  req  = method + " " + path + " HTTP/1.1\r\n";
  req += "Host: " + host + "\r\n";
  if (method == "POST") {
    req += "Content-Type: application/json\r\n";
    req += "Content-Length: " + String(body.length()) + "\r\n";
  }
  req += "Connection: close\r\n\r\n";
  if (method == "POST" && body.length() > 0) {
    req += body;
  }
  client.print(req);

  // Wait for server to start responding — up to 15 s
  unsigned long waitStart = millis();
  while (!client.available() && millis() - waitStart < 15000) {
    wdt_feed();
    delay(50);
  }
  if (!client.available()) {
    log_msg("[HTTP] Timeout waiting for response from: " + host);
    client.stop();
    return "";
  }

  // Read status line — strip \r explicitly
  String statusLine = client.readStringUntil('\n');
  if (statusLine.endsWith("\r")) statusLine.remove(statusLine.length() - 1);
  int statusCode = 0;
  int sp1 = statusLine.indexOf(' ');
  if (sp1 > 0) statusCode = statusLine.substring(sp1 + 1, sp1 + 4).toInt();
  log_msg("[HTTP] Status: " + String(statusCode));

  // Read headers
  String location = "";
  while (client.connected() || client.available()) {
    wdt_feed();
    String line = client.readStringUntil('\n');
    if (line.endsWith("\r")) line.remove(line.length() - 1);
    if (line.length() == 0) break;   // blank line = end of headers

    String lineLower = line;
    lineLower.toLowerCase();
    if (lineLower.startsWith("location:")) {
      location = line.substring(9);   // length of "location:" = 9
      location.trim();
    } else if (lineLower.startsWith("date:")) {
      applyHttpDate(line.substring(5));   // V8-16: free, accurate clock source
    }
  }

  if (statusCode >= 300 && statusCode < 400) {
    if (location.length() > 0) {
      redirectUrl = location;
      log_msg("[HTTP] Redirect to: " + location.substring(0, 80));
    } else {
      log_msg("[HTTP] Redirect with no Location header.");
    }
    client.stop();
    return "";
  }

  // Read body — capped at 512 bytes
  String responseBody = "";
  responseBody.reserve(512);
  unsigned long t0 = millis();
  while ((client.connected() || client.available()) && millis() - t0 < 10000) {
    wdt_feed();
    if (client.available()) {
      responseBody += (char)client.read();
      if (responseBody.length() >= 512) break;
    }
  }
  client.stop();
  return responseBody;
}

// Parse https://hostname/path into host and path components
void parseUrl(const String& url, String& host, String& path) {
  String u = url;
  if (u.startsWith("https://")) u = u.substring(8);
  else if (u.startsWith("http://")) u = u.substring(7);
  int slash = u.indexOf('/');
  if (slash < 0) {
    host = u;
    path = "/";
  } else {
    host = u.substring(0, slash);
    path = u.substring(slash);
  }
}

bool sendToSheets(const String& jsonData) {
  if (WiFi.status() != WL_CONNECTED) return false;

  // ── Step 1: POST to script.google.com, capture the redirect Location ──
  String redirectUrl = "";
  String step1Body = rawHttpRequest("POST", GAS_HOST, GAS_PATH, jsonData, redirectUrl);

  if (redirectUrl.length() == 0 && step1Body.length() == 0) {
    log_msg("[HTTP] Step 1 connect failed — no response.");
    return false;
  }

  if (redirectUrl.length() == 0) {
    if (step1Body.indexOf("success") != -1 && step1Body.indexOf("\"success\":false") == -1) {
      log_msg("[HTTP] Sheets accepted (direct).");
      return true;
    }
    log_msg("[HTTP] Direct response rejected: " + step1Body.substring(0, 120));
    return false;
  }

  log_msg("[HTTP] Redirect → " + redirectUrl.substring(0, 70) + "...");

  // ── Step 2: GET the redirected googleusercontent.com URL ──
  // FIX: HTTP spec says a 302 redirect from POST must be followed with GET.
  // Sending POST here causes Google to return 405 Method Not Allowed.
  String host2, path2;
  parseUrl(redirectUrl, host2, path2);

  String redirectUrl2 = "";
  String body = rawHttpRequest("GET", host2, path2, "", redirectUrl2);

  if (body.length() == 0 && redirectUrl2.length() > 0) {
    log_msg("[HTTP] Double redirect → " + redirectUrl2.substring(0, 70));
    String host3, path3;
    parseUrl(redirectUrl2, host3, path3);
    String unused;
    body = rawHttpRequest("GET", host3, path3, "", unused);
  }

  if (body.indexOf("success") != -1 && body.indexOf("\"success\":false") == -1) {
    log_msg("[HTTP] Sheets accepted.");
    return true;
  }

  log_msg("[HTTP] Sheets rejected: " + body.substring(0, 120));
  return false;
}

// ============================================================
//  SPILL: move oldest LFS_SPILL_FRACTION of lines from
//  LittleFS → SD when LittleFS hits the spill threshold.
//  Atomic: uses temp file so a mid-spill crash can't corrupt data.
// ============================================================
void spillLfsToSd() {
  if (!sdReady) {
    log_msg("[SPILL] SD not ready — spill skipped. LittleFS may fill!");
    return;
  }
  int total = countLinesLfs(LFS_DATA_FILE);
  if (total == 0) return;

  int toSpill = max(1, (int)((float)total * LFS_SPILL_FRACTION));
  log_msg("[SPILL] LFS " + String(lfsUsageFraction() * 100.0f, 1) +
          "% — spilling " + String(toSpill) + "/" + String(total) + " to SD.");

  File src   = LittleFS.open(LFS_DATA_FILE, "r");
  File keep  = LittleFS.open(LFS_TEMP_FILE, "w");
  if (!src || !keep) {
    log_msg("[SPILL] Cannot open LFS files!");
    if (src)  src.close();
    if (keep) keep.close();
    return;
  }
  // Open SD AFTER LFS files are confirmed open — abort if SD fails
  File spill = SD.open(SD_BACKUP_FILE, FILE_APPEND);
  if (!spill) {
    log_msg("[SPILL] Cannot open SD file — aborting to protect data.");
    src.close();
    keep.close();
    LittleFS.remove(LFS_TEMP_FILE);
    return;
  }

  int idx = 0;
  while (src.available()) {
    wdt_feed();
    String line = src.readStringUntil('\n');
    line.trim();
    if (line.length() <= 5) continue;
    if (idx < toSpill) spill.println(line);
    else               keep.println(line);
    idx++;
  }
  src.close();
  spill.close();
  keep.close();

  LittleFS.rename(LFS_TEMP_FILE, LFS_DATA_FILE);   // V8-7: rename overwrites atomically
  log_msg("[SPILL] Done. Moved " + String(toSpill) + " records to SD.");
}

// ============================================================
//  SAVE DATA — main entry point for every reading
// ============================================================
// V8-1: always store first; syncAllToSheets() sends it together with any backlog.
void saveData(const String& jsonLine, bool internetReady) {
  (void)internetReady;
  // Store locally: spill first if needed, then append
  if (lfsUsageFraction() >= LFS_SPILL_THRESHOLD) spillLfsToSd();
  appendToLfs(jsonLine);
  log_msg("[SAVE] Stored in LittleFS. Usage now " +
          String(lfsUsageFraction() * 100.0f, 1) + "%");
}

// ============================================================
//  FLUSH LittleFS → Google Sheets (batched, atomic)
// ============================================================
bool flushLfsToSheets() {
  int total = countLinesLfs(LFS_DATA_FILE);
  if (total == 0) { log_msg("[FLUSH-LFS] Nothing to flush."); return true; }
  log_msg("[FLUSH-LFS] Flushing " + String(total) + " records...");

  File src  = LittleFS.open(LFS_DATA_FILE, "r");
  File keep = LittleFS.open(LFS_TEMP_FILE, "w");
  if (!src || !keep) {
    log_msg("[FLUSH-LFS] Cannot open files!");
    if (src)  src.close();
    if (keep) keep.close();
    return false;
  }

  // FIX: Stack-allocating String batchLines[50] inside a loop burns ~25 KB
  // of stack per iteration on ESP32. Use a fixed small batch with heap strings.
  // V8-18: 40 records per upload, not 10. A sync happens once a month or
  // two, so the backlog is 700-1400 readings and the farmer is standing
  // there with a hotspot on while it drains. At 10 per upload that is up
  // to 144 TLS round trips, close to ten minutes; at 40 it is about three.
  // (V8-2 fixed the real bug here, which was the batch silently
  // overflowing its document and discarding records while counting them
  // as sent. The add() below is checked, so a larger batch is safe.)
  const int BATCH = 40;
  int sent = 0;
  int kept = 0;  // FIX: track failed records with a counter, NOT keep.size().
                 // LittleFS doesn't flush file metadata (size) until close() is
                 // called, so keep.size() always returned 0 even when data was
                 // written — causing "Fully flushed!" to be logged and the temp
                 // file deleted, silently dropping all failed records.

  while (src.available()) {
    wdt_feed();
    std::vector<String> batchLines;
    batchLines.reserve(BATCH);
    DynamicJsonDocument batch(32768);
    JsonArray arr = batch.to<JsonArray>();

    while (src.available() && (int)batchLines.size() < BATCH) {
      String line = src.readStringUntil('\n');
      line.trim();
      if (line.length() <= 5) continue;
      DynamicJsonDocument item(640);
      if (deserializeJson(item, line) == DeserializationError::Ok) {
        if (item["dataType"] == "Current") item["dataType"] = "Backup";
        if (!arr.add(item)) {              // V8-2: doc full — keep for next time
          keep.println(line); kept++;
          log_msg("[FLUSH] Batch doc full — record deferred.");
          break;
        }
        batchLines.push_back(line);
      } else {
        log_msg("[FLUSH] Corrupt line dropped.");
      }
    }
    if (batchLines.empty()) continue;

    String jsonBatch;
    serializeJson(batch, jsonBatch);
    log_msg("[FLUSH-LFS] Sending " + String(batchLines.size()) + " records...");

    if (sendToSheets(jsonBatch)) {
      sent += (int)batchLines.size();
      log_msg("[FLUSH-LFS] OK (" + String(sent) + "/" + String(total) + ")");
    } else {
      log_msg("[FLUSH-LFS] Batch failed — keeping " +
              String(batchLines.size()) + " records.");
      for (auto& l : batchLines) keep.println(l);
      kept += (int)batchLines.size();
    }
  }

  src.close();
  keep.close();
  if (kept > 0) {
    LittleFS.rename(LFS_TEMP_FILE, LFS_DATA_FILE);   // V8-7: overwrite in place
    log_msg("[FLUSH-LFS] Partial — " + String(kept) + " records remain (sent " +
            String(sent) + "/" + String(total) + ").");
    return false;
  }
  LittleFS.remove(LFS_DATA_FILE);
  LittleFS.remove(LFS_TEMP_FILE);
  log_msg("[FLUSH-LFS] All " + String(sent) + " records flushed successfully!");
  return true;
}

// ============================================================
//  FLUSH SD → Google Sheets (batched, graceful if SD missing)
// ============================================================
bool flushSdToSheets() {
  if (!sdReady) {
    log_msg("[FLUSH-SD] SD not ready — skipped gracefully.");
    return true;
  }
  if (!SD.exists(SD_BACKUP_FILE)) {
    log_msg("[FLUSH-SD] No SD backup file.");
    return true;
  }
  int total = countLinesSd(SD_BACKUP_FILE);
  if (total == 0) { SD.remove(SD_BACKUP_FILE); return true; }
  log_msg("[FLUSH-SD] Flushing " + String(total) + " records...");

  File src  = SD.open(SD_BACKUP_FILE, FILE_READ);
  File keep = SD.open(SD_TEMP_FILE,   FILE_WRITE);
  if (!src || !keep) {
    log_msg("[FLUSH-SD] Cannot open SD files!");
    if (src)  src.close();
    if (keep) keep.close();
    return false;
  }

  // V8-18: 40 records per upload, not 10. A sync happens once a month or
  // two, so the backlog is 700-1400 readings and the farmer is standing
  // there with a hotspot on while it drains. At 10 per upload that is up
  // to 144 TLS round trips, close to ten minutes; at 40 it is about three.
  // (V8-2 fixed the real bug here, which was the batch silently
  // overflowing its document and discarding records while counting them
  // as sent. The add() below is checked, so a larger batch is safe.)
  const int BATCH = 40;
  int sent = 0;
  int kept = 0;  // FIX: same keep.size()-before-close() bug as LFS flush

  while (src.available()) {
    wdt_feed();
    std::vector<String> batchLines;
    batchLines.reserve(BATCH);
    DynamicJsonDocument batch(32768);
    JsonArray arr = batch.to<JsonArray>();

    while (src.available() && (int)batchLines.size() < BATCH) {
      String line = src.readStringUntil('\n');
      line.trim();
      if (line.length() <= 5) continue;
      DynamicJsonDocument item(640);
      if (deserializeJson(item, line) == DeserializationError::Ok) {
        if (item["dataType"] == "Current") item["dataType"] = "Backup";
        if (!arr.add(item)) {              // V8-2: doc full — keep for next time
          keep.println(line); kept++;
          log_msg("[FLUSH] Batch doc full — record deferred.");
          break;
        }
        batchLines.push_back(line);
      } else {
        log_msg("[FLUSH] Corrupt line dropped.");
      }
    }
    if (batchLines.empty()) continue;

    String jsonBatch;
    serializeJson(batch, jsonBatch);
    log_msg("[FLUSH-SD] Sending " + String(batchLines.size()) + " records...");

    if (sendToSheets(jsonBatch)) {
      sent += (int)batchLines.size();
      log_msg("[FLUSH-SD] OK (" + String(sent) + "/" + String(total) + ")");
    } else {
      log_msg("[FLUSH-SD] Batch failed — keeping " +
              String(batchLines.size()) + " records.");
      for (auto& l : batchLines) keep.println(l);
      kept += (int)batchLines.size();
    }
  }

  src.close();
  keep.close();
  SD.remove(SD_BACKUP_FILE);
  if (kept > 0) {
    SD.rename(SD_TEMP_FILE, SD_BACKUP_FILE);
    log_msg("[FLUSH-SD] Partial — " + String(kept) + " records remain (sent " +
            String(sent) + "/" + String(total) + ").");
    return false;
  }
  SD.remove(SD_TEMP_FILE);
  log_msg("[FLUSH-SD] All " + String(sent) + " records flushed successfully!");
  return true;
}

// ============================================================
//  FULL SYNC
// ============================================================
void syncAllToSheets() {
  log_msg("[SYNC] Starting full sync (LittleFS then SD)...");
  flushLfsToSheets();
  flushSdToSheets();
  log_msg("[SYNC] Done.");
}

// ============================================================
//  FORWARD DECLARATIONS — web handlers
// ============================================================
void handleRoot();
void handleSave();
void handleBackupDownload();
void handleLiveData();
void handleClearBackup();

// ============================================================
//  CONFIG PORTAL  (AP mode — blocks until user saves)
// ============================================================
void startConfigPortal() {
  log_msg("[Portal] Entering config mode...");

  // FIX: stop any existing WiFi connection before switching to AP mode
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.mode(WIFI_AP);
  // FIX: softAP() can fail silently if called too fast after mode switch;
  // retry up to 3 times.
  bool apOk = false;
  for (int i = 0; i < 3 && !apOk; i++) {
    apOk = WiFi.softAP("wifi - AWD PIPE", "12345678");
    if (!apOk) delay(500);
  }
  if (!apOk) log_msg("[Portal] WARNING: softAP() failed — portal may not be reachable.");
  log_msg("[Portal] AP IP: " + WiFi.softAPIP().toString());

  server.on("/",             handleRoot);
  server.on("/save",         handleSave);
  server.on("/download",     HTTP_GET,  handleBackupDownload);
  server.on("/live-data",    HTTP_GET,  handleLiveData);
  server.on("/clear-backup", HTTP_POST, handleClearBackup);
  server.begin();
  log_msg("[Portal] Web server started. Connect to: wifi - AWD PIPE / 12345678");

  unsigned long portalStart = millis();
  while (true) {
    server.handleClient();
    wdt_feed();
    delay(2);
    if (millis() - portalStart > PORTAL_TIMEOUT_MS) {   // V8-11
      log_msg("[Portal] Timeout — restarting in normal mode.");
      delay(200); ESP.restart();
    }
  }
}

// ============================================================
//  SETUP — all work happens here; loop() is intentionally empty
// ============================================================
void setup() {
  // FIX: Release any GPIOs that were held LOW during deep sleep
  // so the NewPing library can safely use the TRIG_PIN again.
  gpio_hold_dis((gpio_num_t)TRIG_PIN);
  gpio_deep_sleep_hold_dis();

  // FIX: NewPing's global constructor ran BEFORE the hold was released.
  // We must manually re-initialize the pin and let the sensor stabilize.
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  delay(100);

  Serial.begin(115200);
  delay(200);
  log_msg("\n\n[System] === AWD PIPE Water Monitor ===");

  // ── Brownout detector ──
  // FIX: ESP32 resets when supply voltage dips below ~2.45 V. During WiFi TX
  // the radio draws ~380 mA in bursts — enough to cause a brownout on a weak
  // USB cable, cheap LDO, or partially-drained battery.
  // We disable the brownout detector for the duration of setup() where all
  // the WiFi work happens. It is re-enabled before deep sleep.
  // NOTE: This is safe here because if the voltage truly collapses the
  // watchdog will still reset us. The brownout detector only helps catch
  // slow voltage droops — which are better handled by fixing the power supply.
  bodSaved = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG);   // V8-6
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // disable brownout detector
  log_msg("[Power] Brownout detector disabled during operation.");

  // ---- Watchdog ----
  // FIX: esp_task_wdt_init() is deprecated in ESP-IDF v5; use reconfigure.
  // idle_core_mask = 0 means we don't panic if idle tasks stall (safe).
  esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms     = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_err_t wdt_err = esp_task_wdt_reconfigure(&wdt_cfg);
  if (wdt_err != ESP_OK) {
    // First boot — WDT not yet initialised, so init instead of reconfigure
    esp_task_wdt_init(&wdt_cfg);
  }
  esp_task_wdt_add(NULL);
  log_msg("[WDT] Watchdog armed (" + String(WDT_TIMEOUT_S) + "s).");

  // ---- LittleFS ----  V8-10: format only after 3 consecutive failed mounts
  {
    bool ok = LittleFS.begin(false);
    Preferences prefs; prefs.begin("sys", false);
    uint32_t fails = prefs.getUInt("lfsfail", 0);
    if (!ok) {
      fails++;
      log_msg("[LFS] Mount failed (" + String(fails) + "/3).");
      if (fails >= 3) { log_msg("[LFS] Formatting."); ok = LittleFS.begin(true); fails = 0; }
    } else fails = 0;
    prefs.putUInt("lfsfail", fails); prefs.end();
    if (!ok) { log_msg("[LFS] CRITICAL: mount failed — sleeping 10 min.");
               esp_sleep_enable_timer_wakeup(600ULL * 1000000ULL); esp_deep_sleep_start(); }
  }
  // V8-7: merge a leftover flush/spill temp file back (never truncate it)
  if (LittleFS.exists(LFS_TEMP_FILE)) {
    File t = LittleFS.open(LFS_TEMP_FILE, "r"); File d = LittleFS.open(LFS_DATA_FILE, "a");
    int n = 0;
    if (t && d) { while (t.available()) { wdt_feed(); String l = t.readStringUntil('\n'); l.trim(); if (l.length() > 5) { d.println(l); n++; } } }
    bool ok = (t && d); if (t) t.close(); if (d) d.close();
    if (ok) { LittleFS.remove(LFS_TEMP_FILE); log_msg("[LFS] Recovered " + String(n) + " records from temp file."); }
  }
  deviceId = String((uint32_t)(ESP.getEfuseMac() >> 24) & 0xFFFFFF, HEX);
  log_msg("[LFS] Ready. " + String(LittleFS.usedBytes()) +
          "/" + String(LittleFS.totalBytes()) + " bytes used (" +
          String((int)(lfsUsageFraction() * 100)) + "%).");

  // ---- I2C / RTC ----
  Wire.begin(SDA_PIN, SCL_PIN);
  if (rtc.begin()) {
    rtcReady = true;
    setInitialRtcTime();
    log_msg("[RTC] DS3231 ready. Time: " + getTimestamp());
  } else {
    log_msg("[RTC] Not found — timestamps will be invalid.");
  }

  // ---- SD card (optional) ----
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (SD.begin(SD_CS, SPI)) {
    sdReady = true;
    log_msg("[SD] Card ready. " +
            String((float)(SD.cardSize() - SD.usedBytes()) / (1024 * 1024), 1) +
            " MB free.");
  } else {
    log_msg("[SD] Not detected — SD backup disabled (non-fatal).");
  }

  // ---- Config jumper check ----
  pinMode(CONFIG_JUMPER_PIN, INPUT_PULLUP);
  delay(100);
  if (digitalRead(CONFIG_JUMPER_PIN) == LOW) {
    loadConfig();        // pre-fill portal fields
    startConfigPortal(); // never returns — restarts when done
  }

  // ---- Boot reason ----
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  printResetReason(wakeup_reason);

  // ---- Load config ----
  loadConfig();

  // ============================================================
  //  V8-1: SENSOR FIRST, WRITE-AHEAD TO FLASH, THEN NETWORK
  // ============================================================
  log_msg("[Sensor] Reading...");
  int distanceCm = getDistanceCm();
  float waterLevelCm = -1.0f;
  {
    DynamicJsonDocument doc(640);
    bool cv = clockValid();
    doc["timestamp"]  = cv ? getTimestamp() : "";
    doc["clockValid"] = cv;
    doc["deviceId"]   = deviceId;
    doc["network"]    = "pending";
    doc["wifiStrength"] = 0;
    doc["lfsUsedPct"] = (int)(lfsUsageFraction() * 100.0f);
    if (distanceCm < 0) {
      log_msg("[Sensor] FAILURE — fewer than 3 valid echoes.");
      doc["dataType"]   = "SensorError";
      doc["status"]     = "SensorFailure";
      doc["waterLevel"] = nullptr;
    } else {
      waterLevelCm = constrain((float)pipeHeightCm - (float)distanceCm, 0.0f, pipeHeightCm);
      log_msg("[Sensor] Distance=" + String(distanceCm) + "cm  Level=" +
              String(waterLevelCm, 1) + "cm  Status=" + getStatus(waterLevelCm));
      doc["waterLevel"] = waterLevelCm;
      doc["status"]     = getStatus(waterLevelCm);
      doc["dataType"]   = "Current";
    }
    String line; serializeJson(doc, line);
    saveData(line, false);          // write-ahead — safe before any radio work
  }

  // ── TLS note ──
  // WiFiClientSecure instances are now created locally inside sendToSheets()
  // with setInsecure() — no global client needed.

  // ============================================================
  //  WiFi STATE MACHINE  (V8-17)
  //
  //  HOW THIS DEVICE IS ACTUALLY USED
  //  There is no WiFi in the field. Ever. The sync is a deliberate human
  //  action: the farmer switches a phone hotspot on, presses the reset
  //  button, and the device uploads everything it has been buffering.
  //  That happens perhaps once a month or two, or after a harvest.
  //
  //  So the reset button IS the sync command, and it is the primary path
  //  here. On a timer wake there is essentially never a network to find,
  //  and hunting for one is pure battery waste across the ~1400 wakes
  //  between two visits.
  //
  //   • Physical reset (power-on or EN button) → always try. This is the
  //     farmer asking for a sync, and it must never be skipped.
  //   • Buffer filling up → try every WIFI_URGENT_CHECK_BOOTS, so data is
  //     not lost if nobody comes for a very long time.
  //   • Otherwise → one cheap look every WIFI_IDLE_CHECK_BOOTS (about
  //     daily), purely as a safety net in case a hotspot is left on
  //     without anyone pressing reset.
  //
  //  When we do look, we scan first (about 2.5 s) and only spend real
  //  connect time on a network the scan actually saw. The old code
  //  blind-dialled the saved SSID for a full 15 s even in an empty field,
  //  so each check is now roughly six times cheaper than it used to be.
  // ============================================================
  bool internetReady  = false;
  bool attemptWifi    = false;
  bootsSinceSync++;
  bootsSinceAttempt++;

  // ── Is this the farmer asking for a sync? ──────────────────────────
  // V8-19: this decision is fail-safe by design. Missing a sync request
  // is far worse than an unnecessary one: somebody has travelled to the
  // field, switched on a hotspot and pressed the button, and if we ignore
  // that they leave believing the data is uploaded when it is not, and
  // nobody finds out for another month or two. An unnecessary attempt
  // only costs a few seconds of radio.
  //
  // So the rule is inverted from the obvious one. Rather than listing the
  // reset codes that count as a button press, we assume ANY boot that is
  // not the ordinary deep-sleep timer wake IS a sync request, unless it
  // is clearly a fault. Different ESP32 boards report a button press
  // differently (POWERON on most, EXT on some), and an unrecognised code
  // must not silently disable the only sync path this device has.
  esp_reset_reason_t rr = esp_reset_reason();
  bool timerWake = (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) || (rr == ESP_RST_DEEPSLEEP);
  bool faultReset = (rr == ESP_RST_BROWNOUT || rr == ESP_RST_PANIC ||
                     rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT);

  // A tired battery can brown out exactly when the radio switches on, and
  // the farmer's own reset can be the thing that triggers it. So do not
  // dismiss a fault reset outright: allow the first couple to still try,
  // and only start ignoring them once it is plainly a loop.
  if (faultReset) faultResetStreak++;
  else            faultResetStreak = 0;

  bool physicalReset = !timerWake &&
                       (!faultReset || faultResetStreak <= FAULT_RESET_GRACE);

  if (!timerWake) {
    log_msg("[WiFi] Boot was not a timer wake (reset reason " + String((int)rr) + ").");
    if (faultReset) {
      log_msg("[WiFi] That reset looks like a fault, streak=" + String(faultResetStreak) +
              (physicalReset ? " — trying anyway in case it was the reset button."
                             : " — repeated, so not treating it as a sync request."));
    }
  }

  float lfsNow = lfsUsageFraction();
  bool  lfsUrgent = (lfsNow >= 0.60f);

  if (physicalReset) {
    attemptWifi = true;
    wifiFailStreak = 0;      // someone is standing here; start clean
    log_msg("[WiFi] Reset button / power-on — this is a sync request. Trying WiFi.");
  } else if (lfsUrgent && bootsSinceAttempt >= WIFI_URGENT_CHECK_BOOTS) {
    attemptWifi = true;
    log_msg("[WiFi] Buffer at " + String(lfsNow * 100.0f, 0) + "% — checking for a network.");
  } else if (bootsSinceAttempt >= WIFI_IDLE_CHECK_BOOTS) {
    attemptWifi = true;
    log_msg("[WiFi] Routine check (" + String(bootsSinceAttempt) + " boots since the last one).");
  } else {
    log_msg("[WiFi] Skipped — no network expected on a timer wake. " +
            String(bootsSinceSync) + " reading(s) buffered since the last sync.");
  }

  if (attemptWifi) {
    bootsSinceAttempt = 0;
    // ── helper: clean radio reset ──
    auto wifiReset = [&]() {
      WiFi.disconnect(true, true);
      delay(300);
      WiFi.mode(WIFI_STA);
      delay(100);
      wdt_feed();
    };

    // ── helper: try one SSID ──
    auto tryConnect = [&](const String& ssid, const String& pass,
                          uint32_t timeout_ms) -> bool {
      log_msg("[WiFi] Trying: \"" + ssid + "\"" +
              (pass.length() > 0 ? " (secured)" : " (open)"));
      if (pass.length() > 0)
        WiFi.begin(ssid.c_str(), pass.c_str());
      else
        WiFi.begin(ssid.c_str());

      unsigned long t0 = millis();
      while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeout_ms) {
        wdt_feed();
        delay(500);
        wl_status_t st = WiFi.status();                     // V8-8: fail fast
        if ((st == WL_NO_SSID_AVAIL || st == WL_CONNECT_FAILED) && millis() - t0 > 3000) break;
      }
      if (WiFi.status() != WL_CONNECTED) {
        log_msg("[WiFi] Timed out. Status=" + String((int)WiFi.status()));
        wifiReset();
        return false;
      }
      log_msg("[WiFi] Associated. IP=" + WiFi.localIP().toString() +
              " — checking internet...");
      delay(1000);
      wdt_feed();
      if (isInternetAvailable()) {
        log_msg("[WiFi] Internet OK via: " + ssid);
        return true;
      }
      log_msg("[WiFi] No internet on: " + ssid);
      wifiReset();
      return false;
    };

    // ── one acquisition pass: scan, then connect to what the scan saw ──
    auto acquireOnce = [&]() -> bool {
      wifiReset();
      log_msg("[WiFi] Scanning...");
      delay(150);
      wdt_feed();

      struct OpenNet { String ssid; int rssi; };
      std::vector<OpenNet> openNets;
      bool savedInRange = false;
      int  savedRssi    = 0;

      int n = WiFi.scanNetworks(false, false);
      wdt_feed();
      if (n > 0) {
        log_msg("[WiFi] Found " + String(n) + " network(s):");
        for (int i = 0; i < n; i++) {
          String s    = WiFi.SSID(i);
          int    rssi = WiFi.RSSI(i);
          bool   open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
          log_msg("  \"" + s + "\" RSSI=" + String(rssi) +
                  (open ? " OPEN" : " secured"));
          if (s.length() > 0 && s == ssid_config) { savedInRange = true; savedRssi = rssi; }
          if (open && s.length() > 0) openNets.push_back({s, rssi});
        }
      } else {
        log_msg("[WiFi] Nothing on the air.");
      }
      WiFi.scanDelete();

      // the saved hotspot, but only when it is actually there
      if (savedInRange) {
        log_msg("[WiFi] Saved network in range (RSSI " + String(savedRssi) + ").");
        if (tryConnect(ssid_config, pass_config, 15000)) return true;
      } else if (ssid_config.length() > 0) {
        log_msg("[WiFi] Saved network \"" + ssid_config + "\" not in range.");
      }

      // fall back to open networks. Guarded by the fail streak so a
      // permanent captive portal nearby cannot burn radio on every boot.
      if (!openNets.empty()) {
        if (wifiFailStreak >= WIFI_OPEN_SKIP_AFTER_FAILURES) {
          log_msg("[WiFi] " + String(openNets.size()) + " open network(s) present, but " +
                  String(wifiFailStreak) + " failures in a row — skipping them.");
        } else {
          std::sort(openNets.begin(), openNets.end(),
            [](const OpenNet& a, const OpenNet& b){ return a.rssi > b.rssi; });
          int tried = 0;
          for (auto& net : openNets) {
            if (tried >= 2) break;    // strongest two only
            if (tryConnect(net.ssid, "", 10000)) return true;
            tried++;
          }
        }
      }
      return false;
    };

    // ── V8-20: keep trying while the farmer is standing there ──────────
    // A reset means somebody has come to the field to sync. They may press
    // the button before switching the hotspot on, or the phone may take a
    // few seconds to bring it up, so one pass is not enough. Retry for a
    // couple of minutes. A routine timer check makes a single pass, since
    // nobody is waiting and there is nothing to find.
    int passes = physicalReset ? SYNC_REQUEST_PASSES : 1;
    for (int pass = 0; pass < passes && !internetReady; pass++) {
      if (pass > 0) {
        log_msg("[WiFi] No network yet (pass " + String(pass) + " of " + String(passes) +
                "). Waiting " + String(SYNC_REQUEST_GAP_MS / 1000) +
                " s — switch the hotspot on now if it is not already.");
        unsigned long waitStart = millis();
        while (millis() - waitStart < SYNC_REQUEST_GAP_MS) { wdt_feed(); delay(250); }
      }
      internetReady = acquireOnce();
    }

    // ── Outcome ──
    // The streak only gates the open-network fallback. The scan and the
    // saved-network connect always run when we look at all.
    if (internetReady) {
      wifiFailStreak = 0;
      bootsSinceSync = 0;
      log_msg("[WiFi] Online. Streak reset.");
    } else {
      wifiFailStreak++;
      log_msg("[WiFi] Offline this boot. Streak=" + String(wifiFailStreak) +
              ". Reading is stored and will go out on the next connection.");
    }
  }

  // ---- Online tasks ----
  if (internetReady) {

    // ── NTP time sync ──
    // V8-14: only sync when it is actually needed. A healthy DS3231 drifts
    // about a minute a year, so one resync per day is ample. Field logs show
    // iPhone/Android hotspots blocking NTP's UDP port 123 outright, which
    // made the old code burn a full 10 s of radio time (~120 mA) on EVERY
    // online boot for a sync that could never succeed, then print
    // "timestamps will be invalid" while the RTC was keeping perfect time.
    uint32_t todayNum = 0;
    bool needNtp = !clockValid();          // no usable clock at all → must try
    if (rtcReady && clockValid()) {
      todayNum = (uint32_t)(rtc.now().unixtime() / 86400UL);
      if (todayNum != lastNtpDay) needNtp = true;   // first upload of the day
    }

    if (needNtp) {
      // Multiple servers: some hotspots block one but not the others.
      configTime(19800, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
      log_msg("[NTP] Waiting for time sync...");
      {
        struct tm ti;
        unsigned long ntpStart = millis();
        while (millis() - ntpStart < 10000) {
          wdt_feed();
          if (getLocalTime(&ti, 500) && ti.tm_year > 120) {  // year > 2020
            ntpReady = true;
            break;
          }
        }
      }
      if (ntpReady) {
        log_msg("[NTP] Time synced: " + getTimestamp());
        // Also update hardware RTC if present
        if (rtcReady) {
          struct tm ti;
          getLocalTime(&ti, 100);
          rtc.adjust(DateTime(ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday,
                              ti.tm_hour, ti.tm_min, ti.tm_sec));
          lastNtpDay = (uint32_t)(rtc.now().unixtime() / 86400UL);
          log_msg("[NTP] Hardware RTC updated.");
        }
      } else if (clockValid()) {
        // Not a problem: the DS3231 is the primary clock, NTP is only a
        // correction. Common on phone hotspots, which block UDP 123.
        log_msg("[NTP] Unavailable (hotspot blocks it?) — using DS3231 time.");
      } else {
        log_msg("[NTP] Sync failed and no valid RTC — records marked clockValid:false.");
      }
    } else {
      log_msg("[NTP] Skipped — DS3231 valid, already synced today.");
    }

    // FIX: always flush stored data on every wake — original code skipped
    // flush on timer wakeup, so offline records accumulated indefinitely.
    syncAllToSheets();
  }

  wdt_feed();

  // ============================================================
  //  SLEEP DECISION (reading already stored and uploaded above)
  // ============================================================
  if (distanceCm < 0) {
    deepSleepSeconds = configuredSleepSecs;
  } else {
    if (lastWaterLevel >= 0.0f) {
      float change = fabsf(waterLevelCm - lastWaterLevel);
      deepSleepSeconds = (change > WATER_LEVEL_CHANGE_THRESHOLD_CM)
                         ? (uint32_t)SHORT_SLEEP_SECONDS
                         : configuredSleepSecs;
      log_msg("[Sleep] Level change=" + String(change, 1) +
              "cm → sleep=" + String(deepSleepSeconds) + "s");
    } else {
      deepSleepSeconds = configuredSleepSecs;
    }
    lastWaterLevel = waterLevelCm;
  }

  // ---- Enter deep sleep ----
  log_msg("[Sleep] Going to sleep for " + String(deepSleepSeconds) + "s. Bye.");
  Serial.flush();
  // Re-enable brownout detector before sleep — good practice, and required
  // if the chip wakes into a low-power state where voltage is more stable.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, bodSaved);   // V8-6 restore, not "1"
  esp_task_wdt_delete(NULL);
  esp_task_wdt_deinit();

  // FIX: Stop ultrasonic sensor from clicking/vibrating during sleep.
  // We force the TRIG_PIN strictly LOW and tell the RTC to hold it there.
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  gpio_hold_en((gpio_num_t)TRIG_PIN);
  gpio_deep_sleep_hold_en();

  esp_sleep_enable_timer_wakeup((uint64_t)deepSleepSeconds * 1000000ULL);
  esp_deep_sleep_start();
}

void loop() { /* intentionally empty */ }

// ============================================================
//  WEB PORTAL HANDLERS
// ============================================================

void handleRoot() {
  if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD))
    return server.requestAuthentication();

  int lfsLines = countLinesLfs(LFS_DATA_FILE);
  int sdLines  = countLinesSd(SD_BACKUP_FILE);

  // Build page — break into chunks to avoid single huge String allocation
  String html;
  html.reserve(3000);

  html = F("<!DOCTYPE html><html><head>"
           "<meta name='viewport' content='width=device-width,initial-scale=1.0'>"
           "<title>AWD PIPE Monitor</title><style>"
           "*{box-sizing:border-box;margin:0;padding:0}"
           "body{font-family:'Segoe UI',sans-serif;background:#0f172a;color:#e2e8f0;"
                "min-height:100vh;padding:20px}"
           ".card{max-width:480px;margin:30px auto;background:#1e293b;border-radius:16px;"
                 "padding:28px;box-shadow:0 4px 30px rgba(0,0,0,.4)}"
           "h2{color:#38bdf8;margin-bottom:4px;font-size:1.4em}"
           ".sub{color:#64748b;font-size:.85em;margin-bottom:20px}"
           ".sb{display:flex;gap:12px;margin-bottom:20px;flex-wrap:wrap}"
           ".badge{background:#0f172a;border-radius:8px;padding:8px 12px;"
                  "font-size:.8em;flex:1;text-align:center}"
           ".badge span{display:block;color:#38bdf8;font-weight:700;"
                        "font-size:1.1em;margin-top:2px}"
           "label{display:block;color:#94a3b8;font-size:.82em;margin:14px 0 4px}"
           "input,select{width:100%;padding:10px 12px;background:#0f172a;"
                         "border:1px solid #334155;border-radius:8px;"
                         "color:#e2e8f0;font-size:.95em}"
           "input:focus,select:focus{outline:none;border-color:#38bdf8}"
           ".hint{color:#475569;font-size:.75em;margin-top:3px}"
           ".btn{display:block;width:100%;padding:12px;border-radius:8px;border:none;"
                "font-size:.95em;font-weight:600;cursor:pointer;margin-top:10px}"
           ".bp{background:#38bdf8;color:#0f172a}"
           ".bs{background:#1e3a5f;color:#38bdf8;border:1px solid #38bdf8}"
           ".bd{background:#7f1d1d;color:#fca5a5}"
           ".bw{background:#78350f;color:#fde68a}"
           "hr{border:none;border-top:1px solid #334155;margin:20px 0}"
           "#lb{background:#0f172a;border-radius:10px;padding:16px;margin-top:12px;"
               "min-height:60px;text-align:center;font-size:1.1em;color:#38bdf8;display:none}"
           "#cm{text-align:center;min-height:20px;font-size:.85em;"
               "margin-top:8px;color:#fde68a}"
           "</style></head><body><div class='card'>"
           "<h2>AWD PIPE Monitor</h2>"
           "<p class='sub'>Configuration &amp; Status</p>"
           "<div class='sb'>");

  html += "<div class='badge'>LFS Records<span>" + String(lfsLines) + "</span></div>";
  html += "<div class='badge'>LFS Used<span>" +
          String((int)(lfsUsageFraction() * 100)) + "%</span></div>";
  html += "<div class='badge'>SD Records<span>" +
          (sdReady ? String(sdLines) : String("N/A")) + "</span></div>";
  html += F("</div>"
            "<form action='/save' method='post'>"
            "<label>Wi-Fi SSID</label>"
            "<input type='text' name='ssid' value='");
  html += ssid_config;
  html += F("'><label>Wi-Fi Password</label>"
            "<input type='password' name='pass' placeholder='Leave blank to keep current'>"
            "<label>Pipe Height (cm)</label>"
            "<input type='number' step='0.1' name='pipe_height' id='pH' value='");
  html += String(pipeHeightCm);
  html += F("'><label>Update Frequency</label><select name='frequency'>");

  // Frequency options
  const uint32_t freqs[]  = {60, 300, 600, 1800, 3600, 5400};
  const char* labels[] = {"1 Minute","5 Minutes","10 Minutes",
                              "30 Minutes","60 Minutes","90 Minutes"};
  for (int i = 0; i < 6; i++) {
    html += "<option value='" + String(freqs[i]) + "'";
    if (configuredSleepSecs == freqs[i]) html += " selected";
    html += ">" + String(labels[i]) + "</option>";
  }

  html += F("</select>"
            "<label>Custom seconds <span style='color:#475569'>— overrides dropdown</span></label>"
            "<input type='number' name='custom_seconds' placeholder='e.g. 900 for 15 min'>"
            "<button type='submit' class='btn bp' style='margin-top:20px'>"
            "Save &amp; Restart</button></form><hr>"
            "<a href='/download' class='btn bs' "
            "style='text-decoration:none;text-align:center;padding:12px'>"
            "Download LittleFS Backup</a>"
            "<button id='cb' class='btn bd' style='margin-top:8px'>"
            "Clear LittleFS Data</button>"
            "<div id='cm'></div><hr>"
            "<b style='color:#94a3b8'>Live Calibration</b>"
            "<p class='hint' style='margin-top:6px'>"
            "Adjust Pipe Height and watch level update live.</p>"
            "<button id='lb2' class='btn bs' style='margin-top:10px'>"
            "Start Live Reading</button>"
            "<div id='lb'></div></div>"
            "<script>"
            "let lid=null;"
            "const lb2=document.getElementById('lb2'),"
            "lb=document.getElementById('lb'),"
            "pH=document.getElementById('pH');"
            "lb2.addEventListener('click',()=>{"
            "if(lid){clearInterval(lid);lid=null;"
            "lb2.textContent='Start Live Reading';lb2.className='btn bs';"
            "lb.style.display='none';}else{"
            "lb2.textContent='Stop Live Reading';lb2.className='btn bw';"
            "lb.style.display='block';go();lid=setInterval(go,3000);}});"
            "function go(){"
            "fetch('/live-data').then(r=>r.json()).then(d=>{"
            "if(d&&d.distance!==undefined){"
            "const w=Math.max(0,parseFloat(pH.value)-d.distance).toFixed(1);"
            "lb.innerHTML='Sensor: <b>'+d.distance+' cm</b> | Level: <b>'+w+' cm</b>';}"
            "else lb.innerHTML='<span style=color:#f87171>Sensor error</span>';"
            "}).catch(()=>{lb.innerHTML='<span style=color:#f87171>Lost</span>';});}"
            "const cb=document.getElementById('cb'),cm=document.getElementById('cm');"
            "cb.addEventListener('click',()=>{"
            "if(cb.dataset.c){"
            "fetch('/clear-backup',{method:'POST'}).then(r=>r.text()).then(t=>{"
            "cm.textContent=t;delete cb.dataset.c;"
            "cb.textContent='Clear LittleFS Data';cb.className='btn bd';});"
            "}else{cb.dataset.c='1';"
            "cb.textContent='Confirm — Click Again';cb.className='btn bw';"
            "cm.textContent='Permanently deletes all local readings.';}});"
            "</script></body></html>");

  server.send(200, "text/html", html);
}

// ---- /save ----
void handleSave() {
  if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD))
    return server.requestAuthentication();

  String newSsid  = server.arg("ssid");
  String newPass  = server.arg("pass");
  String newPipe  = server.arg("pipe_height");
  String custom   = server.arg("custom_seconds");
  String freq     = server.arg("frequency");

  if (newSsid.length() > 0) ssid_config  = newSsid;
  if (newPass.length() > 0) pass_config  = newPass;
  if (newPipe.length() > 0) pipeHeightCm = newPipe.toFloat();

  long newSleep = 0;                                   // V8-12
  if (custom.length() > 0) newSleep = custom.toInt();
  if (newSleep <= 0)        newSleep = freq.toInt();
  if (newSleep < 10 || newSleep > 86400) newSleep = NORMAL_SLEEP_SECONDS;
  configuredSleepSecs = (uint32_t)newSleep;
  if (pipeHeightCm < 5.0f || pipeHeightCm > 300.0f) pipeHeightCm = 55.0f;

  saveConfig();

  server.send(200, "text/html",
    F("<html><body style='font-family:sans-serif;background:#0f172a;color:#38bdf8;"
      "display:flex;align-items:center;justify-content:center;height:100vh'>"
      "<div style='text-align:center'><h2>Saved!</h2>"
      "<p>Device restarting...</p></div></body></html>"));
  // FIX: shouldRestart flag was set but loop() is intentionally empty so
  // it was never checked — device never actually restarted after save.
  // Call ESP.restart() directly after a short delay for the response to send.
  delay(1500);
  ESP.restart();
}

// ---- /download ----
void handleBackupDownload() {
  if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD))
    return server.requestAuthentication();
  File f = LittleFS.open(LFS_DATA_FILE, "r");
  if (!f) { server.send(404, "text/plain", "No data file."); return; }
  server.sendHeader("Content-Disposition",
                    "attachment; filename=\"awd_pipe_data.txt\"");
  server.streamFile(f, "text/plain");
  f.close();
}

// ---- /live-data ----
void handleLiveData() {
  int d = getDistanceCm();
  if (d < 0)
    server.send(200, "application/json", F("{\"error\":\"sensor_failure\"}"));
  else
    server.send(200, "application/json",
                "{\"distance\":" + String(d) + "}");
}

// ---- /clear-backup ----
void handleClearBackup() {
  if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD))
    return server.requestAuthentication();
  // FIX: also clean up any leftover temp file from an aborted spill/flush
  LittleFS.remove(LFS_TEMP_FILE);
  if (LittleFS.exists(LFS_DATA_FILE) && LittleFS.remove(LFS_DATA_FILE))
    server.send(200, "text/plain", "LittleFS data cleared.");
  else
    server.send(200, "text/plain", "No data to clear.");
}