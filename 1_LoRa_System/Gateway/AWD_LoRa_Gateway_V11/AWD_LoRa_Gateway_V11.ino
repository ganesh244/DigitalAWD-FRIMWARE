/**
 * @file AWD_LoRa_Gateway_V11.ino
 * @brief Gateway V11 — explicit E220 configuration, RSSI logging,
 *        deaf-receiver recovery, crash-safe buffer, heartbeat every cycle.
 *
 * ===================== CHANGES V10.1 → V11 =====================
 * Root cause targeted (from Heartbeat sheet, Feb 28 – Mar 2 2026):
 *   the ESP32 and modem stayed alive and uploaded 24 heartbeats/day,
 *   but "Last LoRa RX" froze for 4 days and recovered without a reboot.
 *   That is a hung E220 receiver, most likely from supply dips caused by
 *   the EC200U sharing the 3.3 V rail. V10.1 could neither detect nor
 *   reset that state.
 *
 * FIX-V11-1:  resetE220ToFactory() removed. C4 C4 C4 is an E32 command;
 *             the E220 (LLCC68) only understands C0/C1/C2. Replaced by
 *             configureE220(): writes the full register block with C0
 *             (address, 9600 8N1, 2.4 kbps, LORA_CHANNEL, max power,
 *             RSSI byte ON, ambient-noise RSSI ON, transparent, LBT off)
 *             and reads it back with C1 to verify. Read-back failure
 *             is the "module hung" signal.
 * FIX-V11-2:  LORA_CHANNEL default 15 = 865.125 MHz. Factory channel 23
 *             (873.125 MHz) sits inside the LTE band-5 downlink used by
 *             Jio and outside India's 865–867 MHz licence-free band.
 *             ALL NODES MUST USE THE SAME CHANNEL (node V6 does).
 * FIX-V11-3:  RSSI byte enabled on the gateway. Every frame is stored as
 *             ts|device|distance|level|rssi and uploaded with "rssi".
 *             Ambient noise floor read at every heartbeat ("noise").
 * FIX-V11-4:  Deaf-receiver recovery ladder in checkLoRaHealth():
 *             45 min silence → reconfigure + verify; still silent →
 *             power-cycle the E220 if LORA_PWR_PIN is wired; after
 *             3 h silence → save queue and restart the ESP32 (max once
 *             per 6 h to avoid boot loops). Heartbeat reports lora_cfg_ok.
 * FIX-V11-5:  Heartbeat sent EVERY cycle, after the data flush, not only
 *             when the buffer was empty. Adds noise, per-node RSSI,
 *             lora_cfg_ok, reset reason, uptime, free heap.
 * FIX-V11-6:  Crash-safe flush: a leftover /lfs_buffer.txt.tmp (reset
 *             during upload) is merged back into the buffer at boot and
 *             before every flush instead of being deleted.
 * FIX-V11-7:  getFormattedTime() wrote 34 bytes into 32-byte buffers when
 *             the RTC returned an invalid year (stack corruption).
 *             Buffers are now TS_LEN (40) and snprintf-bounded.
 * FIX-V11-8:  gsmSendCommand() breaks on ERROR / +CME ERROR, and gsmPOST
 *             waits for CONNECT instead of a fixed 15 s. Saves ~30 s per
 *             batch, so a backlog uploads before the watchdog window.
 * FIX-V11-9:  Modem over-temperature gate raised 60 → 70 °C (EC200U is
 *             rated to +75 °C; the sheet never exceeded 44 °C).
 * FIX-V11-10: Water status: a node reports -1 for a failed sensor read;
 *             the gateway maps level < 0 to "SensorError" instead of
 *             "Low 0 cm".
 * FIX-V11-11: Last LoRa RX timestamp kept in RTC memory so a software
 *             restart does not report NO_DATA.
 *
 * HARDWARE (recommended, not required by this firmware):
 *   - Feed the E220-900T30D from its own 3.3 V/5 V regulator, not the
 *     DevKit 3.3 V pin shared with the EC200U. Add ≥470 µF at the E220.
 *   - Optional: high-side P-MOSFET on the E220 VCC driven by LORA_PWR_PIN
 *     (GPIO5, free since the SD card was removed). Gate through 10 kΩ,
 *     100 kΩ pull-up to VCC; LOW = E220 powered. Set LORA_PWR_PIN to -1
 *     if not wired (default).
 * =================================================================
 *
 * ===================== CHANGES V10 → V10.1 =====================
 * FIX-V10.1-1: resetE220ToFactory() — sends C4 C4 C4 command to
 *              E220 in config mode at boot. Clears any accidentally
 *              set registers (ambient RSSI output, fixed-point mode)
 *              that cause ���� garbage bytes after every packet.
 *              Called once at boot, returns to normal mode after.
 *              Safe because all nodes are also factory default.
 *
 * FIX-V10.1-2: listenToLoRa() drain window — after every complete
 *              frame (\n), drains up to 100ms of trailing non-
 *              printable bytes from ring buffer silently. Prevents
 *              any residual E220 status bytes reaching the parser
 *              even if reset doesn't fully clear them.
 * =================================================================
 *
 * ===================== CHANGES FROM V9 → V10 =====================
 * CHANGE-1:  SD card completely removed. All storage now uses LittleFS
 *            on internal ESP32 flash. Eliminates SPI contention,
 *            loose socket failures, and *** corruption symbols.
 *
 * CHANGE-2:  Buffer file renamed from /buffer.txt to /lfs_buffer.txt
 *            to make it obvious in logs that LittleFS is the source.
 *
 * CHANGE-3:  Config file (/config.json) also moved to LittleFS.
 *            loadConfig() / saveConfig() now use LittleFS directly.
 *
 * CHANGE-4:  tryRemountSD() replaced with checkLittleFS() — simply
 *            verifies LittleFS is mounted; attempts remount if not.
 *
 * CHANGE-5:  Web portal /logs, /download, /clear now serve from
 *            LittleFS /lfs_buffer.txt.
 *
 * CHANGE-6:  Heartbeat now reports LittleFS free bytes instead of
 *            SD free MB.
 *
 * CHANGE-7:  All SD_CS, SD_MOSI, SD_MISO, SD_SCK pin definitions
 *            retained as comments only — GPIO pins now free to reuse.
 *
 * CHANGE-8:  sdReady flag and all SD.xxx() calls removed throughout.
 *            internalMemReady is the single storage health flag.
 *
 * CHANGE-9:  flushAllBuffers() now calls flushSpecificBuffer() once
 *            (LittleFS only). No SD path.
 *
 * CHANGE-10: listenToLoRa() overflow cooldown extended from 10s→30s
 *            to absorb E220 slave boot bursts (FIX for Lora1 missing
 *            data after slave device reset).
 *
 * CHANGE-11: Ring buffer drain added BEFORE idx=0 reset in overflow
 *            handler — prevents partial valid frame being wiped by
 *            stale junk still in ring buffer.
 *
 * LittleFS partition size note:
 *   Default ESP32 partition scheme gives LittleFS ~1.5MB.
 *   At ~192 bytes/record, that holds ~8000 records.
 *   At 20 records/hour (5 devices × 4 tx/hr), buffer lasts ~400 hours
 *   before wrapping — far more than enough for any GSM outage.
 *   Use partition scheme "Default 4MB with spiffs" in Arduino IDE
 *   (or "min_spiffs" variant) to confirm LittleFS gets ≥1MB.
 * =================================================================
 */

// ================== LIBRARIES ==================
#include <WiFi.h>
#include <HardwareSerial.h>
// SD.h and SPI.h removed — no SD card
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <time.h>
#include <Preferences.h>
#include <FS.h>
#include <LittleFS.h>
#include <esp_task_wdt.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/uart.h"
#include <ctype.h>

// ================== DEBUG ==================
#define DEBUG_MODE 1
#if DEBUG_MODE
  #define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...)  Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
  #define DEBUG_PRINTF(...)
#endif

// ================== COMPILE-TIME SWITCHES ==================
#define RADIO_CORE_ENABLED 0

// ================== PINS ==================
#define CONFIG_JUMPER_PIN  15
#define LORA_DIAG_PIN      33
#define LORA_RX_PIN        16
#define LORA_TX_PIN        17
#define LORA_M0            21
#define LORA_M1            19
#define LORA_AUX           18
// FIX-V11-4: optional E220 power switch (P-MOSFET high side, active LOW).
// -1 = not wired. GPIO5 is free since the SD card was removed in V10.
#define LORA_PWR_PIN       -1
#define LORA_PWR_ACTIVE_LOW 1
// FIX-V11-2: 0..83 → 850.125 + CH MHz. 15 = 865.125 MHz (India licence-free).
// MUST match DEVICE_CHANNEL in every node.
#define LORA_CHANNEL       15
#define GSM_RX             26
#define GSM_TX             27
#define GSM_PWR_KEY         4
#define GSM_RST_PIN        32
// SD pins removed — GPIO 5,13,14,35 now free
// #define SD_CS    5
// #define SD_MOSI  13
// #define SD_MISO  35
// #define SD_SCK   14
#define I2C_SDA            22
#define I2C_SCL            23

// ================== STORAGE PATHS ==================
#define LFS_BUFFER_FILE   "/lfs_buffer.txt"
#define LFS_BACKUP_FILE   "/backup.txt"      // legacy name kept for migration read
#define LFS_CONFIG_FILE   "/config.json"
#define LFS_CONFIG_TMP    "/config.tmp"

// ================== TIMING & SIZING ==================
#define WDT_TIMEOUT_SECONDS     180
#define TS_LEN                   40      // FIX-V11-7
#define LORA_RESTART_SILENCE_MS  (3UL*3600UL*1000UL)   // FIX-V11-4 ladder step 3
#define LORA_RESTART_MIN_GAP_MS  (6UL*3600UL*1000UL)   // never restart more often
#define GSM_SILENCE_TIMEOUT     1200000UL
#define LORA_SILENCE_TIMEOUT    2700000UL
#define GHOST_CHECK_INTERVAL     900000UL
#define QUEUE_SAVE_INTERVAL_MS    60000UL
#define LONG_UPTIME_INTERVAL   172800000UL
#define SYSTEM_REFRESH_INTERVAL 604800000UL
#define UPLOAD_BATCH_SIZE            20
#define GSM_BAUD_FAST            115200

// ================== HARDWARE SERIAL ==================
HardwareSerial LoRaSerial(2);
HardwareSerial gsm(1);

// ================== WEB SERVER ==================
WebServer server(80);
const char* CONFIG_USERNAME = "admin";
const char* CONFIG_PASSWORD = "password";

// ================== CLOUD & CONNECTIVITY ==================
char wifi_ssid[64]    = "Default_SSID";
char wifi_pass[64]    = "Default_PASS";
char gscript_url[256] = "https://script.google.com/macros/s/AKfycbwMl7VGQlu4--r5DjptzE8JF5XXDoRIWSnYJ-0qCuYBEQnLbaBvXHzBNmuQcgjiynnf/exec";
char deviceNicknames[5][32] = {"Plot 1","Plot 2","Plot 3","Plot 4","Plot 5"};
long connection_interval_min = 60;

// ================== KNOWN DEVICES ==================
const char* knownDevices[] = {
    "T22D(Lora1)","T22D(Lora2)","T22D(Lora3)","T22D(Lora4)","T22D(Lora5)"
};

// ================== WATER THRESHOLDS ==================
const float THRESHOLD_LOW    =  7.0f;
const float THRESHOLD_GOOD   = 15.0f;
const float THRESHOLD_EXCESS = 20.0f;

// ================== RTC ==================
RTC_DS3231 rtc;

// ================== LORA ISR RING BUFFER ==================
#define LORA_ISR_BUFFER 2048
volatile char     loraIsrBuf[LORA_ISR_BUFFER];
volatile uint16_t loraIsrWrite        = 0;
volatile uint16_t loraIsrRead         = 0;
volatile uint32_t loraOverflowCount   = 0;
volatile uint32_t uartHwOverflowCount = 0;
portMUX_TYPE loraMux = portMUX_INITIALIZER_UNLOCKED;

// ================== SOFTWARE QUEUE ==================
#define QUEUE_SIZE    100
#define QUEUE_MSG_LEN 192
char loraMessageQueue[QUEUE_SIZE][QUEUE_MSG_LEN];
int  queueWriteIndex = 0;
int  queueReadIndex  = 0;
volatile uint32_t queueOverflowCount = 0;

// ================== FLUSH MUTEX ==================
portMUX_TYPE flushMux = portMUX_INITIALIZER_UNLOCKED;
bool isFlushing = false;

struct FlushResult { bool dataWasPresent; int recordsUploaded; };

// ================== GLOBAL STATE FLAGS ==================
Preferences preferences;
bool gsmPowered         = false;
bool gsmFullyPowered    = false;
bool gsmBandsConfigured = false;
bool internalMemReady   = false;   // Single storage health flag (replaces sdReady)
bool rtcReady           = false;
bool shouldSaveConfig   = false;
bool isScheduledUploadTime = false;
bool gsmBusyHTTP        = false;
bool radioRefreshing    = false;

// ================== TELEMETRY CACHE ==================
const char* simOperator    = "Unknown";
char lastKnownData[5][64]  = {};
unsigned long lastLoraRxPerDevice[5] = {0,0,0,0,0};
int  lastRssiPerDevice[5]  = {0,0,0,0,0};   // FIX-V11-3 dBm, 0 = never heard
int  lastNoiseDbm          = 0;             // FIX-V11-3 ambient noise at heartbeat
bool loraCfgOk             = false;         // FIX-V11-1 last C1 read-back matched
uint32_t loraCfgFailCount  = 0;
RTC_DATA_ATTR static char lastLoRaTimestamp[TS_LEN] = "NO_DATA";  // FIX-V11-11
RTC_DATA_ATTR static uint32_t rtcMagic = 0;
RTC_DATA_ATTR static bool rtcRestartedForSilence = false;
RTC_DATA_ATTR static int silenceRestartCount = 0;
int  currentGsmStrength    = 0;
float currentGsmTemp       = 0.0f;
int  currentGsmBaud        = GSM_BAUD_FAST;

// ================== TIMERS ==================
unsigned long bootTime               = 0;
unsigned long lastGsmActivityTime    = 0;
unsigned long lastGhostCheck         = 0;
unsigned long lastQueueSave          = 0;
unsigned long lastScheduledTask      = 0;
unsigned long scheduled_task_interval_ms = 3600000UL;
unsigned long lastUartMaintenance    = 0;
unsigned long lastRadioRefresh       = 0;
unsigned long lastRadioModeRefresh   = 0;
unsigned long lastLoraRxTime         = 0;
unsigned long lastLongUptimeMaintenance = 0;
unsigned long lastSystemRefresh      = 0;
unsigned long radioRefreshStart      = 0;
int           loraRecoveryAttempts   = 0;
RTC_DATA_ATTR static int loraRestartCount = 0;

// ================== FORWARD DECLARATIONS ==================
void handleRoot();
void handleSave();
void handleLogs();
void handleDownload();
void handleClear();
void performScheduledTasks();
void gsmSendCommand(const char* cmd, uint32_t timeout,
                    char* outBuffer = NULL, size_t outSize = 0,
                    const char* expectedResponse = "OK",
                    bool clearBuffer = true);
const char* getWaterStatus(float waterLevel);
bool isKnownDevice(const char* dev);
void saveQueueToStorage();
FlushResult flushAllBuffers();
FlushResult flushSpecificBuffer(fs::FS& fs, const char* fileName, const char* fsName);
bool sendHeartbeat();
void saveConfig();
void getFormattedTime(char* outBuf);
bool gsmPOST(const char* json);
void gsmDisconnect();
bool gsmConnect();
void print_reset_reason();
void validateHardwareState();
void startConfigPortal();
void exorciseZombie();
bool checkGSMStateRobust();
bool attemptBootSync(int durationSec);
void gsmHardReset();
void listenToLoRa();
void processLoRaFrame(const char* frame, int rssiDbm);
void checkLoRaHealth();
void reInitLoRaUART();
void performLoRaRecovery();
bool syncRTCfromGSM();
void updateGsmActivity(unsigned long ts = 0);
void drainGsmUart(uint32_t durationMs);
int  getGsmStrength();
float getGsmTemperature();
const char* autoDetectApn();
bool checkLittleFS();      // CHANGE-4: replaces tryRemountSD()
bool configureE220(bool persist); // FIX-V11-1
bool readE220Config(uint8_t* out6);
bool readE220Rssi(int* noiseDbm, int* lastDbm);
void e220PowerCycle();
void recoverLeftoverTemp(); // FIX-V11-6
void onLoraRx();
void initializeDefaultConfig();
bool loadConfig();
void gsmPowerOn();
bool waitForSim();
void migrateLegacyBackup(); // migrate old /backup.txt if present

// ======================================================================
//  CHANGE-4: checkLittleFS — verifies / remounts internal flash FS
// ======================================================================
bool checkLittleFS() {
    if (internalMemReady) return true;
    DEBUG_PRINTLN("[LFS] Attempting remount...");
    internalMemReady = LittleFS.begin(false);
    if (internalMemReady) DEBUG_PRINTLN("[LFS] Remount SUCCESS.");
    else                  DEBUG_PRINTLN("[LFS] Remount FAILED. Storage unavailable!");
    return internalMemReady;
}

// ======================================================================
//  FIX-V11-1: Explicit E220 register configuration (replaces C4 reset)
//
//  E220 (LLCC68) register interface, config mode (M0=M1=HIGH, 9600 8N1):
//    C0 00 06 <ADDH ADDL REG0 REG1 REG2 REG3>  write, saved on power-down
//    C2 00 06 <...>                            write, volatile
//    C1 00 06                                  read → C1 00 06 <6 bytes>
//    wrong format                              → FF FF FF
//  Normal mode, only when REG1 bit5 (ambient RSSI) is set:
//    C0 C1 C2 C3 00 02                         → C1 00 02 <noise> <lastRSSI>
//    dBm = -(256 - value)
// ======================================================================
static const uint8_t E220_REG0 = 0x62;               // 9600 baud, 8N1, 2.4 kbps air
static const uint8_t E220_REG1 = 0x20;               // 200 B sub-packet, ambient RSSI ON, max power
static const uint8_t E220_REG2 = (uint8_t)LORA_CHANNEL;
static const uint8_t E220_REG3 = 0x83;               // RSSI byte ON, transparent, LBT off, WOR 2000 ms

static bool waitAuxHigh(uint32_t timeoutMs) {
    unsigned long t0 = millis();
    while (digitalRead(LORA_AUX) == LOW) {
        if (millis() - t0 > timeoutMs) return false;
        delay(1);
    }
    return true;
}

static void e220EnterConfigMode() {
    digitalWrite(LORA_M0, HIGH);
    digitalWrite(LORA_M1, HIGH);
    delay(50);
    waitAuxHigh(200);
    delay(5);
    while (LoRaSerial.available()) LoRaSerial.read();
}

static void e220LeaveConfigMode() {
    digitalWrite(LORA_M0, LOW);
    digitalWrite(LORA_M1, LOW);
    delay(50);
    waitAuxHigh(200);
    delay(5);
}

// Collect up to n bytes within timeoutMs. Returns bytes read.
static int e220Read(uint8_t* buf, int n, uint32_t timeoutMs) {
    int idx = 0; unsigned long t0 = millis();
    while (millis() - t0 < timeoutMs && idx < n) {
        if (LoRaSerial.available()) buf[idx++] = (uint8_t)LoRaSerial.read();
        else delay(1);
    }
    return idx;
}

// Read the 6 config registers. Caller must be in config mode.
static bool e220ReadRegsRaw(uint8_t* out6) {
    while (LoRaSerial.available()) LoRaSerial.read();
    uint8_t cmd[] = {0xC1, 0x00, 0x06};
    LoRaSerial.write(cmd, 3); LoRaSerial.flush();
    uint8_t resp[16] = {0};
    int n = e220Read(resp, 9, 300);
    if (n == 9 && resp[0] == 0xC1 && resp[1] == 0x00 && resp[2] == 0x06) {
        memcpy(out6, resp + 3, 6); return true;
    }
    DEBUG_PRINTF("[LORA] C1 read failed (%d bytes):", n);
    for (int i = 0; i < n; i++) DEBUG_PRINTF(" %02X", resp[i]);
    DEBUG_PRINTLN("");
    return false;
}

bool readE220Config(uint8_t* out6) {
    e220EnterConfigMode();
    bool ok = e220ReadRegsRaw(out6);
    e220LeaveConfigMode();
    return ok;
}

// Write (C0 = persistent, C2 = volatile) and verify. Sets loraCfgOk.
bool configureE220(bool persist) {
    DEBUG_PRINTF("[LORA] Configuring E220: ch=%d (%.3f MHz), RSSI byte ON, ambient RSSI ON\n",
                 LORA_CHANNEL, 850.125f + LORA_CHANNEL);
    esp_task_wdt_reset();
    e220EnterConfigMode();

    uint8_t cmd[9] = { (uint8_t)(persist ? 0xC0 : 0xC2), 0x00, 0x06,
                       0x00, 0x00, E220_REG0, E220_REG1, E220_REG2, E220_REG3 };
    LoRaSerial.write(cmd, 9); LoRaSerial.flush();
    uint8_t resp[16] = {0};
    int n = e220Read(resp, 9, 400);
    DEBUG_PRINTF("[LORA] write response (%d bytes):", n);
    for (int i = 0; i < n; i++) DEBUG_PRINTF(" %02X", resp[i]);
    DEBUG_PRINTLN("");
    delay(persist ? 100 : 20);   // C0 commits to the module's non-volatile memory

    uint8_t regs[6] = {0};
    bool ok = e220ReadRegsRaw(regs);
    if (ok) {
        ok = (regs[2] == E220_REG0 && regs[3] == E220_REG1 &&
              regs[4] == E220_REG2 && (regs[5] & 0xD0) == (E220_REG3 & 0xD0));
        DEBUG_PRINTF("[LORA] read-back: %02X %02X %02X %02X %02X %02X → %s\n",
                     regs[0], regs[1], regs[2], regs[3], regs[4], regs[5],
                     ok ? "MATCH" : "MISMATCH");
    }
    e220LeaveConfigMode();
    reInitLoRaUART();   // drop any bytes emitted around the mode change
    loraCfgOk = ok;
    if (ok) loraCfgFailCount = 0; else loraCfgFailCount++;
    DEBUG_PRINTLN(ok ? "[LORA] E220 configured and verified."
                     : "[LORA] E220 NOT responding / config mismatch.");
    return ok;
}

// Ambient noise and last-packet RSSI (normal mode). Returns false on timeout.
bool readE220Rssi(int* noiseDbm, int* lastDbm) {
    if (gsmBusyHTTP) return false;
    // Quiesce the parser: anything already in the ring is a real frame.
    listenToLoRa();
    while (LoRaSerial.available()) LoRaSerial.read();
    uint8_t cmd[] = {0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x02};
    LoRaSerial.write(cmd, 6); LoRaSerial.flush();
    uint8_t resp[8] = {0};
    int n = e220Read(resp, 5, 150);
    if (n == 5 && resp[0] == 0xC1 && resp[1] == 0x00 && resp[2] == 0x02) {
        if (noiseDbm) *noiseDbm = -(256 - (int)resp[3]);
        if (lastDbm)  *lastDbm  = -(256 - (int)resp[4]);
        return true;
    }
    // Not an RSSI answer: push whatever came back into the ring so a
    // frame that arrived meanwhile is not lost (printable bytes only).
    portENTER_CRITICAL(&loraMux);
    for (int i = 0; i < n; i++) {
        uint16_t next = (loraIsrWrite + 1) % LORA_ISR_BUFFER;
        if (next != loraIsrRead) { loraIsrBuf[loraIsrWrite] = (char)resp[i]; loraIsrWrite = next; }
    }
    portEXIT_CRITICAL(&loraMux);
    return false;
}

// FIX-V11-4: hard power cycle through the optional MOSFET.
void e220PowerCycle() {
#if LORA_PWR_PIN >= 0
    DEBUG_PRINTLN("[LORA] Power-cycling E220 via LORA_PWR_PIN...");
    pinMode(LORA_PWR_PIN, OUTPUT);
    digitalWrite(LORA_PWR_PIN, LORA_PWR_ACTIVE_LOW ? HIGH : LOW);   // off
    digitalWrite(LORA_M0, LOW); digitalWrite(LORA_M1, LOW);
    for (int i = 0; i < 3; i++) { delay(500); esp_task_wdt_reset(); }
    digitalWrite(LORA_PWR_PIN, LORA_PWR_ACTIVE_LOW ? LOW : HIGH);   // on
    delay(300);
    waitAuxHigh(1500);
    reInitLoRaUART();
    configureE220(false);
#else
    DEBUG_PRINTLN("[LORA] LORA_PWR_PIN not wired — cannot power-cycle E220.");
#endif
}

// FIX-V11-6: a reset during flushSpecificBuffer() leaves /lfs_buffer.txt.tmp.
// Merge it back instead of deleting it.
void recoverLeftoverTemp() {
    if (!internalMemReady) return;
    char tmpName[64];
    snprintf(tmpName, sizeof(tmpName), "%s.tmp", LFS_BUFFER_FILE);
    if (!LittleFS.exists(tmpName)) return;
    DEBUG_PRINTLN("[LFS] Leftover flush temp found — merging back into buffer.");
    File src = LittleFS.open(tmpName, "r");
    File dst = LittleFS.open(LFS_BUFFER_FILE, "a");
    int lines = 0;
    if (src && dst) {
        while (src.available()) {
            esp_task_wdt_reset();
            String line = src.readStringUntil('\n');
            line.trim();
            if (line.length() > 2) { dst.println(line); lines++; }
        }
    }
    if (src) src.close();
    if (dst) dst.close();
    if (src && dst) { LittleFS.remove(tmpName); DEBUG_PRINTF("[LFS] Merged %d lines.\n", lines); }
}

// ======================================================================
//  Migration: move old /backup.txt entries into /lfs_buffer.txt once
// ======================================================================
void migrateLegacyBackup() {
    if (!internalMemReady) return;
    if (!LittleFS.exists(LFS_BACKUP_FILE)) return;
    DEBUG_PRINTLN("[LFS] Migrating legacy /backup.txt → /lfs_buffer.txt ...");
    File src  = LittleFS.open(LFS_BACKUP_FILE, "r");
    File dest = LittleFS.open(LFS_BUFFER_FILE, "a");
    if (src && dest) {
        while (src.available()) {
            String line = src.readStringUntil('\n');
            if (line.length() > 2) dest.println(line);
        }
        DEBUG_PRINTLN("[LFS] Migration complete.");
    }
    if (src)  src.close();
    if (dest) dest.close();
    LittleFS.remove(LFS_BACKUP_FILE);
}

// ======================================================================
//  reInitLoRaUART (unchanged from V9)
// ======================================================================
void reInitLoRaUART() {
    LoRaSerial.end();
    delay(100);
    LoRaSerial.setRxBufferSize(1024);
    LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    delay(100);
    uart_set_rx_full_threshold(UART_NUM_2, 16);
    uart_set_rx_timeout(UART_NUM_2, 1);
    portENTER_CRITICAL(&loraMux);
    loraIsrRead = loraIsrWrite;
    portEXIT_CRITICAL(&loraMux);
    unsigned long _drainStart = millis();
    while (millis() - _drainStart < 500) {
        while (LoRaSerial.available()) LoRaSerial.read();
        delay(10);
    }
    portENTER_CRITICAL(&loraMux);
    loraIsrRead = loraIsrWrite;
    portEXIT_CRITICAL(&loraMux);
    DEBUG_PRINTLN("[LORA] UART reinitialized — thresholds applied.");
}

// ======================================================================
//  LoRa ISR hardware drain into ring buffer
// ======================================================================
void onLoraRx() {
    int avail = LoRaSerial.available();
    if (avail == 0) return;
    if (avail >= 1000) {
        uartHwOverflowCount++;
        DEBUG_PRINTLN("[WARN] UART HW Buffer saturated!");
    }
    portENTER_CRITICAL(&loraMux);
    while (LoRaSerial.available()) {
        char c = LoRaSerial.read();
        uint16_t next = (loraIsrWrite + 1) % LORA_ISR_BUFFER;
        if (next != loraIsrRead) {
            loraIsrBuf[loraIsrWrite] = c;
            loraIsrWrite = next;
        } else {
            loraOverflowCount++;
        }
    }
    portEXIT_CRITICAL(&loraMux);
}

void updateGsmActivity(unsigned long ts) {
    lastGsmActivityTime = (ts == 0) ? millis() : ts;
}

void drainGsmUart(uint32_t durationMs) {
    unsigned long start = millis();
    while (millis() - start < durationMs) {
        while (gsm.available()) gsm.read();
        esp_task_wdt_reset();
        delay(5);
    }
}

void exorciseZombie() {
    DEBUG_PRINTLN("[EXORCIST] Attempting to cure Zombie Mode...");
    drainGsmUart(100);
    gsm.write(27); delay(100);
    gsm.write(26); delay(500);
    gsm.print("+++"); delay(1000);
    gsm.println("AT"); delay(500);
    while (gsm.available()) { DEBUG_PRINT((char)gsm.read()); }
    DEBUG_PRINTLN("\n[EXORCIST] Ritual complete.");
    updateGsmActivity();
}

void gsmHardReset() {
    DEBUG_PRINTLN("\n[GSM_RST] HARDWARE RESET INITIATED...");
    gsmFullyPowered    = false;
    gsmPowered         = false;
    gsmBandsConfigured = false;
    pinMode(GSM_RST_PIN, OUTPUT);
    digitalWrite(GSM_RST_PIN, LOW); delay(1000);
    pinMode(GSM_RST_PIN, INPUT);
    DEBUG_PRINTLN("[GSM_RST] Waiting 10s for hardware init...");
    for (int i = 0; i < 10; i++) { delay(1000); esp_task_wdt_reset(); }
    gsm.end(); delay(100);
    gsm.begin(GSM_BAUD_FAST, SERIAL_8N1, GSM_RX, GSM_TX);
    drainGsmUart(50);
    updateGsmActivity();
}

bool checkGSMStateRobust() {
    while (gsm.available()) gsm.read();
    static uint16_t missCount = 0;
    gsm.println("AT");
    char buf[64] = {0}; int idx = 0;
    unsigned long start = millis();
    while (millis() - start < 350) {
        while (gsm.available()) {
            char c = gsm.read();
            if (idx < 63) buf[idx++] = c;
        }
        buf[idx] = '\0';
        if (strstr(buf, "OK")) { missCount = 0; updateGsmActivity(); return true; }
        delay(5);
    }
    if (missCount < 10) missCount++;
    if (missCount < 3) return true;
    return false;
}

bool attemptBootSync(int durationSec) {
    DEBUG_PRINTF("[GSM_BOOT] Listening for boot (Max %ds)...\n", durationSec);
    unsigned long start = millis();
    unsigned long lastPing = 0;
    while (millis() - start < (unsigned long)(durationSec * 1000)) {
        esp_task_wdt_reset();
        if (millis() - start > 4000) {
            if (millis() - lastPing > 500) { gsm.println("AT"); lastPing = millis(); }
        }
        char buf[64]; int idx = 0;
        unsigned long cmdStart = millis();
        while (millis() - cmdStart < 500) {
            while (gsm.available()) {
                char c = gsm.read();
                if (isPrintable(c)) DEBUG_PRINT(c);
                if (idx > 60) idx = 0;
                if (idx < 63) buf[idx++] = c;
                buf[idx] = '\0';
                if (idx >= 2 && buf[idx-2] == 'O' && buf[idx-1] == 'K') {
                    DEBUG_PRINTF("\n[GSM_BOOT] Module Woke Up at %d baud!\n", currentGsmBaud);
                    updateGsmActivity(); return true;
                }
            }
            delay(10);
        }
        if (millis() - start > 4000) DEBUG_PRINT(".");
    }
    DEBUG_PRINTLN("\n[GSM_BOOT] Timed out.");
    if (currentGsmBaud != GSM_BAUD_FAST) {
        gsm.end(); gsm.begin(GSM_BAUD_FAST, SERIAL_8N1, GSM_RX, GSM_TX);
        drainGsmUart(50); currentGsmBaud = GSM_BAUD_FAST;
    }
    return false;
}

bool waitForSim() {
    DEBUG_PRINTLN("[GSM] Checking SIM Card status...");
    for (int i = 0; i < 5; i++) {
        esp_task_wdt_reset();
        char resp[64];
        gsmSendCommand("AT+CPIN?", 2000, resp, sizeof(resp));
        if (strstr(resp, "READY") != NULL) { DEBUG_PRINTLN("[GSM] SIM READY."); return true; }
        unsigned long _simWait = millis();
        while (millis() - _simWait < 1000) { listenToLoRa(); delay(20); }
    }
    DEBUG_PRINTLN("[GSM] FATAL: SIM not detected.");
    return false;
}

void gsmSendCommand(const char* cmd, uint32_t timeout,
                    char* outBuffer, size_t outSize,
                    const char* expectedResponse, bool clearBuffer) {
    if (clearBuffer) { while (gsm.available()) gsm.read(); }
    updateGsmActivity();
    DEBUG_PRINTF("[GSM_CMD] -> %s\n", cmd);
    gsm.println(cmd);
    char dummyBuffer[256];
    char* targetBuffer = (outBuffer != NULL && outSize > 0) ? outBuffer : dummyBuffer;
    size_t targetSize  = (outBuffer != NULL && outSize > 0) ? outSize   : sizeof(dummyBuffer);
    memset(targetBuffer, 0, targetSize);
    int idx = 0;
    unsigned long startTime = millis();
    unsigned long lastFeed  = millis();
    while (millis() - startTime < timeout) {
#if RADIO_CORE_ENABLED == 0
        listenToLoRa();
#endif
        server.handleClient();
        if (millis() - lastFeed > 1000) { esp_task_wdt_reset(); lastFeed = millis(); }
        delay(1); yield();
        if (gsm.available()) {
            char c = (char)gsm.read();
            if (idx < (int)(targetSize - 1)) targetBuffer[idx++] = c;
            targetBuffer[idx] = '\0';
            if (expectedResponse && expectedResponse[0] != '\0' &&
                strstr(targetBuffer, expectedResponse) != NULL) {
                if (strchr(targetBuffer, '\n')) drainGsmUart(20);
                break;
            }
            // FIX-V11-8: a rejected command should not cost the full timeout
            if (expectedResponse && expectedResponse[0] != '\0' &&
                (strstr(targetBuffer, "\r\nERROR") != NULL ||
                 strstr(targetBuffer, "+CME ERROR") != NULL)) {
                drainGsmUart(20);
                break;
            }
        }
    }
    targetBuffer[targetSize - 1] = '\0';
    DEBUG_PRINTF("[GSM_RSP] %s\n", targetBuffer);
    updateGsmActivity();
}

void validateHardwareState() {
    DEBUG_PRINTLN("[MAINT] Validating hardware state...");
    if (millis() - bootTime < 20000) { DEBUG_PRINTLN("[HEALTH] Boot grace. Skipping."); return; }
    bool isActuallyOn = checkGSMStateRobust();
    if (isActuallyOn) {
        drainGsmUart(20);
        char cellBuf[256];
        gsmSendCommand("AT+QENG=\"servingcell\"", 3000, cellBuf, sizeof(cellBuf), NULL, true);
        if (strlen(cellBuf) < 5) {
            if (!checkGSMStateRobust()) { gsmHardReset(); gsmFullyPowered = false; return; }
        }
        if (strstr(cellBuf, "SEARCH") != NULL || strstr(cellBuf, "LIMSRV") != NULL) {
            updateGsmActivity(); gsmFullyPowered = true; return;
        }
        char stackBuf[128];
        gsmSendCommand("AT+QISTATE?", 1500, stackBuf, sizeof(stackBuf), NULL, true);
        if (strlen(stackBuf) < 2) { gsmHardReset(); gsmFullyPowered = false; }
        else gsmFullyPowered = true;
    } else {
        gsmFullyPowered = false; gsmPowered = false;
    }
}

void gsmPowerOn() {
    DEBUG_PRINTLN("[GSM_PWR] Smart & Safe Boot...");
    if (!gsmFullyPowered) {
        gsm.end(); gsm.begin(GSM_BAUD_FAST, SERIAL_8N1, GSM_RX, GSM_TX); drainGsmUart(50);
    }
    if (checkGSMStateRobust()) {
        delay(1500);
        char resp[32];
        gsmSendCommand("AT", 1000, resp, sizeof(resp), "OK", true);
        if (strstr(resp, "OK") != NULL) {
            gsmFullyPowered = true;
            DEBUG_PRINTLN("[GSM_PWR] Already ON. Skipping Pulse.");
            exorciseZombie(); return;
        }
    }
    gsmFullyPowered = false;
    pinMode(GSM_PWR_KEY, OUTPUT);
    digitalWrite(GSM_PWR_KEY, HIGH); delay(100);
    esp_task_wdt_reset();
    digitalWrite(GSM_PWR_KEY, LOW);  delay(2800);
    esp_task_wdt_reset();
    digitalWrite(GSM_PWR_KEY, HIGH); delay(1500);
    esp_task_wdt_reset();
    if (attemptBootSync(40)) { gsmFullyPowered = true; return; }
    gsmHardReset();
    esp_task_wdt_reset();
    digitalWrite(GSM_PWR_KEY, LOW);  delay(2800);
    esp_task_wdt_reset();
    digitalWrite(GSM_PWR_KEY, HIGH); delay(1000);
    esp_task_wdt_reset();
    if (attemptBootSync(40)) { gsmFullyPowered = true; DEBUG_PRINTLN("[GSM_PWR] RECOVERED!"); }
    else { DEBUG_PRINTLN("[GSM_PWR] FATAL: Unresponsive."); gsmFullyPowered = false; }
}

void gsmDisconnect() {
    DEBUG_PRINTLN("[GSM_OFF] Deactivating data context...");
    gsmSendCommand("AT+QIDEACT=1", 5000);
    drainGsmUart(1000);
    gsmPowered = false;
    updateGsmActivity();
    delay(200); drainGsmUart(50);
}

const char* autoDetectApn() {
    DEBUG_PRINTLN("[GSM] Auto-detecting APN...");
    char copsBuf[128];
    gsmSendCommand("AT+COPS?", 3000, copsBuf, sizeof(copsBuf));
    simOperator = "Unknown";
    if (strstr(copsBuf, "Jio") || strstr(copsBuf, "JIO") || strstr(copsBuf, "jio"))
        { simOperator = "Jio";    return "jionet"; }
    if (strstr(copsBuf, "Airtel") || strstr(copsBuf, "airtel") || strstr(copsBuf, "AIRTEL"))
        { simOperator = "Airtel"; return "airtelgprs.com"; }
    if (strstr(copsBuf, "Vi") || strstr(copsBuf, "VI") ||
        strstr(copsBuf, "Vodafone") || strstr(copsBuf, "Idea"))
        { simOperator = "Vi";     return "www"; }
    if (strstr(copsBuf, "BSNL") || strstr(copsBuf, "CellOne"))
        { simOperator = "BSNL";   return "bsnlnet"; }
    char imsiBuf[64];
    gsmSendCommand("AT+CIMI", 3000, imsiBuf, sizeof(imsiBuf));
    char imsi[32] = {0}; int idx = 0;
    for (int i = 0; imsiBuf[i] != '\0' && idx < 31; i++)
        if (isdigit(imsiBuf[i])) imsi[idx++] = imsiBuf[i];
    imsi[idx] = '\0';
    if (strlen(imsi) < 6) { simOperator = "Hologram"; return "hologram"; }
    if (strncmp(imsi, "405", 3) == 0) { simOperator = "Jio";    return "jionet"; }
    if (strncmp(imsi, "404", 3) == 0 || strncmp(imsi, "406", 3) == 0)
        { simOperator = "Airtel"; return "airtelgprs.com"; }
    simOperator = "Hologram"; return "hologram";
}

int getGsmStrength() {
    if (!gsmFullyPowered) return 0;
    char csqBuf[32];
    gsmSendCommand("AT+CSQ", 1000, csqBuf, sizeof(csqBuf), "OK", true);
    char* colon = strchr(csqBuf, ':');
    char* comma = strchr(csqBuf, ',');
    if (colon && comma && comma > colon) { *comma = '\0'; return atoi(colon + 1); }
    return 0;
}

float getGsmTemperature() {
    if (!gsmFullyPowered) return 0.0f;
    char tempBuf[64];
    gsmSendCommand("AT+QTEMP", 1000, tempBuf, sizeof(tempBuf), "OK", true);
    char* colon = strchr(tempBuf, ':');
    return colon ? atof(colon + 1) : 0.0f;
}

bool syncRTCfromGSM() {
    if (!rtcReady) return false;
    char cclkBuf[64];
    gsmSendCommand("AT+CCLK?", 2000, cclkBuf, sizeof(cclkBuf));
    char* quoteStart = strchr(cclkBuf, '"');
    char* quoteEnd   = strrchr(cclkBuf, '"');
    if (!quoteStart || !quoteEnd || quoteEnd <= quoteStart) return false;
    *quoteEnd = '\0';
    char* timeStr = quoteStart + 1;
    if (strlen(timeStr) < 17) return false;
    int yy = atoi(timeStr), mm = atoi(timeStr+3), dd = atoi(timeStr+6);
    int hh = atoi(timeStr+9), min = atoi(timeStr+12), ss = atoi(timeStr+15);
    int fullYear = 2000 + yy;
    if (fullYear < 2024 || fullYear > 2035) {
        DEBUG_PRINTF("[RTC] REJECTED: Year %d outside 2024-2035.\n", fullYear); return false;
    }
    if (mm < 1 || mm > 12 || dd < 1 || dd > 31) return false;
    int tzQuarters = 0;
    char* tzPos = strchr(timeStr, '+');
    if (tzPos) tzQuarters = atoi(tzPos + 1);
    else { tzPos = strchr(timeStr, '-'); if (tzPos) tzQuarters = -atoi(tzPos + 1); }
    DateTime dt(fullYear, mm, dd, hh, min, ss);
    if (tzQuarters == 0 &&
        (strcmp(simOperator,"Jio")==0 || strcmp(simOperator,"Airtel")==0 ||
         strcmp(simOperator,"Vi")==0  || strcmp(simOperator,"BSNL")==0)) {
        dt = dt + TimeSpan(0,5,30,0);
    } else if (tzQuarters != 0) {
        dt = dt + TimeSpan(0, tzQuarters*15/60, tzQuarters*15%60, 0);
    }
    rtc.adjust(dt);
    DEBUG_PRINTF("[RTC] Synced: %04d-%02d-%02d %02d:%02d:%02d\n",
                 dt.year(),dt.month(),dt.day(),dt.hour(),dt.minute(),dt.second());
    return true;
}

bool gsmConnect() {
    gsmPowerOn();
    if (!gsmFullyPowered) return false;
    if (gsmPowered) return true;
    char respAT[32];
    gsmSendCommand("AT", 1000, respAT, sizeof(respAT), "OK", false);
    if (strstr(respAT, "OK") == NULL) return false;
    if (!waitForSim()) return false;
    gsmSendCommand("AT+CTZU=1", 2000);
    if (!gsmBandsConfigured) {
        gsmSendCommand("AT+QCFG=\"nwscanmode\",3,1", 2000);
        gsmSendCommand("AT+QCFG=\"band\",0,80000A0E85,0", 2000);
        gsmBandsConfigured = true;
    }
    unsigned long startTime = millis();
    while (millis() - startTime < 45000) {
        esp_task_wdt_reset();
        char ceregResponse[64];
        gsmSendCommand("AT+CEREG?", 1200, ceregResponse, sizeof(ceregResponse), "OK", false);
        char* commaPos = strchr(ceregResponse, ',');
        int stat = (commaPos != NULL) ? atoi(commaPos + 1) : -1;
        if (stat == 1 || stat == 5) {
            const char* detectedApn = autoDetectApn();
            syncRTCfromGSM();
            gsmSendCommand("AT+QIDEACT=1", 10000); delay(1000);
            char apnCmd[128];
            snprintf(apnCmd, sizeof(apnCmd), "AT+QICSGP=1,1,\"%s\",\"\",\"\",1", detectedApn);
            gsmSendCommand(apnCmd, 5000); delay(1000);
            char respAct[64];
            gsmSendCommand("AT+QIACT=1", 15000, respAct, sizeof(respAct));
            if (strstr(respAct, "OK") != NULL) { gsmPowered = true; return true; }
            return false;
        }
        DEBUG_PRINT(".");
        unsigned long _regWait = millis();
        while (millis() - _regWait < (unsigned long)(1800 + random(0,600))) {
            listenToLoRa(); esp_task_wdt_reset(); delay(50);
        }
    }
    return false;
}

bool gsmPOST(const char* json) {
    if (!gsmPowered) return false;
    gsmBusyHTTP = true;
    drainGsmUart(100); delay(120); drainGsmUart(20);
    gsmSendCommand("AT+QHTTPCFG=\"contenttype\",1", 2000);
    char urlCmd[64];
    snprintf(urlCmd, sizeof(urlCmd), "AT+QHTTPURL=%d,80", (int)strlen(gscript_url));
    char urlResp[64];
    gsmSendCommand(urlCmd, 15000, urlResp, sizeof(urlResp), "CONNECT", true);
    if (strstr(urlResp, "CONNECT") == NULL) { gsmBusyHTTP = false; return false; }
    gsm.print(gscript_url);
    {   // wait for the modem to accept the URL (OK), max 3 s
        char okBuf[32] = {0}; int oi = 0; unsigned long t0 = millis();
        while (millis() - t0 < 3000) {
            while (gsm.available()) { char c = (char)gsm.read(); if (oi < 31) okBuf[oi++] = c; }
            if (strstr(okBuf, "OK") || strstr(okBuf, "ERROR")) break;
            listenToLoRa(); delay(5);
        }
        if (strstr(okBuf, "ERROR")) { gsmBusyHTTP = false; return false; }
    }
    char postCmd[64];
    snprintf(postCmd, sizeof(postCmd), "AT+QHTTPPOST=%d,90,90", (int)strlen(json));
    char postResp[64];
    gsmSendCommand(postCmd, 15000, postResp, sizeof(postResp), "CONNECT", true);
    if (strstr(postResp, "CONNECT") == NULL) { gsmBusyHTTP = false; return false; }
    gsm.print(json);
    updateGsmActivity();
    char finalResponse[256];
    memset(finalResponse, 0, sizeof(finalResponse));
    int idx = 0;
    unsigned long startTime = millis();
    unsigned long lastFeed  = millis();
    while (millis() - startTime < 90000) {
        updateGsmActivity();
        onLoraRx();
#if RADIO_CORE_ENABLED == 0
        listenToLoRa();
#endif
        server.handleClient();
        if ((loraIsrWrite - loraIsrRead + LORA_ISR_BUFFER) % LORA_ISR_BUFFER > 700) {
#if RADIO_CORE_ENABLED == 0
            listenToLoRa();
#endif
        }
        if (millis() - lastFeed > 1000) { esp_task_wdt_reset(); lastFeed = millis(); }
        delay(1); yield();
        while (gsm.available()) {
            char c = (char)gsm.read();
            if (idx < 255) finalResponse[idx++] = c;
            finalResponse[idx] = '\0';
        }
        if (strstr(finalResponse, "+QHTTPPOST:") != NULL) break;
    }
    updateGsmActivity();
    bool success = (strstr(finalResponse, "+QHTTPPOST: 0,200") != NULL ||
                    strstr(finalResponse, "+QHTTPPOST: 0,302") != NULL);
    if (success) gsmSendCommand("AT+QHTTPREAD=80", 10000);
    gsmBusyHTTP = false;
    return success;
}

// FIX-V11-7: outBuf must be at least TS_LEN bytes. Never overruns.
void getFormattedTime(char* outBuf) {
    if (rtcReady) {
        DateTime now = rtc.now();
        if (now.year() < 2024 || now.year() > 2035)
            snprintf(outBuf, TS_LEN, "2024-01-01 00:00:00 (RTC_INVALID)");
        else
            snprintf(outBuf, TS_LEN, "%04d-%02d-%02d %02d:%02d:%02d",
                     now.year(),now.month(),now.day(),
                     now.hour(),now.minute(),now.second());
    } else {
        snprintf(outBuf, TS_LEN, "1970-01-01 00:00:00 (RTC_ERR)");
    }
}

void print_reset_reason() {
    esp_reset_reason_t reason = esp_reset_reason();
    DEBUG_PRINT("[SYS] Reset reason: ");
    switch (reason) {
        case ESP_RST_POWERON:   DEBUG_PRINTLN("Power on");   break;
        case ESP_RST_SW:        DEBUG_PRINTLN("Software");   break;
        case ESP_RST_PANIC:     DEBUG_PRINTLN("Panic");      break;
        case ESP_RST_BROWNOUT:  DEBUG_PRINTLN("Brownout!");  break;
        case ESP_RST_TASK_WDT:  DEBUG_PRINTLN("WDT (loop)"); break;
        default:                DEBUG_PRINTLN("Other");      break;
    }
}

const char* getWaterStatus(float waterLevel) {
    if (waterLevel < 0.0f)             return "SensorError";   // FIX-V11-10: node sent -1
    if (waterLevel < THRESHOLD_LOW)    return "Low";
    if (waterLevel < THRESHOLD_GOOD)   return "Good";
    if (waterLevel < THRESHOLD_EXCESS) return "Excess";
    return "Flood Alert";
}

bool isKnownDevice(const char* dev) {
    for (int i = 0; i < 5; i++)
        if (strcmp(dev, knownDevices[i]) == 0) return true;
    return false;
}

// ======================================================================
//  Config: load from LittleFS (CHANGE-3)
// ======================================================================
void initializeDefaultConfig() { saveConfig(); }

bool loadConfig() {
    if (!internalMemReady) return false;
    File configFile = LittleFS.open(LFS_CONFIG_FILE, "r");
    if (!configFile) { initializeDefaultConfig(); return false; }
    static StaticJsonDocument<1024> doc;
    doc.clear();
    DeserializationError error = deserializeJson(doc, configFile);
    configFile.close();
    if (error) return false;
    const char* _ssid = doc["wifi_ssid"]   | (const char*)wifi_ssid;
    const char* _pass = doc["wifi_pass"]   | (const char*)wifi_pass;
    const char* _url  = doc["gscript_url"] | (const char*)gscript_url;
    strncpy(wifi_ssid,   _ssid, sizeof(wifi_ssid)-1);   wifi_ssid[sizeof(wifi_ssid)-1]=0;
    strncpy(wifi_pass,   _pass, sizeof(wifi_pass)-1);   wifi_pass[sizeof(wifi_pass)-1]=0;
    strncpy(gscript_url, _url,  sizeof(gscript_url)-1); gscript_url[sizeof(gscript_url)-1]=0;
    connection_interval_min = doc["connection_interval_min"] | connection_interval_min;
    JsonArray nicknames = doc["nicknames"];
    if (!nicknames.isNull()) {
        for (int i = 0; i < 5; i++) {
            if (i < (int)nicknames.size() && nicknames[i].is<const char*>()) {
                strncpy(deviceNicknames[i], nicknames[i].as<const char*>(), 31);
                deviceNicknames[i][31] = '\0';
            }
        }
    }
    scheduled_task_interval_ms = connection_interval_min * 60UL * 1000UL;
    return true;
}

// ======================================================================
//  Config: atomic save to LittleFS (CHANGE-3)
// ======================================================================
void saveConfig() {
    if (!internalMemReady) return;
    File tempFile = LittleFS.open(LFS_CONFIG_TMP, "w");
    if (!tempFile) return;
    static StaticJsonDocument<1024> doc;
    doc.clear();
    doc["wifi_ssid"]               = wifi_ssid;
    doc["wifi_pass"]               = wifi_pass;
    doc["gscript_url"]             = gscript_url;
    doc["connection_interval_min"] = connection_interval_min;
    JsonArray nicknames = doc.createNestedArray("nicknames");
    for (int i = 0; i < 5; i++) nicknames.add(deviceNicknames[i]);
    serializeJson(doc, tempFile);
    tempFile.close();
    if (LittleFS.exists(LFS_CONFIG_FILE)) LittleFS.remove(LFS_CONFIG_FILE);
    LittleFS.rename(LFS_CONFIG_TMP, LFS_CONFIG_FILE);
}

// ======================================================================
//  Queue: drain pending frames to LittleFS only (CHANGE-8, CHANGE-9)
// ======================================================================
void saveQueueToStorage() {
    bool localFlushState;
    portENTER_CRITICAL(&flushMux);
    localFlushState = isFlushing;
    int items = (queueWriteIndex - queueReadIndex + QUEUE_SIZE) % QUEUE_SIZE;
    portEXIT_CRITICAL(&flushMux);
    if (localFlushState && items < QUEUE_SIZE - 10) return;
    if (items == 0) return;

    // CHANGE-4: verify LittleFS before write
    if (!checkLittleFS()) {
        DEBUG_PRINTLN("[LFS] Storage unavailable — queue drain skipped!");
        return;
    }

    File logFile = LittleFS.open(LFS_BUFFER_FILE, "a");
    if (!logFile) { DEBUG_PRINTLN("[LFS] Cannot open buffer file!"); return; }

    while (true) {
        char localMsg[QUEUE_MSG_LEN];
        portENTER_CRITICAL(&flushMux);
        if (queueReadIndex == queueWriteIndex) { portEXIT_CRITICAL(&flushMux); break; }
        strncpy(localMsg, loraMessageQueue[queueReadIndex], QUEUE_MSG_LEN - 1);
        localMsg[QUEUE_MSG_LEN - 1] = '\0';
        queueReadIndex = (queueReadIndex + 1) % QUEUE_SIZE;
        portEXIT_CRITICAL(&flushMux);

        // Cache last known data per device
        char* firstPipe = strchr(localMsg, '|');
        if (firstPipe) {
            char* msgContent = firstPipe + 1;
            for (int i = 0; i < 5; i++) {
                if (strncmp(msgContent, knownDevices[i], strlen(knownDevices[i])) == 0) {
                    strncpy(lastKnownData[i], msgContent, 63);
                    lastKnownData[i][63] = '\0';
                    break;
                }
            }
        }
        logFile.println(localMsg);
    }
    logFile.close();
}

// ======================================================================
//  Upload: flush LittleFS buffer (CHANGE-9 — one path only)
// ======================================================================
FlushResult flushSpecificBuffer(fs::FS& fs, const char* fileName, const char* fsName) {
    FlushResult result = {false, 0};
    if (!fs.exists(fileName)) return result;
    DEBUG_PRINTF("[UPLOAD] Found data in %s (%s). Uploading...\n", fsName, fileName);
    char tempFileName[64];
    snprintf(tempFileName, sizeof(tempFileName), "%s.tmp", fileName);
    if (fs.exists(tempFileName)) recoverLeftoverTemp();   // FIX-V11-6: merge, never delete
    if (!fs.exists(fileName)) return result;
    if (!fs.rename(fileName, tempFileName)) {
        DEBUG_PRINTLN("[UPLOAD] rename to .tmp failed — buffer kept for next cycle.");
        return result;
    }
    File flushFile = fs.open(tempFileName, "r");
    if (!flushFile) return result;
    result.dataWasPresent = true;

    static char batchBuffer[UPLOAD_BATCH_SIZE][QUEUE_MSG_LEN];
    int batchCount = 0;

    while (flushFile.available()) {
        esp_task_wdt_reset();
#if RADIO_CORE_ENABLED == 0
        listenToLoRa();
#endif
        server.handleClient();
        int len = flushFile.readBytesUntil('\n', batchBuffer[batchCount], QUEUE_MSG_LEN - 1);
        batchBuffer[batchCount][len] = '\0';
        if (len > 0 && batchBuffer[batchCount][len-1] == '\r') {
            batchBuffer[batchCount][--len] = '\0';
        }
        if (len < 2) continue;
        batchCount++;

        if (batchCount >= UPLOAD_BATCH_SIZE || !flushFile.available()) {
            char ts[TS_LEN]; getFormattedTime(ts);

            // CHANGE-6: LittleFS free bytes instead of SD free MB
            size_t lfsTotal = LittleFS.totalBytes();
            size_t lfsUsed  = LittleFS.usedBytes();
            float lfsFreeKB = (lfsTotal - lfsUsed) / 1024.0f;

            const char* networkType = gsmPowered ? "GSM" :
                (WiFi.status() == WL_CONNECTED ? "WiFi" : "Offline");

            static char jsonPayload[4608];
            int payloadLen = snprintf(jsonPayload, sizeof(jsonPayload),
                "{\"upload_ts\":\"%s\",\"network\":\"%s\",\"simOperator\":\"%s\","
                "\"gsmStrength\":%d,\"lfsFreeKB\":%.1f,\"readings\":[",
                ts, networkType, simOperator, currentGsmStrength, lfsFreeKB);

            bool firstEntry = true;
            for (int i = 0; i < batchCount; i++) {
                char* line = batchBuffer[i];
                char* p1 = strchr(line, '|');  if (!p1) continue;
                char* p2 = strchr(p1+1, '|');  if (!p2) continue;
                char* p3 = strchr(p2+1, '|');
                char gateway_rx_ts[TS_LEN];
                int ts_len = p1 - line; if (ts_len > TS_LEN-1) ts_len = TS_LEN-1;
                strncpy(gateway_rx_ts, line, ts_len); gateway_rx_ts[ts_len] = '\0';
                char dev[32];
                int dev_len = p2-(p1+1); if (dev_len > 31) dev_len = 31;
                strncpy(dev, p1+1, dev_len); dev[dev_len] = '\0';
                if (!isKnownDevice(dev)) continue;
                char tx_data[16]; float waterLevel = 0.0f; int rssi = 0;
                if (p3) {
                    int data_len = p3-(p2+1); if (data_len > 15) data_len = 15;
                    strncpy(tx_data, p2+1, data_len); tx_data[data_len] = '\0';
                    waterLevel = atof(p3+1);
                    char* p4 = strchr(p3+1, '|');           // FIX-V11-3: optional rssi
                    if (p4) rssi = atoi(p4+1);
                } else { strcpy(tx_data, "N/A"); }
                const char* status   = getWaterStatus(waterLevel);
                const char* commaStr = firstEntry ? "" : ",";
                char wlStr[16];
                if (waterLevel < 0.0f) strcpy(wlStr, "null");   // FIX-V11-10
                else snprintf(wlStr, sizeof(wlStr), "%.1f", waterLevel);
                int written = snprintf(jsonPayload + payloadLen,
                    sizeof(jsonPayload) - payloadLen,
                    "%s{\"device\":\"%s\",\"tx_data\":\"%s\","
                    "\"gateway_rx_ts\":\"%s\",\"waterLevel\":%s,\"status\":\"%s\",\"rssi\":%d}",
                    commaStr, dev, tx_data, gateway_rx_ts, wlStr, status, rssi);
                if (written > 0 && written < (int)(sizeof(jsonPayload)-payloadLen)) {
                    payloadLen += written; firstEntry = false;
                } else { payloadLen = sizeof(jsonPayload)-4; break; }
            }
            if (payloadLen > (int)(sizeof(jsonPayload)-4)) payloadLen = sizeof(jsonPayload)-4;
            strcpy(jsonPayload + payloadLen, "]}");

            if (gsmPOST(jsonPayload)) {
                result.recordsUploaded += batchCount;
            } else {
                File newBuffer = fs.open(fileName, "a");
                if (newBuffer) {
                    for (int i = 0; i < batchCount; i++) newBuffer.println(batchBuffer[i]);
                    newBuffer.close();
                }
            }
            batchCount = 0;
        }
    }
    flushFile.close();
    fs.remove(tempFileName);
    if (fs.exists(fileName)) {
        File f = fs.open(fileName, "r");
        if (f && f.size() == 0) { f.close(); fs.remove(fileName); }
        else if (f) f.close();
    }
    return result;
}

// ======================================================================
//  Upload: flush LittleFS only (CHANGE-9)
// ======================================================================
FlushResult flushAllBuffers() {
    FlushResult total = {false, 0};
    portENTER_CRITICAL(&flushMux);
    isFlushing = true;
    portEXIT_CRITICAL(&flushMux);

    if (internalMemReady) {
        FlushResult r = flushSpecificBuffer(LittleFS, LFS_BUFFER_FILE, "LittleFS");
        total.dataWasPresent  |= r.dataWasPresent;
        total.recordsUploaded += r.recordsUploaded;
    }
    // SD path removed

    portENTER_CRITICAL(&flushMux);
    isFlushing = false;
    portEXIT_CRITICAL(&flushMux);
    return total;
}

// ======================================================================
//  Heartbeat (CHANGE-6: reports LittleFS free KB)
// ======================================================================
bool sendHeartbeat() {
    char ts[TS_LEN]; getFormattedTime(ts);
    char devStatus[128] = {0}; char devBuf[24];
    for (int _di = 0; _di < 5; _di++) {
        unsigned long silenceMin = (lastLoraRxPerDevice[_di] == 0) ?
            9999 : (millis() - lastLoraRxPerDevice[_di]) / 60000UL;
        snprintf(devBuf, sizeof(devBuf), "%s%lu", _di > 0 ? "," : "", silenceMin);
        strncat(devStatus, devBuf, sizeof(devStatus)-strlen(devStatus)-1);
    }
    size_t lfsFreeKB = (LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024;
    // FIX-V11-5: per-node RSSI, noise floor, module health, reset reason
    char rssiList[64] = {0}; char rb[12];
    for (int _di = 0; _di < 5; _di++) {
        snprintf(rb, sizeof(rb), "%s%d", _di > 0 ? "," : "", lastRssiPerDevice[_di]);
        strncat(rssiList, rb, sizeof(rssiList)-strlen(rssiList)-1);
    }
    const char* rst = "Other";
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:  rst = "PowerOn";  break;
        case ESP_RST_SW:       rst = "Software"; break;
        case ESP_RST_PANIC:    rst = "Panic";    break;
        case ESP_RST_BROWNOUT: rst = "Brownout"; break;
        case ESP_RST_TASK_WDT: rst = "WDT";      break;
        default: break;
    }
    static char jsonBuf[1024];
    snprintf(jsonBuf, sizeof(jsonBuf),
        "{\"type\":\"heartbeat\",\"upload_ts\":\"%s\","
        "\"last_lora_rx\":\"%s\",\"gsm\":\"%s\","
        "\"gateway\":\"ALIVE\",\"simOperator\":\"%s\","
        "\"gsmStrength\":%d,\"temp\":%.1f,"
        "\"lora_overflows\":%lu,\"queue_overflows\":%lu,"
        "\"hw_overflows\":%lu,\"lora_recovery\":%d,"
        "\"lora_restarts\":%d,\"lfsFreeKB\":%zu,"
        "\"dev_silence_min\":[%s],"
        "\"noise\":%d,\"dev_rssi\":[%s],\"lora_cfg_ok\":%d,"
        "\"reset\":\"%s\",\"uptime_min\":%lu,\"heap\":%u,"
        "\"channel\":%d,\"fw\":\"GW-V11\"}",
        ts, lastLoRaTimestamp, gsmPowered ? "OK" : "OFF", simOperator,
        currentGsmStrength, currentGsmTemp,
        loraOverflowCount, queueOverflowCount, uartHwOverflowCount,
        loraRecoveryAttempts, loraRestartCount, lfsFreeKB, devStatus,
        lastNoiseDbm, rssiList, loraCfgOk ? 1 : 0,
        rst, (unsigned long)(millis() / 60000UL), (unsigned)ESP.getFreeHeap(),
        LORA_CHANNEL);
    return gsmPOST(jsonBuf);
}

// ======================================================================
//  LoRa frame enqueue (unchanged)
// ======================================================================
void processLoRaFrame(const char* frame, int rssiDbm) {
    if (strlen(frame) < 5) return;
    if (strncmp(frame, "T22D(", 5) != 0) return;
    loraRecoveryAttempts = 0;
    rtcRestartedForSilence = false;      // FIX-V11-4: a frame re-arms the restart ladder
    lastLoraRxTime = millis();
    for (int _di = 0; _di < 5; _di++) {
        if (strncmp(frame, knownDevices[_di], strlen(knownDevices[_di])) == 0) {
            lastLoraRxPerDevice[_di] = millis();
            if (rssiDbm != 0) lastRssiPerDevice[_di] = rssiDbm;   // FIX-V11-3
            break;
        }
    }
    char ts[TS_LEN]; getFormattedTime(ts);
    strncpy(lastLoRaTimestamp, ts, sizeof(lastLoRaTimestamp)-1);
    lastLoRaTimestamp[sizeof(lastLoRaTimestamp)-1] = '\0';
    DEBUG_PRINTF("[LORA] RX: %s  (RSSI %d dBm)\n", frame, rssiDbm);
    bool queueOverflowed = false;
    portENTER_CRITICAL(&flushMux);
    // Stored line: ts|device|distance|level|rssi  (rssi added in V11)
    snprintf(loraMessageQueue[queueWriteIndex], QUEUE_MSG_LEN-1, "%s|%s|%d", ts, frame, rssiDbm);
    loraMessageQueue[queueWriteIndex][QUEUE_MSG_LEN-1] = '\0';
    queueWriteIndex = (queueWriteIndex + 1) % QUEUE_SIZE;
    if (queueWriteIndex == queueReadIndex) {
        queueReadIndex = (queueReadIndex + 1) % QUEUE_SIZE;
        queueOverflowCount++; queueOverflowed = true;
    }
    portEXIT_CRITICAL(&flushMux);
    if (queueOverflowed) DEBUG_PRINTLN("[WARN] Queue overflow! Dropping oldest.");
}

// ======================================================================
//  CHANGE-10/11: listenToLoRa — 30s cooldown + drain-before-reset
// ======================================================================
void listenToLoRa() {
    onLoraRx();

    static char rxBuf[QUEUE_MSG_LEN];
    static int  idx = 0;
    static unsigned long partialStart = 0;

    static uint32_t lastOverflowSeen  = 0;
    static unsigned long lastOverflowReset = 0;

    // CHANGE-10: 30s cooldown (was 10s) to absorb E220 slave boot bursts
    if (loraOverflowCount != lastOverflowSeen &&
        (millis() - lastOverflowReset > 30000)) {
        lastOverflowSeen  = loraOverflowCount;
        lastOverflowReset = millis();
        // CHANGE-11: drain ring buffer FIRST, then reset parser
        portENTER_CRITICAL(&loraMux);
        loraIsrRead = loraIsrWrite;   // discard all buffered junk atomically
        portEXIT_CRITICAL(&loraMux);
        idx = 0;                      // NOW safe to reset — ring is clean
        DEBUG_PRINTLN("[LORA] Overflow cleared — ring drained, parser reset.");
    }

    if (idx > 0 && (millis() - partialStart > 2000)) {
        DEBUG_PRINTLN("[LORA] Partial frame timeout. Discarding.");
        idx = 0;
    }

    while (loraIsrRead != loraIsrWrite) {
        portENTER_CRITICAL(&loraMux);
        char c = loraIsrBuf[loraIsrRead];
        loraIsrRead = (loraIsrRead + 1) % LORA_ISR_BUFFER;
        portEXIT_CRITICAL(&loraMux);

        if (c == '\r') continue;
        if (c == '\n') {
            rxBuf[idx] = '\0';
            int frameLen = idx;
            idx = 0;

            // FIX-V11-3: with REG3 bit7 set the E220 appends ONE RSSI byte
            // after every received packet (dBm = -(256 - byte)). Wait up to
            // 60 ms for it. A printable byte instead means a new frame has
            // already started — leave it in the ring. Any further
            // non-printable bytes inside a 100 ms window are discarded
            // (FIX-V10.1-2 behaviour retained).
            int rssiDbm = 0;
            bool rssiTaken = false;
            unsigned long drainEnd = millis() + 100;
            unsigned long rssiEnd  = millis() + 60;
            while (millis() < drainEnd) {
                onLoraRx();
                if (loraIsrRead == loraIsrWrite) {
                    if (rssiTaken || millis() > rssiEnd) break;
                    delay(2); continue;
                }
                portENTER_CRITICAL(&loraMux);
                uint8_t b = (uint8_t)loraIsrBuf[loraIsrRead];
                portEXIT_CRITICAL(&loraMux);
                if (b >= 0x20 && b < 0x7F) break;       // next frame — stop
                portENTER_CRITICAL(&loraMux);
                loraIsrRead = (loraIsrRead + 1) % LORA_ISR_BUFFER;
                portEXIT_CRITICAL(&loraMux);
                if (!rssiTaken && b >= 0x80) { rssiDbm = -(256 - (int)b); rssiTaken = true; }
            }
            if (frameLen >= 5) processLoRaFrame(rxBuf, rssiDbm);
            continue;
        }
        if (isprint((unsigned char)c) && idx < (int)(sizeof(rxBuf)-1)) {
            if (idx == 0) partialStart = millis();
            rxBuf[idx++] = c;
        } else { idx = 0; partialStart = 0; }
    }
}

// FIX-V11-4: recovery ladder for a deaf receiver.
//   step 1 (45 min silent) : volatile re-config (C2) + read-back verify
//   step 2 (90 min)        : power-cycle if LORA_PWR_PIN wired, else persistent
//                            re-config (C0)
//   step 3+ (every 45 min) : re-config again; after 3 h total silence save
//                            the queue and restart the ESP32 — at most once
//                            until a frame is heard again (no boot loops).
void performLoRaRecovery() {
    DEBUG_PRINTF("[RECOVERY] LoRa silent >45 min. Attempt %d.\n", loraRecoveryAttempts);
    esp_task_wdt_reset();
    if (loraRecoveryAttempts == 1) {
        configureE220(false);
    } else if (loraRecoveryAttempts == 2) {
#if LORA_PWR_PIN >= 0
        e220PowerCycle();
#else
        configureE220(true);
#endif
    } else {
        configureE220(false);
    }
#if LORA_PWR_PIN >= 0
    if (!loraCfgOk) e220PowerCycle();
#endif
    lastLoraRxTime = millis();   // re-arm the 45-min timer
    DEBUG_PRINTF("[RECOVERY] Done. cfg_ok=%d\n", loraCfgOk ? 1 : 0);
}

void checkLoRaHealth() {
    static unsigned long silenceStart = 0;
    if (lastLoraRxTime == 0) return;
    if ((millis() - lastLoraRxTime) <= LORA_SILENCE_TIMEOUT) {
        if (loraRecoveryAttempts == 0) silenceStart = 0;
        return;
    }
    if (silenceStart == 0) silenceStart = lastLoraRxTime;
    loraRecoveryAttempts++;
    performLoRaRecovery();

    if (millis() - silenceStart > LORA_RESTART_SILENCE_MS && !rtcRestartedForSilence) {
        DEBUG_PRINTLN("[RECOVERY] Silent >3 h — saving queue and restarting gateway.");
        rtcRestartedForSilence = true;
        silenceRestartCount++;
        loraRestartCount++;
        saveQueueToStorage();
        delay(500);
        ESP.restart();
    }
}

// ======================================================================
//  SETUP
// ======================================================================
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    Serial.begin(115200);
    delay(100);
    DEBUG_PRINTLN("\n\n[SYS] Booting LoRa Gateway V11 (E220 verified config, RSSI, deaf recovery)...");

    pinMode(CONFIG_JUMPER_PIN, INPUT_PULLUP);
    pinMode(LORA_DIAG_PIN,     INPUT_PULLUP);
    pinMode(LORA_AUX,          INPUT_PULLUP);
    delay(10);

    esp_task_wdt_deinit();
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms     = WDT_TIMEOUT_SECONDS * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);

    print_reset_reason();
    pinMode(GSM_PWR_KEY, OUTPUT);
    digitalWrite(GSM_PWR_KEY, HIGH);

    // Mount LittleFS — format on first boot if blank
    if (LittleFS.begin(true)) {  // true = format on fail
        DEBUG_PRINTLN("[SYS] LittleFS Mounted.");
        internalMemReady = true;
        migrateLegacyBackup();  // one-time migration from V9 /backup.txt
        recoverLeftoverTemp();  // FIX-V11-6
    }
    // FIX-V11-11: RTC-memory state survives ESP.restart() but not power loss
    if (rtcMagic != 0xB0B0B0B0UL) {
        rtcMagic = 0xB0B0B0B0UL;
        strncpy(lastLoRaTimestamp, "NO_DATA", sizeof(lastLoRaTimestamp));
        rtcRestartedForSilence = false;
        silenceRestartCount = 0;
        loraRestartCount = 0;
    } else {
        DEBUG_PRINTLN("[FATAL] LittleFS Mount Failed! Storage unavailable.");
        internalMemReady = false;
    }

    #ifdef GSM_RST_PIN
    pinMode(GSM_RST_PIN, INPUT);
    #endif

    pinMode(LORA_M0, OUTPUT);
    pinMode(LORA_M1, OUTPUT);
    digitalWrite(LORA_M0, LOW);
    digitalWrite(LORA_M1, LOW);

#if LORA_PWR_PIN >= 0
    pinMode(LORA_PWR_PIN, OUTPUT);
    digitalWrite(LORA_PWR_PIN, LORA_PWR_ACTIVE_LOW ? LOW : HIGH);   // E220 on
    delay(300);
#endif
    reInitLoRaUART();
    waitAuxHigh(1500);
    // FIX-V11-1: write and verify the full register block (persistent)
    if (!configureE220(true)) {
        delay(500); esp_task_wdt_reset();
        configureE220(true);   // one retry; result reported in heartbeat
    }
    delay(200);
    esp_task_wdt_reset();
    while (LoRaSerial.available()) LoRaSerial.read();
    portENTER_CRITICAL(&loraMux);
    loraIsrRead = loraIsrWrite;
    portEXIT_CRITICAL(&loraMux);

    gsm.begin(GSM_BAUD_FAST, SERIAL_8N1, GSM_RX, GSM_TX);
    drainGsmUart(50);
    Wire.begin(I2C_SDA, I2C_SCL);

    delay(1000);
    esp_task_wdt_reset();

    if (checkGSMStateRobust()) {
        gsmFullyPowered = true;
        DEBUG_PRINTLN("[SYS] Modem ALIVE at boot.");
    } else {
        gsmFullyPowered = false;
        DEBUG_PRINTLN("[SYS] Modem appears OFF.");
    }

    rtcReady = rtc.begin();
    if (!rtcReady) {
        DEBUG_PRINTLN("[WARN] RTC not found!");
    } else {
        DateTime now = rtc.now();
        if (now.year() < 2024 || now.year() > 2035) {
            DEBUG_PRINTF("[RTC] Invalid year %d. Writing placeholder.\n", now.year());
            rtc.adjust(DateTime(2024, 1, 1, 0, 0, 0));
        }
        if (rtc.lostPower()) DEBUG_PRINTLN("[SYS] RTC lost power — will sync via GSM.");
    }

    // SD initialization removed entirely

    if (digitalRead(CONFIG_JUMPER_PIN) == LOW) {
        DEBUG_PRINTLN("[SYS] Config Jumper detected! Starting Portal.");
        startConfigPortal();
    }

    loadConfig();
    DEBUG_PRINTF("[SYS] Upload interval: %ld min.\n", connection_interval_min);

    // Print LittleFS storage info at boot
    if (internalMemReady) {
        DEBUG_PRINTF("[LFS] Total: %zu KB  Used: %zu KB  Free: %zu KB\n",
            LittleFS.totalBytes()/1024,
            LittleFS.usedBytes()/1024,
            (LittleFS.totalBytes()-LittleFS.usedBytes())/1024);
    }

    bootTime                  = millis();
    lastGsmActivityTime       = millis();
    lastUartMaintenance       = millis();
    lastRadioRefresh          = millis();
    lastRadioModeRefresh      = millis();
    lastLoraRxTime            = millis();
    lastLongUptimeMaintenance = millis();
    lastSystemRefresh         = millis();

    performScheduledTasks();
    lastScheduledTask = millis();

    DEBUG_PRINTLN("\n[SYS] Setup complete. Listening for LoRa...");
}

// ======================================================================
//  MAIN LOOP
// ======================================================================
void loop() {
    esp_task_wdt_reset();

#if RADIO_CORE_ENABLED == 0
    listenToLoRa();
#endif

    server.handleClient();
    unsigned long now = millis();

    int itemsPending = 0;
    portENTER_CRITICAL(&flushMux);
    itemsPending = (queueWriteIndex - queueReadIndex + QUEUE_SIZE) % QUEUE_SIZE;
    portEXIT_CRITICAL(&flushMux);

    if (itemsPending >= 5 ||
        (now - lastQueueSave > QUEUE_SAVE_INTERVAL_MS && itemsPending > 0)) {
        lastQueueSave = now;
        saveQueueToStorage();
    }

    static unsigned long lastEmergencyUpload = 0;
    if (itemsPending > QUEUE_SIZE - 20 &&
        (now - lastEmergencyUpload > 300000UL)) {
        DEBUG_PRINTLN("[MAINT] Queue near full — forcing upload.");
        esp_task_wdt_reset();
        lastEmergencyUpload = now;
        performScheduledTasks();
    }

    if (!isScheduledUploadTime && gsmFullyPowered &&
        (now - lastGhostCheck > GHOST_CHECK_INTERVAL)) {
        lastGhostCheck = now;
        if (checkGSMStateRobust()) {
            float temp = getGsmTemperature();
            if (temp > 70.0f) { gsmDisconnect(); }   // FIX-V11-9
            else {
                int csq = getGsmStrength();
                if (csq != 99 && csq > 0 && csq < 3) gsmHardReset();
            }
        }
    }

    if (isScheduledUploadTime && gsmFullyPowered && !gsmBusyHTTP &&
        ((uint32_t)(now - lastGsmActivityTime) > GSM_SILENCE_TIMEOUT)) {
        gsmHardReset(); gsmFullyPowered = false; gsmPowered = false;
        updateGsmActivity(now);
    }

    if (now - lastUartMaintenance > 2700000UL) {
        drainGsmUart(300); lastUartMaintenance = now;
    }

    if (now - lastRadioRefresh > 3600000UL && !isScheduledUploadTime && gsmFullyPowered) {
        gsmSendCommand("AT+CFUN=0", 5000); esp_task_wdt_reset();
        listenToLoRa(); delay(2000); esp_task_wdt_reset();
        gsmSendCommand("AT+CFUN=1", 5000);
        gsmPowered = false; gsmBandsConfigured = false;
        lastRadioRefresh = now;
    }

    if (now - lastRadioModeRefresh > 43200000UL) {
        radioRefreshing = true; radioRefreshStart = now;
        digitalWrite(LORA_M0, HIGH); digitalWrite(LORA_M1, HIGH);
        delay(200);
        digitalWrite(LORA_M0, LOW);  digitalWrite(LORA_M1, LOW);
        lastRadioModeRefresh = now;
    }

    if (radioRefreshing && (now - radioRefreshStart > 10000)) {
        radioRefreshing = false;
    }

    if (now - lastLongUptimeMaintenance > LONG_UPTIME_INTERVAL &&
        !isScheduledUploadTime && gsmFullyPowered) {
        gsmDisconnect();
        gsmSendCommand("AT+CFUN=0", 5000); esp_task_wdt_reset();
        listenToLoRa(); delay(3000); esp_task_wdt_reset();
        gsmSendCommand("AT+CFUN=1", 5000);
        gsmPowered = false; gsmBandsConfigured = false;
        lastLongUptimeMaintenance = now;
    }

    if (now - lastSystemRefresh > SYSTEM_REFRESH_INTERVAL) {
        lastSystemRefresh = now;
        saveQueueToStorage(); delay(500);
        ESP.restart();
    }

    if (now - lastScheduledTask >= scheduled_task_interval_ms) {
        while (now - lastScheduledTask >= scheduled_task_interval_ms)
            lastScheduledTask += scheduled_task_interval_ms;
        performScheduledTasks();
        saveQueueToStorage();
    }

    static unsigned long lastIdle = 0;
    if (millis() - lastIdle > 5) { lastIdle = millis(); delay(1); }

    if (lastLoraRxTime > 0 && !gsmBusyHTTP) checkLoRaHealth();

    static unsigned long auxLowStart = 0;
    if (millis() > 30000 && digitalRead(LORA_AUX) == LOW) {
        if (!radioRefreshing) {
            if (auxLowStart == 0) auxLowStart = millis();
            if (millis() - auxLowStart > 45000 &&
                (millis() - lastLoraRxTime > 30000) && !gsmBusyHTTP) {
                reInitLoRaUART(); auxLowStart = 0;
            }
        }
    } else if (digitalRead(LORA_AUX) == HIGH) {
        auxLowStart = 0;
        if (radioRefreshing) radioRefreshing = false;
    }
}

// ======================================================================
//  SCHEDULED TASKS
// ======================================================================
void performScheduledTasks() {
    DEBUG_PRINTLN("\n--- [MAINT] Starting scheduled cycle ---");
    isScheduledUploadTime = true;
    esp_task_wdt_reset();

    // CHANGE-4: verify LittleFS instead of SD remount
    checkLittleFS();

    validateHardwareState();

    if (gsmConnect()) {
        esp_task_wdt_reset();
        currentGsmTemp = getGsmTemperature();
        if (currentGsmTemp > 70.0f) {   // FIX-V11-9
            DEBUG_PRINTF("[MAINT] Modem %.1f C — skipping this upload.\n", currentGsmTemp);
            gsmDisconnect(); isScheduledUploadTime = false; return;
        }
        currentGsmStrength = getGsmStrength();
        FlushResult flushRes = flushAllBuffers();
        DEBUG_PRINTF("[MAINT] Uploaded %d records.\n", flushRes.recordsUploaded);
        // FIX-V11-5: heartbeat every cycle, after the data
        int noise = 0;
        if (readE220Rssi(&noise, NULL)) lastNoiseDbm = noise;
        else DEBUG_PRINTLN("[LORA] Ambient RSSI read failed.");
        sendHeartbeat();
        gsmDisconnect();
    } else {
        DEBUG_PRINTLN("[MAINT] GSM Connection Failed.");
    }

    isScheduledUploadTime = false;
    DEBUG_PRINTLN("--- [MAINT] Cycle finished ---\n");
    esp_task_wdt_reset();
}

// ======================================================================
//  WEB PORTAL — Root (CHANGE-5: SD status removed, LittleFS info shown)
// ======================================================================
void handleRoot() {
    esp_task_wdt_reset();
    if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD))
        return server.requestAuthentication();

    String currentIP = WiFi.localIP().toString();
    size_t lfsTotal  = internalMemReady ? LittleFS.totalBytes() : 0;
    size_t lfsUsed   = internalMemReady ? LittleFS.usedBytes()  : 0;
    size_t lfsFree   = lfsTotal - lfsUsed;

    static char htmlBuf[5120];
    int len = snprintf(htmlBuf, sizeof(htmlBuf),
        "<!DOCTYPE html><html><head><title>LoRa Gateway Config</title>"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "<style>"
        "body{font-family:Arial,sans-serif;background:#f4f4f4;margin:0;padding:20px;"
              "display:flex;flex-direction:column;align-items:center;}"
        ".container{padding:20px;max-width:600px;width:100%%;background:white;"
                   "border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);margin-bottom:20px;}"
        "input[type=text],input[type=password],input[type=number]{"
              "width:100%%;padding:12px;margin:6px 0 16px 0;"
              "box-sizing:border-box;border:1px solid #ccc;border-radius:4px;}"
        "button,.btn{width:100%%;text-decoration:none;display:inline-block;"
              "text-align:center;padding:14px;background-color:#4CAF50;color:white;"
              "border:none;cursor:pointer;border-radius:4px;font-size:16px;margin:5px 0;}"
        "button:hover,.btn:hover{background-color:#45a049;}"
        ".btn-secondary{background-color:#008CBA;}"
        ".btn-danger{background-color:#f44336;}"
        "h2,h3{text-align:center;color:#333;}"
        "hr{border:0;height:1px;background:#ddd;margin:20px 0;}"
        "label{font-weight:bold;display:block;margin-bottom:5px;}"
        "</style></head><body>"
        "<div class=\"container\">"
        "<h2>LoRa Gateway V11</h2>"
        "<h3>System Status</h3>"
        "<p>WiFi: %s</p>"
        "<p>GSM: %s</p>"
        "<p>RTC: %s</p>"
        "<p>LittleFS: %s — Total: %zu KB / Used: %zu KB / Free: %zu KB</p>"
        "<p>Last LoRa RX: <b>%s</b></p>"
        "<p>Next Upload: ~%lu min</p>"
        "<p>LoRa Recovery: %d | ISR OVF: %lu | Queue OVF: %lu | HW OVF: %lu</p>"
        "<hr>"
        "<form action=\"/save\" method=\"POST\">"
        "<h3>Connectivity</h3>"
        "<label>WiFi SSID:<input type=\"text\" name=\"ssid\" value=\"%s\"></label>"
        "<label>WiFi Password:<input type=\"password\" name=\"pass\" "
               "placeholder=\"Leave blank to keep current\"></label>"
        "<label>Google Script URL:<input type=\"text\" name=\"gscript\" value=\"%s\"></label>"
        "<hr>"
        "<label>Upload Interval (minutes):<input type=\"number\" name=\"conn_interval\" "
               "value=\"%ld\" min=\"1\"></label>"
        "<hr><h3>Device Nicknames</h3>",
        (WiFi.status() == WL_CONNECTED ?
            ("Connected (" + currentIP + ")").c_str() : "Disconnected"),
        gsmPowered       ? "Active"  : "Inactive",
        rtcReady         ? "OK"      : "Error",
        internalMemReady ? "OK"      : "ERROR",
        lfsTotal/1024, lfsUsed/1024, lfsFree/1024,
        lastLoRaTimestamp,
        scheduled_task_interval_ms / 60000,
        loraRecoveryAttempts, loraOverflowCount, queueOverflowCount, uartHwOverflowCount,
        wifi_ssid, gscript_url, connection_interval_min
    );

    for (int i = 0; i < 5; i++) {
        if (len < (int)sizeof(htmlBuf)-1) {
            len += snprintf(htmlBuf+len, sizeof(htmlBuf)-len,
                "<label>%s Nickname:<input type=\"text\" name=\"nick%d\" value=\"%s\"></label>",
                knownDevices[i], i, deviceNicknames[i]);
        }
    }

    if (len < (int)sizeof(htmlBuf)-1) {
        snprintf(htmlBuf+len, sizeof(htmlBuf)-len,
            "<br><button type=\"submit\">Save &amp; Restart</button></form></div>"
            "<div class=\"container\"><h3>Log Management (LittleFS)</h3>"
            "<a href=\"/logs\" target=\"_blank\" class=\"btn btn-secondary\">View Logs</a>"
            "<a href=\"/download\" class=\"btn\">Download lfs_buffer.txt</a>"
            "<a href=\"/clear\" class=\"btn btn-danger\" "
               "onclick=\"return confirm('Delete all buffered logs?');\">Clear Log Buffer</a>"
            "</div></body></html>");
    }
    server.send(200, "text/html", htmlBuf);
}

// ======================================================================
//  WEB PORTAL — Save
// ======================================================================
void handleSave() {
    if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD))
        return server.requestAuthentication();
    strncpy(wifi_ssid, server.arg("ssid").c_str(), sizeof(wifi_ssid)-1); wifi_ssid[sizeof(wifi_ssid)-1]=0;
    if (server.arg("pass").length() > 0) {
        strncpy(wifi_pass, server.arg("pass").c_str(), sizeof(wifi_pass)-1); wifi_pass[sizeof(wifi_pass)-1]=0;
    }
    strncpy(gscript_url, server.arg("gscript").c_str(), sizeof(gscript_url)-1); gscript_url[sizeof(gscript_url)-1]=0;
    connection_interval_min = server.arg("conn_interval").toInt();
    if (connection_interval_min < 1) connection_interval_min = 1;
    for (int i = 0; i < 5; i++) {
        String key = "nick"; key += i;
        strncpy(deviceNicknames[i], server.arg(key).c_str(), 31);
        deviceNicknames[i][31] = '\0';
    }
    saveConfig();
    server.send(200, "text/plain", "Saved. Restarting...");
    shouldSaveConfig = true;
    delay(1000); ESP.restart();
}

// ======================================================================
//  WEB PORTAL — Logs / Download / Clear (CHANGE-5: LittleFS paths)
// ======================================================================
void handleLogs() {
    if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD))
        return server.requestAuthentication();
    if (!internalMemReady || !LittleFS.exists(LFS_BUFFER_FILE)) {
        server.send(404, "text/plain", "No log data found.");
        return;
    }
    File f = LittleFS.open(LFS_BUFFER_FILE, "r");
    server.streamFile(f, "text/plain");
    f.close();
}

void handleDownload() {
    if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD))
        return server.requestAuthentication();
    if (!internalMemReady || !LittleFS.exists(LFS_BUFFER_FILE)) {
        server.send(404, "text/plain", "No log data found.");
        return;
    }
    server.sendHeader("Content-Disposition", "attachment; filename=lora_gateway_logs.txt");
    File f = LittleFS.open(LFS_BUFFER_FILE, "r");
    server.streamFile(f, "text/plain");
    f.close();
}

void handleClear() {
    if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD))
        return server.requestAuthentication();
    String msg;
    if (internalMemReady && LittleFS.remove(LFS_BUFFER_FILE))
        msg = "Log buffer cleared. Redirecting...";
    else
        msg = "Clear failed. Redirecting...";
    server.send(200, "text/html",
        "<html><head><meta http-equiv='refresh' content='3;url=/'></head>"
        "<body><h1>" + msg + "</h1></body></html>");
}

// ======================================================================
//  WEB PORTAL — AP startup
// ======================================================================
void startConfigPortal() {
    WiFi.softAP("LoRaGateway-Setup", "12345678");
    server.on("/",         HTTP_GET,  handleRoot);
    server.on("/save",     HTTP_POST, handleSave);
    server.on("/logs",     HTTP_GET,  handleLogs);
    server.on("/download", HTTP_GET,  handleDownload);
    server.on("/clear",    HTTP_GET,  handleClear);
    server.begin();
    DEBUG_PRINTLN("[Config] Portal running at " + WiFi.softAPIP().toString());
    while (!shouldSaveConfig) {
        esp_task_wdt_reset();
        server.handleClient();
        delay(10);
    }
    delay(1000);
    WiFi.softAPdisconnect(true);
    ESP.restart();
}
