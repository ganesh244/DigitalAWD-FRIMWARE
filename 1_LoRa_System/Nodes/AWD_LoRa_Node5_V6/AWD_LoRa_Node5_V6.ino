/**
 * @file AWD_LoRa_Node5_V6.ino
 * @brief LoRa Slave Transmitter — T22D(Lora5)
 * @version V6
 *
 * ===================== FIX LIST V6 =====================
 * FIX-V6-1: E220 configured on EVERY wake with a volatile C2 register
 *           write while the module is in mode 3 (sleep/config):
 *           9600 8N1, 2.4 kbps, DEVICE_CHANNEL, 22 dBm, transparent,
 *           RSSI byte OFF. No more reliance on factory defaults, and the
 *           channel moves off 873.125 MHz (LTE band-5 downlink) to
 *           865.125 MHz. MUST MATCH LORA_CHANNEL IN THE GATEWAY.
 * FIX-V6-2: After the payload is handed to the E220 the node now waits
 *           for AUX to go LOW (transmission started) and HIGH again
 *           (transmission finished) before sleeping. V5.1 slept 100 ms
 *           after the UART flush, while a 2.4 kbps packet is still on
 *           the air for several hundred ms.
 * FIX-V6-3: E220 put into mode 3 (M0=M1=HIGH, ~3 µA) and the pin states
 *           held through deep sleep (gpio_hold). V5.1 left the module in
 *           receive mode (~12 mA) all night. TRIG is also held LOW so the
 *           ultrasonic sensor cannot free-run.
 * FIX-V6-4: A failed sensor read is reported as distance -1 / level -1.
 *           V5.1 reported "pipe height / 0 cm" which the sheet and the
 *           dashboard showed as a genuine dry pipe ("Low"). The gateway
 *           V11 maps -1 to status "SensorError".
 * FIX-V6-5: Payload unchanged otherwise: T22D(LoraN)|distance|level
 * ========================================================
 *
 * ===================== FIX LIST V5.1 =====================
 * FIX-V5.1-1: E220 readiness now detected via the AUX pin instead of
 *             a fixed delay. CORRECTED from an earlier draft of this
 *             fix that used an unverified flat 1000ms delay -- that
 *             number was never checked against the datasheet and was
 *             wrong. Per the official E220 datasheet, AUX is driven
 *             LOW during power-on self-check / mode change and HIGH
 *             once the module is actually ready; the real settle
 *             time is far shorter than 1000ms (datasheet: ~1ms when
 *             idle; independent oscilloscope testing on this same
 *             module family: ~2-40ms). The previous 200ms total delay
 *             was occasionally too short, causing the first
 *             transmission after a reset to fire mid-boot and be
 *             silently dropped. The fix now polls the AUX pin for its
 *             rising edge (100ms safety timeout if AUX is unconnected
 *             or faulty) so the wait is always exactly as long as
 *             needed -- correct on every boot, and faster on battery
 *             than a long fixed guess would be.
 *             REQUIRES: E220 AUX pin wired to ESP32 GPIO18 on this
 *             node (same as the gateway's AUX wiring).
 *
 * FIX-V5.1-2: First-boot stagger state moved from RTC_DATA_ATTR to
 *             RTC_NOINIT_ATTR + magic-number check. RTC_DATA_ATTR is
 *             re-initialized on every EN/reset-button press, which
 *             caused the device-specific stagger sleep (0/4/8/12/16
 *             min) to re-trigger on EVERY physical reset -- not just
 *             true power-on. Only Lora1 (0s stagger) appeared to
 *             respond "immediately"; all other nodes silently sat in
 *             stagger sleep for minutes before transmitting. Now the
 *             stagger only ever runs once per real power cycle, and
 *             every node transmits within seconds of a reset button
 *             press, matching Lora1's behaviour.
 * ==========================================================
 *
 * ===================== FIX LIST V5 =====================
 * FIX-1: WDT safe — esp_task_wdt_deinit() called before init.
 *         WDT set to 120s (covers sensor + LoRa + any delays).
 *
 * FIX-2: First-boot stagger via DEEP SLEEP (not busy-wait).
 *         RTC_DATA_ATTR tracks if stagger has been done.
 *         Device sleeps during stagger = zero battery waste.
 *         Stagger: 960s offset from power-on.
 *
 * FIX-3: Per-cycle jitter removed from active time.
 *         Random drift (0-30s) added to SLEEP duration only.
 *         Device never wastes active power on jitter again.
 *
 * FIX-4: Config portal 10-minute auto-timeout.
 *         If user forgets to configure, device reboots to
 *         normal mode instead of draining battery forever.
 *
 * FIX-5: Brownout detector disabled for solar/battery supply.
 *
 * FIX-6: Sleep default changed to 1200s (20 min).
 *         Random 0-300s added per cycle = 20-25 min range.
 *         Prevents permanent re-synchronization after first drift.
 *
 * FIX-7: sleepSeconds changed to uint32_t (safe for 1200+).
 *
 * FIX-8: Sensor returns average of 3 valid readings for accuracy.
 *
 * STAGGER PLAN (prevents collision on simultaneous deployment):
 *   Lora1 → 0 min offset  (transmits first)
 *   Lora2 → 4 min offset
 *   Lora3 → 8 min offset
 *   Lora4 → 12 min offset
 *   Lora5 → 16 min offset
 *   After first cycle, random sleep drift keeps them spread permanently.
 * ==========================================================
 */

#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "esp_sleep.h"
#include <esp_task_wdt.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/gpio.h"

// ================== DEVICE IDENTITY ==================
// DO NOT CHANGE — uniquely identifies this node to the gateway
#define DEVICE_ID       "T22D(Lora5)"
#define DEVICE_NUMBER   5   // 1-5
// FIX-V6-1: 0..83 → 850.125 + CH MHz. 15 = 865.125 MHz. Same on all nodes + gateway.
#define DEVICE_CHANNEL  15

// ================== HARDWARE PINS ==================
#define CONFIG_JUMPER_PIN  15  // Pull LOW on boot to enter config mode
#define LORA_RX_PIN        16
#define LORA_TX_PIN        17
#define LORA_M0            21
#define LORA_M1            19
#define LORA_AUX           18  // FIX-V5.1-1 (corrected): used to detect real E220 ready state
#define SENSOR_TRIG_PIN    13
#define SENSOR_ECHO_PIN    12

// ================== TIMING ==================
// FIX-2: First-boot deep-sleep stagger (seconds).
// Offsets transmissions across all 5 devices on first deployment.
#define DEVICE_STAGGER_SEC  960UL

// FIX-6: Base sleep + random jitter = 20-25 min total per cycle.
#define SLEEP_BASE_SEC      1200UL   // 20 min base
#define SLEEP_JITTER_SEC    300UL               // 0-5 min random = 20-25 min total

// ================== HARDWARE ==================
HardwareSerial LoRaSerial(2);
WebServer server(80);

// ================== CONFIG ==================
const char* CONFIG_USERNAME = "admin";
const char* CONFIG_PASSWORD = "password";
// Config portal SSID is unique per device so you can configure them independently
const char* AP_SSID = "LoRaTX-Lora5-Setup";

// ================== PERSISTENT STATE (survives deep sleep AND reset button) ==================
// FIX-V5.1-2: RTC_NOINIT_ATTR + magic number — unlike RTC_DATA_ATTR, this
// survives EN/reset-button presses, not just deep-sleep wake. Without this,
// every physical reset re-ran the first-boot stagger sleep (0/4/8/12/16 min
// depending on device), making it look like only Lora1 (0s stagger)
// responded immediately on reset.
#define BOOT_MAGIC_VALUE 0xA5A5A5A5UL

RTC_NOINIT_ATTR static uint32_t bootMagic;
RTC_NOINIT_ATTR static bool     firstBootDone;
RTC_NOINIT_ATTR static bool     staggerSleeping;
RTC_NOINIT_ATTR static uint32_t cycleCount;

// ================== CONFIG VARIABLES ==================
// FIX-7: uint32_t for safe handling of 1200+ second values
uint32_t sleepBaseSec = SLEEP_BASE_SEC;
float    pipeHeightCm = 60.0f;

Preferences preferences;
float lastRawDistanceCm = 0.0f;

enum OperatingMode { NORMAL_MODE, CONFIG_MODE };
OperatingMode currentMode;

// ================== RESET REASON ==================
void print_reset_reason() {
    esp_reset_reason_t r = esp_reset_reason();
    Serial.print("[SYS] Reset: ");
    switch (r) {
        case ESP_RST_POWERON:   Serial.println("Power on");         break;
        case ESP_RST_DEEPSLEEP: Serial.println("Deep sleep wake");  break;
        case ESP_RST_SW:        Serial.println("Software reset");   break;
        case ESP_RST_PANIC:     Serial.println("Panic/exception");  break;
        case ESP_RST_BROWNOUT:  Serial.println("Brownout!");        break;
        case ESP_RST_TASK_WDT:  Serial.println("WDT (loop frozen)");break;
        default:                Serial.println("Other");            break;
    }
}

// ================== CONFIG LOAD/SAVE ==================
void loadConfig() {
    preferences.begin("lora-tx", true);
    sleepBaseSec = preferences.getUInt("sleep", SLEEP_BASE_SEC);
    pipeHeightCm = preferences.getFloat("height", 60.0f);
    preferences.end();
    // Safety clamps
    if (sleepBaseSec < 300)   sleepBaseSec = 300;    // min 5 min
    if (sleepBaseSec > 3600)  sleepBaseSec = 3600;   // max 60 min
    if (pipeHeightCm <= 0)    pipeHeightCm = 60.0f;
    if (pipeHeightCm > 500)   pipeHeightCm = 500.0f;
    Serial.printf("[Config] sleep=%lus  pipe=%.1fcm\n", sleepBaseSec, pipeHeightCm);
}

void saveConfig() {
    preferences.begin("lora-tx", false);
    preferences.putUInt("sleep", sleepBaseSec);
    preferences.putFloat("height", pipeHeightCm);
    preferences.end();
    Serial.println("[Config] Saved to flash.");
}

// ================== FIX-8: SENSOR (average of valid readings) ==================
float getWaterLevel() {
    pinMode(SENSOR_TRIG_PIN, OUTPUT);
    pinMode(SENSOR_ECHO_PIN, INPUT_PULLDOWN);  // PULLDOWN: GPIO12 stays LOW at boot (strapping pin safety)
    delay(50);  // Sensor warm-up

    float sumCm = 0;
    int   validCount = 0;

    for (int attempt = 0; attempt < 5; attempt++) {
        esp_task_wdt_reset();

        digitalWrite(SENSOR_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(SENSOR_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(SENSOR_TRIG_PIN, LOW);

        long dur = pulseIn(SENSOR_ECHO_PIN, HIGH, 25000); // 25ms = ~4.3m max range
        if (dur > 0) {
            float d = dur * 0.0343f / 2.0f;
            // Sanity check: reading must be within pipe bounds
            if (d > 0 && d <= pipeHeightCm + 5.0f) {
                sumCm += d;
                validCount++;
            }
        }
        delay(30); // inter-ping delay (avoid echo overlap)
    }

    float distanceCm;
    if (validCount > 0) {
        distanceCm = sumCm / validCount;  // Use average of valid readings
    } else {
        // FIX-V6-4: report the failure instead of pretending the pipe is empty
        lastRawDistanceCm = -1.0f;
        Serial.println("[SENSOR] All readings failed. Reporting SensorError (-1).");
        return -1.0f;
    }

    lastRawDistanceCm = distanceCm;
    float waterLevel  = pipeHeightCm - distanceCm;
    return constrain(waterLevel, 0.0f, pipeHeightCm);
}

// ================== FIX-V6-1: E220 volatile configuration ==================
// Register block (E220 / LLCC68):
//   ADDH ADDL REG0 REG1 REG2 REG3
//   REG0 0x62 = 9600 baud, 8N1, 2.4 kbps air
//   REG1 0x00 = 200 B sub-packet, ambient RSSI off, 22 dBm
//   REG2 = channel
//   REG3 0x03 = RSSI byte off, transparent, LBT off, WOR 2000 ms
// C2 = write without saving to the module's non-volatile memory (safe to
// repeat every wake). Response echoes C1 00 06 + 6 bytes. Wrong format → FF FF FF.
static bool waitAux(int level, uint32_t timeoutMs) {
    unsigned long t0 = millis();
    while (digitalRead(LORA_AUX) != level) {
        if (millis() - t0 > timeoutMs) return false;
        delay(1);
        esp_task_wdt_reset();
    }
    return true;
}

bool configureE220Volatile() {
    // Module must be in mode 3 (M0=M1=HIGH) — set by caller.
    while (LoRaSerial.available()) LoRaSerial.read();
    uint8_t cmd[9] = {0xC2, 0x00, 0x06, 0x00, 0x00, 0x62, 0x00, (uint8_t)DEVICE_CHANNEL, 0x03};
    LoRaSerial.write(cmd, 9); LoRaSerial.flush();
    uint8_t resp[9] = {0}; int n = 0; unsigned long t0 = millis();
    while (millis() - t0 < 300 && n < 9) {
        if (LoRaSerial.available()) resp[n++] = (uint8_t)LoRaSerial.read();
        else delay(1);
    }
    bool ok = (n == 9 && resp[0] == 0xC1 && resp[7] == (uint8_t)DEVICE_CHANNEL);
    Serial.printf("[LORA] C2 config %s (%d bytes:", ok ? "OK" : "FAILED", n);
    for (int i = 0; i < n; i++) Serial.printf(" %02X", resp[i]);
    Serial.println(")");
    return ok;
}

// ================== WEB PORTAL ==================
void handleRoot() {
    if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD))
        return server.requestAuthentication();

    char html[3072]; // TX-2 FIX: increased from 2048 — template alone is ~1500 chars
    snprintf(html, sizeof(html),
        "<!DOCTYPE html><html><head><title>%s Config</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        "body{font-family:sans-serif;background:#f4f4f4;margin:0;padding:20px;}"
        ".box{background:#fff;border-radius:8px;box-shadow:0 2px 5px rgba(0,0,0,.1);"
              "padding:20px;max-width:500px;margin:10px auto;}"
        "input{width:100%%;padding:10px;margin:8px 0;border:1px solid #ccc;"
              "border-radius:4px;box-sizing:border-box;}"
        "button{background:#4CAF50;color:#fff;padding:13px;width:100%%;border:none;"
               "border-radius:4px;cursor:pointer;font-size:16px;}"
        "label{font-weight:bold;display:block;margin-top:12px;color:#555;}"
        "h2,p{text-align:center;} .live{font-size:2.5em;font-weight:bold;text-align:center;}"
        "</style></head><body>"
        "<div class='box'><h2>%s</h2>"
        "<p>Cycle #%lu | Raw: <b id='r'>--</b> cm | Level: <b id='l'>--</b> cm</p></div>"
        "<div class='box'>"
        "<form action='/save' method='POST'>"
        "<label>Sleep Interval Base (seconds, 300-3600):</label>"
        "<input type='number' name='sleep' min='300' max='3600' value='%lu'>"
        "<label>Pipe Height (cm):</label>"
        "<input type='number' step='0.1' name='height' value='%.1f'>"
        "<br><br><button type='submit'>Save &amp; Restart</button>"
        "</form></div>"
        "<script>"
        "setInterval(()=>fetch('/data').then(r=>r.text()).then(d=>{"
        "const p=d.split(',');if(p.length==2){"
        "document.getElementById('r').innerText=parseFloat(p[0]).toFixed(2);"
        "document.getElementById('l').innerText=parseFloat(p[1]).toFixed(2);"
        "}}),1500);"
        "</script></body></html>",
        DEVICE_ID, DEVICE_ID, cycleCount, sleepBaseSec, pipeHeightCm);

    server.send(200, "text/html", html);
}

void handleSave() {
    if (!server.authenticate(CONFIG_USERNAME, CONFIG_PASSWORD)) return;
    if (server.hasArg("sleep")) {
        sleepBaseSec = (uint32_t)server.arg("sleep").toInt();
        if (sleepBaseSec < 300)  sleepBaseSec = 300;
        if (sleepBaseSec > 3600) sleepBaseSec = 3600;
    }
    if (server.hasArg("height")) {
        pipeHeightCm = server.arg("height").toFloat();
        if (pipeHeightCm <= 0 || pipeHeightCm > 500) pipeHeightCm = 60.0f;
    }
    saveConfig();
    server.send(200, "text/html",
        "<html><body style='font-family:sans-serif;text-align:center;padding:40px;'>"
        "<h2>Saved! Restarting in normal mode...</h2>"
        "<p>You can now disconnect from the WiFi AP.</p>"
        "</body></html>");
    delay(1500);
    ESP.restart();
}

void handleLiveData() {
    float level = getWaterLevel();
    server.send(200, "text/plain",
        String(lastRawDistanceCm, 2) + "," + String(level, 2));
}

// ================== NORMAL TRANSMISSION ==================
void runNormalMode() {
    loadConfig();

    // FIX-V5.1-2: On a TRUE power-on (battery first connected, or
    // reconnected after full power loss), bootMagic will contain garbage
    // (RTC_NOINIT_ATTR is not zero-initialized). Detect that and reset all
    // first-boot state exactly once. On any later EN/reset button press,
    // bootMagic still holds BOOT_MAGIC_VALUE, so this block is skipped and
    // the device goes straight to normal transmission.
    if (bootMagic != BOOT_MAGIC_VALUE) {
        bootMagic       = BOOT_MAGIC_VALUE;
        firstBootDone   = false;
        staggerSleeping = false;
        cycleCount      = 0;
        Serial.println("[SYS] True power-on detected — first-boot state reset.");
    }

    // ----------------------------------------------------------------
    // FIX-2: FIRST-BOOT STAGGER — deep sleep only, zero power waste
    //
    // On very first power-on (or after factory reset of RTC memory),
    // this device sleeps for DEVICE_STAGGER_SEC before its first
    // transmission. This staggers all 5 devices so they never collide
    // on the first cycle even if powered on simultaneously.
    //
    // This sleep happens ONLY ONCE in the device lifetime.
    // After that, per-cycle random jitter keeps them staggered.
    // ----------------------------------------------------------------
    // FIX-E: Two-phase stagger using separate flags
    // staggerSleeping=true means "we are in the middle of stagger sleep"
    // firstBootDone=true means "stagger is fully complete, normal operation"
    // This correctly handles power-cut during stagger sleep.
    if (!firstBootDone && !staggerSleeping) {
        // Very first power-on — initiate stagger sleep if needed
        if (DEVICE_STAGGER_SEC > 0) {
            staggerSleeping = true;  // Mark BEFORE sleep
            Serial.printf("[TX] First boot! Stagger sleep: %lus\n", DEVICE_STAGGER_SEC);
            Serial.flush();
            esp_sleep_enable_timer_wakeup((uint64_t)DEVICE_STAGGER_SEC * 1000000ULL);
            esp_deep_sleep_start();
        }
        // Device 1 (stagger=0): skip stagger, mark done immediately
        firstBootDone  = true;
        staggerSleeping = false;
    }
    if (staggerSleeping && !firstBootDone) {
        // Woke up FROM the stagger sleep — stagger is now complete
        firstBootDone  = true;
        staggerSleeping = false;
        Serial.println("[TX] Stagger complete. Starting normal operation.");
    }

    cycleCount++;
    Serial.printf("\n[TX] Wake cycle #%lu — %s\n", cycleCount, DEVICE_ID);

    // FIX-V6-3: release the pad holds from the previous deep sleep
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis((gpio_num_t)LORA_M0);
    gpio_hold_dis((gpio_num_t)LORA_M1);
    gpio_hold_dis((gpio_num_t)SENSOR_TRIG_PIN);

    // FIX-V6-1: start in mode 3 (sleep/config), write the volatile config,
    // then drop to mode 0 (normal). Mode 3 is also where the module was
    // parked during deep sleep, so this costs no extra transition.
    pinMode(LORA_M0, OUTPUT);
    pinMode(LORA_M1, OUTPUT);
    digitalWrite(LORA_M0, HIGH);
    digitalWrite(LORA_M1, HIGH);
    pinMode(LORA_AUX, INPUT_PULLDOWN);
    LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    delay(20);
    waitAux(HIGH, 500);
    delay(5);
    bool cfgOk = configureE220Volatile();
    if (!cfgOk) { delay(50); cfgOk = configureE220Volatile(); }
    LoRaSerial.end();
    delay(5);

    // Normal mode (M0=0, M1=0) — mode change is accepted only when AUX is HIGH
    digitalWrite(LORA_M0, LOW);
    digitalWrite(LORA_M1, LOW);

    // FIX-V5.1-1 (CORRECTED): per the official E220 datasheet, the
    // module drives AUX LOW during power-on self-check / mode change,
    // then HIGH once it is actually ready. The previous version of this
    // fix used a blind 1000ms delay, which was an UNVERIFIED guess and
    // has since been checked against the datasheet -- the real
    // requirement is far shorter (datasheet: ~1ms when idle; community
    // hardware testing on this exact module: ~2-40ms via oscilloscope).
    // Polling AUX is correct because it reacts to the module's ACTUAL
    // ready state instead of a fixed guess, and it doesn't waste solar
    // battery life waiting longer than necessary on every wake cycle.
    // FIX-V5.1-3: INPUT_PULLDOWN (not bare INPUT) prevents a floating-pin
    // race on the very first reset after flashing. Without a defined pull,
    // GPIO18 can read a stray HIGH before the E220 has actually started
    // driving AUX, making the code below skip the wait entirely and
    // transmit before the module is ready -- this was the root cause of
    // "first reset after flash = no data, second reset = works".
    pinMode(LORA_AUX, INPUT_PULLDOWN);

    unsigned long auxWaitStart = millis();
    const unsigned long AUX_TIMEOUT_MS = 100; // generous safety ceiling
    while (digitalRead(LORA_AUX) == LOW) {
        if (millis() - auxWaitStart > AUX_TIMEOUT_MS) {
            Serial.println("[LORA] WARNING: AUX did not go HIGH within timeout — proceeding anyway.");
            break;
        }
        delay(1);
        esp_task_wdt_reset();
    }
    delay(2);  // small margin per datasheet guidance after AUX rising edge

    LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    delay(20);  // brief UART stabilisation after begin()

    // Drain any E220 boot bytes before transmitting
    // E220 outputs ~100-200 bytes on power-on that must be discarded
    while (LoRaSerial.available()) LoRaSerial.read();

    // FIX-8: Take averaged sensor reading
    float waterLevel = getWaterLevel();
    Serial.printf("[TX] Distance: %.2fcm | Water: %.2fcm\n",
                  lastRawDistanceCm, waterLevel);

    // Build payload: T22D(LoraX)|raw_sensor_distance|water_level
    char payload[96];
    snprintf(payload, sizeof(payload), "%s|%.2f|%.2f",
             DEVICE_ID,
             waterLevel < 0 ? -1.0f : pipeHeightCm - waterLevel,  // FIX-V6-4
             waterLevel);

    Serial.printf("[TX] Payload: %s\n", payload);

    // Transmit (LoRaSerial.println adds \r\n which gateway parser handles)
    LoRaSerial.println(payload);
    LoRaSerial.flush();

    // FIX-V6-2: wait for the air transmission to finish. AUX drops LOW
    // while the E220 buffers + transmits, then returns HIGH. At 2.4 kbps a
    // ~30-byte packet is on the air for ~400 ms.
    if (waitAux(LOW, 300)) {
        if (!waitAux(HIGH, 3000)) Serial.println("[LORA] WARNING: AUX stayed LOW after TX.");
    } else {
        Serial.println("[LORA] AUX never dropped — TX may not have started (AUX wired?).");
        delay(600);   // fall back to a fixed wait long enough for the packet
    }
    delay(20);

    // ----------------------------------------------------------------
    // FIX-6: RANDOM SLEEP (20-25 min range)
    //
    // Base sleep + random 0-SLEEP_JITTER_SEC seconds.
    // Uses hardware entropy (esp_random) — truly random each cycle.
    // This permanently drifts synchronized devices apart after
    // only a few cycles, eliminating long-term collision risk.
    // ----------------------------------------------------------------
    uint32_t jitterSec = (uint32_t)(esp_random() % (SLEEP_JITTER_SEC + 1));
    uint32_t totalSec  = sleepBaseSec + jitterSec;
    Serial.printf("[TX] Sleeping %lus (base=%lu + jitter=%lu)\n",
                  totalSec, sleepBaseSec, jitterSec);
    Serial.flush();

    // FIX-F: End LoRaSerial before sleep — prevents ~5mA UART leakage during sleep
    LoRaSerial.end();

    // FIX-V6-3: park the E220 in mode 3 (deep sleep, ~3 µA) and hold the
    // pins through ESP32 deep sleep. Also hold TRIG LOW so the HC-SR04
    // cannot be triggered by a floating pin.
    digitalWrite(LORA_M0, HIGH);
    digitalWrite(LORA_M1, HIGH);
    pinMode(SENSOR_TRIG_PIN, OUTPUT);
    digitalWrite(SENSOR_TRIG_PIN, LOW);
    delay(5);
    gpio_hold_en((gpio_num_t)LORA_M0);
    gpio_hold_en((gpio_num_t)LORA_M1);
    gpio_hold_en((gpio_num_t)SENSOR_TRIG_PIN);
    gpio_deep_sleep_hold_en();

    // FIX-G: Flush and end Serial before sleep
    Serial.flush();
    Serial.end();

    esp_sleep_enable_timer_wakeup((uint64_t)totalSec * 1000000ULL);
    esp_deep_sleep_start();
}

// ================== SETUP ==================
void setup() {
    // FIX-5: Disable brownout detector for solar/battery immunity
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.begin(115200);
    delay(200);
    Serial.printf("\n\n[SYS] %s V6 Booting (ch %d)...\n", DEVICE_ID, DEVICE_CHANNEL);
    print_reset_reason();

    // FIX-1: Deinit WDT before init (prevents "already initialized" error on wake)
    esp_task_wdt_deinit();
    esp_task_wdt_config_t wdt = {
        .timeout_ms     = 120000,  // 120s — covers sensor + LoRa + all delays safely
        .idle_core_mask = 0,
        .trigger_panic  = true
    };
    esp_task_wdt_init(&wdt);
    esp_task_wdt_add(NULL);

    pinMode(CONFIG_JUMPER_PIN, INPUT_PULLUP);
    delay(100);

    if (digitalRead(CONFIG_JUMPER_PIN) == LOW) {
        currentMode = CONFIG_MODE;
        Serial.println("[SYS] Config mode — jumper detected.");
        loadConfig();

        WiFi.softAP(AP_SSID, "12345678");
        Serial.println("[Config] AP: " + WiFi.softAPIP().toString());
        Serial.printf("[Config] SSID: %s\n", AP_SSID);

        server.on("/",     HTTP_GET,  handleRoot);
        server.on("/save", HTTP_POST, handleSave);
        server.on("/data", HTTP_GET,  handleLiveData);
        server.begin();
        Serial.println("[Config] Portal active at 192.168.4.1");
    } else {
        currentMode = NORMAL_MODE;
    }
}

// ================== LOOP ==================
void loop() {
    if (currentMode == CONFIG_MODE) {
        // FIX-4: Config portal auto-timeout (10 minutes)
        // Prevents battery drain if user forgets to close config mode
        static unsigned long configStart = 0;
        if (configStart == 0) configStart = millis();

        if (millis() - configStart > 600000UL) {  // 10 minutes
            Serial.println("[Config] Timeout! Rebooting to normal mode.");
            delay(500);
            ESP.restart();
        }

        esp_task_wdt_reset();
        server.handleClient();
        delay(1);
    } else {
        // Normal mode: runs once, then deep sleeps
        runNormalMode();
    }
}
