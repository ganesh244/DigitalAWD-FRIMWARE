/**
 * @file Code.gs
 * @brief LoRa Gateway & Settings Manager (V11 Firmware Compatible)
 * @version 7.0
 *
 * CHANGES FROM V6.1 -> V7.0 (gateway V11):
 * - Data sheet: new column 13 "RSSI (dBm)" from reading.rssi. Existing
 *   sheets get the header appended on the right (no data shifted).
 * - Data sheet: status "SensorError" (node sensor failed, waterLevel null)
 *   gets its own colour; Water Level cell is left blank, not 0.
 * - Heartbeat sheet: new columns 19-27 — Noise (dBm), Lora1-5 RSSI (dBm),
 *   LoRa Cfg OK, Reset Reason, Uptime (min), Free Heap, Channel, Firmware.
 *   Same append-on-the-right migration as before.
 * - doGet: ?heartbeat=1 returns the Heartbeat tab as JSON (newest first,
 *   supports ?days= and ?limit=) so a dashboard can show gateway health.
 *
 * CHANGES FROM V6.0 -> V6.1:
 * - Gateway V10 removed the SD card entirely and switched to LittleFS
 *   for buffering. The firmware now sends "lfsFreeKB" instead of
 *   "sdFreeMB" in both the data-upload payload and the heartbeat.
 * - Data sheet: "SD Free (MB)" column renamed to "LittleFS Free (KB)",
 *   now populated from payload.lfsFreeKB (1 decimal place).
 * - Heartbeat sheet: new column 18 "LittleFS Free (KB)" added via the
 *   same auto-migration mechanism used for the V9 columns — existing
 *   sheets get the column appended on the right without disrupting
 *   any existing data.
 *
 * CHANGES FROM V5.0 -> V6.0:
 * - Heartbeat sheet: Added 9 new columns (cols 8-16) for diagnostic fields
 *   introduced in gateway V9: lora_overflows, queue_overflows, hw_overflows,
 *   lora_recovery, and dev_silence_min[5] (per-device silence in minutes).
 * - Schema migration: getAndPrepareHeartbeatSheet() auto-adds missing columns
 *   to existing sheets without disrupting existing data alignment.
 * - Data sheet and Settings sheet: UNCHANGED — no column reordering.
 * - All existing rows in the Heartbeat sheet remain aligned (new cols = blank
 *   for old rows, populated from V9 firmware onward).
 */

// --- CONFIGURATION ---
const BATCHED_DATA_SHEET_NAME = 'AWD_Gateway_Data';
const SETTINGS_SHEET_NAME     = 'AppSettings';
const HEARTBEAT_SHEET_NAME    = 'Heartbeat';

// Canonical heartbeat column definitions — ORDER MUST NEVER CHANGE
// To add future columns, append to this array only.
const HEARTBEAT_HEADERS = [
  "Upload TS",            // col 1  — payload: upload_ts
  "Gateway Status",       // col 2  — payload: gateway
  "GSM Module",           // col 3  — payload: gsm
  "Operator",             // col 4  — payload: simOperator
  "Signal (CSQ)",         // col 5  — payload: gsmStrength
  "Modem Temp (°C)",      // col 6  — payload: temp
  "Last LoRa RX",         // col 7  — payload: last_lora_rx
  // --- V9 additions (cols 8-16) ---
  "LoRa ISR Overflows",   // col 8  — payload: lora_overflows
  "Queue Overflows",      // col 9  — payload: queue_overflows
  "HW FIFO Overflows",    // col 10 — payload: hw_overflows
  "LoRa Recoveries",      // col 11 — payload: lora_recovery
  "LoRa Restarts",        // col 12 — payload: lora_restarts (survives software resets)
  "Lora1 Silence (min)",  // col 13 — payload: dev_silence_min[0]
  "Lora2 Silence (min)",  // col 14 — payload: dev_silence_min[1]
  "Lora3 Silence (min)",  // col 15 — payload: dev_silence_min[2]
  "Lora4 Silence (min)",  // col 16 — payload: dev_silence_min[3]
  "Lora5 Silence (min)",  // col 17 — payload: dev_silence_min[4]
  // --- V10.1 addition (col 18) ---
  "LittleFS Free (KB)",   // col 18 — payload: lfsFreeKB
  // --- V11 additions (cols 19-27) ---
  "Noise (dBm)",          // col 19 — payload: noise (ambient RSSI at heartbeat)
  "Lora1 RSSI (dBm)",     // col 20 — payload: dev_rssi[0] (last packet)
  "Lora2 RSSI (dBm)",     // col 21
  "Lora3 RSSI (dBm)",     // col 22
  "Lora4 RSSI (dBm)",     // col 23
  "Lora5 RSSI (dBm)",     // col 24
  "LoRa Cfg OK",          // col 25 — payload: lora_cfg_ok (1 = E220 answered C1 read-back)
  "Reset Reason",         // col 26 — payload: reset
  "Uptime (min)",         // col 27 — payload: uptime_min
  "Free Heap",            // col 28 — payload: heap
  "Channel",              // col 29 — payload: channel
  "Firmware"              // col 30 — payload: fw
];

// Data sheet headers — ORDER MUST NEVER CHANGE, append only.
const DATA_HEADERS = [
  "Gateway Received Time", "Device ID", "Transmitter Data", "Water Level (cm)",
  "Status", "Network", "Batch Upload Time", "SIM Operator",
  "WiFi Strength (dBm)", "GSM Strength (CSQ)", "LittleFS Free (KB)", "Source",
  // --- V7 addition ---
  "RSSI (dBm)"            // col 13 — payload: reading.rssi (0 = unknown)
];

// ==========================================
// ENTRY POINTS
// ==========================================

/**
 * Routes GET requests: getSettings action or data retrieval.
 */
function doGet(e) {
  try {
    if (e.parameter && e.parameter.action === 'getSettings') {
      return handleGetSettings(e);
    }
    if (e.parameter && (e.parameter.heartbeat === '1' || e.parameter.action === 'heartbeat')) {
      return handleGetHeartbeat(e);
    }
    return handleGetData(e);
  } catch (err) {
    return createJSONOutput({ status: "error", message: err.message });
  }
}

/**
 * Routes POST requests: settings save, heartbeat, or batched telemetry.
 */
function doPost(e) {
  try {
    const payload = JSON.parse(e.postData.contents);

    if (payload.action === 'saveSetting') {
      return handleSaveSetting(payload);
    }
    if (payload.type === 'heartbeat') {
      return handleHeartbeat(payload);
    }
    if (payload.readings && Array.isArray(payload.readings)) {
      return handleLogData(payload);
    }

    throw new Error("Invalid payload format received.");
  } catch (err) {
    return createJSONOutput({ result: "error", message: err.message });
  }
}

// ==========================================
// SECTION 1: SETTINGS MANAGEMENT (UNCHANGED)
// ==========================================

function getAndPrepareSettingsSheet(ss) {
  let sheet = ss.getSheetByName(SETTINGS_SHEET_NAME);
  if (!sheet) {
    sheet = ss.insertSheet(SETTINGS_SHEET_NAME);
    sheet.appendRow(["DeviceID", "Key", "Value", "Timestamp"]);
    sheet.setFrozenRows(1);
    sheet.getRange(1, 1, 1, 4).setFontWeight("bold");
  }
  return sheet;
}

function handleGetSettings(e) {
  const ss = SpreadsheetApp.getActiveSpreadsheet();
  const sheet = getAndPrepareSettingsSheet(ss);
  const data = sheet.getDataRange().getValues();
  const settings = {};
  for (let i = 1; i < data.length; i++) {
    const deviceId = data[i][0];
    const key      = data[i][1];
    let   value    = data[i][2];
    try { value = JSON.parse(value); } catch(e) {}
    if (!settings[deviceId]) settings[deviceId] = {};
    settings[deviceId][key] = value;
  }
  return createJSONOutput(settings);
}

function handleSaveSetting(payload) {
  const ss    = SpreadsheetApp.getActiveSpreadsheet();
  const sheet = getAndPrepareSettingsSheet(ss);
  const { deviceId, key, value } = payload;
  if (!deviceId || !key) throw new Error("Missing deviceId or key");

  const data       = sheet.getDataRange().getValues();
  let rowToUpdate  = -1;
  for (let i = 1; i < data.length; i++) {
    if (data[i][0] === deviceId && data[i][1] === key) {
      rowToUpdate = i + 1;
      break;
    }
  }
  const timestamp = new Date().toISOString();
  const valString = JSON.stringify(value);
  if (rowToUpdate > 0) {
    sheet.getRange(rowToUpdate, 3).setValue(valString);
    sheet.getRange(rowToUpdate, 4).setValue(timestamp);
  } else {
    sheet.appendRow([deviceId, key, valString, timestamp]);
  }
  return createJSONOutput({ result: "success", device: deviceId, key: key });
}

// ==========================================
// SECTION 2: HEARTBEAT DIAGNOSTICS (UPDATED)
// ==========================================

/**
 * Prepares the Heartbeat sheet, adding any missing V9/V10.1 columns to the
 * right of existing data so that old rows are never misaligned.
 *
 * Migration logic:
 *   - If sheet does not exist: create with full 18-column header.
 *   - If sheet exists with fewer columns (V5/V9 era): append the missing
 *     headers (V9's 9 columns and/or V10.1's "LittleFS Free (KB)").
 *   - If sheet already has all 18 columns: no-op.
 *
 * Returns an object { sheet, colIndex } where colIndex maps header name → 1-based column.
 */
function getAndPrepareHeartbeatSheet(ss) {
  let sheet = ss.getSheetByName(HEARTBEAT_SHEET_NAME);

  if (!sheet) {
    // Fresh sheet — write all headers at once
    sheet = ss.insertSheet(HEARTBEAT_SHEET_NAME);
    sheet.getRange(1, 1, 1, HEARTBEAT_HEADERS.length)
         .setValues([HEARTBEAT_HEADERS])
         .setFontWeight("bold");
    sheet.setFrozenRows(1);
    SpreadsheetApp.flush();
  } else {
    // Sheet exists — check which headers are already present
    const lastCol      = sheet.getLastColumn();
    const existingHdrs = lastCol > 0
      ? sheet.getRange(1, 1, 1, lastCol).getValues()[0]
      : [];

    const existingSet = new Set(existingHdrs.map(h => String(h).trim()));

    // Append any missing headers to the right (never insert/reorder)
    const missing = HEARTBEAT_HEADERS.filter(h => !existingSet.has(h));
    if (missing.length > 0) {
      const startCol = lastCol + 1;
      sheet.getRange(1, startCol, 1, missing.length)
           .setValues([missing])
           .setFontWeight("bold");
      SpreadsheetApp.flush();
    }
  }

  // Build a column-index map from the now-complete header row
  const totalCols = sheet.getLastColumn();
  const headerRow = sheet.getRange(1, 1, 1, totalCols).getValues()[0];
  const colIndex  = {};
  headerRow.forEach((h, i) => { colIndex[String(h).trim()] = i + 1; });

  return { sheet, colIndex };
}

function handleHeartbeat(payload) {
  const ss = SpreadsheetApp.getActiveSpreadsheet();
  const { sheet, colIndex } = getAndPrepareHeartbeatSheet(ss);

  // Parse dev_silence_min — arrives as array [n0,n1,n2,n3,n4]
  const silence = Array.isArray(payload.dev_silence_min)
    ? payload.dev_silence_min
    : [0, 0, 0, 0, 0];

  // Build a sparse row array indexed by column position
  const totalCols = sheet.getLastColumn();
  const row       = new Array(totalCols).fill("");

  function set(headerName, value) {
    const col = colIndex[headerName];
    if (col) row[col - 1] = (value !== undefined && value !== null) ? value : "";
  }

  // V5 original fields — columns 1-7 (order preserved exactly)
  set("Upload TS",          payload.upload_ts    || new Date().toISOString());
  set("Gateway Status",     payload.gateway      || "UNKNOWN");
  set("GSM Module",         payload.gsm          || "UNKNOWN");
  set("Operator",           payload.simOperator  || "UNKNOWN");
  set("Signal (CSQ)",       payload.gsmStrength  || 0);
  set("Modem Temp (°C)",    payload.temp         || 0.0);
  set("Last LoRa RX",       payload.last_lora_rx || "NO_DATA");

  // V9 new fields — columns 8-16
  set("LoRa ISR Overflows",  payload.lora_overflows   || 0);
  set("Queue Overflows",     payload.queue_overflows  || 0);
  set("HW FIFO Overflows",   payload.hw_overflows     || 0);
  set("LoRa Recoveries",     payload.lora_recovery    || 0);
  set("LoRa Restarts",       payload.lora_restarts    || 0);
  set("Lora1 Silence (min)", silence[0] !== undefined ? silence[0] : "");
  set("Lora2 Silence (min)", silence[1] !== undefined ? silence[1] : "");
  set("Lora3 Silence (min)", silence[2] !== undefined ? silence[2] : "");
  set("Lora4 Silence (min)", silence[3] !== undefined ? silence[3] : "");
  set("Lora5 Silence (min)", silence[4] !== undefined ? silence[4] : "");

  // V10.1 addition — col 18
  set("LittleFS Free (KB)", payload.lfsFreeKB !== undefined ? payload.lfsFreeKB : "");

  // V11 additions — cols 19-30 (blank for older gateways)
  const rssi = Array.isArray(payload.dev_rssi) ? payload.dev_rssi : [];
  set("Noise (dBm)",       payload.noise !== undefined ? payload.noise : "");
  set("Lora1 RSSI (dBm)",  rssi[0] !== undefined && rssi[0] !== 0 ? rssi[0] : "");
  set("Lora2 RSSI (dBm)",  rssi[1] !== undefined && rssi[1] !== 0 ? rssi[1] : "");
  set("Lora3 RSSI (dBm)",  rssi[2] !== undefined && rssi[2] !== 0 ? rssi[2] : "");
  set("Lora4 RSSI (dBm)",  rssi[3] !== undefined && rssi[3] !== 0 ? rssi[3] : "");
  set("Lora5 RSSI (dBm)",  rssi[4] !== undefined && rssi[4] !== 0 ? rssi[4] : "");
  set("LoRa Cfg OK",       payload.lora_cfg_ok !== undefined ? payload.lora_cfg_ok : "");
  set("Reset Reason",      payload.reset      || "");
  set("Uptime (min)",      payload.uptime_min !== undefined ? payload.uptime_min : "");
  set("Free Heap",         payload.heap       !== undefined ? payload.heap : "");
  set("Channel",           payload.channel    !== undefined ? payload.channel : "");
  set("Firmware",          payload.fw         || "");

  sheet.appendRow(row);

  return createJSONOutput({ result: "success", message: "Heartbeat Logged" });
}

// ==========================================
// SECTION 3: LORA DATA LOGGING (UNCHANGED)
// ==========================================

function getAndPrepareDataSheet(ss) {
  let sheet = ss.getSheetByName(BATCHED_DATA_SHEET_NAME);
  if (!sheet) {
    sheet = ss.insertSheet(BATCHED_DATA_SHEET_NAME);
  }
  if (sheet.getLastRow() < 1) {
    sheet.getRange(1, 1, 1, DATA_HEADERS.length)
         .setValues([DATA_HEADERS])
         .setFontWeight("bold");
    sheet.setFrozenRows(1);
    SpreadsheetApp.flush();
  } else {
    // V7 migration: append any missing header on the right, never reorder
    const lastCol = sheet.getLastColumn();
    const existing = new Set(sheet.getRange(1, 1, 1, lastCol).getValues()[0].map(h => String(h).trim()));
    const missing = DATA_HEADERS.filter(h => !existing.has(h));
    if (missing.length > 0) {
      sheet.getRange(1, lastCol + 1, 1, missing.length).setValues([missing]).setFontWeight("bold");
      SpreadsheetApp.flush();
    }
  }
  return sheet;
}

function handleLogData(payload) {
  const ss    = SpreadsheetApp.getActiveSpreadsheet();
  const sheet = getAndPrepareDataSheet(ss);

  const batchUploadTime = payload.upload_ts    || new Date().toISOString();
  const network         = payload.network      || "N/A";
  const simOperator     = payload.simOperator  || "N/A";
  const wifiStrength    = payload.wifiStrength != null ? payload.wifiStrength : "";
  const gsmStrength     = payload.gsmStrength  != null ? payload.gsmStrength  : "";
  const lfsFreeKB       = payload.lfsFreeKB    != null ? Number(payload.lfsFreeKB).toFixed(1) : "";

  const colorMap = {
    "Low":         "#FFC0CB",
    "Good":        "#98FB98",
    "Excess":      "#FFFFE0",
    "Flood Alert": "#D8BFD8",
    "SensorError": "#D3D3D3"   // V7: node sensor failed (level null)
  };

  // Column positions from the (possibly migrated) header row
  const totalCols = sheet.getLastColumn();
  const headerRow = sheet.getRange(1, 1, 1, totalCols).getValues()[0].map(h => String(h).trim());
  const colIndex  = {};
  headerRow.forEach((h, i) => { colIndex[h] = i; });

  const newRows        = [];
  const backgroundColors = [];

  payload.readings.forEach(reading => {
    const row = new Array(totalCols).fill("");
    const put = (name, value) => { if (colIndex[name] !== undefined) row[colIndex[name]] = value; };
    const sensorFailed = reading.status === "SensorError" || reading.waterLevel == null;
    put("Gateway Received Time", reading.gateway_rx_ts || "");
    put("Device ID",             reading.device        || "Unknown Device");
    put("Transmitter Data",      sensorFailed ? "" : (reading.tx_data || ""));
    put("Water Level (cm)",      sensorFailed ? "" : reading.waterLevel);
    put("Status",                reading.status        || "N/A");
    put("Network",               network);
    put("Batch Upload Time",     batchUploadTime);
    put("SIM Operator",          simOperator);
    put("WiFi Strength (dBm)",   wifiStrength);
    put("GSM Strength (CSQ)",    gsmStrength);
    put("LittleFS Free (KB)",    lfsFreeKB);
    put("Source",                reading.source        || "live");
    put("RSSI (dBm)",            (reading.rssi !== undefined && reading.rssi !== 0) ? reading.rssi : "");
    newRows.push(row);
    backgroundColors.push([colorMap[reading.status] || "#FFFFFF"]);
  });

  if (newRows.length > 0) {
    const startRow = sheet.getLastRow() + 1;
    sheet.getRange(startRow, 1, newRows.length, totalCols).setValues(newRows);
    const statusCol = (colIndex["Status"] !== undefined) ? colIndex["Status"] + 1 : 5;
    sheet.getRange(startRow, statusCol, newRows.length, 1).setBackgrounds(backgroundColors);
  }

  return createJSONOutput({
    result: "success",
    message: `${newRows.length} records processed.`
  });
}

function handleGetHeartbeat(e) {
  return sheetToJson(SpreadsheetApp.getActiveSpreadsheet().getSheetByName(HEARTBEAT_SHEET_NAME), e);
}

function handleGetData(e) {
  return sheetToJson(SpreadsheetApp.getActiveSpreadsheet().getSheetByName(BATCHED_DATA_SHEET_NAME), e);
}

// Shared: newest-first rows of a sheet as JSON, with ?days=N and ?limit=N.
function sheetToJson(sheet, e) {
  if (!sheet) {
    return createJSONOutput({ status: "error", message: "Sheet not found." });
  }

  const values = sheet.getDataRange().getValues();
  if (values.length <= 1) return createJSONOutput([]);

  const headers = values[0];
  let rows      = values.slice(1).reverse(); // newest first

  // ?days=N filter
  if (e.parameter && e.parameter.days && e.parameter.days !== 'all') {
    const days = parseInt(e.parameter.days);
    if (!isNaN(days) && days > 0) {
      const cutoff = new Date();
      cutoff.setDate(cutoff.getDate() - days);
      rows = rows.filter(row => {
        try { return new Date(row[0]) >= cutoff; } catch(err) { return true; }
      });
    }
  }

  // ?limit=N filter
  if (e.parameter && e.parameter.limit) {
    const limit = parseInt(e.parameter.limit);
    if (!isNaN(limit)) rows = rows.slice(0, limit);
  }

  const jsonData = rows.map(row => {
    const record = {};
    headers.forEach((header, i) => {
      record[header] = (row[i] instanceof Date) ? row[i].toISOString() : row[i];
    });
    return record;
  });

  return createJSONOutput(jsonData);
}

// ==========================================
// UTILITIES
// ==========================================

function createJSONOutput(data) {
  return ContentService.createTextOutput(JSON.stringify(data))
    .setMimeType(ContentService.MimeType.JSON);
}
