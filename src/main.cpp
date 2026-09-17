/*
 * Water flow meter - ESP32-CAM + YF-S401
 *
 * - Pulses are counted in an interrupt on GPIO13
 * - Every 10 seconds, pulses are sampled and converted to pulses/min
 * - 10-second records are buffered and persisted to SD
 * - Minute history is aggregated from six 10-second samples per minute
 * - AP mode serves a LittleFS-hosted dashboard + JSON/CSV APIs
 *
 * Camera is NOT initialized -> its pins are free for SD_MMC + sensor.
 *
 * --- async_tcp watchdog notes (read this before touching handlers) -------
 * Every server.on(...) lambda runs inside the "async_tcp" FreeRTOS task,
 * which is subscribed to the 5s task watchdog. Two rules follow from that,
 * and every handler below is written to respect them:
 *
 *   1. Never take a mutex with portMAX_DELAY from a handler. Always use a
 *      short pdMS_TO_TICKS() timeout and fail the request (503) if it
 *      expires. loop()-side code (sampleFlow, flushBufferToSD) is NOT in
 *      the watchdogged task, so it's fine for it to wait longer for the
 *      same locks.
 *   2. Never do unbounded work (reading a whole file into RAM, serializing
 *      under a lock that's also needed every second by loop()) inside a
 *      handler. Snapshot shared state under a short lock, release it, then
 *      do the slow part outside the lock; stream large payloads in bounded
 *      chunks instead of buffering them whole.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebSerial.h>
#include <SD_MMC.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <vector>
#include <set>
#include <memory>
#include <algorithm>

// ---------- Pin assignments (AI-Thinker ESP32-CAM, camera disabled) ------
#define FLOW_SENSOR_PIN   13   // interrupt-capable, free when camera is off
// SD_MMC 1-bit mode uses fixed pins on ESP32-CAM: CLK=14, CMD=15, D0=2
// (D1/D2/D3 normally used by camera - we skip them by using 1-bit mode)

// ---------- Timing ---------------------------------------------------------
static const uint32_t MEASUREMENT_INTERVAL_MS = 1000;  // 1-second windows: raw sensor sampling rate
static const uint32_t HISTORY_INTERVAL_MS = 1000;      // 1 point per second: matches the sample rate so
                                                         // the chart and the current-reading card always
                                                         // show the exact same value at the exact same time
static const uint32_t FLUSH_INTERVAL_MS = 10000;       // persist every 10 seconds of samples
static const size_t BUFFER_CAPACITY = 24;               // ~24 seconds of 1s records (see FLUSH_INTERVAL_MS)

static const uint32_t HEAP_LOG_INTERVAL_MS = 30000;    // log heap status every 30 seconds
static uint32_t lastHeapLogMs = 0;

// These are derived from the two intervals above rather than hardcoded, so
// changing either interval can't silently desync the rpm card, the chart,
// and the CSV export from each other again. HISTORY_INTERVAL_MS must be an
// integer multiple of MEASUREMENT_INTERVAL_MS for the bucketing math below
// to line up; this is checked (and logged) once at boot in setup().
static const uint32_t PULSES_PER_MINUTE_SCALE = 60000 / MEASUREMENT_INTERVAL_MS;  // scales one raw
                                                                                    // sample's pulses to pulses/min
static const uint32_t SAMPLES_PER_HISTORY_POINT = HISTORY_INTERVAL_MS / MEASUREMENT_INTERVAL_MS; // raw
                                                                                    // samples folded into one chart point
static const uint32_t HISTORY_POINT_SCALE = 60000 / HISTORY_INTERVAL_MS;          // scales a finalized
                                                                                    // bucket's pulse sum to pulses/min

// The live in-RAM chart only needs a recent window, not the whole session
// (which stays fully available on SD for export regardless). Bounding
// minuteHistory to this many points keeps both its RAM footprint and the
// /api/history response size constant no matter how long the device runs.
static const uint32_t LIVE_WINDOW_MS = 10UL * 60UL * 1000UL; // 10 minutes
static const size_t MAX_HISTORY_POINTS = LIVE_WINDOW_MS / HISTORY_INTERVAL_MS;

// ---------- Mutex wait budgets ---------------------------------------------
// Handler-side waits (async_tcp task): short and bounded, see file header.
static const TickType_t HANDLER_LOCK_TIMEOUT = pdMS_TO_TICKS(100);
static const TickType_t HANDLER_SD_TIMEOUT   = pdMS_TO_TICKS(300);

// ---------- AP credentials ---------------------------------------------
static const char* AP_SSID = "FlowMeter";
static const char* AP_PASS = "flowmeter123"; // 8+ chars required by WiFi lib

// ---------- Captive portal ------------------------------------------------
// Resolves every DNS query to the AP IP so OS captive-portal probes (and any
// stray hostname lookups) land on this device instead of failing to resolve.
static const byte DNS_PORT = 53;
DNSServer dnsServer;

// ---------- Shared state between ISR and loop -----------------------------
volatile uint32_t pulseCount = 0;
volatile uint32_t lastPulseMicros = 0;

// Minimum time between counted pulses, in microseconds. Filters out noise/
// ringing that would otherwise be counted as real pulses. Tune this down if
// your real flow rate needs a higher max frequency than ~500 Hz allows;
// tune it up if you still see suspiciously flat/maxed-out readings.
static const uint32_t MIN_PULSE_INTERVAL_US = 2000; // caps at 500 Hz

void IRAM_ATTR onPulse() {
  uint32_t now = micros();
  // micros() wraps every ~71 min; this subtraction is wrap-safe.
  if ((uint32_t)(now - lastPulseMicros) >= MIN_PULSE_INTERVAL_US) {
    pulseCount++;
    lastPulseMicros = now;
  }
}

// Guards SD_MMC access shared between loop() (sampling/flush) and the
// AsyncTCP task (history/export handlers), which run concurrently.
SemaphoreHandle_t sdMutex;

// Guards the RAM record buffer (buffer/bufferLen) shared between loop()
// (sampling/flush) and the AsyncTCP task (history/export handlers).
SemaphoreHandle_t bufferMutex;

// Guards the incrementally-built minute history (minuteHistory + the live
// in-progress minute accumulator) shared between loop() (sampleFlow) and
// the AsyncTCP task (the /api/history handler's current-session fast path).
SemaphoreHandle_t historyMutex;

// ---------- Record buffer ---------------------------------------------
struct FlowRecord {
  uint32_t t_ms;             // elapsed since boot, recorded at end of 10s window
  uint32_t pulse_count_10s;  // pulses counted in this 10-second interval
  uint32_t pulses_per_minute;
  uint32_t total_pulses;     // cumulative pulses since boot
};

FlowRecord buffer[BUFFER_CAPACITY];
size_t bufferLen = 0;

// ---------- Incremental minute history (current session only) -----------
// Built up one finalized minute at a time as sampleFlow() runs, so
// /api/history can serve the current session without re-reading and
// re-parsing the whole SD log file on every poll. Historical files
// (requested via ?file=) still use the full-replay path since they're
// viewed rarely, not polled continuously.
struct MinutePoint {
  uint32_t t_ms;
  uint32_t pulses_per_minute;
};
std::vector<MinutePoint> minuteHistory;

bool minuteAccActive = false;
uint32_t minuteAccIndex = 0;
uint32_t minuteAccPulseSum = 0;
uint32_t minuteAccSampleCount = 0;

uint32_t totalPulses = 0;
uint32_t latestPulseCount10s = 0;
uint32_t latestPulsesPerMinute = 0;
uint32_t latestMeasurementElapsedMs = 0;
bool hasMeasurement = false;

uint32_t lastMeasurementMs = 0;
uint32_t lastFlushMs = 0;

bool sdReady = false;
// A fresh log file is created every boot (see initLogFile()); LOG_PATH
// always points at the file for the current session.
String LOG_PATH;
const char* BOOT_SEQ_PATH = "/boot_seq.txt";

// Real files present on LittleFS at boot, used to keep the static handler
// from swallowing every unmatched path (see buildAssetIndex()/setupServer()).
std::set<String> knownAssets;

AsyncWebServer server(80);

bool parseFlowRecordLine(const String &line, FlowRecord &record) {
  unsigned long tMs = 0;
  unsigned long pulseCount10s = 0;
  unsigned long pulsesPerMinute = 0;
  unsigned long totalPulsesParsed = 0;

  // Preferred schema: t_ms,pulse_count_10s,pulses_per_minute,total_pulses
  if (sscanf(line.c_str(), "%lu,%lu,%lu,%lu", &tMs, &pulseCount10s, &pulsesPerMinute, &totalPulsesParsed) == 4) {
    record.t_ms = static_cast<uint32_t>(tMs);
    record.pulse_count_10s = static_cast<uint32_t>(pulseCount10s);
    record.pulses_per_minute = static_cast<uint32_t>(pulsesPerMinute);
    record.total_pulses = static_cast<uint32_t>(totalPulsesParsed);
    return true;
  }

  // Backward-compatible parse for old schema: t_ms,pulses_sample,pulses_total_acc
  if (sscanf(line.c_str(), "%lu,%lu,%lu", &tMs, &pulseCount10s, &totalPulsesParsed) == 3) {
    record.t_ms = static_cast<uint32_t>(tMs);
    record.pulse_count_10s = static_cast<uint32_t>(pulseCount10s);
    record.pulses_per_minute = static_cast<uint32_t>(pulseCount10s * 6);
    record.total_pulses = static_cast<uint32_t>(totalPulsesParsed);
    return true;
  }

  return false;
}

// Only accepts paths this firmware itself could have generated, so the
// "file" query param can never be used to open arbitrary SD paths.
bool isValidLogFile(const String &path) {
  static const char *PREFIX = "/flow_log_";
  static const char *SUFFIX = ".csv";
  size_t prefixLen = strlen(PREFIX);
  size_t suffixLen = strlen(SUFFIX);

  if (path.length() <= prefixLen + suffixLen) return false;
  if (!path.startsWith(PREFIX) || !path.endsWith(SUFFIX)) return false;

  String middle = path.substring(prefixLen, path.length() - suffixLen);
  if (middle.length() == 0) return false;
  for (size_t i = 0; i < middle.length(); i++) {
    if (!isDigit(middle[i])) return false;
  }
  return true;
}

// ---------- SD helpers ---------------------------------------------------
// Reads/increments a persistent boot counter and returns a unique log path
// for this session, e.g. "/flow_log_0007.csv". Must be called with sdMutex
// already held or before the server/loop start touching SD.
String allocateLogPath() {
  unsigned long seq = 0;
  if (SD_MMC.exists(BOOT_SEQ_PATH)) {
    File f = SD_MMC.open(BOOT_SEQ_PATH, FILE_READ);
    if (f) {
      seq = f.parseInt();
      f.close();
    }
  }

  File f = SD_MMC.open(BOOT_SEQ_PATH, FILE_WRITE);
  if (f) {
    f.print(seq + 1);
    f.close();
  }

  char path[32];
  snprintf(path, sizeof(path), "/flow_log_%04lu.csv", seq);
  return String(path);
}

// setup()-only: runs once before the server or loop() ever touch SD, so a
// generous wait here is fine - there's no contention yet and nothing is
// watchdogged at this point.
void ensureLogHeader() {
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    Serial.println("[SD] log header skipped, sdMutex busy");
    WebSerial.println("[SD] log header skipped, sdMutex busy");
    return;
  }
  if (!SD_MMC.exists(LOG_PATH)) {
    File f = SD_MMC.open(LOG_PATH, FILE_WRITE);
    if (f) {
      f.println("t_ms,pulse_count_10s,pulses_per_minute,total_pulses");
      f.close();
    }
  }
  xSemaphoreGive(sdMutex);
}

// Creates a brand-new log file for this boot session, so past runs are
// preserved as separate files instead of being appended to. setup()-only,
// same reasoning as ensureLogHeader() above.
void initLogFile() {
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    Serial.println("[SD] log init skipped, sdMutex busy");
    WebSerial.println("[SD] log init skipped, sdMutex busy");
    return;
  }
  LOG_PATH = allocateLogPath();
  xSemaphoreGive(sdMutex);
  ensureLogHeader();
}

// Lists log files on SD (newest first), for the export file picker. Called
// from the /api/files handler, i.e. from the async_tcp task - bounded wait.
bool listLogFiles(std::vector<String> &names) {
  if (xSemaphoreTake(sdMutex, HANDLER_SD_TIMEOUT) != pdTRUE) {
    return false;
  }
  File root = SD_MMC.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      String name = String(entry.name());
      if (!name.startsWith("/")) {
        name = "/" + name;
      }
      if (name.startsWith("/flow_log_") && name.endsWith(".csv")) {
        names.push_back(name);
      }
      entry = root.openNextFile();
    }
    root.close();
  }
  xSemaphoreGive(sdMutex);

  std::sort(names.begin(), names.end(), std::greater<String>());
  return true;
}

// loop()-only (not the watchdogged async_tcp task), so waiting for the
// locks here can afford to be generous - the readers on the handler side
// now hold them only briefly, so contention windows are short anyway.
void flushBufferToSD() {
  if (!sdReady) return;

  // Snapshot under the buffer lock, then release it before touching SD so
  // concurrent /api/history and /api/export.csv reads are never blocked by
  // slow SD I/O.
  FlowRecord snapshot[BUFFER_CAPACITY];
  size_t snapshotLen;

  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  snapshotLen = bufferLen;
  memcpy(snapshot, buffer, sizeof(FlowRecord) * snapshotLen);
  xSemaphoreGive(bufferMutex);

  if (snapshotLen == 0) return;

  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    Serial.println("[SD] flush skipped, sdMutex busy");
    WebSerial.println("[SD] flush skipped, sdMutex busy");
    return;
  }
  File f = SD_MMC.open(LOG_PATH, FILE_APPEND);
  if (!f) {
    xSemaphoreGive(sdMutex);
    Serial.println("[SD] failed to open log for append");
    WebSerial.println("[SD] failed to open log for append");
    return;
  }
  for (size_t i = 0; i < snapshotLen; i++) {
    f.printf("%lu,%lu,%lu,%lu\n",
             snapshot[i].t_ms,
             snapshot[i].pulse_count_10s,
             snapshot[i].pulses_per_minute,
             snapshot[i].total_pulses);
  }
  f.close();
  xSemaphoreGive(sdMutex);

  // Only drop the records we actually persisted; anything appended while we
  // were writing to SD (e.g. sampleFlow's buffer-full retry) stays queued.
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  if (bufferLen >= snapshotLen) {
    memmove(buffer, buffer + snapshotLen, sizeof(FlowRecord) * (bufferLen - snapshotLen));
    bufferLen -= snapshotLen;
  }
  xSemaphoreGive(bufferMutex);

  Serial.printf("[SD] flushed %u records\n", (unsigned)snapshotLen);
  WebSerial.printf("[SD] flushed %u records\n", (unsigned)snapshotLen);
}

// ---------- Sensor sampling (loop()-only) --------------------------------
void sampleFlow(uint32_t measurementElapsedMs) {
  noInterrupts();
  uint32_t pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  uint32_t pulsesPerMinute = pulses * PULSES_PER_MINUTE_SCALE;

  totalPulses += pulses;
  latestPulseCount10s = pulses;
  latestPulsesPerMinute = pulsesPerMinute;
  latestMeasurementElapsedMs = measurementElapsedMs;
  hasMeasurement = true;

  FlowRecord record;
  record.t_ms = measurementElapsedMs;
  record.pulse_count_10s = pulses;
  record.pulses_per_minute = pulsesPerMinute;
  record.total_pulses = totalPulses;

  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  bool bufferFull = (bufferLen >= BUFFER_CAPACITY);
  if (!bufferFull) {
    buffer[bufferLen++] = record;
  }
  xSemaphoreGive(bufferMutex);

  if (bufferFull) {
    // buffer full before scheduled flush - flush now to avoid losing data
    flushBufferToSD();
    xSemaphoreTake(bufferMutex, portMAX_DELAY);
    if (bufferLen < BUFFER_CAPACITY) {
      buffer[bufferLen++] = record;
    }
    xSemaphoreGive(bufferMutex);
  }

  // Fold this sample into the current history bucket, finalizing it the
  // moment enough samples have accumulated (rather than waiting for the
  // next sample to prove the bucket is over) so the chart's newest point
  // shows up in the same tick as the rpm card, not a bucket later.
  xSemaphoreTake(historyMutex, portMAX_DELAY);
  if (!minuteAccActive) {
    minuteAccActive = true;
    minuteAccIndex = (measurementElapsedMs - 1) / HISTORY_INTERVAL_MS;
  }
  minuteAccPulseSum += pulses;
  minuteAccSampleCount++;

  if (minuteAccSampleCount >= SAMPLES_PER_HISTORY_POINT) {
    MinutePoint point;
    point.t_ms = (minuteAccIndex + 1) * HISTORY_INTERVAL_MS;
    point.pulses_per_minute = minuteAccPulseSum * HISTORY_POINT_SCALE;
    minuteHistory.push_back(point);
    // Bounded to a rolling window (~600 points at 1/sec for 10 minutes) -
    // one erase-from-front per second is negligible work on this hardware,
    // and it's far simpler than a manual circular-buffer index for a
    // vector this small.
    if (minuteHistory.size() > MAX_HISTORY_POINTS) {
      minuteHistory.erase(minuteHistory.begin());
    }
    minuteAccActive = false;
    minuteAccPulseSum = 0;
    minuteAccSampleCount = 0;
  }
  xSemaphoreGive(historyMutex);
}

// ---------- Static-asset filtering -----------------------------------------
// Walks LittleFS once at boot and records every real file. serveStatic()'s
// filter then only matches requests against this set (plus "/"), instead of
// matching every non-/api/ path. Without this, serveStatic's broad filter
// claims requests like /generate_204 or /favicon.ico before they ever reach
// the captive-portal redirect in onNotFound() - each one then falls through
// AsyncStaticWebHandler's internal "try .gz, then plain, then index.html"
// fallback, doing several synchronous flash reads per bogus request, all
// inside the watchdogged async_tcp task. A burst of these around Wi-Fi
// association is exactly what was tripping the task watchdog.
void buildAssetIndex() {
  File root = LittleFS.open("/");
  if (!root) return;
  File f = root.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      String name = String(f.name());
      if (!name.startsWith("/")) {
        name = "/" + name;
      }
      knownAssets.insert(name);
    }
    f = root.openNextFile();
  }
  root.close();
  Serial.printf("[LittleFS] indexed %u asset(s)\n", (unsigned)knownAssets.size());
}

// ---------- CSV export state (async_tcp task, streamed) -------------------
// Carried by shared_ptr into the chunked-response callback so it lives
// exactly as long as the response does and is freed automatically when it
// finishes - no manual cleanup path to get wrong on the error branches.
struct ExportState {
  File file;
  bool fileOpen = false;
  bool headerPending = false;
  bool includeBuffer = false;
  bool bufferDone = false;
  FlowRecord bufSnapshot[BUFFER_CAPACITY];
  size_t bufLen = 0;
  size_t bufIndex = 0;
};

// ---------- Web server routes --------------------------------------------
void setupServer() {
  WebSerial.begin(&server);

  if (!LittleFS.begin(true)) {
    Serial.println("[LittleFS] mount failed");
    WebSerial.println("[LittleFS] mount failed");
  } else {
    buildAssetIndex();
  }

  // WebSerial: bi-directional console over WebSocket, for debugging
  /* Optional: Attach a callback to receive data from the web interface
  WebSerial.onMessage([](uint8_t *data, size_t len) {
    // Handle incoming commands if necessary
  }); */

  // Current reading based on the latest completed 10-second measurement.
  server.on("/api/current", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["elapsed_ms"] = millis();
    doc["pulse_count_10s"] = latestPulseCount10s;
    doc["pulses_per_minute"] = latestPulsesPerMinute;
    doc["total_pulses"] = totalPulses;
    doc["measurement_interval_ms"] = MEASUREMENT_INTERVAL_MS;
    doc["has_measurement"] = hasMeasurement;
    doc["sd_ready"] = sdReady;
    doc["current_file"] = LOG_PATH;

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  server.on("/api/history", HTTP_GET, [](AsyncWebServerRequest *request) {
    bool wantsCurrent = !request->hasParam("file");
    String requestedFile = LOG_PATH;
    if (!wantsCurrent) {
      requestedFile = request->getParam("file")->value();
      if (!isValidLogFile(requestedFile)) {
        request->send(400, "text/plain", "Invalid log file");
        return;
      }
    }
    bool isCurrentFile = wantsCurrent || (requestedFile == LOG_PATH);

    // Fast path: the current session's history is already finalized
    // incrementally in RAM by sampleFlow(), so this is O(points), not
    // O(entire log file), and needs no SD access at all. This is what
    // keeps continuous 1-second polling from bogging down as a session
    // runs for a long time.
    //
    // The lock is held only long enough to copy the (small, bounded)
    // vector and the accumulator fields - all serialization happens after
    // it's released, so a slow/backed-up TCP send here can never delay
    // sampleFlow()'s once-a-second write to the same data.
    if (isCurrentFile) {
      std::vector<MinutePoint> historySnapshot;
      bool haveActive = false;
      uint32_t activeIndex = 0, activeSum = 0, activeCount = 0;

      if (xSemaphoreTake(historyMutex, HANDLER_LOCK_TIMEOUT) != pdTRUE) {
        request->send(503, "text/plain", "Busy, try again");
        return;
      }
      historySnapshot = minuteHistory;
      haveActive = minuteAccActive;
      activeIndex = minuteAccIndex;
      activeSum = minuteAccPulseSum;
      activeCount = minuteAccSampleCount;
      xSemaphoreGive(historyMutex);

      AsyncResponseStream *response = request->beginResponseStream("application/json");
      response->print("{\"interval_ms\":");
      response->print(HISTORY_INTERVAL_MS);
      response->print(",\"measurement_interval_ms\":");
      response->print(MEASUREMENT_INTERVAL_MS);
      response->print(",\"points\":[");

      bool firstPoint = true;
      for (const MinutePoint &point : historySnapshot) {
        if (!firstPoint) {
          response->print(',');
        }
        response->printf("{\"t_ms\":%lu,\"pulses_per_minute\":%lu}",
                         static_cast<unsigned long>(point.t_ms),
                         static_cast<unsigned long>(point.pulses_per_minute));
        firstPoint = false;
      }
      response->print(']');
      if (haveActive && activeCount > 0 && activeCount < SAMPLES_PER_HISTORY_POINT) {
        response->printf(",\"partial_minute\":{\"t_ms\":%lu,\"samples\":%lu,\"pulse_count_sum\":%lu}",
                         static_cast<unsigned long>((activeIndex + 1) * HISTORY_INTERVAL_MS),
                         static_cast<unsigned long>(activeCount),
                         static_cast<unsigned long>(activeSum));
      }
      response->print('}');
      request->send(response);
      return;
    }

    // Historical-file path: viewing an old session is a rare, one-off
    // action (not continuously polled), so replaying the whole file here
    // is fine.
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->print("{\"interval_ms\":");
    response->print(HISTORY_INTERVAL_MS);
    response->print(",\"measurement_interval_ms\":");
    response->print(MEASUREMENT_INTERVAL_MS);
    response->print(",\"points\":[");

    bool firstPoint = true;
    bool hasPartialMinute = false;
    uint32_t partialMinuteElapsedMs = 0;
    uint32_t partialMinutePulseCount = 0;
    uint32_t partialMinuteSampleCount = 0;

    bool minuteActive = false;
    uint32_t minuteIndex = 0;
    uint32_t minutePulseSum = 0;
    uint32_t minuteSampleCount = 0;

    auto emitMinute = [&](uint32_t idx, uint32_t pulsesPerMinutePoint) {
      if (!firstPoint) {
        response->print(',');
      }
      response->printf("{\"t_ms\":%lu,\"pulses_per_minute\":%lu}",
                       static_cast<unsigned long>((idx + 1) * HISTORY_INTERVAL_MS),
                       static_cast<unsigned long>(pulsesPerMinutePoint));
      firstPoint = false;
    };

    auto finalizeMinute = [&]() {
      if (!minuteActive) {
        return;
      }

      if (minuteSampleCount >= SAMPLES_PER_HISTORY_POINT) {
        emitMinute(minuteIndex, minutePulseSum * HISTORY_POINT_SCALE);
      } else if (minuteSampleCount > 0) {
        hasPartialMinute = true;
        partialMinuteElapsedMs = (minuteIndex + 1) * HISTORY_INTERVAL_MS;
        partialMinutePulseCount = minutePulseSum;
        partialMinuteSampleCount = minuteSampleCount;
      }

      minuteActive = false;
      minutePulseSum = 0;
      minuteSampleCount = 0;
    };

    auto consumeRecord = [&](const FlowRecord &record) {
      if (record.t_ms == 0) {
        return;
      }

      uint32_t recordMinuteIndex = (record.t_ms - 1) / HISTORY_INTERVAL_MS;
      if (!minuteActive) {
        minuteActive = true;
        minuteIndex = recordMinuteIndex;
      } else if (recordMinuteIndex != minuteIndex) {
        finalizeMinute();
        minuteActive = true;
        minuteIndex = recordMinuteIndex;
      }

      minutePulseSum += record.pulse_count_10s;
      minuteSampleCount++;
    };

    if (xSemaphoreTake(sdMutex, HANDLER_SD_TIMEOUT) != pdTRUE) {
      request->send(503, "text/plain", "SD busy, try again");
      return;
    }
    if (sdReady && SD_MMC.exists(requestedFile)) {
      File f = SD_MMC.open(requestedFile, FILE_READ);
      if (f) {
        while (f.available()) {
          String line = f.readStringUntil('\n');
          line.trim();
          if (line.isEmpty() || line.startsWith("t_ms,")) {
            continue;
          }

          FlowRecord record;
          if (parseFlowRecordLine(line, record)) {
            consumeRecord(record);
          }
        }
        f.close();
      }
    }
    xSemaphoreGive(sdMutex);

    // This path only ever runs for an explicitly requested historical
    // file (the current session returns earlier via the fast path above),
    // so there's no live RAM buffer to merge in here - it's all on SD.
    finalizeMinute();

    response->print(']');
    if (hasPartialMinute) {
      response->printf(",\"partial_minute\":{\"t_ms\":%lu,\"samples\":%lu,\"pulse_count_sum\":%lu}",
                       static_cast<unsigned long>(partialMinuteElapsedMs),
                       static_cast<unsigned long>(partialMinuteSampleCount),
                       static_cast<unsigned long>(partialMinutePulseCount));
    }
    response->print('}');
    request->send(response);
  });

  // Lists log files available on SD, newest first, so the dashboard can
  // offer a file picker for export/history review.
  server.on("/api/files", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    JsonArray files = doc["files"].to<JsonArray>();

    if (sdReady) {
      std::vector<String> logFiles;
      if (!listLogFiles(logFiles)) {
        request->send(503, "text/plain", "SD busy, try again");
        return;
      }
      for (const String &name : logFiles) {
        JsonObject entry = files.add<JsonObject>();
        entry["name"] = name;
        entry["current"] = (name == LOG_PATH);
      }
    }
    doc["current"] = LOG_PATH;

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  // Streams the CSV instead of buffering it in RAM: for a historical file
  // this is a near-verbatim byte copy straight from SD (no per-line String
  // allocation, no quadratic response-buffer growth); for the current
  // session it copies from SD the same way and then appends the small
  // (<= BUFFER_CAPACITY records) unflushed tail from RAM. Locks are taken
  // AFTER the response object would otherwise be created, and released
  // well before the (potentially large) SD copy runs, so a busy-503 here
  // never leaks a response object the way the old buffered version could.
  server.on("/api/export.csv", HTTP_GET, [](AsyncWebServerRequest *request) {
    String requestedFile = LOG_PATH;
    if (request->hasParam("file")) {
      requestedFile = request->getParam("file")->value();
      if (!isValidLogFile(requestedFile)) {
        request->send(400, "text/plain", "Invalid log file");
        return;
      }
    }
    bool isCurrentFile = (requestedFile == LOG_PATH);

    // Historical files only ever exist on SD, so those still require it.
    // The current session, however, is also held in the RAM buffer, so it
    // can still be exported even if SD never mounted or failed mid-run.
    if (!sdReady && !isCurrentFile) {
      request->send(503, "text/plain", "SD card not available");
      return;
    }

    auto state = std::make_shared<ExportState>();

    if (sdReady) {
      if (xSemaphoreTake(sdMutex, HANDLER_SD_TIMEOUT) != pdTRUE) {
        request->send(503, "text/plain", "SD busy, try again");
        return;
      }
      if (SD_MMC.exists(requestedFile)) {
        state->file = SD_MMC.open(requestedFile, FILE_READ);
        state->fileOpen = (bool)state->file;
      }
      xSemaphoreGive(sdMutex);
    }
    // If we couldn't open the file (missing, or SD unavailable), the SD
    // side of the response is skipped and this handler writes the header
    // itself so the output is still a well-formed CSV.
    state->headerPending = !state->fileOpen;

    if (isCurrentFile) {
      if (xSemaphoreTake(bufferMutex, HANDLER_LOCK_TIMEOUT) != pdTRUE) {
        if (state->fileOpen) state->file.close();
        request->send(503, "text/plain", "Buffer busy, try again");
        return;
      }
      state->bufLen = bufferLen;
      memcpy(state->bufSnapshot, buffer, sizeof(FlowRecord) * bufferLen);
      xSemaphoreGive(bufferMutex);
      state->includeBuffer = true;
    }

    String fileName = requestedFile;
    fileName.replace("/", "");
    if (fileName.isEmpty()) {
      fileName = "flow_log_current.csv";
    }

    AsyncWebServerResponse *response = request->beginChunkedResponse(
      "text/csv",
      [state](uint8_t *buf, size_t maxLen, size_t /*index*/) -> size_t {
        size_t written = 0;

        if (state->headerPending) {
          static const char *HEADER = "t_ms,pulse_count_10s,pulses_per_minute,total_pulses\n";
          size_t len = strlen(HEADER);
          if (len > maxLen) {
            return 0; // pathologically small chunk size; won't happen in practice
          }
          memcpy(buf, HEADER, len);
          written = len;
          state->headerPending = false;
        }

        // Raw byte copy straight from the file - it's already valid CSV
        // (including its own header, which is why we only add one above
        // when there was no file to read one from), so there's no need to
        // parse lines here the way the old handler did.
        if (state->fileOpen && written < maxLen) {
          int n = state->file.read(buf + written, maxLen - written);
          if (n > 0) {
            written += (size_t)n;
          } else {
            state->file.close();
            state->fileOpen = false;
          }
        }

        // Small (<= BUFFER_CAPACITY records), always fits in the
        // remainder of a chunk; if it somehow doesn't, the unwritten
        // records are simply retried on the next callback invocation.
        if (written < maxLen && state->includeBuffer && !state->bufferDone) {
          while (state->bufIndex < state->bufLen) {
            char line[64];
            int len = snprintf(line, sizeof(line), "%lu,%lu,%lu,%lu\n",
                                (unsigned long)state->bufSnapshot[state->bufIndex].t_ms,
                                (unsigned long)state->bufSnapshot[state->bufIndex].pulse_count_10s,
                                (unsigned long)state->bufSnapshot[state->bufIndex].pulses_per_minute,
                                (unsigned long)state->bufSnapshot[state->bufIndex].total_pulses);
            if (len < 0) {
              state->bufIndex++;
              continue;
            }
            if (written + (size_t)len > maxLen) {
              break; // retry this record on the next call
            }
            memcpy(buf + written, line, len);
            written += (size_t)len;
            state->bufIndex++;
          }
          if (state->bufIndex >= state->bufLen) {
            state->bufferDone = true;
          }
        }

        return written; // 0 once every phase is exhausted -> ends the response
      });

    response->addHeader("Content-Disposition", "attachment; filename=" + fileName);
    request->send(response);
  });


  // Captive portal detection endpoints for various OS connectivity checks.
  server.on("/generate_204", HTTP_ANY, [](AsyncWebServerRequest *request) {
    request->redirect("/");
  });
  server.on("/hotspot-detect.html", HTTP_ANY, [](AsyncWebServerRequest *request) {
      request->redirect("/");
  });
  server.on("/ncsi.txt", HTTP_ANY, [](AsyncWebServerRequest *request) {
      request->redirect("/");
  });
  server.on("/connecttest.txt", HTTP_ANY, [](AsyncWebServerRequest *request) {
      request->redirect("/");
  });
  server.on("/redirect", HTTP_ANY, [](AsyncWebServerRequest *request) {
      request->redirect("/");
  });

  // Serve the frontend from LittleFS (data/index.html -> uploaded via
  // `pio run --target uploadfs`), fully decoupled from firmware logic.
  // Filtered against the boot-time asset index (see buildAssetIndex())
  // rather than a blanket "not /api/" filter, so this handler only ever
  // claims requests for files that actually exist. Everything else -
  // captive-portal probes included - now correctly reaches onNotFound()
  // below without touching the filesystem at all.
  server.serveStatic("/", LittleFS, "/")
  .setDefaultFile("index.html")
  .setFilter([](AsyncWebServerRequest *r) {
    if (r->url().startsWith("/api/")) return false;
    if (r->url() == "/") return true;
    return knownAssets.count(r->url()) > 0;
  });

  // ---- Captive portal detection ----
  // OS connectivity checks hit fixed, well-known paths (any hostname,
  // since DNS resolves everything to us) that don't exist as real files,
  // so with the asset-index filter above they all land here. Redirecting
  // to "/" makes the OS recognize a captive portal and open it in a
  // browser/portal webview. This one handler covers every OS's probe path
  // (Android's /generate_204, iOS/macOS's /hotspot-detect.html, Windows'
  // /ncsi.txt, and anything else) instead of needing one entry per OS.
  server.onNotFound([](AsyncWebServerRequest *request) {
    if (request->url().startsWith("/api/")) {
      request->send(404, "application/json", "{\"error\":\"Not found\"}");
      return;
    }
    request->redirect("/");
  });

  server.begin();
}

// ---------- Setup / loop --------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.printf("[BOOT] reset reason: %d, free heap: %u\n",(int)esp_reset_reason(), ESP.getFreeHeap());

  if (60000 % MEASUREMENT_INTERVAL_MS != 0) {
    Serial.println("[CONFIG] WARNING: 60000 is not evenly divisible by MEASUREMENT_INTERVAL_MS - pulses_per_minute will be slightly off");
  }
  if (HISTORY_INTERVAL_MS % MEASUREMENT_INTERVAL_MS != 0 || SAMPLES_PER_HISTORY_POINT == 0) {
    Serial.println("[CONFIG] WARNING: HISTORY_INTERVAL_MS must be a positive multiple of MEASUREMENT_INTERVAL_MS - history bucketing will be wrong");
  }

  sdMutex = xSemaphoreCreateMutex();
  bufferMutex = xSemaphoreCreateMutex();
  historyMutex = xSemaphoreCreateMutex();

  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), onPulse, RISING);

  // Mount SD before the server starts, so /api/history and /api/export.csv
  // never race against the initial mount attempt.
  if (SD_MMC.begin("/sdcard", true)) {
    sdReady = true;
    initLogFile();
    Serial.printf("[SD] mounted ok (1-bit mode), logging to %s\n", LOG_PATH.c_str());
  } else {
    Serial.println("[SD] mount failed - logging disabled, AP+API still work");
  }

  WiFi.mode(WIFI_AP);
  IPAddress local_IP(10, 0, 0, 1);     // Target IP address
  IPAddress gateway(10, 0, 0, 1);      // Gateway (typically matches IP for AP)
  IPAddress subnet(255, 255, 255, 0);  // Subnet mask
  WiFi.softAPConfig(local_IP, gateway, subnet); // URL is http://10.0.0.1/
  WiFi.softAP(AP_SSID, AP_PASS);

  // Captive portal: resolve every hostname to the AP IP so phones detect
  // this network as a captive portal and prompt the user to open it.
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  setupServer();

  if (sdReady) {
    WebSerial.printf("[SD] mounted ok (1-bit mode), logging to %s\n", LOG_PATH.c_str());
  } else {
    WebSerial.println("[SD] mount failed - logging disabled, AP+API still work");
  }

  Serial.printf("[AP] started, IP: %s\n", WiFi.softAPIP().toString().c_str());
  WebSerial.printf("[AP] started, IP: %s\n", WiFi.softAPIP().toString().c_str());

  lastMeasurementMs = millis();
  lastFlushMs = millis();
}

void loop() {
  WebSerial.loop();
  dnsServer.processNextRequest(); // lightweight, non-blocking DNS capture

  uint32_t now = millis();

  // Log heap status periodically.
  if (now - lastHeapLogMs >= HEAP_LOG_INTERVAL_MS) {
    lastHeapLogMs = now;
    Serial.printf("[HEAP] free=%u largest=%u minfree=%u\n",ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap());
  }

  // Perform any other periodic tasks here.
  while ((uint32_t)(now - lastMeasurementMs) >= MEASUREMENT_INTERVAL_MS) {
    lastMeasurementMs += MEASUREMENT_INTERVAL_MS;
    sampleFlow(lastMeasurementMs);
  }

  // Flush any remaining data to the SD card periodically.
  if (now - lastFlushMs >= FLUSH_INTERVAL_MS) {
    lastFlushMs = now;
    flushBufferToSD();
  }

  delay(1); // Short delay to yield to other tasks and avoid watchdog resets.
}
