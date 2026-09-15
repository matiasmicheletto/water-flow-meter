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

void ensureLogHeader() {
  xSemaphoreTake(sdMutex, portMAX_DELAY);
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
// preserved as separate files instead of being appended to.
void initLogFile() {
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  LOG_PATH = allocateLogPath();
  xSemaphoreGive(sdMutex);
  ensureLogHeader();
}

// Lists log files on SD (newest first), for the export file picker.
std::vector<String> listLogFiles() {
  std::vector<String> names;
  xSemaphoreTake(sdMutex, portMAX_DELAY);
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
  return names;
}

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

  xSemaphoreTake(sdMutex, portMAX_DELAY);
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

// ---------- Sensor sampling ---------------------------------------------
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

// ---------- Web server routes --------------------------------------------
void setupServer() {
  WebSerial.begin(&server);

  if (!LittleFS.begin(true)) {
    Serial.println("[LittleFS] mount failed");
    WebSerial.println("[LittleFS] mount failed");
  }

  // WebSerial: bi-directional console over WebSocket, for debugging
  /* Optional: Attach a callback to receive data from the web interface
  WebSerial.onMessage([](uint8_t *data, size_t len) {
    // Handle incoming commands if necessary
  }); */
  // Serve the frontend from LittleFS (data/index.html -> uploaded via
  // `pio run --target uploadfs`), fully decoupled from firmware logic.
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

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
    if (isCurrentFile) {
      AsyncResponseStream *response = request->beginResponseStream("application/json");
      response->print("{\"interval_ms\":");
      response->print(HISTORY_INTERVAL_MS);
      response->print(",\"measurement_interval_ms\":");
      response->print(MEASUREMENT_INTERVAL_MS);
      response->print(",\"points\":[");

      xSemaphoreTake(historyMutex, portMAX_DELAY);
      bool firstPoint = true;
      for (const MinutePoint &point : minuteHistory) {
        if (!firstPoint) {
          response->print(',');
        }
        response->printf("{\"t_ms\":%lu,\"pulses_per_minute\":%lu}",
                         static_cast<unsigned long>(point.t_ms),
                         static_cast<unsigned long>(point.pulses_per_minute));
        firstPoint = false;
      }
      response->print(']');
      if (minuteAccActive && minuteAccSampleCount > 0 && minuteAccSampleCount < 6) {
        response->printf(",\"partial_minute\":{\"t_ms\":%lu,\"samples\":%lu,\"pulse_count_sum\":%lu}",
                         static_cast<unsigned long>((minuteAccIndex + 1) * HISTORY_INTERVAL_MS),
                         static_cast<unsigned long>(minuteAccSampleCount),
                         static_cast<unsigned long>(minuteAccPulseSum));
      }
      xSemaphoreGive(historyMutex);

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

    xSemaphoreTake(sdMutex, portMAX_DELAY);
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
    // so there's no live RAM buffer to merge in here — it's all on SD.
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
      for (const String &name : listLogFiles()) {
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

    String fileName = requestedFile;
    fileName.replace("/", "");
    if (fileName.isEmpty()) {
      fileName = "flow_log_current.csv";
    }

    AsyncResponseStream *response = request->beginResponseStream("text/csv");
    response->addHeader("Content-Disposition", "attachment; filename=" + fileName);
    response->print("t_ms,pulse_count_10s,pulses_per_minute,total_pulses\n");

    xSemaphoreTake(sdMutex, portMAX_DELAY);
    if (sdReady && SD_MMC.exists(requestedFile)) {
      File f = SD_MMC.open(requestedFile, FILE_READ);
      if (f) {
        while (f.available()) {
          String line = f.readStringUntil('\n');
          line.trim();
          if (line.isEmpty() || line.startsWith("t_ms,")) {
            continue;
          }
          response->println(line);
        }
        f.close();
      }
    }
    xSemaphoreGive(sdMutex);

    if (isCurrentFile) {
      FlowRecord snapshot[BUFFER_CAPACITY];
      size_t snapshotLen;
      xSemaphoreTake(bufferMutex, portMAX_DELAY);
      snapshotLen = bufferLen;
      memcpy(snapshot, buffer, sizeof(FlowRecord) * snapshotLen);
      xSemaphoreGive(bufferMutex);

      for (size_t i = 0; i < snapshotLen; i++) {
        response->printf("%lu,%lu,%lu,%lu\n",
                         snapshot[i].t_ms,
                         snapshot[i].pulse_count_10s,
                         snapshot[i].pulses_per_minute,
                         snapshot[i].total_pulses);
      }
    }

    request->send(response);
  });

  // ---- Captive portal detection ----
  // OS connectivity checks hit these fixed, well-known paths (any hostname,
  // since DNS resolves everything to us). Redirecting them to "/" makes the
  // OS recognize a captive portal and open it in a browser/portal webview.
  auto redirectToApp = [](AsyncWebServerRequest *request) {
    request->redirect("/");
  };
  server.on("/generate_204", HTTP_GET, redirectToApp);       // Android
  server.on("/gen_204", HTTP_GET, redirectToApp);            // Android
  server.on("/hotspot-detect.html", HTTP_GET, redirectToApp); // iOS/macOS
  server.on("/connecttest.txt", HTTP_GET, redirectToApp);    // Windows
  server.on("/ncsi.txt", HTTP_GET, redirectToApp);           // Windows
  server.on("/success.txt", HTTP_GET, redirectToApp);        // ChromeOS/other

  // Any other unmatched path falls back to the app so the portal still
  // opens, except unknown /api/ routes which must surface as real 404s.
  server.onNotFound([redirectToApp](AsyncWebServerRequest *request) {
    if (request->url().startsWith("/api/")) {
      request->send(404, "application/json", "{\"error\":\"Not found\"}");
      return;
    }
    redirectToApp(request);
  });

  server.begin();
}

// ---------- Setup / loop --------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

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

  while ((uint32_t)(now - lastMeasurementMs) >= MEASUREMENT_INTERVAL_MS) {
    lastMeasurementMs += MEASUREMENT_INTERVAL_MS;
    sampleFlow(lastMeasurementMs);
  }

  if (now - lastFlushMs >= FLUSH_INTERVAL_MS) {
    lastFlushMs = now;
    flushBufferToSD();
  }
}