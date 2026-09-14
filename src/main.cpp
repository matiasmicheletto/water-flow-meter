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
static const uint32_t MEASUREMENT_INTERVAL_MS = 10000; // 10-second windows
static const uint32_t HISTORY_INTERVAL_MS = 60000;     // 1 point per minute
static const uint32_t FLUSH_INTERVAL_MS = 10000;       // persist every 10-second window
static const size_t BUFFER_CAPACITY = 24;              // >4 minutes of 10s records

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

// ---------- Record buffer ---------------------------------------------
struct FlowRecord {
  uint32_t t_ms;             // elapsed since boot, recorded at end of 10s window
  uint32_t pulse_count_10s;  // pulses counted in this 10-second interval
  uint32_t pulses_per_minute;
  uint32_t total_pulses;     // cumulative pulses since boot
};

FlowRecord buffer[BUFFER_CAPACITY];
size_t bufferLen = 0;

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
  if (!sdReady || bufferLen == 0) return;

  xSemaphoreTake(sdMutex, portMAX_DELAY);
  File f = SD_MMC.open(LOG_PATH, FILE_APPEND);
  if (!f) {
    xSemaphoreGive(sdMutex);
    Serial.println("[SD] failed to open log for append");
    WebSerial.println("[SD] failed to open log for append");
    return;
  }
  for (size_t i = 0; i < bufferLen; i++) {
    f.printf("%lu,%lu,%lu,%lu\n",
             buffer[i].t_ms,
             buffer[i].pulse_count_10s,
             buffer[i].pulses_per_minute,
             buffer[i].total_pulses);
  }
  f.close();
  xSemaphoreGive(sdMutex);

  Serial.printf("[SD] flushed %u records\n", (unsigned)bufferLen);
  WebSerial.printf("[SD] flushed %u records\n", (unsigned)bufferLen);
  bufferLen = 0;
}

// ---------- Sensor sampling ---------------------------------------------
void sampleFlow(uint32_t measurementElapsedMs) {
  noInterrupts();
  uint32_t pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  uint32_t pulsesPerMinute = pulses * 6;

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

  if (bufferLen < BUFFER_CAPACITY) {
    buffer[bufferLen++] = record;
  } else {
    // buffer full before scheduled flush - flush now to avoid losing data
    flushBufferToSD();
    if (bufferLen < BUFFER_CAPACITY) {
      buffer[bufferLen++] = record;
    }
  }
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
    String requestedFile = LOG_PATH;
    if (request->hasParam("file")) {
      requestedFile = request->getParam("file")->value();
    }
    bool isCurrentFile = (requestedFile == LOG_PATH);

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

      if (minuteSampleCount >= 6) {
        emitMinute(minuteIndex, minutePulseSum);
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

    if (isCurrentFile) {
      for (size_t i = 0; i < bufferLen; i++) {
        consumeRecord(buffer[i]);
      }
    }
    xSemaphoreGive(sdMutex);

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
    }
    bool isCurrentFile = (requestedFile == LOG_PATH);

    if (!sdReady) {
      request->send(503, "text/plain", "SD card not available");
      return;
    }

    String fileName = requestedFile;
    fileName.replace("/", "");

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
      for (size_t i = 0; i < bufferLen; i++) {
        response->printf("%lu,%lu,%lu,%lu\n",
                         buffer[i].t_ms,
                         buffer[i].pulse_count_10s,
                         buffer[i].pulses_per_minute,
                         buffer[i].total_pulses);
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

  // Any other unmatched path (e.g. a probe using an unlisted URL/hostname)
  // falls back to the app instead of a bare 404, so the portal still opens.
  server.onNotFound(redirectToApp);

  server.begin();
}

// ---------- Setup / loop --------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  sdMutex = xSemaphoreCreateMutex();

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
  WiFi.softAPConfig(local_IP, gateway, subnet); // URL is http://10.0.0.1:8080
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

  Serial.print("[AP] started, IP: ");
  WebSerial.print("[AP] started, IP: ");
  Serial.println(WiFi.softAPIP());
  WebSerial.println(WiFi.softAPIP());

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