/*
 * Water flow meter - ESP32-CAM + YF-S401
 * Step 1: sensor pulse sampling (1s) + buffered SD logging (flush ~60s)
 *         + AP mode + web server serving LittleFS index.html
 *         + JSON endpoints for current reading, history, and CSV export
 *
 * Camera is NOT initialized -> its pins are free for SD_MMC + sensor.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebSerial.h>
#include <SD_MMC.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// ---------- Pin assignments (AI-Thinker ESP32-CAM, camera disabled) ------
#define FLOW_SENSOR_PIN   13   // interrupt-capable, free when camera is off
// SD_MMC 1-bit mode uses fixed pins on ESP32-CAM: CLK=14, CMD=15, D0=2
// (D1/D2/D3 normally used by camera - we skip them by using 1-bit mode)

// ---------- Timing ---------------------------------------------------------
static const uint32_t SAMPLE_INTERVAL_MS = 1000;   // measure every 1s
static const uint32_t FLUSH_INTERVAL_MS  = 60000;  // dump to SD every ~60s
static const size_t   BUFFER_CAPACITY    = 70;      // ~70s of margin before flush
static const size_t   HISTORY_LIMIT      = 120;     // recent points served to the dashboard

// ---------- AP credentials ---------------------------------------------
static const char* AP_SSID = "FlowMeter";
static const char* AP_PASS = "flowmeter123"; // 8+ chars required by WiFi lib

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
  uint32_t t_ms;              // millis() at time of sample (relative timestamp)
  uint32_t pulses_sample;     // pulses counted during this sample window
  uint32_t pulses_total_acc;  // cumulative pulses since boot
};

FlowRecord buffer[BUFFER_CAPACITY];
size_t bufferLen = 0;

uint32_t totalPulses = 0;
uint32_t currentPulseCount = 0;   // latest reading, exposed via /api/current
uint32_t currentSampleTime = 0;
uint32_t measurementStartMs = 0;

uint32_t lastSampleMs = 0;
uint32_t lastFlushMs = 0;

bool sdReady = false;
const char* LOG_PATH = "/flow_log.csv";

AsyncWebServer server(80);

void appendHistoryRecord(FlowRecord *records, size_t &count, const FlowRecord &record) {
  if (count < HISTORY_LIMIT) {
    records[count++] = record;
    return;
  }

  memmove(records, records + 1, sizeof(FlowRecord) * (HISTORY_LIMIT - 1));
  records[HISTORY_LIMIT - 1] = record;
}

bool parseFlowRecordLine(const String &line, FlowRecord &record) {
  unsigned long tMs = 0;
  unsigned long pulsesSample = 0;
  unsigned long pulsesTotalAcc = 0;

  if (sscanf(line.c_str(), "%lu,%lu,%lu", &tMs, &pulsesSample, &pulsesTotalAcc) != 3) {
    return false;
  }

  record.t_ms = static_cast<uint32_t>(tMs);
  record.pulses_sample = static_cast<uint32_t>(pulsesSample);
  record.pulses_total_acc = static_cast<uint32_t>(pulsesTotalAcc);
  return true;
}

// ---------- SD helpers ---------------------------------------------------
void ensureLogHeader() {
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  if (!SD_MMC.exists(LOG_PATH)) {
    File f = SD_MMC.open(LOG_PATH, FILE_WRITE);
    if (f) {
      f.println("t_ms,pulses_sample,pulses_total_acc");
      f.close();
    }
  }
  xSemaphoreGive(sdMutex);
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
    f.printf("%lu,%lu,%lu\n",
             buffer[i].t_ms, buffer[i].pulses_sample, buffer[i].pulses_total_acc);
  }
  f.close();
  xSemaphoreGive(sdMutex);

  Serial.printf("[SD] flushed %u records\n", (unsigned)bufferLen);
  WebSerial.printf("[SD] flushed %u records\n", (unsigned)bufferLen);
  bufferLen = 0;
}

// ---------- Sensor sampling ---------------------------------------------
void sampleFlow() {
  noInterrupts();
  uint32_t pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  uint32_t sampleTimestampMs = millis();
  if (measurementStartMs == 0) {
    measurementStartMs = sampleTimestampMs;
  }

  totalPulses += pulses;
  currentPulseCount = pulses;
  currentSampleTime = sampleTimestampMs - measurementStartMs;

  if (bufferLen < BUFFER_CAPACITY) {
    buffer[bufferLen].t_ms = currentSampleTime;
    buffer[bufferLen].pulses_sample = pulses;
    buffer[bufferLen].pulses_total_acc = totalPulses;
    bufferLen++;
  } else {
    // buffer full before scheduled flush - flush now to avoid losing data
    flushBufferToSD();
    buffer[bufferLen].t_ms = currentSampleTime;
    buffer[bufferLen].pulses_sample = pulses;
    buffer[bufferLen].pulses_total_acc = totalPulses;
    bufferLen++;
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

  // Current reading, polled by the page every second.
  server.on("/api/current", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["t_ms"] = currentSampleTime;
    doc["pulses_sample"] = currentPulseCount;
    doc["pulses_total_acc"] = totalPulses;
    doc["buffered"] = bufferLen;
    doc["sd_ready"] = sdReady;

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  server.on("/api/history", HTTP_GET, [](AsyncWebServerRequest *request) {
    static FlowRecord history[HISTORY_LIMIT]; // static: keeps it off the AsyncTCP task's stack
    size_t historyCount = 0;

    xSemaphoreTake(sdMutex, portMAX_DELAY);
    if (sdReady && SD_MMC.exists(LOG_PATH)) {
      File f = SD_MMC.open(LOG_PATH, FILE_READ);
      if (f) {
        while (f.available()) {
          String line = f.readStringUntil('\n');
          line.trim();
          if (line.isEmpty() || line.startsWith("t_ms,")) {
            continue;
          }

          FlowRecord record;
          if (parseFlowRecordLine(line, record)) {
            appendHistoryRecord(history, historyCount, record);
          }
        }
        f.close();
      }
    }

    for (size_t i = 0; i < bufferLen; i++) {
      appendHistoryRecord(history, historyCount, buffer[i]);
    }
    xSemaphoreGive(sdMutex);

    JsonDocument doc;
    JsonArray points = doc["points"].to<JsonArray>();
    for (size_t i = 0; i < historyCount; i++) {
      JsonObject point = points.add<JsonObject>();
      point["t_ms"] = history[i].t_ms;
      point["pulses_sample"] = history[i].pulses_sample;
      point["pulses_total_acc"] = history[i].pulses_total_acc;
    }
    doc["sample_interval_ms"] = SAMPLE_INTERVAL_MS;
    doc["measurement_started"] = measurementStartMs != 0;

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  server.on("/api/export.csv", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/csv");
    response->addHeader("Content-Disposition", "attachment; filename=flow-history.csv");
    response->print("t_ms,pulses_sample,pulses_total_acc\n");

    xSemaphoreTake(sdMutex, portMAX_DELAY);
    if (sdReady && SD_MMC.exists(LOG_PATH)) {
      File f = SD_MMC.open(LOG_PATH, FILE_READ);
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

    for (size_t i = 0; i < bufferLen; i++) {
      response->printf("%lu,%lu,%lu\n",
                       buffer[i].t_ms,
                       buffer[i].pulses_sample,
                       buffer[i].pulses_total_acc);
    }

    request->send(response);
  });

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
    ensureLogHeader();
    Serial.println("[SD] mounted ok (1-bit mode)");
  } else {
    Serial.println("[SD] mount failed - logging disabled, AP+API still work");
  }

  WiFi.mode(WIFI_AP);
  IPAddress local_IP(10, 0, 0, 1);     // Target IP address
  IPAddress gateway(10, 0, 0, 1);      // Gateway (typically matches IP for AP)
  IPAddress subnet(255, 255, 255, 0);  // Subnet mask
  WiFi.softAPConfig(local_IP, gateway, subnet); // URL is http://10.0.0.1:8080
  WiFi.softAP(AP_SSID, AP_PASS);

  setupServer();

  if (sdReady) {
    WebSerial.println("[SD] mounted ok (1-bit mode)");
  } else {
    WebSerial.println("[SD] mount failed - logging disabled, AP+API still work");
  }

  Serial.print("[AP] started, IP: ");
  WebSerial.print("[AP] started, IP: ");
  Serial.println(WiFi.softAPIP());
  WebSerial.println(WiFi.softAPIP());

  lastSampleMs = millis();
  lastFlushMs = millis();
}

void loop() {
  WebSerial.loop();

  uint32_t now = millis();

  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    sampleFlow();
  }

  if (now - lastFlushMs >= FLUSH_INTERVAL_MS) {
    lastFlushMs = now;
    flushBufferToSD();
  }
}