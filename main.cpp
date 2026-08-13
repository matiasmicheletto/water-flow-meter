/*
 * Water flow meter - ESP32-CAM + YF-S401
 * Step 1: sensor sampling (1s) + buffered SD logging (flush ~60s)
 *         + AP mode + web server serving LittleFS index.html
 *         + /api/current JSON endpoint (polled by the page)
 *
 * Camera is NOT initialized -> its pins are free for SD_MMC + sensor.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <SD_MMC.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// ---------- Pin assignments (AI-Thinker ESP32-CAM, camera disabled) ------
#define FLOW_SENSOR_PIN   13   // interrupt-capable, free when camera is off
// SD_MMC 1-bit mode uses fixed pins on ESP32-CAM: CLK=14, CMD=15, D0=2
// (D1/D2/D3 normally used by camera - we skip them by using 1-bit mode)

// ---------- Flow sensor calibration --------------------------------------
// YF-S401: verify this constant against your specific unit's datasheet.
// Commonly cited around 5880 pulses/L for the small-bore YF-S401 (~98 Hz per L/min).
// CHANGE THIS to match your sensor's actual calibration sheet.
static const float PULSES_PER_LITER = 5880.0f;

// ---------- Timing ---------------------------------------------------------
static const uint32_t SAMPLE_INTERVAL_MS = 1000;   // measure every 1s
static const uint32_t FLUSH_INTERVAL_MS  = 60000;  // dump to SD every ~60s
static const size_t   BUFFER_CAPACITY    = 70;      // ~70s of margin before flush

// ---------- AP credentials ---------------------------------------------
static const char* AP_SSID = "FlowMeter";
static const char* AP_PASS = "flowmeter123"; // 8+ chars required by WiFi lib

// ---------- Shared state between ISR and loop -----------------------------
volatile uint32_t pulseCount = 0;

void IRAM_ATTR onPulse() {
  pulseCount++;
}

// ---------- Record buffer ---------------------------------------------
struct FlowRecord {
  uint32_t t_ms;       // millis() at time of sample (relative timestamp)
  float    flow_lpm;   // instantaneous flow, liters/minute
  float    liters_acc; // cumulative liters since boot
};

FlowRecord buffer[BUFFER_CAPACITY];
size_t bufferLen = 0;

float totalLiters = 0.0f;
float currentFlowLpm = 0.0f;   // latest reading, exposed via /api/current
uint32_t currentSampleTime = 0;

uint32_t lastSampleMs = 0;
uint32_t lastFlushMs = 0;

bool sdReady = false;
const char* LOG_PATH = "/flow_log.csv";

AsyncWebServer server(80);

// ---------- SD helpers ---------------------------------------------------
void ensureLogHeader() {
  if (!SD_MMC.exists(LOG_PATH)) {
    File f = SD_MMC.open(LOG_PATH, FILE_WRITE);
    if (f) {
      f.println("t_ms,flow_lpm,liters_acc");
      f.close();
    }
  }
}

void flushBufferToSD() {
  if (!sdReady || bufferLen == 0) return;

  File f = SD_MMC.open(LOG_PATH, FILE_APPEND);
  if (!f) {
    Serial.println("[SD] failed to open log for append");
    return;
  }
  for (size_t i = 0; i < bufferLen; i++) {
    f.printf("%lu,%.3f,%.3f\n",
             buffer[i].t_ms, buffer[i].flow_lpm, buffer[i].liters_acc);
  }
  f.close();
  Serial.printf("[SD] flushed %u records\n", (unsigned)bufferLen);
  bufferLen = 0;
}

// ---------- Sensor sampling ---------------------------------------------
void sampleFlow() {
  noInterrupts();
  uint32_t pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  // pulses in this 1s window -> instantaneous flow rate (L/min)
  float litersThisSample = pulses / PULSES_PER_LITER;
  float flow_lpm = litersThisSample * 60.0f; // 1s window -> scale to per-minute

  totalLiters += litersThisSample;
  currentFlowLpm = flow_lpm;
  currentSampleTime = millis();

  if (bufferLen < BUFFER_CAPACITY) {
    buffer[bufferLen].t_ms = currentSampleTime;
    buffer[bufferLen].flow_lpm = flow_lpm;
    buffer[bufferLen].liters_acc = totalLiters;
    bufferLen++;
  } else {
    // buffer full before scheduled flush - flush now to avoid losing data
    flushBufferToSD();
    buffer[bufferLen].t_ms = currentSampleTime;
    buffer[bufferLen].flow_lpm = flow_lpm;
    buffer[bufferLen].liters_acc = totalLiters;
    bufferLen++;
  }
}

// ---------- Web server routes --------------------------------------------
void setupServer() {
  if (!LittleFS.begin(true)) {
    Serial.println("[LittleFS] mount failed");
  }

  // Serve the frontend from LittleFS (data/index.html -> uploaded via
  // `pio run --target uploadfs`), fully decoupled from firmware logic.
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // Current reading, polled by the page every second.
  server.on("/api/current", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["t_ms"] = currentSampleTime;
    doc["flow_lpm"] = currentFlowLpm;
    doc["liters_acc"] = totalLiters;
    doc["buffered"] = bufferLen;
    doc["sd_ready"] = sdReady;

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  server.begin();
}

// ---------- Setup / loop --------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), onPulse, RISING);

  // SD in 1-bit mode: frees D1/D2/D3 lines that camera would otherwise use.
  if (SD_MMC.begin("/sdcard", true)) {
    sdReady = true;
    ensureLogHeader();
    Serial.println("[SD] mounted ok (1-bit mode)");
  } else {
    Serial.println("[SD] mount failed - logging disabled, AP+API still work");
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("[AP] started, IP: ");
  Serial.println(WiFi.softAPIP());

  setupServer();

  lastSampleMs = millis();
  lastFlushMs = millis();
}

void loop() {
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
