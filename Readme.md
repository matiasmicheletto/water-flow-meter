# Water Flow Meter (ESP32-CAM + YF-S401)

ESP32-CAM firmware and dashboard for pulse-based flow monitoring.

The firmware is the source of truth for pulse counting and flow-rate calculation:

- Pulses are counted in an interrupt on GPIO13.
- Pulses are measured in fixed 10-second windows.
- Instantaneous flow is reported as pulses/minute using:
	- `pulses_per_minute = pulse_count_10s * 6`
- Historical chart data is aggregated to one point per minute.

## Hardware

- Board: AI-Thinker ESP32-CAM
- Flow sensor: YF-S401
- Flow sensor pulse pin: GPIO13 (`FLOW_SENSOR_PIN`)
- Storage: microSD card via `SD_MMC` in 1-bit mode
- Frontend storage: `LittleFS` (`data/index.html`)

Notes:

- Camera is intentionally not initialized so sensor/SD pin usage is preserved.
- Sensor input uses `INPUT_PULLUP` and interrupt on rising edge.
- If pulse noise causes overcounting, tune `MIN_PULSE_INTERVAL_US` in firmware.

## Firmware Architecture

### Interrupt-based pulse counting

- ISR only does lightweight work:
	- Reads `micros()`
	- Applies minimum pulse interval filter (`MIN_PULSE_INTERVAL_US`)
	- Increments a volatile pulse counter
- No SD, JSON, serial logging, or heavy work inside ISR.

### 10-second measurement windows

Every `10000 ms` in the main loop:

1. Atomically read/reset interrupt pulse counter.
2. Compute `pulses_per_minute = pulse_count_10s * 6`.
3. Update cumulative `total_pulses`.
4. Append one 10-second record to in-memory buffer.

10-second CSV/log record schema:

```csv
t_ms,pulse_count_10s,pulses_per_minute,total_pulses
```

Where `t_ms` is elapsed milliseconds since ESP32 boot.

### Minute-level historical aggregation

- Historical chart points are one point per minute.
- Each minute point is aggregated from six 10-second windows:
	- `minute_pulses_per_minute = sum(pulse_count_10s over 6 windows)`
- Partial minute at the end of session is not emitted as a chart point.
	- It is exposed as `partial_minute` in `/api/history`.

### SD logging

- 10-second records are buffered in RAM and flushed periodically to SD.
- SD remains the persistent source for full session history.
- `/api/history` and `/api/export.csv` merge SD data plus current in-memory buffer.
- No sliding-window truncation of history.

## Dashboard

The dashboard (served from LittleFS) provides three primary cards:

1. Current flow (`pulses/min`) from latest completed 10-second measurement.
2. Elapsed time since ESP32 boot (`HH:MM:SS`).
3. Total pulses since boot (SI-formatted: `k`, `M`, `B`).

Historical chart:

- X axis: elapsed time (`t_ms`) since reset
- Y axis: `pulses/min`
- One historical point per minute
- Complete session history from startup
- Frontend-only deterministic subsampling for rendering very large histories
	- First and last points are preserved
	- Underlying API history remains intact

Polling behavior:

- Current metrics: every 2 seconds
- History: every 10 seconds
- Elapsed card updates every second between polls for smoother UX

## API

### `GET /api/current`

Example:

```json
{
	"elapsed_ms": 123450,
	"pulse_count_10s": 15,
	"pulses_per_minute": 90,
	"total_pulses": 12345,
	"measurement_interval_ms": 10000,
	"has_measurement": true,
	"sd_ready": true
}
```

### `GET /api/history`

Example:

```json
{
	"interval_ms": 60000,
	"measurement_interval_ms": 10000,
	"points": [
		{ "t_ms": 60000, "pulses_per_minute": 72 },
		{ "t_ms": 120000, "pulses_per_minute": 84 }
	],
	"partial_minute": {
		"t_ms": 180000,
		"samples": 2,
		"pulse_count_sum": 19
	}
}
```

`partial_minute` is optional and appears only when the current minute is incomplete.

### `GET /api/export.csv`

Exports complete 10-second measurement history (SD + RAM buffer):

```csv
t_ms,pulse_count_10s,pulses_per_minute,total_pulses
```

## Running On Hardware (PlatformIO)

Project config is `esp32cam` + Arduino framework + `115200` monitor/upload speed + LittleFS.

From repository root:

1. Build firmware:

```bash
/home/matias/.platformio/penv/bin/platformio run
```

2. Upload firmware:

```bash
/home/matias/.platformio/penv/bin/platformio run --target upload --upload-port /dev/ttyUSB0
```

3. Upload LittleFS dashboard files:

```bash
/home/matias/.platformio/penv/bin/platformio run --target uploadfs --upload-port /dev/ttyUSB0
```

4. Open serial monitor:

```bash
/home/matias/.platformio/penv/bin/platformio device monitor --port /dev/ttyUSB0 --baud 115200
```

If upload fails, set ESP32-CAM in flashing mode (IO0 to GND or hold IO0 button as required by your adapter) and retry.

## Accessing The Dashboard On Device

- Connect to AP: `FlowMeter`
- Password: `flowmeter123`
- Open: `http://10.0.0.1/`

## Running The Simulator

The simulator exposes the same API contract and serves the same dashboard UI.

From repository root:

```bash
pip install -r requirements.txt
uvicorn simulator.main:app --reload
```

Open `http://127.0.0.1:8000/`.

Useful options:

- Start with long pre-generated history:

```bash
SIM_INITIAL_MINUTES=5000 uvicorn simulator.main:app --reload
```

- Reproducible simulation randomness:

```bash
SIM_RANDOM_SEED=123 uvicorn simulator.main:app --reload
```

Simulation behavior:

- Elapsed time continuously increases.
- 10-second measurements are generated over time.
- `pulses_per_minute` changes dynamically with noise and occasional shifts.
- Minute history grows from session start and is not a sliding window.