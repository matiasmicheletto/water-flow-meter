
# Water Flow Meter (ESP32-CAM)

This project samples a flow sensor as raw pulse counts (no unit conversion in firmware),
stores timestamped records, and hosts a mobile-first web dashboard.

## What It Does

- Samples pulses every second.
- Stores records as relative timestamps from measurement start.
- Flushes buffered records to SD card periodically.
- Hosts a dashboard over AP mode with:
	- Current pulse count (latest sample)
	- Historical chart
	- CSV export button (phone-friendly)

## Data Format

All records use this CSV schema:

```csv
t_ms,pulses_sample,pulses_total_acc
```

- `t_ms`: elapsed milliseconds since measurement start
- `pulses_sample`: pulses seen in the sampling window
- `pulses_total_acc`: cumulative pulse count since measurement start

## API Endpoints

- `GET /api/current`
	- Returns the current sample and status:
	- `t_ms`, `pulses_sample`, `pulses_total_acc`, `buffered`, `sd_ready`
- `GET /api/history`
	- Returns recent historical points for the dashboard chart.
- `GET /api/export.csv`
	- Streams CSV export (SD data + in-memory buffered records).

## Flashing And Running

1. Upload the hosted web files (LittleFS image) first:

```bash
pio run --target uploadfs --upload-port /dev/ttyUSB0
```

2. Upload firmware:

```bash
pio run --target upload --upload-port /dev/ttyUSB0
```

3. Open serial monitor:

```bash
pio device monitor --port /dev/ttyUSB0 --baud 115200
```

Alternative monitor:

```bash
picocom -b 115200 /dev/ttyUSB0
```

## Accessing The Dashboard

- Device starts an AP named `FlowMeter`.
- Connect your phone/laptop to that AP.
- Open `http://10.0.0.1/` in the browser.

The dashboard is mobile-first and includes an export button that will use native
share when supported (otherwise it falls back to file download).