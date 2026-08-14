# Simulator

This FastAPI app simulates the ESP32 flow-meter API and serves the same dashboard UI from `data/index.html`.

## Run

From repository root:

```bash
pip install -r requirements.txt
uvicorn simulator.main:app --reload
```

Open `http://127.0.0.1:8000/`.

## API

- `GET /api/current`
- `GET /api/history`
- `GET /api/export.csv`

These routes match the firmware contract.

## Long-history testing

Set `SIM_INITIAL_MINUTES` before starting Uvicorn to start with pre-generated elapsed time/history.

Example:

```bash
SIM_INITIAL_MINUTES=5000 uvicorn simulator.main:app --reload
```

Optional deterministic randomness:

```bash
SIM_RANDOM_SEED=123 uvicorn simulator.main:app --reload
```
