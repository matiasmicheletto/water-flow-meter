import os
import random
import threading
import time
from pathlib import Path
from typing import Dict, List

from fastapi import FastAPI
from fastapi.responses import PlainTextResponse
from fastapi.staticfiles import StaticFiles

MEASUREMENT_INTERVAL_MS = 10_000
HISTORY_INTERVAL_MS = 60_000


class FlowSimulator:
    def __init__(self, initial_minutes: int = 0, seed: int = 42) -> None:
        self._rng = random.Random(seed)
        self._lock = threading.Lock()

        self._boot_monotonic = time.monotonic() - (initial_minutes * 60.0)
        self._last_window_index = 0

        self._total_pulses = 0
        self._latest_pulse_count_10s = 0
        self._latest_pulses_per_minute = 0
        self._has_measurement = False

        self._minute_window_count = 0
        self._minute_pulse_sum = 0

        self._records_10s: List[Dict[str, int]] = []
        self._minute_points: List[Dict[str, int]] = []

        self._target_ppm = 90.0
        self._drift_step = 0

    def _elapsed_ms_now(self) -> int:
        return int((time.monotonic() - self._boot_monotonic) * 1000)

    def _next_window_pulse_count(self) -> int:
        if self._drift_step <= 0:
            self._target_ppm += self._rng.uniform(-20.0, 20.0)
            self._target_ppm = max(0.0, min(1500.0, self._target_ppm))
            self._drift_step = self._rng.randint(3, 12)
        else:
            self._drift_step -= 1

        if self._rng.random() < 0.04:
            self._target_ppm += self._rng.uniform(-80.0, 120.0)
            self._target_ppm = max(0.0, min(2000.0, self._target_ppm))

        expected_10s = self._target_ppm / 6.0
        pulses = int(round(expected_10s + self._rng.gauss(0.0, 1.8)))
        return max(0, pulses)

    def _append_measurement_window(self, window_index: int) -> None:
        elapsed_ms = window_index * MEASUREMENT_INTERVAL_MS
        pulse_count_10s = self._next_window_pulse_count()
        pulses_per_minute = pulse_count_10s * 6

        self._total_pulses += pulse_count_10s
        self._latest_pulse_count_10s = pulse_count_10s
        self._latest_pulses_per_minute = pulses_per_minute
        self._has_measurement = True

        self._records_10s.append(
            {
                "t_ms": elapsed_ms,
                "pulse_count_10s": pulse_count_10s,
                "pulses_per_minute": pulses_per_minute,
                "total_pulses": self._total_pulses,
            }
        )

        self._minute_window_count += 1
        self._minute_pulse_sum += pulse_count_10s

        if self._minute_window_count == 6:
            self._minute_points.append(
                {
                    "t_ms": elapsed_ms,
                    "pulses_per_minute": self._minute_pulse_sum,
                }
            )
            self._minute_window_count = 0
            self._minute_pulse_sum = 0

    def advance_to_now(self) -> None:
        with self._lock:
            elapsed_ms = self._elapsed_ms_now()
            window_index_now = elapsed_ms // MEASUREMENT_INTERVAL_MS

            while self._last_window_index < window_index_now:
                self._last_window_index += 1
                self._append_measurement_window(self._last_window_index)

    def snapshot_current(self) -> Dict[str, int | bool]:
        self.advance_to_now()
        with self._lock:
            return {
                "elapsed_ms": self._elapsed_ms_now(),
                "pulse_count_10s": self._latest_pulse_count_10s,
                "pulses_per_minute": self._latest_pulses_per_minute,
                "total_pulses": self._total_pulses,
                "measurement_interval_ms": MEASUREMENT_INTERVAL_MS,
                "has_measurement": self._has_measurement,
                "sd_ready": True,
            }

    def snapshot_history(self) -> Dict[str, object]:
        self.advance_to_now()
        with self._lock:
            payload: Dict[str, object] = {
                "interval_ms": HISTORY_INTERVAL_MS,
                "measurement_interval_ms": MEASUREMENT_INTERVAL_MS,
                "points": list(self._minute_points),
            }

            if self._minute_window_count > 0:
                minute_index = self._last_window_index // 6
                payload["partial_minute"] = {
                    "t_ms": (minute_index + 1) * HISTORY_INTERVAL_MS,
                    "samples": self._minute_window_count,
                    "pulse_count_sum": self._minute_pulse_sum,
                }

            return payload

    def export_csv(self) -> str:
        self.advance_to_now()
        with self._lock:
            lines = ["t_ms,pulse_count_10s,pulses_per_minute,total_pulses"]
            lines.extend(
                f"{r['t_ms']},{r['pulse_count_10s']},{r['pulses_per_minute']},{r['total_pulses']}"
                for r in self._records_10s
            )
            return "\n".join(lines) + "\n"


initial_minutes = int(os.getenv("SIM_INITIAL_MINUTES", "0"))
seed = int(os.getenv("SIM_RANDOM_SEED", "42"))
sim = FlowSimulator(initial_minutes=initial_minutes, seed=seed)
app = FastAPI(title="Water Flow Meter Simulator")


@app.get("/api/current")
def api_current() -> Dict[str, int | bool]:
    return sim.snapshot_current()


@app.get("/api/history")
def api_history() -> Dict[str, object]:
    return sim.snapshot_history()


@app.get("/api/export.csv", response_class=PlainTextResponse)
def api_export_csv() -> PlainTextResponse:
    body = sim.export_csv()
    headers = {"Content-Disposition": "attachment; filename=flow-history.csv"}
    return PlainTextResponse(content=body, media_type="text/csv", headers=headers)


project_root = Path(__file__).resolve().parents[1]
web_root = project_root / "data"
app.mount("/", StaticFiles(directory=str(web_root), html=True), name="dashboard")
