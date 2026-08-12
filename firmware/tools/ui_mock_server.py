#!/usr/bin/env python3
"""Local ESP32 API simulator for repeatable UI development.

This server is intentionally kept outside the firmware build. It serves the
embedded ``web_page.html`` and implements the small subset of HTTP endpoints
used by the page, so the phone workflow can be tested without motors or an IMU.

Run from anywhere:
    python firmware/tools/ui_mock_server.py --port 8765
Then open http://127.0.0.1:8765/.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit


WEB_PAGE = Path(__file__).resolve().parents[1] / "components" / "comm" / "web_page.html"


class DeviceState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.boot_id = 20260812
        self.mode = 0
        self.stabilize = False
        self.estop = False
        self.height_cm = 170.0
        self.shank_override_cm = 0.0
        self.training_state = "idle"
        self.session_id = 0
        self.session_started = 0.0
        self.accumulated_ms = 0
        self.last_step_count = 0
        self.goal_enabled = False
        self.goal_rom_min_deg = 15.0
        self.goal_rom_max_deg = 45.0
        self.goal_cadence_spm = 30.0
        self.goal_cadence_tolerance_spm = 4.5
        self.capture_state = "empty"
        self.capture_label = "neutral"
        self.capture_started = 0.0
        self.capture_samples = 0

    def elapsed_ms(self) -> int:
        elapsed = self.accumulated_ms
        if self.training_state == "running":
            elapsed += int((time.monotonic() - self.session_started) * 1000)
        return max(0, elapsed)

    def pause_clock(self) -> None:
        if self.training_state == "running":
            self.accumulated_ms = self.elapsed_ms()

    def training_json(self) -> dict[str, object]:
        elapsed = self.elapsed_ms()
        if self.training_state == "running":
            steps = max(self.last_step_count, elapsed // 1800)
            self.last_step_count = steps
        else:
            steps = self.last_step_count
        completed = steps > 0
        shank_cm = self.shank_override_cm or self.height_cm * 0.2386
        rom = 29.6 if completed else 0.0
        cadence = 33.0 if completed else 0.0
        quality_flags = 0
        if completed and self.goal_enabled:
            if rom < self.goal_rom_min_deg:
                quality_flags |= 1
            if rom > self.goal_rom_max_deg:
                quality_flags |= 2
            if cadence < self.goal_cadence_spm - self.goal_cadence_tolerance_spm:
                quality_flags |= 128
            if cadence > self.goal_cadence_spm + self.goal_cadence_tolerance_spm:
                quality_flags |= 256
        step_cm = 2.0 * shank_cm * math.sin(math.radians(rom / 2.0)) if completed else 0.0
        phase = "settled"
        if self.training_state == "running":
            phase = ("lifting", "returning", "settled")[(elapsed // 600) % 3]
        return {
            "algo": "single-leg-mvp-0.1",
            "device_boot": self.boot_id,
            "state": self.training_state,
            "phase": phase,
            "id": self.session_id,
            "elapsed_ms": elapsed,
            "mode": self.mode,
            "steps": steps,
            "qualified": steps,
            "invalid": 0,
            "score_pct": 92.0 if completed else 0.0,
            "rom_last_deg": rom,
            "rom_avg_deg": rom,
            "peak_speed_dps": 54.2 if completed else 0.0,
            "cycle_s": 1.8 if completed else 0.0,
            "lift_s": 0.8 if completed else 0.0,
            "return_s": 1.0 if completed else 0.0,
            "cadence_spm": cadence,
            "lateral_deg": 2.4 if completed else 0.0,
            "return_error_deg": 1.2 if completed else 0.0,
            "height_cm": self.height_cm,
            "leg_cm_est": self.height_cm * 0.5199,
            "shank_cm": shank_cm,
            "shank_measured": bool(self.shank_override_cm),
            "step_cm_est": step_cm,
            "step_avg_cm_est": step_cm,
            "intervention_mean_pct": 12.0 if completed else 0.0,
            "intervention_peak_pct": 26.0 if completed else 0.0,
            "correction_load_index": 8.5 if completed else 0.0,
            "quality_flags": quality_flags,
            "goal_enabled": self.goal_enabled,
            "goal_rom_min_deg": self.goal_rom_min_deg,
            "goal_rom_max_deg": self.goal_rom_max_deg,
            "goal_cadence_spm": self.goal_cadence_spm,
            "goal_cadence_tolerance_spm": self.goal_cadence_tolerance_spm,
            "fall_stage": 0,
            "fall_events": 0,
            "accel_g": 1.01,
            "gyro_dps": 12.4 if self.training_state == "running" else 0.2,
            "fall_tilt_deg": 0.0,
            "event": "mock",
            "limits": {
                "lift_start_deg": 8,
                "rom_min_deg": 15,
                "rom_max_deg": 45,
                "rom_limit_deg": 60,
                "speed_min_dps": 10,
                "speed_max_dps": 90,
                "cycle_min_ms": 800,
                "cycle_max_ms": 8000,
                "lateral_max_deg": 8,
                "return_deg": 5,
            },
        }

    def status_json(self) -> dict[str, object]:
        elapsed = self.elapsed_ms()
        moving = self.training_state == "running"
        pitch = 18.0 * max(0.0, math.sin(elapsed / 900.0 * math.pi)) if moving else 0.0
        roll = 2.4 * math.sin(elapsed / 700.0) if moving else 0.0
        correction = int(round(roll * 18.0)) if self.stabilize and not self.estop else 0
        t1 = max(-1000, min(1000, -correction))
        t3 = max(-1000, min(1000, correction))
        return {
            "cal": 0,
            "r": round(roll, 1),
            "p": round(pitch, 1),
            "y": 0.8,
            "g": 44.0 if moving else 0.2,
            "gx": round(roll * 4.0, 1),
            "gy": 44.0 if moving else 0.2,
            "gz": 1.0,
            "ax": 0.02,
            "ay": 0.01,
            "az": 1.01,
            "seq": elapsed // 20,
            "age": 8,
            "imu": 1,
            "stab": int(self.stabilize),
            "mde": self.mode,
            "es": int(self.estop),
            "po": 0,
            "ro": correction,
            "stop": 0,
            "m": [
                {"s": t1, "c": 1500 + t1 // 2, "t": 1500 + t1 // 2},
                {"s": 0, "c": 1500, "t": 1500},
                {"s": t3, "c": 1500 + t3 // 2, "t": 1500 + t3 // 2},
            ],
        }

    def capture_json(self) -> dict[str, object]:
        if self.capture_state == "recording":
            self.capture_samples = min(1200, int((time.monotonic() - self.capture_started) * 50))
            if self.capture_samples >= 1200:
                self.capture_state = "ready"
        duration_ms = self.capture_samples * 20
        return {
            "state": self.capture_state,
            "label": self.capture_label,
            "samples": self.capture_samples,
            "capacity": 1200,
            "duration_ms": duration_ms,
            "rate_hz": 50.0 if self.capture_samples else 0.0,
            "full": int(self.capture_samples >= 1200),
            "download_ready": int(self.capture_state == "ready" and self.capture_samples > 0),
        }

    def command(self, command: str) -> str:
        parts = command.strip().split()
        upper = [p.upper() for p in parts]
        training_locked = self.training_state in {"running", "paused"}
        if not parts:
            return "ERR:empty"
        if upper[0] == "STOP":
            self.pause_clock()
            self.estop = True
            self.stabilize = False
            if self.training_state == "running":
                self.training_state = "paused"
            return "STOP_OK"
        if upper[0] == "START":
            self.estop = False
            return "START_OK"
        if upper[0] == "MODE" and len(parts) > 1:
            if training_locked:
                return "MODE_ERR:training_active"
            modes = {"ASSIST": 0, "ACTIVE": 1, "IMPEDANCE": 2}
            self.mode = modes.get(upper[1], self.mode)
            return "MODE_OK"
        if upper[0] == "STAB" and len(parts) > 1:
            if training_locked:
                return "STAB_ERR:training_active"
            if upper[1] == "ON":
                self.stabilize = True
            elif upper[1] == "OFF":
                self.stabilize = False
            return "STAB_OK"
        if upper[:2] == ["TRAIN", "HEIGHT"] and len(parts) > 2:
            if training_locked:
                return "TRAIN_ERR:training_active"
            self.height_cm = float(parts[2])
            return "TRAIN_OK:height"
        if upper[:2] == ["TRAIN", "SHANK"] and len(parts) > 2:
            if training_locked:
                return "TRAIN_ERR:training_active"
            self.shank_override_cm = float(parts[2])
            return "TRAIN_OK:shank"
        if upper[:2] == ["TRAIN", "GOAL"]:
            if training_locked:
                return "TRAIN_ERR:training_active"
            if len(parts) == 3 and upper[2] == "OFF":
                self.goal_enabled = False
                return "TRAIN_OK:goal=off"
            if len(parts) != 5:
                return "TRAIN_ERR:goal_range"
            rom_min = float(parts[2])
            rom_max = float(parts[3])
            cadence = float(parts[4])
            if rom_min < 8.0 or rom_max > 60.0 or rom_max - rom_min < 2.0:
                return "TRAIN_ERR:goal_range"
            if cadence < 5.0 or cadence > 75.0:
                return "TRAIN_ERR:goal_range"
            self.goal_enabled = True
            self.goal_rom_min_deg = rom_min
            self.goal_rom_max_deg = rom_max
            self.goal_cadence_spm = cadence
            self.goal_cadence_tolerance_spm = max(3.0, cadence * 0.15)
            return "TRAIN_OK:goal"
        if upper[:2] == ["TRAIN", "START"]:
            if self.estop:
                return "TRAIN_ERR:estop"
            self.session_id += 1
            self.training_state = "running"
            self.session_started = time.monotonic()
            self.accumulated_ms = 0
            self.last_step_count = 0
            self.mode = 1
            return "TRAIN_OK:running"
        if upper[:2] == ["TRAIN", "PAUSE"]:
            self.pause_clock()
            self.training_state = "paused"
            return "TRAIN_OK:paused"
        if upper[:2] == ["TRAIN", "RESUME"]:
            if self.estop:
                return "TRAIN_ERR:estop"
            self.training_state = "running"
            self.session_started = time.monotonic()
            return "TRAIN_OK:running"
        if upper[:2] == ["TRAIN", "STOP"]:
            self.pause_clock()
            self.training_state = "finished"
            return "TRAIN_OK:finished"
        if upper[:2] == ["TRAIN", "RESET"]:
            self.training_state = "idle"
            self.accumulated_ms = 0
            self.last_step_count = 0
            return "TRAIN_OK:reset"
        if upper[:2] == ["CAPTURE", "START"]:
            self.capture_label = parts[2] if len(parts) > 2 else "neutral"
            self.capture_state = "recording"
            self.capture_started = time.monotonic()
            self.capture_samples = 0
            return "CAPTURE_OK:recording"
        if upper[:2] == ["CAPTURE", "STOP"]:
            self.capture_json()
            self.capture_state = "ready" if self.capture_samples else "empty"
            return "CAPTURE_OK:ready" if self.capture_samples else "CAPTURE_OK:empty"
        if upper[:2] == ["CAPTURE", "CLEAR"]:
            self.capture_state = "empty"
            self.capture_samples = 0
            return "CAPTURE_OK:empty"
        return "MOCK_OK"


STATE = DeviceState()


class Handler(BaseHTTPRequestHandler):
    server_version = "AlwaysbeA-UI-Mock/1.0"

    def log_message(self, fmt: str, *args: object) -> None:
        return

    def send_bytes(self, body: bytes, content_type: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, value: object) -> None:
        self.send_bytes(json.dumps(value, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        path = urlsplit(self.path).path
        with STATE.lock:
            if path == "/":
                self.send_bytes(WEB_PAGE.read_bytes(), "text/html; charset=utf-8")
            elif path == "/api/status":
                self.send_json(STATE.status_json())
            elif path == "/api/training/status":
                self.send_json(STATE.training_json())
            elif path == "/api/capture/status":
                self.send_json(STATE.capture_json())
            elif path == "/api/capture.csv":
                rows = (
                    "\ufeff# schema=alwaysbea-calibration-capture-v1\r\n"
                    "# label=mock\r\n"
                    "timestamp_ms,elapsed_ms,sequence,roll_deg,pitch_deg\r\n"
                    "1000,0,1,0.00,0.00\r\n"
                ).encode("utf-8")
                self.send_bytes(rows, "text/csv; charset=utf-8")
            else:
                self.send_bytes(b"not found", "text/plain; charset=utf-8", 404)

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        path = urlsplit(self.path).path
        if path != "/api/cmd":
            self.send_bytes(b"not found", "text/plain; charset=utf-8", 404)
            return
        length = int(self.headers.get("Content-Length", "0"))
        command = self.rfile.read(length).decode("utf-8", errors="replace")
        with STATE.lock:
            response = STATE.command(command)
        self.send_bytes(response.encode("utf-8"), "text/plain; charset=utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve the AlwaysbeA UI with a mock ESP32 API")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    if not WEB_PAGE.is_file():
        raise SystemExit(f"UI not found: {WEB_PAGE}")
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"AlwaysbeA UI mock: http://{args.host}:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
