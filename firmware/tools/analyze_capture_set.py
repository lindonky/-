#!/usr/bin/env python3
"""Compare several AlwaysbeA calibration captures on a common baseline.

This tool replays engineering fall-detector gates and reports separability. It
does not estimate clinical sensitivity/specificity or select patient limits.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Callable

from analyze_calibration_capture import CaptureError, load_capture, percentile


FALL_ANGLE_TRIGGER_DEG = 65.0
FALL_ANGLE_HOLD_MS = 400
FALL_ACCEL_LOW_G = 0.55
FALL_ACCEL_HIGH_G = 1.60


def _median_dt(rows: list[dict[str, float | int]]) -> float:
    dt = [
        int(b["timestamp_ms"]) - int(a["timestamp_ms"])
        for a, b in zip(rows, rows[1:])
        if int(b["timestamp_ms"]) > int(a["timestamp_ms"])
    ]
    return float(statistics.median(dt)) if dt else 0.0


def _max_run_ms(
    samples: list[dict[str, float]], predicate: Callable[[dict[str, float]], bool],
    sample_period_ms: float,
) -> float:
    started: float | None = None
    last: float | None = None
    best = 0.0
    for sample in samples:
        now = sample["timestamp_ms"]
        if predicate(sample):
            if started is None:
                started = now
            last = now
        elif started is not None and last is not None:
            best = max(best, last - started + sample_period_ms)
            started = last = None
    if started is not None and last is not None:
        best = max(best, last - started + sample_period_ms)
    return best


def _replay_two_method(samples: list[dict[str, float]]) -> dict[str, Any]:
    angle_started: float | None = None
    candidate_reason = "none"
    confirmed_at: float | None = None

    for sample in samples:
        now = sample["timestamp_ms"]
        if sample["accel_mag_g"] <= FALL_ACCEL_LOW_G or sample["accel_mag_g"] >= FALL_ACCEL_HIGH_G:
            candidate_reason = "acceleration"
            confirmed_at = now
            break
        if sample["tilt_deg"] >= FALL_ANGLE_TRIGGER_DEG:
            if angle_started is None:
                angle_started = now
            elif now - angle_started >= FALL_ANGLE_HOLD_MS:
                candidate_reason = "angle"
                confirmed_at = now
                break
        else:
            angle_started = None

    return {
        "confirmed": confirmed_at is not None,
        "confirmed_elapsed_ms": None if confirmed_at is None else int(confirmed_at - samples[0]["timestamp_ms"]),
        "candidate_reason": candidate_reason,
    }


def analyze_one(path: Path) -> dict[str, Any]:
    metadata, rows = load_capture(path)
    start = int(rows[0]["timestamp_ms"])
    # Firmware snapshots the first valid sample at TRAIN START, so the safety
    # replay must use the same reference rather than a retrospective filter.
    baseline_pitch = float(rows[0]["motion_deg"])
    baseline_roll = float(rows[0]["lateral_deg"])
    samples: list[dict[str, float]] = []
    for row in rows:
        rel_pitch = float(row["motion_deg"]) - baseline_pitch
        rel_roll = float(row["lateral_deg"]) - baseline_roll
        accel_mag = math.sqrt(sum(float(row[name]) ** 2 for name in ("ax_g", "ay_g", "az_g")))
        gyro_mag = math.sqrt(sum(float(row[name]) ** 2 for name in ("gx_dps", "gy_dps", "gz_dps")))
        samples.append({
            "timestamp_ms": float(row["timestamp_ms"]),
            "elapsed_ms": float(int(row["timestamp_ms"]) - start),
            "rel_pitch_deg": rel_pitch,
            "rel_roll_deg": rel_roll,
            "tilt_deg": max(abs(rel_pitch), abs(rel_roll)),
            "accel_mag_g": accel_mag,
            "gyro_mag_dps": gyro_mag,
        })

    period = _median_dt(rows)
    angle_run = _max_run_ms(
        samples,
        lambda s: s["tilt_deg"] >= FALL_ANGLE_TRIGGER_DEG,
        period,
    )
    accel_trips = sum(
        s["accel_mag_g"] <= FALL_ACCEL_LOW_G or s["accel_mag_g"] >= FALL_ACCEL_HIGH_G
        for s in samples
    )
    peak = max(samples, key=lambda s: s["tilt_deg"])
    return {
        "file": path.name,
        "label": metadata.get("label", "unlabeled"),
        "samples": len(rows),
        "duration_ms": int(rows[-1]["timestamp_ms"]) - start,
        "rate_hz": (len(rows) - 1) * 1000.0 / (int(rows[-1]["timestamp_ms"]) - start),
        "baseline_motion_deg": baseline_pitch,
        "baseline_lateral_deg": baseline_roll,
        "motion_range_rel_deg": max(s["rel_pitch_deg"] for s in samples) - min(s["rel_pitch_deg"] for s in samples),
        "lateral_range_rel_deg": max(s["rel_roll_deg"] for s in samples) - min(s["rel_roll_deg"] for s in samples),
        "peak_tilt_deg": peak["tilt_deg"],
        "peak_tilt_elapsed_ms": int(peak["elapsed_ms"]),
        "gyro_peak_dps": max(s["gyro_mag_dps"] for s in samples),
        "gyro_p95_dps": percentile((s["gyro_mag_dps"] for s in samples), 0.95),
        "accel_min_g": min(s["accel_mag_g"] for s in samples),
        "accel_max_g": max(s["accel_mag_g"] for s in samples),
        "angle_over_threshold_max_ms": angle_run,
        "acceleration_trip_samples": accel_trips,
        "two_method_replay": _replay_two_method(samples),
    }


def markdown(report: dict[str, Any]) -> str:
    lines = [
        "# AlwaysbeA 多标签标定对比",
        "",
        "| 标签 | 样本/Hz | 主轴范围 | 横向范围 | 峰值倾角 | 陀螺峰值 | 合加速度范围 | ≥65°最长 | 加速度越界点 | 双路回放 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for item in report["captures"]:
        lines.append(
            f"| `{item['label']}` | {item['samples']}/{item['rate_hz']:.2f} | "
            f"{item['motion_range_rel_deg']:.1f}° | {item['lateral_range_rel_deg']:.1f}° | "
            f"{item['peak_tilt_deg']:.1f}° | {item['gyro_peak_dps']:.1f}°/s | "
            f"{item['accel_min_g']:.2f}–{item['accel_max_g']:.2f} g | "
            f"{item['angle_over_threshold_max_ms'] / 1000:.1f} s | "
            f"{item['acceleration_trip_samples']} | "
            f"{'触发/' + item['two_method_replay']['candidate_reason'] if item['two_method_replay']['confirmed'] else '不触发'} |"
        )
    lines.extend([
        "",
        "双路回放参数：相对训练开始姿态的 pitch/roll 任一达到 65°并连续 0.4 s，或者合加速度 ≤0.55 g / ≥1.60 g；任一路确认即触发。",
        "",
        "> 单次工程样本只能验证这四段回放的可分性，不能给出临床敏感度、特异度或安全证明。",
        "",
    ])
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--format", choices=("markdown", "json"), default="markdown")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        report = {"captures": [analyze_one(path) for path in args.csv]}
    except (OSError, CaptureError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    sys.stdout.write(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n"
        if args.format == "json" else markdown(report)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
