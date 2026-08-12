#!/usr/bin/env python3
"""Summarize an AlwaysbeA calibration capture without clinical inference.

The report is intentionally descriptive: ranges, timing/data quality, dominant
axis clues, and active-mode PWM safety consistency. It does not select patient
thresholds or claim gait/fall accuracy.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Iterable


SCHEMA = "alwaysbea-calibration-capture-v1"
REQUIRED_COLUMNS = {
    "timestamp_ms", "elapsed_ms", "sequence", "roll_deg", "pitch_deg",
    "yaw_deg", "motion_deg", "lateral_deg", "gx_dps", "gy_dps", "gz_dps",
    "ax_g", "ay_g", "az_g", "pitch_out", "roll_out", "t1_pulse_us",
    "t2_pulse_us", "t3_pulse_us", "mode", "imu_valid", "estop", "stabilize",
}
FLOAT_COLUMNS = {
    "roll_deg", "pitch_deg", "yaw_deg", "motion_deg", "lateral_deg",
    "gx_dps", "gy_dps", "gz_dps", "ax_g", "ay_g", "az_g",
    "pitch_out", "roll_out",
}
INT_COLUMNS = REQUIRED_COLUMNS - FLOAT_COLUMNS


class CaptureError(ValueError):
    pass


def _number(value: str, name: str, line: int) -> float | int:
    try:
        return float(value) if name in FLOAT_COLUMNS else int(value)
    except (TypeError, ValueError) as exc:
        raise CaptureError(f"line {line}: invalid {name}={value!r}") from exc


def load_capture(path: Path) -> tuple[dict[str, str], list[dict[str, float | int]]]:
    metadata: dict[str, str] = {}
    data_lines: list[str] = []
    try:
        text = path.read_text(encoding="utf-8-sig")
    except UnicodeDecodeError as exc:
        raise CaptureError("CSV must be UTF-8/UTF-8-BOM") from exc

    metadata_done = False
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if not metadata_done and line.startswith("#"):
            body = line[1:].strip()
            for item in body.split(","):
                if "=" in item:
                    key, value = item.split("=", 1)
                    metadata[key.strip()] = value.strip()
            continue
        metadata_done = True
        data_lines.append(raw)

    if metadata.get("schema") != SCHEMA:
        raise CaptureError(
            f"unsupported or missing schema: {metadata.get('schema')!r}"
        )
    if not data_lines:
        raise CaptureError("CSV contains no header/data")

    reader = csv.DictReader(data_lines)
    columns = set(reader.fieldnames or [])
    missing = sorted(REQUIRED_COLUMNS - columns)
    if missing:
        raise CaptureError("missing columns: " + ", ".join(missing))

    rows: list[dict[str, float | int]] = []
    for line_no, raw in enumerate(reader, start=2):
        rows.append({name: _number(raw[name], name, line_no) for name in REQUIRED_COLUMNS})
    if len(rows) < 2:
        raise CaptureError("capture needs at least 2 samples")
    return metadata, rows


def percentile(values: Iterable[float], q: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    pos = (len(ordered) - 1) * q
    lo, hi = math.floor(pos), math.ceil(pos)
    if lo == hi:
        return float(ordered[lo])
    return float(ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo))


def stats(values: Iterable[float]) -> dict[str, float]:
    data = [float(v) for v in values]
    return {
        "min": min(data),
        "max": max(data),
        "mean": statistics.fmean(data),
        "p95_abs": percentile((abs(v) for v in data), 0.95),
        "range": max(data) - min(data),
    }


def analyze(metadata: dict[str, str], rows: list[dict[str, float | int]]) -> dict[str, Any]:
    timestamps = [int(r["timestamp_ms"]) for r in rows]
    sequences = [int(r["sequence"]) for r in rows]
    positive_dt = [b - a for a, b in zip(timestamps, timestamps[1:]) if b > a]
    nonpositive_dt = sum(b <= a for a, b in zip(timestamps, timestamps[1:]))
    missing_seq = sum(max(0, b - a - 1) for a, b in zip(sequences, sequences[1:]) if b > a)
    nonmonotonic_seq = sum(b <= a for a, b in zip(sequences, sequences[1:]))
    duration_ms = timestamps[-1] - timestamps[0]
    measured_hz = (len(rows) - 1) * 1000.0 / duration_ms if duration_ms > 0 else 0.0

    axis = {name: stats(r[name] for r in rows) for name in (
        "roll_deg", "pitch_deg", "yaw_deg", "motion_deg", "lateral_deg",
        "gx_dps", "gy_dps", "gz_dps", "ax_g", "ay_g", "az_g",
        "pitch_out", "roll_out",
    )}
    motion_range = axis["motion_deg"]["range"]
    lateral_range = axis["lateral_deg"]["range"]
    raw_ranges = {name: axis[name]["range"] for name in ("roll_deg", "pitch_deg", "yaw_deg")}
    dominant_axis = max(raw_ranges, key=raw_ranges.get)
    dominant_ratio = raw_ranges[dominant_axis] / max(
        0.001, sum(v for k, v in raw_ranges.items() if k != dominant_axis)
    )

    accel_magnitude = [math.sqrt(float(r["ax_g"]) ** 2 + float(r["ay_g"]) ** 2 + float(r["az_g"]) ** 2) for r in rows]
    pwm = {name: stats(r[name] for r in rows) for name in (
        "t1_pulse_us", "t2_pulse_us", "t3_pulse_us"
    )}
    active_rows = [r for r in rows if int(r["mode"]) == 1]
    active_t2_violations = [r for r in active_rows if int(r["t2_pulse_us"]) != 1500]

    warnings: list[str] = []
    if nonpositive_dt:
        warnings.append(f"{nonpositive_dt} non-monotonic/duplicate timestamps")
    if nonmonotonic_seq:
        warnings.append(f"{nonmonotonic_seq} non-monotonic/duplicate sequence transitions")
    if missing_seq:
        warnings.append(f"{missing_seq} missing IMU sequence values")
    if any(int(r["imu_valid"]) != 1 for r in rows):
        warnings.append("one or more rows are marked IMU invalid")
    if any(int(r["estop"]) == 1 for r in rows):
        warnings.append("emergency stop was active in part of this capture")
    if active_t2_violations:
        warnings.append(
            f"active-mode T2 was not 1500 us in {len(active_t2_violations)} samples"
        )
    if measured_hz < 10.0:
        warnings.append(f"measured sample rate is low ({measured_hz:.1f} Hz)")
    if motion_range < 3.0 and metadata.get("label") not in {"neutral", "strap-loose"}:
        warnings.append("motion range is below 3 deg for a non-neutral label")
    if lateral_range > motion_range and metadata.get("label") in {"normal-lift", "small-lift", "fast-lift"}:
        warnings.append("lateral range exceeds motion range for a lift-labelled capture")

    return {
        "schema": metadata.get("schema"),
        "label": metadata.get("label", "unlabeled"),
        "exported_at": metadata.get("exported_at"),
        "device_boot": metadata.get("device_boot"),
        "samples": len(rows),
        "duration_ms": duration_ms,
        "measured_rate_hz": measured_hz,
        "timing": {
            "median_dt_ms": statistics.median(positive_dt) if positive_dt else 0.0,
            "p95_dt_ms": percentile(positive_dt, 0.95),
            "nonpositive_dt": nonpositive_dt,
            "missing_sequence_values": missing_seq,
            "nonmonotonic_sequence": nonmonotonic_seq,
        },
        "axis": axis,
        "accel_magnitude_g": stats(accel_magnitude),
        "pwm_us": pwm,
        "dominant_raw_angle_axis": dominant_axis,
        "dominant_axis_ratio": dominant_ratio,
        "active_mode": {
            "samples": len(active_rows),
            "t2_neutral_samples": len(active_rows) - len(active_t2_violations),
            "t2_violations": len(active_t2_violations),
            "t2_min_us": min((int(r["t2_pulse_us"]) for r in active_rows), default=0),
            "t2_max_us": max((int(r["t2_pulse_us"]) for r in active_rows), default=0),
        },
        "flags": {
            "imu_invalid_samples": sum(int(r["imu_valid"]) != 1 for r in rows),
            "estop_samples": sum(int(r["estop"]) == 1 for r in rows),
            "stabilize_enabled_samples": sum(int(r["stabilize"]) == 1 for r in rows),
        },
        "warnings": warnings,
        "disclaimer": "Descriptive engineering statistics only; not a clinical threshold or diagnosis.",
    }


def markdown(report: dict[str, Any], source: str) -> str:
    axis = report["axis"]
    active = report["active_mode"]
    pwm = report["pwm_us"]
    timing = report["timing"]
    warnings = report["warnings"]
    lines = [
        "# AlwaysbeA 标定采集客观统计",
        "",
        f"- 文件：`{source}`",
        f"- 标签：`{report['label']}`",
        f"- 样本：{report['samples']}；时长：{report['duration_ms'] / 1000:.2f} s；实测采样率：{report['measured_rate_hz']:.2f} Hz",
        f"- 时间间隔中位数/P95：{timing['median_dt_ms']:.1f}/{timing['p95_dt_ms']:.1f} ms",
        f"- 缺失序号：{timing['missing_sequence_values']}；时间戳非递增：{timing['nonpositive_dt']}",
        f"- 原始角度主变化轴：`{report['dominant_raw_angle_axis']}`；主轴比：{report['dominant_axis_ratio']:.2f}",
        "",
        "## 运动与信号范围",
        "",
        "| 信号 | 最小 | 最大 | 范围 | P95 绝对值 |",
        "|---|---:|---:|---:|---:|",
    ]
    for name in ("motion_deg", "lateral_deg", "roll_deg", "pitch_deg", "yaw_deg", "gx_dps", "gy_dps", "gz_dps"):
        s = axis[name]
        lines.append(f"| `{name}` | {s['min']:.3f} | {s['max']:.3f} | {s['range']:.3f} | {s['p95_abs']:.3f} |")
    lines.extend([
        "",
        "## 主动模式 T2 中位一致性",
        "",
        f"- 主动模式样本：{active['samples']}；T2=1500 μs：{active['t2_neutral_samples']}；违反：{active['t2_violations']}。",
        f"- T2 范围：{active['t2_min_us']}–{active['t2_max_us']} μs。",
        "",
        "| 通道 | 最小脉宽 | 最大脉宽 | 平均脉宽 |",
        "|---|---:|---:|---:|",
        f"| T1 | {pwm['t1_pulse_us']['min']:.0f} | {pwm['t1_pulse_us']['max']:.0f} | {pwm['t1_pulse_us']['mean']:.1f} |",
        f"| T2 | {pwm['t2_pulse_us']['min']:.0f} | {pwm['t2_pulse_us']['max']:.0f} | {pwm['t2_pulse_us']['mean']:.1f} |",
        f"| T3 | {pwm['t3_pulse_us']['min']:.0f} | {pwm['t3_pulse_us']['max']:.0f} | {pwm['t3_pulse_us']['mean']:.1f} |",
        "",
        "## 警告",
        "",
    ])
    lines.extend(f"- {w}" for w in warnings)
    if not warnings:
        lines.append("- 未发现格式、时间连续性或主动模式 T2 中位一致性警告。")
    lines.extend([
        "",
        "> 本报告只给出描述性工程统计，不自动选择患者阈值，也不构成临床诊断、疗效结论或跌倒安全证明。",
        "",
    ])
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="exported calibration capture CSV")
    parser.add_argument("--format", choices=("markdown", "json"), default="markdown")
    parser.add_argument("--output", type=Path, help="write report to a file instead of stdout")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        metadata, rows = load_capture(args.csv)
        report = analyze(metadata, rows)
    except (OSError, CaptureError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    output = json.dumps(report, ensure_ascii=False, indent=2) + "\n" if args.format == "json" else markdown(report, str(args.csv))
    if args.output:
        args.output.write_text(output, encoding="utf-8")
    else:
        sys.stdout.write(output)
    return 1 if report["warnings"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
