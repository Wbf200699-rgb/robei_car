#!/usr/bin/env python3
"""Analyze a test-only UWB JSONL capture and generate reproducible evidence."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
from typing import Any, Iterable


SCHEMA = "little_car_uwb_test_v1"
COUNTER_FIELDS = (
    "good_frames",
    "crc_errors",
    "uwb_tx",
    "queue_drops",
    "rx_bytes",
    "framing_errors",
    "position_frames",
    "pass_frames",
    "raw_frames",
    "position_start_requests",
)


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def percentile(values: Iterable[float], percent: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def numeric_stats(values: Iterable[float]) -> dict[str, float | int | None]:
    numbers = [float(value) for value in values if math.isfinite(float(value))]
    if not numbers:
        return {
            "count": 0,
            "minimum": None,
            "mean": None,
            "median": None,
            "p95": None,
            "maximum": None,
        }
    return {
        "count": len(numbers),
        "minimum": min(numbers),
        "mean": statistics.fmean(numbers),
        "median": statistics.median(numbers),
        "p95": percentile(numbers, 95.0),
        "maximum": max(numbers),
    }


def fraction(numerator: int, denominator: int) -> float | None:
    return None if denominator == 0 else numerator / denominator


def load_capture(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    metadata: dict[str, Any] = {}
    samples: list[dict[str, Any]] = []
    errors: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as capture_file:
        for line_number, raw_line in enumerate(capture_file, start=1):
            line = raw_line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSON on line {line_number}: {exc}") from exc
            if not isinstance(record, dict):
                raise ValueError(f"line {line_number} is not a JSON object")
            record_type = record.get("record_type")
            if record_type == "metadata" and not metadata:
                metadata = record
            elif record_type == "sample":
                samples.append(record)
            elif record_type == "error":
                errors.append(record)
    if not samples:
        raise ValueError("capture contains no sample records")
    return metadata, samples, errors


def counter_summary(samples: list[dict[str, Any]], field: str) -> dict[str, int | None]:
    values: list[int] = []
    for sample in samples:
        status = (sample.get("state") or {}).get("gateway_status") or {}
        value = status.get(field)
        if isinstance(value, int) and not isinstance(value, bool):
            values.append(value)
    if not values:
        return {"start": None, "end": None, "delta": None, "resets": 0}
    delta = 0
    resets = 0
    for previous, current in zip(values, values[1:]):
        if current >= previous:
            delta += current - previous
        else:
            resets += 1
            delta += current
    return {"start": values[0], "end": values[-1], "delta": delta, "resets": resets}


def tag_summary(samples: list[dict[str, Any]], tag_id: int) -> dict[str, Any]:
    present = 0
    fresh = 0
    valid = 0
    usable = 0
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    ages: list[float] = []
    distances: list[list[float]] = [[], [], [], []]

    for sample in samples:
        fixes = (sample.get("state") or {}).get("tag_fixes") or {}
        fix = fixes.get(str(tag_id))
        if not isinstance(fix, dict):
            continue
        present += 1
        is_fresh = bool(fix.get("fresh"))
        is_valid = bool(fix.get("position_valid"))
        fresh += int(is_fresh)
        valid += int(is_valid)
        age = finite_number(fix.get("age_ms"))
        if age is not None:
            ages.append(age)
        if not (is_fresh and is_valid):
            continue
        x = finite_number(fix.get("x_cm"))
        y = finite_number(fix.get("y_cm"))
        z = finite_number(fix.get("z_cm"))
        if x is None or y is None:
            continue
        usable += 1
        xs.append(x)
        ys.append(y)
        if z is not None:
            zs.append(z)
        for index, value in enumerate(list(fix.get("distances_cm") or [])[:4]):
            number = finite_number(value)
            if number is not None and number >= 0:
                distances[index].append(number)

    median_x = statistics.median(xs) if xs else None
    median_y = statistics.median(ys) if ys else None
    radial_deviations = (
        [math.hypot(x - median_x, y - median_y) for x, y in zip(xs, ys)]
        if median_x is not None and median_y is not None
        else []
    )
    rms_radius = (
        math.sqrt(statistics.fmean(value * value for value in radial_deviations))
        if radial_deviations
        else None
    )
    return {
        "present_samples": present,
        "fresh_fraction": fraction(fresh, len(samples)),
        "position_valid_fraction": fraction(valid, len(samples)),
        "usable_fraction": fraction(usable, len(samples)),
        "median_position_cm": {"x": median_x, "y": median_y, "z": statistics.median(zs) if zs else None},
        "age_ms": numeric_stats(ages),
        "spread_from_median_cm": {
            "rms_radius": rms_radius,
            "p95_radius": percentile(radial_deviations, 95.0),
            "maximum_radius": max(radial_deviations) if radial_deviations else None,
            "note": "Interpretable as positioning jitter only when the vehicle was stationary.",
        },
        "anchor_distance_cm": {
            anchor: numeric_stats(values)
            for anchor, values in zip(("A", "B", "C", "D"), distances)
        },
    }


def analyze_records(
    metadata: dict[str, Any],
    samples: list[dict[str, Any]],
    errors: list[dict[str, Any]],
    source: str = "synthetic",
) -> dict[str, Any]:
    capture_times = [
        value
        for sample in samples
        if (value := finite_number(sample.get("captured_at_unix"))) is not None
    ]
    duration_s = max(capture_times) - min(capture_times) if len(capture_times) >= 2 else 0.0
    connected_count = 0
    fresh_gateway_count = 0
    four_tag_count = 0
    kinematics_valid_count = 0
    pose_quality: Counter[str] = Counter()
    points: list[tuple[float, float]] = []
    speeds: list[float] = []
    yaw_rates: list[float] = []
    rectangle_values: dict[str, list[float]] = {
        "front_width_cm": [],
        "rear_width_cm": [],
        "left_length_cm": [],
        "right_length_cm": [],
        "fix_skew_ms": [],
    }

    for sample in samples:
        state = sample.get("state") or {}
        derived = sample.get("derived") or {}
        pose = state.get("pose") or {}
        connected_count += int(bool(state.get("gateway_connected")))
        fresh_gateway_count += int(bool(state.get("gateway_data_fresh")))
        fresh_tag_count = derived.get("fresh_tag_count")
        if not isinstance(fresh_tag_count, int):
            fixes = state.get("tag_fixes") or {}
            fresh_tag_count = sum(
                int(bool(fix.get("fresh")) and bool(fix.get("position_valid")))
                for fix in fixes.values()
                if isinstance(fix, dict)
            )
        four_tag_count += int(fresh_tag_count == 4)
        pose_quality[str(pose.get("quality", "none"))] += 1

        if bool(derived.get("valid")):
            kinematics_valid_count += 1
            x = finite_number(pose.get("x_cm"))
            y = finite_number(pose.get("y_cm"))
            if x is not None and y is not None:
                points.append((x, y))
            speed = finite_number(derived.get("speed_cm_s"))
            yaw_rate = finite_number(derived.get("yaw_rate_deg_s"))
            if speed is not None:
                speeds.append(speed)
            if yaw_rate is not None:
                yaw_rates.append(yaw_rate)

        for field, values in rectangle_values.items():
            value = finite_number(pose.get(field))
            if value is not None:
                values.append(value)

    path_length = sum(
        math.hypot(current[0] - previous[0], current[1] - previous[1])
        for previous, current in zip(points, points[1:])
    )
    displacement = (
        math.hypot(points[-1][0] - points[0][0], points[-1][1] - points[0][1])
        if len(points) >= 2
        else 0.0
    )
    error_types = Counter(str(error.get("error_type", "unknown")) for error in errors)
    metadata_ok = (
        metadata.get("schema") == SCHEMA
        and metadata.get("test_only") is True
        and metadata.get("competition_design_input") is False
    )
    warnings: list[str] = []
    if not metadata_ok:
        warnings.append("Capture metadata does not contain the complete test-only boundary markers.")
    if duration_s < 10.0:
        warnings.append("Capture is shorter than 10 seconds; quality statistics are only a smoke test.")
    if connected_count < len(samples):
        warnings.append("The S3 gateway was disconnected for at least part of the capture.")
    if fresh_gateway_count < len(samples):
        warnings.append("Gateway status was stale for at least part of the capture.")
    if four_tag_count < len(samples):
        warnings.append("At least one sample did not contain four fresh position-valid tag fixes.")

    return {
        "schema": "little_car_uwb_test_analysis_v1",
        "test_only": True,
        "competition_design_input": False,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "source": source,
        "capture_metadata": metadata,
        "metadata_boundary_valid": metadata_ok,
        "sample_summary": {
            "samples": len(samples),
            "error_records": len(errors),
            "duration_s": duration_s,
            "effective_sample_hz": (len(samples) - 1) / duration_s if duration_s > 0 else None,
            "gateway_connected_fraction": fraction(connected_count, len(samples)),
            "gateway_fresh_fraction": fraction(fresh_gateway_count, len(samples)),
            "four_fresh_valid_tags_fraction": fraction(four_tag_count, len(samples)),
            "kinematics_valid_fraction": fraction(kinematics_valid_count, len(samples)),
            "pose_quality_counts": dict(sorted(pose_quality.items())),
            "error_type_counts": dict(sorted(error_types.items())),
        },
        "gateway_counter_deltas": {
            field: counter_summary(samples, field) for field in COUNTER_FIELDS
        },
        "tags": {str(tag_id): tag_summary(samples, tag_id) for tag_id in range(4)},
        "vehicle_pose": {
            "path_length_cm": path_length,
            "displacement_cm": displacement,
            "speed_cm_s": numeric_stats(speeds),
            "yaw_rate_deg_s": numeric_stats(yaw_rates),
            "absolute_yaw_rate_deg_s": numeric_stats(abs(value) for value in yaw_rates),
            "front_width_cm": numeric_stats(rectangle_values["front_width_cm"]),
            "rear_width_cm": numeric_stats(rectangle_values["rear_width_cm"]),
            "left_length_cm": numeric_stats(rectangle_values["left_length_cm"]),
            "right_length_cm": numeric_stats(rectangle_values["right_length_cm"]),
            "fix_skew_ms": numeric_stats(rectangle_values["fix_skew_ms"]),
        },
        "warnings": warnings,
    }


def fmt(value: Any, digits: int = 3) -> str:
    return "--" if value is None else f"{float(value):.{digits}f}"


def pct(value: Any) -> str:
    return "--" if value is None else f"{float(value) * 100.0:.1f}%"


def render_report(summary: dict[str, Any]) -> str:
    sample = summary["sample_summary"]
    vehicle = summary["vehicle_pose"]
    lines = [
        "# UWB test telemetry report",
        "",
        "> TEST ONLY: this capture is an engineering observation aid. It is not part of the competition architecture, score claim, or vehicle control/safety loop.",
        "",
        "## Capture",
        "",
        f"- Source: `{summary['source']}`",
        f"- Samples: {sample['samples']}",
        f"- Duration: {fmt(sample['duration_s'])} s",
        f"- Effective sample rate: {fmt(sample['effective_sample_hz'])} Hz",
        f"- Gateway connected: {pct(sample['gateway_connected_fraction'])}",
        f"- Gateway data fresh: {pct(sample['gateway_fresh_fraction'])}",
        f"- Four fresh valid tags: {pct(sample['four_fresh_valid_tags_fraction'])}",
        f"- Kinematics-valid samples: {pct(sample['kinematics_valid_fraction'])}",
        f"- Pose qualities: `{json.dumps(sample['pose_quality_counts'], ensure_ascii=True)}`",
        f"- Capture errors: {sample['error_records']} `{json.dumps(sample['error_type_counts'], ensure_ascii=True)}`",
        "",
        "## Motion",
        "",
        f"- Center path length: {fmt(vehicle['path_length_cm'])} cm",
        f"- Start-to-end displacement: {fmt(vehicle['displacement_cm'])} cm",
        f"- Speed median / p95 / max: {fmt(vehicle['speed_cm_s']['median'])} / {fmt(vehicle['speed_cm_s']['p95'])} / {fmt(vehicle['speed_cm_s']['maximum'])} cm/s",
        f"- Absolute yaw rate median / p95 / max: {fmt(vehicle['absolute_yaw_rate_deg_s']['median'])} / {fmt(vehicle['absolute_yaw_rate_deg_s']['p95'])} / {fmt(vehicle['absolute_yaw_rate_deg_s']['maximum'])} deg/s",
        f"- Fix skew median / p95 / max: {fmt(vehicle['fix_skew_ms']['median'])} / {fmt(vehicle['fix_skew_ms']['p95'])} / {fmt(vehicle['fix_skew_ms']['maximum'])} ms",
        "",
        "## Rectangle",
        "",
        "| Edge | Median (cm) | P95 (cm) | Samples |",
        "|---|---:|---:|---:|",
    ]
    for label, key in (
        ("Front width", "front_width_cm"),
        ("Rear width", "rear_width_cm"),
        ("Left length", "left_length_cm"),
        ("Right length", "right_length_cm"),
    ):
        stats = vehicle[key]
        lines.append(
            f"| {label} | {fmt(stats['median'])} | {fmt(stats['p95'])} | {stats['count']} |"
        )

    lines.extend(
        [
            "",
            "## Tag quality",
            "",
            "| Tag | Fresh | Usable | Median X/Y (cm) | RMS radius (cm) | P95 radius (cm) |",
            "|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for tag_id, tag in summary["tags"].items():
        position = tag["median_position_cm"]
        spread = tag["spread_from_median_cm"]
        lines.append(
            f"| {tag_id} | {pct(tag['fresh_fraction'])} | {pct(tag['usable_fraction'])} | "
            f"{fmt(position['x'])} / {fmt(position['y'])} | {fmt(spread['rms_radius'])} | "
            f"{fmt(spread['p95_radius'])} |"
        )

    lines.extend(
        [
            "",
            "The radius values are stationary jitter only when the vehicle remained stationary for the entire capture.",
            "",
            "## Gateway counters",
            "",
            "| Counter | Start | End | Accumulated delta | Resets |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for field, counter in summary["gateway_counter_deltas"].items():
        lines.append(
            f"| `{field}` | {counter['start']} | {counter['end']} | {counter['delta']} | {counter['resets']} |"
        )

    if summary["warnings"]:
        lines.extend(["", "## Warnings", ""])
        lines.extend(f"- {warning}" for warning in summary["warnings"])
    lines.append("")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path, help="JSONL generated by capture_uwb_test_telemetry.py")
    parser.add_argument("--summary-output", type=Path)
    parser.add_argument("--report-output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    capture_path = args.capture.resolve()
    metadata, samples, errors = load_capture(capture_path)
    summary = analyze_records(metadata, samples, errors, str(capture_path))
    summary_path = (args.summary_output or capture_path.with_suffix(".summary.json")).resolve()
    report_path = (args.report_output or capture_path.with_suffix(".report.md")).resolve()
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=True) + "\n", encoding="utf-8"
    )
    report_path.write_text(render_report(summary), encoding="utf-8")
    sample = summary["sample_summary"]
    print(
        "RESULT "
        f"samples={sample['samples']} errors={sample['error_records']} "
        f"duration_s={sample['duration_s']:.3f} "
        f"four_tag_valid={pct(sample['four_fresh_valid_tags_fraction'])}"
    )
    print(f"RESULT summary={summary_path}")
    print(f"RESULT report={report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
