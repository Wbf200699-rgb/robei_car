#!/usr/bin/env python3
"""Capture test-only UWB positioning snapshots from the public server API."""

from __future__ import annotations

import argparse
from collections import deque
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


SCHEMA = "little_car_uwb_test_v1"
DEFAULT_STATE_URL = "https://relay.example.invalid/api/v1/state"
DEFAULT_INTERVAL_S = 0.1
VALID_POSE_QUALITIES = {"full", "unconfigured"}


def read_env_value(path: Path, key: str) -> str | None:
    if not path.exists():
        return None
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        name, value = line.split("=", 1)
        if name.strip() == key:
            return value.strip().strip('"').strip("'")
    return None


def resolve_token(env_file: Path) -> str:
    token = os.getenv("UWB_MOBILE_TOKEN") or read_env_value(env_file, "UWB_MOBILE_TOKEN")
    if not token:
        raise RuntimeError(
            "UWB_MOBILE_TOKEN is unavailable; set it in the environment or server/.env"
        )
    return token


def fetch_state(url: str, token: str, timeout_s: float) -> dict[str, Any]:
    request = Request(
        url,
        headers={
            "Accept": "application/json",
            "User-Agent": "little-car-uwb-test-capture/1",
            "X-API-Token": token,
        },
    )
    with urlopen(request, timeout=timeout_s) as response:
        payload = json.load(response)
    if not isinstance(payload, dict):
        raise ValueError("state endpoint did not return a JSON object")
    return payload


def compact_state(state: dict[str, Any]) -> dict[str, Any]:
    """Keep only positioning and transport-health fields in test captures."""
    return {
        "gateway_connected": bool(state.get("gateway_connected")),
        "gateway_id": state.get("gateway_id"),
        "gateway_status": state.get("gateway_status") or {},
        "gateway_status_age_ms": state.get("gateway_status_age_ms"),
        "gateway_data_fresh": bool(state.get("gateway_data_fresh")),
        "tag_fixes": state.get("tag_fixes") or {},
        "pose": state.get("pose") or {},
        "protocol_errors": state.get("protocol_errors"),
    }


def normalized_angle_delta(current_deg: float, previous_deg: float) -> float:
    return (current_deg - previous_deg + 180.0) % 360.0 - 180.0


def valid_tag_count(state: dict[str, Any]) -> int:
    fixes = state.get("tag_fixes") or {}
    return sum(
        1
        for fix in fixes.values()
        if bool(fix.get("fresh")) and bool(fix.get("position_valid"))
    )


class KinematicsEstimator:
    """Estimate velocity over a short window to reduce single-fix UWB jitter."""

    def __init__(self, window_s: float) -> None:
        if window_s <= 0:
            raise ValueError("kinematics window must be positive")
        self.window_s = window_s
        self.samples: deque[tuple[float, float, float, float]] = deque()

    def update(self, captured_at: float, state: dict[str, Any]) -> dict[str, Any]:
        pose = state.get("pose") or {}
        x_cm = pose.get("x_cm")
        y_cm = pose.get("y_cm")
        heading_deg = pose.get("heading_deg")
        quality = str(pose.get("quality", "none"))
        fresh_tags = valid_tag_count(state)
        valid = (
            bool(state.get("gateway_connected"))
            and bool(state.get("gateway_data_fresh"))
            and fresh_tags == 4
            and quality in VALID_POSE_QUALITIES
            and x_cm is not None
            and y_cm is not None
            and heading_deg is not None
        )
        result: dict[str, Any] = {
            "valid": valid,
            "fresh_tag_count": fresh_tags,
            "vx_cm_s": None,
            "vy_cm_s": None,
            "speed_cm_s": None,
            "yaw_rate_deg_s": None,
            "window_s": None,
        }
        if not valid:
            self.samples.clear()
            return result

        sample = (captured_at, float(x_cm), float(y_cm), float(heading_deg))
        self.samples.append(sample)
        while len(self.samples) > 1 and captured_at - self.samples[0][0] > self.window_s:
            self.samples.popleft()
        if len(self.samples) < 2:
            return result

        first = self.samples[0]
        elapsed = sample[0] - first[0]
        if elapsed < min(0.15, self.window_s / 2.0):
            return result
        vx = (sample[1] - first[1]) / elapsed
        vy = (sample[2] - first[2]) / elapsed
        result.update(
            {
                "vx_cm_s": vx,
                "vy_cm_s": vy,
                "speed_cm_s": math.hypot(vx, vy),
                "yaw_rate_deg_s": normalized_angle_delta(sample[3], first[3]) / elapsed,
                "window_s": elapsed,
            }
        )
        return result


def safe_label(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip()).strip("_.")
    return slug or "unlabeled"


def default_output_path(repo_root: Path, label: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return (
        repo_root
        / "little_car"
        / "test_results"
        / "uwb_test_telemetry"
        / f"{stamp}_{safe_label(label)}.jsonl"
    )


def csv_columns() -> list[str]:
    fields = [
        "schema",
        "test_only",
        "competition_design_input",
        "label",
        "sample_index",
        "captured_at_unix",
        "elapsed_s",
        "gateway_connected",
        "gateway_data_fresh",
        "gateway_status_age_ms",
        "position_frames",
        "crc_errors",
        "framing_errors",
        "pose_quality",
        "pose_x_cm",
        "pose_y_cm",
        "pose_heading_deg",
        "fix_skew_ms",
        "front_width_cm",
        "rear_width_cm",
        "left_length_cm",
        "right_length_cm",
        "fresh_tag_count",
        "kinematics_valid",
        "vx_cm_s",
        "vy_cm_s",
        "speed_cm_s",
        "yaw_rate_deg_s",
    ]
    for tag_id in range(4):
        fields.extend(
            [
                f"tag{tag_id}_fresh",
                f"tag{tag_id}_valid",
                f"tag{tag_id}_age_ms",
                f"tag{tag_id}_x_cm",
                f"tag{tag_id}_y_cm",
                f"tag{tag_id}_z_cm",
                f"tag{tag_id}_distance_a_cm",
                f"tag{tag_id}_distance_b_cm",
                f"tag{tag_id}_distance_c_cm",
                f"tag{tag_id}_distance_d_cm",
            ]
        )
    return fields


def flatten_sample(record: dict[str, Any]) -> dict[str, Any]:
    state = record["state"]
    status = state.get("gateway_status") or {}
    pose = state.get("pose") or {}
    derived = record.get("derived") or {}
    row: dict[str, Any] = {
        "schema": record["schema"],
        "test_only": record["test_only"],
        "competition_design_input": record["competition_design_input"],
        "label": record["label"],
        "sample_index": record["sample_index"],
        "captured_at_unix": record["captured_at_unix"],
        "elapsed_s": record["elapsed_s"],
        "gateway_connected": state.get("gateway_connected"),
        "gateway_data_fresh": state.get("gateway_data_fresh"),
        "gateway_status_age_ms": state.get("gateway_status_age_ms"),
        "position_frames": status.get("position_frames"),
        "crc_errors": status.get("crc_errors"),
        "framing_errors": status.get("framing_errors"),
        "pose_quality": pose.get("quality"),
        "pose_x_cm": pose.get("x_cm"),
        "pose_y_cm": pose.get("y_cm"),
        "pose_heading_deg": pose.get("heading_deg"),
        "fix_skew_ms": pose.get("fix_skew_ms"),
        "front_width_cm": pose.get("front_width_cm"),
        "rear_width_cm": pose.get("rear_width_cm"),
        "left_length_cm": pose.get("left_length_cm"),
        "right_length_cm": pose.get("right_length_cm"),
        "fresh_tag_count": derived.get("fresh_tag_count"),
        "kinematics_valid": derived.get("valid"),
        "vx_cm_s": derived.get("vx_cm_s"),
        "vy_cm_s": derived.get("vy_cm_s"),
        "speed_cm_s": derived.get("speed_cm_s"),
        "yaw_rate_deg_s": derived.get("yaw_rate_deg_s"),
    }
    fixes = state.get("tag_fixes") or {}
    for tag_id in range(4):
        fix = fixes.get(str(tag_id), {})
        distances = list(fix.get("distances_cm") or [])
        distances.extend([None] * (4 - len(distances)))
        row.update(
            {
                f"tag{tag_id}_fresh": fix.get("fresh"),
                f"tag{tag_id}_valid": fix.get("position_valid"),
                f"tag{tag_id}_age_ms": fix.get("age_ms"),
                f"tag{tag_id}_x_cm": fix.get("x_cm"),
                f"tag{tag_id}_y_cm": fix.get("y_cm"),
                f"tag{tag_id}_z_cm": fix.get("z_cm"),
                f"tag{tag_id}_distance_a_cm": distances[0],
                f"tag{tag_id}_distance_b_cm": distances[1],
                f"tag{tag_id}_distance_c_cm": distances[2],
                f"tag{tag_id}_distance_d_cm": distances[3],
            }
        )
    return row


def format_console(record: dict[str, Any]) -> str:
    state = record["state"]
    pose = state.get("pose") or {}
    derived = record.get("derived") or {}

    def number(value: Any, digits: int = 1) -> str:
        return "--" if value is None else f"{float(value):.{digits}f}"

    return (
        f"t={record['elapsed_s']:7.1f}s "
        f"gw={'ON' if state.get('gateway_connected') else 'OFF'} "
        f"fresh={'Y' if state.get('gateway_data_fresh') else 'N'} "
        f"tags={derived.get('fresh_tag_count', 0)}/4 "
        f"q={pose.get('quality', 'none')} "
        f"xy=({number(pose.get('x_cm'))},{number(pose.get('y_cm'))})cm "
        f"yaw={number(pose.get('heading_deg'))}deg "
        f"v={number(derived.get('speed_cm_s'))}cm/s "
        f"w={number(derived.get('yaw_rate_deg_s'))}deg/s"
    )


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=DEFAULT_STATE_URL)
    parser.add_argument("--env-file", type=Path, default=repo_root / "server" / ".env")
    parser.add_argument("--label", default="unlabeled")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 runs until Ctrl+C")
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL_S)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--kinematics-window", type=float, default=0.8)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.interval < 0.05:
        raise SystemExit("--interval must be at least 0.05 seconds")
    if args.duration < 0:
        raise SystemExit("--duration cannot be negative")
    if args.timeout <= 0:
        raise SystemExit("--timeout must be positive")

    repo_root = Path(__file__).resolve().parents[1]
    output = args.output or default_output_path(repo_root, args.label)
    output = output.resolve()
    csv_output = output.with_suffix(".csv")
    output.parent.mkdir(parents=True, exist_ok=True)
    token = resolve_token(args.env_file.resolve())
    estimator = KinematicsEstimator(args.kinematics_window)
    started_wall = time.time()
    started_mono = time.monotonic()
    next_poll = started_mono
    sample_index = 0
    error_count = 0

    metadata = {
        "schema": SCHEMA,
        "test_only": True,
        "record_type": "metadata",
        "label": args.label,
        "started_at_unix": started_wall,
        "started_at_utc": datetime.fromtimestamp(started_wall, timezone.utc).isoformat(),
        "interval_s": args.interval,
        "kinematics_window_s": args.kinematics_window,
        "source": "server_state_snapshot",
        "units": {"position": "cm", "speed": "cm/s", "heading": "deg"},
        "competition_design_input": False,
    }

    print(f"Test-only UWB capture: {output}")
    print("Press Ctrl+C to stop." if args.duration == 0 else f"Duration: {args.duration:.1f} s")
    try:
        with output.open("w", encoding="utf-8", newline="\n") as json_file, csv_output.open(
            "w", encoding="utf-8", newline=""
        ) as csv_file:
            json_file.write(json.dumps(metadata, ensure_ascii=True, separators=(",", ":")) + "\n")
            writer = csv.DictWriter(csv_file, fieldnames=csv_columns())
            writer.writeheader()

            while True:
                now_mono = time.monotonic()
                elapsed = now_mono - started_mono
                if args.duration and elapsed >= args.duration:
                    break
                if now_mono < next_poll:
                    time.sleep(next_poll - now_mono)
                next_poll = max(next_poll + args.interval, time.monotonic())

                captured_at = time.time()
                try:
                    raw_state = fetch_state(args.url, token, args.timeout)
                    state = compact_state(raw_state)
                    derived = estimator.update(captured_at, state)
                    record = {
                        "schema": SCHEMA,
                        "test_only": True,
                        "competition_design_input": False,
                        "record_type": "sample",
                        "label": args.label,
                        "sample_index": sample_index,
                        "captured_at_unix": captured_at,
                        "captured_at_utc": datetime.fromtimestamp(
                            captured_at, timezone.utc
                        ).isoformat(),
                        "elapsed_s": captured_at - started_wall,
                        "state": state,
                        "derived": derived,
                    }
                    json_file.write(
                        json.dumps(record, ensure_ascii=True, separators=(",", ":")) + "\n"
                    )
                    writer.writerow(flatten_sample(record))
                    json_file.flush()
                    csv_file.flush()
                    if sample_index == 0 or sample_index % max(1, round(1.0 / args.interval)) == 0:
                        print(format_console(record))
                    sample_index += 1
                except (HTTPError, URLError, TimeoutError, ValueError, OSError) as exc:
                    error_count += 1
                    error_record = {
                        "schema": SCHEMA,
                        "test_only": True,
                        "competition_design_input": False,
                        "record_type": "error",
                        "captured_at_unix": captured_at,
                        "elapsed_s": captured_at - started_wall,
                        "error_type": type(exc).__name__,
                        "error": str(exc),
                    }
                    json_file.write(
                        json.dumps(error_record, ensure_ascii=True, separators=(",", ":")) + "\n"
                    )
                    json_file.flush()
                    print(f"capture error: {type(exc).__name__}: {exc}")
    except KeyboardInterrupt:
        print("Capture stopped by user.")

    elapsed = time.monotonic() - started_mono
    print(f"RESULT samples={sample_index} errors={error_count} duration_s={elapsed:.3f}")
    print(f"RESULT jsonl={output}")
    print(f"RESULT csv={csv_output}")
    return 0 if sample_index > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
