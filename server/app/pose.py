from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import math
import statistics
import time

from .protocol import PositionReport
from .vehicle_geometry_generated import (
    FRONT_LEFT_TAG,
    FRONT_RIGHT_TAG,
    MAXIMUM_FIX_AGE_S,
    MAXIMUM_FIX_SKEW_S,
    REAR_LEFT_TAG,
    REAR_RIGHT_TAG,
    VEHICLE_DIMENSION_TOLERANCE_M,
    VEHICLE_TAG_LENGTH_M,
    VEHICLE_TAG_WIDTH_M,
)


@dataclass(frozen=True)
class PoseConfig:
    front_left_tag: int = FRONT_LEFT_TAG
    front_right_tag: int = FRONT_RIGHT_TAG
    rear_left_tag: int = REAR_LEFT_TAG
    rear_right_tag: int = REAR_RIGHT_TAG
    vehicle_length_cm: float = VEHICLE_TAG_LENGTH_M * 100.0
    vehicle_width_cm: float = VEHICLE_TAG_WIDTH_M * 100.0
    dimension_tolerance_cm: float = VEHICLE_DIMENSION_TOLERANCE_M * 100.0
    maximum_fix_age_s: float = MAXIMUM_FIX_AGE_S
    maximum_fix_skew_s: float = MAXIMUM_FIX_SKEW_S

    @property
    def role_map(self) -> dict[str, int]:
        return {
            "front_left": self.front_left_tag,
            "front_right": self.front_right_tag,
            "rear_left": self.rear_left_tag,
            "rear_right": self.rear_right_tag,
        }


@dataclass(frozen=True)
class TimedFix:
    report: PositionReport
    received_at: float


@dataclass(frozen=True)
class VehiclePose:
    quality: str
    x_cm: float | None
    y_cm: float | None
    heading_deg: float | None
    valid_tags: tuple[int, ...]
    fix_skew_ms: float | None
    rigid_rectangle: bool | None
    front_width_cm: float | None
    rear_width_cm: float | None
    left_length_cm: float | None
    right_length_cm: float | None
    updated_at: float

    def as_dict(self) -> dict[str, object]:
        return {
            "quality": self.quality,
            "x_cm": self.x_cm,
            "y_cm": self.y_cm,
            "heading_deg": self.heading_deg,
            "valid_tags": list(self.valid_tags),
            "fix_skew_ms": self.fix_skew_ms,
            "rigid_rectangle": self.rigid_rectangle,
            "front_width_cm": self.front_width_cm,
            "rear_width_cm": self.rear_width_cm,
            "left_length_cm": self.left_length_cm,
            "right_length_cm": self.right_length_cm,
            "updated_at": self.updated_at,
        }


def _distance(a: PositionReport, b: PositionReport) -> float:
    return math.hypot(float(a.x_cm - b.x_cm), float(a.y_cm - b.y_cm))


def _midpoint(a: PositionReport, b: PositionReport) -> tuple[float, float]:
    return ((a.x_cm + b.x_cm) / 2.0, (a.y_cm + b.y_cm) / 2.0)


def _fit_rigid_pose(
    role_reports: dict[str, PositionReport], length_cm: float, width_cm: float
) -> tuple[float, float, float, float] | None:
    """Fit translation and rotation without allowing the vehicle to deform."""
    local = {
        "front_left": (length_cm / 2.0, width_cm / 2.0),
        "front_right": (length_cm / 2.0, -width_cm / 2.0),
        "rear_left": (-length_cm / 2.0, width_cm / 2.0),
        "rear_right": (-length_cm / 2.0, -width_cm / 2.0),
    }
    samples = [
        (local[role][0], local[role][1], float(report.x_cm), float(report.y_cm))
        for role, report in role_reports.items()
    ]
    if len(samples) < 2 or length_cm <= 0 or width_cm <= 0:
        return None

    local_x = sum(item[0] for item in samples) / len(samples)
    local_y = sum(item[1] for item in samples) / len(samples)
    measured_x = sum(item[2] for item in samples) / len(samples)
    measured_y = sum(item[3] for item in samples) / len(samples)
    dot = 0.0
    cross = 0.0
    for lx, ly, mx, my in samples:
        centered_lx = lx - local_x
        centered_ly = ly - local_y
        centered_mx = mx - measured_x
        centered_my = my - measured_y
        dot += centered_lx * centered_mx + centered_ly * centered_my
        cross += centered_lx * centered_my - centered_ly * centered_mx
    if abs(dot) + abs(cross) < 1e-9:
        return None

    heading_rad = math.atan2(cross, dot)
    cosine = math.cos(heading_rad)
    sine = math.sin(heading_rad)
    center_x = measured_x - (cosine * local_x - sine * local_y)
    center_y = measured_y - (sine * local_x + cosine * local_y)
    squared_error = 0.0
    for lx, ly, mx, my in samples:
        predicted_x = center_x + cosine * lx - sine * ly
        predicted_y = center_y + sine * lx + cosine * ly
        squared_error += (predicted_x - mx) ** 2 + (predicted_y - my) ** 2
    rms_residual = math.sqrt(squared_error / len(samples))
    return center_x, center_y, math.degrees(heading_rad), rms_residual


class PoseEstimator:
    def __init__(self, config: PoseConfig | None = None) -> None:
        self.config = config or PoseConfig()
        self._fixes: dict[int, TimedFix] = {}
        self._position_history: dict[int, deque[PositionReport]] = {}

    def update(self, report: PositionReport, received_at: float | None = None) -> VehiclePose:
        timestamp = time.time() if received_at is None else received_at
        filtered = report
        if report.position_valid:
            # LinkPG coordinates can jump by several centimetres while a tag is
            # stationary. Five samples reject isolated UWB multipath outliers
            # without adding enough delay to affect the phone map noticeably.
            history = self._position_history.setdefault(report.tag_id, deque(maxlen=5))
            history.append(report)
            filtered = PositionReport(
                gateway_sequence=report.gateway_sequence,
                tag_id=report.tag_id,
                flags=report.flags,
                x_cm=int(round(statistics.median(item.x_cm for item in history))),
                y_cm=int(round(statistics.median(item.y_cm for item in history))),
                z_cm=int(round(statistics.median(item.z_cm for item in history))),
                distances_cm=report.distances_cm,
            )
        self._fixes[report.tag_id] = TimedFix(report=filtered, received_at=timestamp)
        return self.estimate(timestamp)

    def latest_report(self, tag_id: int) -> PositionReport | None:
        timed = self._fixes.get(tag_id)
        return None if timed is None else timed.report

    def estimate(self, now: float | None = None) -> VehiclePose:
        timestamp = time.time() if now is None else now
        role_reports: dict[str, PositionReport] = {}
        role_timestamps: list[float] = []
        valid_ids: list[int] = []

        for role, tag_id in self.config.role_map.items():
            timed = self._fixes.get(tag_id)
            if timed is None:
                continue
            if timestamp - timed.received_at > self.config.maximum_fix_age_s:
                continue
            if not timed.report.position_valid:
                continue
            role_reports[role] = timed.report
            role_timestamps.append(timed.received_at)
            valid_ids.append(tag_id)

        if not role_reports:
            return VehiclePose(
                quality="none",
                x_cm=None,
                y_cm=None,
                heading_deg=None,
                valid_tags=(),
                fix_skew_ms=None,
                rigid_rectangle=None,
                front_width_cm=None,
                rear_width_cm=None,
                left_length_cm=None,
                right_length_cm=None,
                updated_at=timestamp,
            )

        center_x = sum(fix.x_cm for fix in role_reports.values()) / len(role_reports)
        center_y = sum(fix.y_cm for fix in role_reports.values()) / len(role_reports)
        fix_skew_ms = (max(role_timestamps) - min(role_timestamps)) * 1000.0

        required = {"front_left", "front_right", "rear_left", "rear_right"}
        if self.config.vehicle_length_cm <= 0 or self.config.vehicle_width_cm <= 0:
            if set(role_reports) != required:
                return VehiclePose(
                    quality="partial", x_cm=center_x, y_cm=center_y, heading_deg=None,
                    valid_tags=tuple(sorted(valid_ids)), fix_skew_ms=fix_skew_ms,
                    rigid_rectangle=None, front_width_cm=None, rear_width_cm=None,
                    left_length_cm=None, right_length_cm=None, updated_at=timestamp,
                )
            front_mid = _midpoint(role_reports["front_left"], role_reports["front_right"])
            rear_mid = _midpoint(role_reports["rear_left"], role_reports["rear_right"])
            heading = math.degrees(math.atan2(
                front_mid[1] - rear_mid[1], front_mid[0] - rear_mid[0]
            ))
            return VehiclePose(
                quality="unconfigured", x_cm=center_x, y_cm=center_y,
                heading_deg=heading, valid_tags=tuple(sorted(valid_ids)),
                fix_skew_ms=fix_skew_ms, rigid_rectangle=None,
                front_width_cm=None, rear_width_cm=None, left_length_cm=None,
                right_length_cm=None, updated_at=timestamp,
            )
        rigid_fit = _fit_rigid_pose(
            role_reports, self.config.vehicle_length_cm, self.config.vehicle_width_cm
        )
        if rigid_fit is None:
            return VehiclePose(
                quality="partial",
                x_cm=center_x,
                y_cm=center_y,
                heading_deg=None,
                valid_tags=tuple(sorted(valid_ids)),
                fix_skew_ms=fix_skew_ms,
                rigid_rectangle=None,
                front_width_cm=None,
                rear_width_cm=None,
                left_length_cm=None,
                right_length_cm=None,
                updated_at=timestamp,
            )

        center_x, center_y, heading, rms_residual = rigid_fit
        front_width = (_distance(role_reports["front_left"], role_reports["front_right"])
                       if {"front_left", "front_right"} <= set(role_reports) else None)
        rear_width = (_distance(role_reports["rear_left"], role_reports["rear_right"])
                      if {"rear_left", "rear_right"} <= set(role_reports) else None)
        left_length = (_distance(role_reports["front_left"], role_reports["rear_left"])
                       if {"front_left", "rear_left"} <= set(role_reports) else None)
        right_length = (_distance(role_reports["front_right"], role_reports["rear_right"])
                        if {"front_right", "rear_right"} <= set(role_reports) else None)
        rigid = rms_residual <= self.config.dimension_tolerance_cm
        if fix_skew_ms > self.config.maximum_fix_skew_s * 1000.0:
            quality = "unsynchronized"
        elif rigid and set(role_reports) == required:
            quality = "full"
        elif rigid:
            quality = "partial_rigid"
        else:
            quality = "rejected"
        return VehiclePose(
            quality=quality,
            x_cm=center_x,
            y_cm=center_y,
            heading_deg=heading,
            valid_tags=tuple(sorted(valid_ids)),
            fix_skew_ms=fix_skew_ms,
            rigid_rectangle=rigid,
            front_width_cm=front_width,
            rear_width_cm=rear_width,
            left_length_cm=left_length,
            right_length_cm=right_length,
            updated_at=timestamp,
        )

    def fixes_as_dict(self, now: float | None = None) -> dict[str, object]:
        timestamp = time.time() if now is None else now
        return {
            str(tag_id): {
                "gateway_sequence": timed.report.gateway_sequence,
                "flags": f"{timed.report.flags:08X}",
                "position_valid": timed.report.position_valid,
                "x_cm": timed.report.x_cm,
                "y_cm": timed.report.y_cm,
                "z_cm": timed.report.z_cm,
                "distances_cm": list(timed.report.distances_cm),
                "received_at": timed.received_at,
                "age_ms": max(0.0, (timestamp - timed.received_at) * 1000.0),
                "fresh": timestamp - timed.received_at <= self.config.maximum_fix_age_s,
            }
            for tag_id, timed in sorted(self._fixes.items())
        }
