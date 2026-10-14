from __future__ import annotations

from dataclasses import dataclass
import time

from .protocol import ProtocolError, crc8


LIDAR_MAP_VERSION = "1"
LIDAR_PART_COUNT = 6
LIDAR_POINTS_PER_PART = 12
LIDAR_POINT_COUNT = LIDAR_PART_COUNT * LIDAR_POINTS_PER_PART
LIDAR_PARTIAL_TIMEOUT_S = 1.5


@dataclass(frozen=True)
class LidarPoint:
    bin_index: int
    angle_deg: float
    distance_mm: int | None
    quality_class: int

    def as_dict(self) -> dict[str, object]:
        return {
            "bin": self.bin_index,
            "angle_deg": self.angle_deg,
            "distance_mm": self.distance_mm,
            "quality": self.quality_class,
        }


@dataclass(frozen=True)
class LidarChunk:
    sequence: int
    tick_10ms: int
    part: int
    total: int
    points: tuple[LidarPoint, ...]


@dataclass(frozen=True)
class LidarScan:
    sender_id: int
    sequence: int
    tick_10ms: int
    points: tuple[LidarPoint, ...]
    received_at: float

    def as_dict(self) -> dict[str, object]:
        return {
            "sender_id": self.sender_id,
            "sequence": self.sequence,
            "tick_10ms": self.tick_10ms,
            "valid_points": sum(point.distance_mm is not None for point in self.points),
            "received_at": self.received_at,
            "points": [point.as_dict() for point in self.points],
        }


def _parse_hex(text: str, field: str) -> int:
    try:
        return int(text, 16)
    except ValueError as exc:
        raise ProtocolError(f"lidar {field} is not hexadecimal") from exc


def parse_lidar_chunk(line: str) -> LidarChunk:
    """Parse one 49-byte @R line; a direct-UART newline is optional."""

    content = line
    if content.endswith("\n"):
        content = content[:-1]
        if content.endswith("\r"):
            content = content[:-1]
    if len(content) != 49:
        raise ProtocolError(f"lidar map line length is {len(content)}, expected 49")
    if not content.startswith("@R" + LIDAR_MAP_VERSION):
        raise ProtocolError("unsupported lidar map frame version")

    expected_crc = _parse_hex(content[47:49], "CRC")
    actual_crc = crc8(content[1:47].encode("ascii"))
    if actual_crc != expected_crc:
        raise ProtocolError(
            f"lidar map CRC mismatch: got {expected_crc:02X}, expected {actual_crc:02X}"
        )

    sequence = _parse_hex(content[3:5], "sequence")
    tick_10ms = _parse_hex(content[5:9], "tick")
    part = _parse_hex(content[9:10], "part")
    total = _parse_hex(content[10:11], "part count")
    if total != LIDAR_PART_COUNT or not 0 <= part < total:
        raise ProtocolError("invalid lidar map part/count")

    points: list[LidarPoint] = []
    payload = content[11:47]
    for local_index in range(LIDAR_POINTS_PER_PART):
        packed = _parse_hex(payload[local_index * 3 : local_index * 3 + 3], "point")
        quality_class = (packed >> 10) & 0x03
        distance_code = packed & 0x03FF
        bin_index = part * LIDAR_POINTS_PER_PART + local_index
        points.append(
            LidarPoint(
                bin_index=bin_index,
                angle_deg=bin_index * 5.0 + 2.5,
                distance_mm=None if distance_code == 0x03FF else distance_code * 16,
                quality_class=quality_class,
            )
        )
    return LidarChunk(
        sequence=sequence,
        tick_10ms=tick_10ms,
        part=part,
        total=total,
        points=tuple(points),
    )


@dataclass
class _PartialScan:
    updated_at: float
    chunks: dict[int, LidarChunk]


class LidarScanAssembler:
    def __init__(self, timeout_s: float = LIDAR_PARTIAL_TIMEOUT_S) -> None:
        self.timeout_s = timeout_s
        self._partial: dict[tuple[int, int, int], _PartialScan] = {}

    def _expire(self, now: float) -> None:
        stale = [
            key
            for key, partial in self._partial.items()
            if now - partial.updated_at > self.timeout_s
        ]
        for key in stale:
            self._partial.pop(key, None)

    def add(
        self, sender_id: int, chunk: LidarChunk, now: float | None = None
    ) -> LidarScan | None:
        timestamp = time.time() if now is None else now
        self._expire(timestamp)
        key = (sender_id, chunk.sequence, chunk.tick_10ms)

        # A new part 0 supersedes incomplete older scans from the same vehicle.
        if chunk.part == 0:
            for old_key in [item for item in self._partial if item[0] == sender_id and item != key]:
                self._partial.pop(old_key, None)

        partial = self._partial.setdefault(key, _PartialScan(timestamp, {}))
        partial.updated_at = timestamp
        partial.chunks[chunk.part] = chunk
        if len(partial.chunks) != LIDAR_PART_COUNT:
            return None
        if set(partial.chunks) != set(range(LIDAR_PART_COUNT)):
            return None

        points = tuple(
            point
            for part in range(LIDAR_PART_COUNT)
            for point in partial.chunks[part].points
        )
        self._partial.pop(key, None)
        if len(points) != LIDAR_POINT_COUNT:
            raise ProtocolError("assembled lidar scan does not contain 72 points")
        return LidarScan(
            sender_id=sender_id,
            sequence=chunk.sequence,
            tick_10ms=chunk.tick_10ms,
            points=points,
            received_at=timestamp,
        )
