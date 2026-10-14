import asyncio

import pytest

from app.lidar import LidarScanAssembler, parse_lidar_chunk
from app.pose import PoseEstimator
from app.protocol import ProtocolError, build_fragment, crc8
from app.runtime import VehicleRuntime


def build_map_line(sequence: int, tick: int, part: int, words: list[int]) -> str:
    assert len(words) == 12
    body = f"R1{sequence:02X}{tick:04X}{part:X}6" + "".join(
        f"{word & 0xFFF:03X}" for word in words
    )
    return "@" + body + f"{crc8(body.encode('ascii')):02X}"


def test_parse_lidar_chunk_decodes_distance_quality_and_optional_newline() -> None:
    words = [0x84B, 0xC7D, 0xFFF] + [0x401 + index for index in range(9)]
    chunk = parse_lidar_chunk(build_map_line(0x2A, 0x1234, 0, words) + "\n")

    assert chunk.sequence == 0x2A
    assert chunk.tick_10ms == 0x1234
    assert chunk.part == 0
    assert chunk.points[0].distance_mm == 0x04B * 16
    assert chunk.points[0].quality_class == 2
    assert chunk.points[0].angle_deg == 2.5
    assert chunk.points[1].quality_class == 3
    assert chunk.points[2].distance_mm is None


def test_parse_lidar_chunk_rejects_corrupt_crc() -> None:
    line = build_map_line(1, 2, 0, [0xFFF] * 12)
    corrupt = line[:-2] + ("00" if line[-2:] != "00" else "01")
    with pytest.raises(ProtocolError, match="CRC mismatch"):
        parse_lidar_chunk(corrupt)


def test_lidar_scan_assembler_requires_all_six_matching_parts() -> None:
    assembler = LidarScanAssembler()
    complete = None
    for part in (0, 2, 1, 3, 5):
        chunk = parse_lidar_chunk(build_map_line(7, 100, part, [0x401 + part] * 12))
        complete = assembler.add(1, chunk, now=1.0 + part * 0.01)
        assert complete is None

    final_chunk = parse_lidar_chunk(build_map_line(7, 100, 4, [0x405] * 12))
    complete = assembler.add(1, final_chunk, now=1.1)
    assert complete is not None
    assert len(complete.points) == 72
    assert complete.points[0].bin_index == 0
    assert complete.points[-1].bin_index == 71
    assert complete.points[24].distance_mm == 3 * 16


def test_runtime_reassembles_map_lines_and_publishes_latest_scan() -> None:
    runtime = VehicleRuntime(PoseEstimator())

    async def exercise() -> None:
        for part in range(6):
            words = [0x400 | (part * 12 + index + 1) for index in range(12)]
            line = build_map_line(0x33, 0x4567, part, words).encode("ascii")
            envelope = build_fragment(0x80 + part, 0, 1, line)
            await runtime.handle_gateway_message(
                f"PASS,{part + 1},1,{envelope.hex().upper()}"
            )

    asyncio.run(exercise())
    snapshot = runtime.snapshot()
    assert snapshot["lidar_scan_fresh"] is True
    assert snapshot["lidar_scan"]["sequence"] == 0x33
    assert snapshot["lidar_scan"]["tick_10ms"] == 0x4567
    assert snapshot["lidar_scan"]["valid_points"] == 72
    assert len(snapshot["lidar_scan"]["points"]) == 72
    assert runtime.latest_vehicle_lines["@R"].startswith("@R13345675")
