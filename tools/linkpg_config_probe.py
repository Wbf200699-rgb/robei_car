#!/usr/bin/env python3
"""Read LinkPG registers and optionally configure a verified main anchor."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import struct
import time
from dataclasses import asdict, dataclass

import serial
from serial.tools import list_ports


BAUD_CODES = {
    0: 4800,
    1: 9600,
    2: 14400,
    3: 38400,
    5: 56000,
    6: 57600,
    7: 115200,
    8: 128000,
    9: 256000,
}
ROLE_NAMES = {0: "tag", 1: "secondary_anchor", 2: "main_anchor"}
POSITION_NAMES = {0: "one_to_one", 1: "2d", 2: "3d"}
RUN_MODE_NAMES = {
    0: "stopped",
    1: "single_manual",
    2: "continuous_manual",
    3: "single_auto_output",
    4: "continuous_auto_output",
    5: "boot_single_manual",
    6: "boot_continuous_manual",
    7: "boot_single_auto_output",
    8: "boot_continuous_auto_output",
    9: "cancel_boot_auto",
    10: "remote_config",
}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_read(modbus_id: int, address: int, count: int) -> bytes:
    body = struct.pack(">BBHH", modbus_id, 0x03, address, count)
    return body + struct.pack("<H", crc16_modbus(body))


def build_write_registers(modbus_id: int, address: int, values: list[int]) -> bytes:
    if not values or len(values) > 60 or any(not 0 <= value <= 0xFFFF for value in values):
        raise ValueError("invalid Modbus register write")
    data = b"".join(struct.pack(">H", value) for value in values)
    body = struct.pack(">BBHHB", modbus_id, 0x10, address, len(values), len(data)) + data
    return body + struct.pack("<H", crc16_modbus(body))


def build_write_register(modbus_id: int, address: int, value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise ValueError("invalid Modbus register value")
    body = struct.pack(">BBHH", modbus_id, 0x06, address, value)
    return body + struct.pack("<H", crc16_modbus(body))


def read_valid_frame(port: serial.Serial, expected_data_bytes: int) -> bytes:
    deadline = time.monotonic() + port.timeout
    buffer = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(max(1, port.in_waiting))
        if not chunk:
            continue
        buffer.extend(chunk)

        while len(buffer) >= 5:
            if buffer[0] != 1 or buffer[1] != 0x03:
                del buffer[0]
                continue
            frame_length = 3 + buffer[2] + 2
            if frame_length > 260:
                del buffer[0]
                continue
            if len(buffer) < frame_length:
                break
            frame = bytes(buffer[:frame_length])
            del buffer[:frame_length]
            expected_crc = int.from_bytes(frame[-2:], "little")
            if crc16_modbus(frame[:-2]) != expected_crc:
                continue
            if frame[2] == expected_data_bytes:
                return frame
    raise TimeoutError(f"no {expected_data_bytes}-byte Modbus response")


def read_registers(port: serial.Serial, address: int, count: int) -> list[int]:
    port.reset_input_buffer()
    port.write(build_read(1, address, count))
    port.flush()
    frame = read_valid_frame(port, count * 2)
    return [int.from_bytes(frame[3 + index * 2 : 5 + index * 2], "big") for index in range(count)]


def read_write_ack(
    port: serial.Serial, function: int, address: int, count_or_value: int
) -> None:
    deadline = time.monotonic() + port.timeout
    buffer = bytearray()
    expected = struct.pack(">BBHH", 1, function, address, count_or_value)
    while time.monotonic() < deadline:
        chunk = port.read(max(1, port.in_waiting))
        if not chunk:
            continue
        buffer.extend(chunk)
        while len(buffer) >= 8:
            if buffer[0] != 1:
                del buffer[0]
                continue
            function = buffer[1]
            if function == 0x03:
                frame_length = 3 + buffer[2] + 2
            elif function in (0x06, 0x10):
                frame_length = 8
            else:
                del buffer[0]
                continue
            if frame_length > 260:
                del buffer[0]
                continue
            if len(buffer) < frame_length:
                break
            frame = bytes(buffer[:frame_length])
            del buffer[:frame_length]
            if crc16_modbus(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
                continue
            if frame[:6] == expected:
                return
    raise TimeoutError(f"no Modbus write acknowledgement for address {address}")


def write_registers(port: serial.Serial, address: int, values: list[int]) -> None:
    port.reset_input_buffer()
    port.write(build_write_registers(1, address, values))
    port.flush()
    read_write_ack(port, 0x10, address, len(values))


def write_register(port: serial.Serial, address: int, value: int) -> None:
    port.reset_input_buffer()
    port.write(build_write_register(1, address, value))
    port.flush()
    read_write_ack(port, 0x06, address, value)


@dataclass
class LinkPgConfig:
    port: str
    baud_bps: int | None
    modbus_id: int
    ranging_method: str
    positioning_mode: str
    role: str
    secondary_anchor_id: int
    tag_id: int
    channel_code: int
    air_rate_code: int
    enabled_anchor_mask: int
    anchor_coordinates_cm: dict[str, list[int]]
    run_mode: str
    hardware_positioning: bool
    tag_count: int
    tag_ids: list[int]
    tag_output_enabled: bool
    tag_output_flags: int
    tag_output_protocol: str
    output_flags: int


def probe(port_name: str, timeout: float) -> LinkPgConfig:
    with serial.Serial(
        port_name,
        115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        write_timeout=timeout,
        rtscts=False,
        dsrdtr=False,
    ) as port:
        basic = read_registers(port, 0, 6)
        anchor_masks = read_registers(port, 9, 2)
        coordinates = read_registers(port, 11, 12)
        positioning = read_registers(port, 59, 18)
        output_config = read_registers(port, 112, 3)

    mode_register = basic[2]
    role_register = basic[3]
    device_id_register = basic[4]
    radio_register = basic[5]
    positioning_control = positioning[0]
    tag_control = positioning[1]
    tag_count = tag_control & 0xFF
    packed_ids = positioning[2:]
    tag_ids: list[int] = []
    for packed in packed_ids:
        tag_ids.extend((packed & 0xFF, packed >> 8))

    return LinkPgConfig(
        port=port_name,
        baud_bps=BAUD_CODES.get(basic[0]),
        modbus_id=basic[1],
        ranging_method="HDS-TWR" if (mode_register >> 8) == 1 else "DS-TWR+",
        positioning_mode=POSITION_NAMES.get(mode_register & 0xFF, f"unknown_{mode_register & 0xFF}"),
        role=ROLE_NAMES.get(role_register, f"unknown_{role_register}"),
        secondary_anchor_id=device_id_register >> 8,
        tag_id=device_id_register & 0xFF,
        channel_code=radio_register >> 8,
        air_rate_code=radio_register & 0xFF,
        enabled_anchor_mask=(anchor_masks[0] << 16) | anchor_masks[1],
        anchor_coordinates_cm={
            name: coordinates[index * 3 : index * 3 + 3]
            for index, name in enumerate(("A", "B", "C", "D"))
        },
        run_mode=RUN_MODE_NAMES.get(positioning_control, f"unknown_{positioning_control}"),
        hardware_positioning=(tag_control >> 8) == 1,
        tag_count=tag_count,
        tag_ids=tag_ids[:tag_count],
        tag_output_enabled=(output_config[0] >> 8) == 1,
        tag_output_flags=output_config[0] & 0xFF,
        tag_output_protocol={0: "ascii", 1: "modbus"}.get(
            output_config[1], f"unknown_{output_config[1]}"
        ),
        output_flags=output_config[2],
    )


def save_evidence(config: LinkPgConfig, suffix: str, evidence_dir: Path) -> Path:
    evidence_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = evidence_dir / f"main_anchor_A_{timestamp}_{suffix}.json"
    path.write_text(json.dumps(asdict(config), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def configure_main_2d(
    port_name: str, timeout: float, ab_cm: int, bc_cm: int, evidence_dir: Path
) -> LinkPgConfig:
    before = probe(port_name, timeout)
    if before.role != "main_anchor":
        raise RuntimeError(f"refusing configuration because {port_name} is {before.role}, not main_anchor")
    before_path = save_evidence(before, "before", evidence_dir)

    mode_value = 0x0101 if before.ranging_method == "HDS-TWR" else 0x0001
    coordinates = [
        0, 0, 0,
        ab_cm, 0, 0,
        ab_cm, bc_cm, 0,
        0, bc_cm, 0,
    ]
    with serial.Serial(
        port_name,
        115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        write_timeout=timeout,
        rtscts=False,
        dsrdtr=False,
    ) as port:
        if before.run_mode != "stopped":
            write_register(port, 59, 0)
            time.sleep(0.25)

        # Coordinates are accepted while still in ranging mode. Writing them
        # first avoids asking the firmware to enter 2D mode with a degenerate
        # all-zero anchor layout.
        write_registers(port, 11, coordinates)
        time.sleep(0.25)
        if read_registers(port, 11, len(coordinates)) != coordinates:
            raise RuntimeError("coordinate write did not verify before mode change")

        write_register(port, 2, mode_value)
        time.sleep(0.5)
        write_register(port, 59, 4)
        time.sleep(0.25)

    after = probe(port_name, timeout)
    expected_coordinates = {
        "A": [0, 0, 0],
        "B": [ab_cm, 0, 0],
        "C": [ab_cm, bc_cm, 0],
        "D": [0, bc_cm, 0],
    }
    if after.positioning_mode != "2d":
        raise RuntimeError(f"2D mode verification failed: {after.positioning_mode}")
    if after.anchor_coordinates_cm != expected_coordinates:
        raise RuntimeError(f"coordinate verification failed: {after.anchor_coordinates_cm}")
    if after.run_mode != "continuous_auto_output":
        raise RuntimeError(f"run-mode verification failed: {after.run_mode}")
    after_path = save_evidence(after, "after", evidence_dir)
    print(f"evidence_before={before_path}")
    print(f"evidence_after={after_path}")
    return after


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port such as COM12")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--list", action="store_true", help="list candidate serial ports and exit")
    parser.add_argument("--configure-main-2d", action="store_true")
    parser.add_argument("--ab-cm", type=int, default=150)
    parser.add_argument("--bc-cm", type=int, default=170)
    parser.add_argument("--confirm-write", default="")
    parser.add_argument("--evidence-dir", type=Path, default=Path("docs/uwb_config_evidence"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.list or not args.port:
        ports = [
            {"device": item.device, "description": item.description, "manufacturer": item.manufacturer}
            for item in list_ports.comports()
        ]
        print(json.dumps(ports, ensure_ascii=False, indent=2))
        return 0 if args.list else 2

    if args.configure_main_2d:
        if args.confirm_write != "CONFIGURE_MAIN_A_2D":
            raise SystemExit("refusing write: pass --confirm-write CONFIGURE_MAIN_A_2D")
        if not 1 <= args.ab_cm <= 10000 or not 1 <= args.bc_cm <= 10000:
            raise SystemExit("AB/BC dimensions must be between 1 and 10000 cm")
        config = configure_main_2d(
            args.port, args.timeout, args.ab_cm, args.bc_cm, args.evidence_dir
        )
    else:
        config = probe(args.port, args.timeout)
    data = asdict(config)
    if args.json:
        print(json.dumps(data, ensure_ascii=False, indent=2))
    else:
        for key, value in data.items():
            print(f"{key}={value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
