from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Union


PROTOCOL_VERSION = "1"
MAX_UWB_PAYLOAD = 74
MAX_FRAGMENT_DATA = 64


class ProtocolError(ValueError):
    pass


def crc8(data: bytes) -> int:
    """CRC-8, polynomial 0x07, init 0x00, no reflection."""
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def _hex_byte(value: int) -> bytes:
    return f"{value & 0xFF:02X}".encode("ascii")


def _parse_hex_byte(raw: bytes, field: str) -> int:
    if len(raw) != 2:
        raise ProtocolError(f"{field} must contain two hex digits")
    try:
        return int(raw.decode("ascii"), 16)
    except (UnicodeDecodeError, ValueError) as exc:
        raise ProtocolError(f"{field} is not hexadecimal") from exc


@dataclass(frozen=True)
class CommandEnvelope:
    sequence: int
    command: str


@dataclass(frozen=True)
class AckEnvelope:
    sequence: int
    command: str
    result: int


@dataclass(frozen=True)
class FragmentEnvelope:
    message_sequence: int
    part: int
    total: int
    data: bytes


Envelope = Union[CommandEnvelope, AckEnvelope, FragmentEnvelope]


def build_command(sequence: int, command: str) -> bytes:
    if len(command) != 1 or ord(command) < 0x20 or ord(command) > 0x7E:
        raise ProtocolError("command must be one printable ASCII character")
    body = b"C" + PROTOCOL_VERSION.encode("ascii") + _hex_byte(sequence) + command.encode("ascii")
    payload = body + _hex_byte(crc8(body))
    if len(payload) > MAX_UWB_PAYLOAD:
        raise AssertionError("command envelope exceeds LinkPG payload limit")
    return payload


def build_ack(sequence: int, command: str, result: int) -> bytes:
    if len(command) != 1 or not 0 <= result <= 3:
        raise ProtocolError("invalid acknowledgement fields")
    body = (
        b"A"
        + PROTOCOL_VERSION.encode("ascii")
        + _hex_byte(sequence)
        + command.encode("ascii")
        + bytes((ord("0") + result,))
    )
    return body + _hex_byte(crc8(body))


def build_fragment(message_sequence: int, part: int, total: int, data: bytes) -> bytes:
    if not 1 <= total <= 15 or not 0 <= part < total:
        raise ProtocolError("invalid fragment index/count")
    if len(data) > MAX_FRAGMENT_DATA:
        raise ProtocolError("fragment data exceeds 64 bytes")
    body = (
        b"F"
        + PROTOCOL_VERSION.encode("ascii")
        + _hex_byte(message_sequence)
        + f"{part:X}{total:X}".encode("ascii")
        + _hex_byte(len(data))
        + data
    )
    payload = body + _hex_byte(crc8(body))
    if len(payload) > MAX_UWB_PAYLOAD:
        raise AssertionError("fragment envelope exceeds LinkPG payload limit")
    return payload


def _require_crc(payload: bytes, body_length: int) -> bytes:
    expected_length = body_length + 2
    if len(payload) < expected_length:
        raise ProtocolError("truncated envelope")
    body = payload[:body_length]
    expected = _parse_hex_byte(payload[body_length:expected_length], "CRC-8")
    if crc8(body) != expected:
        raise ProtocolError("CRC-8 mismatch")
    if any(payload[expected_length:]):
        raise ProtocolError("non-zero bytes follow envelope")
    return body


def parse_envelope(payload: bytes) -> Envelope:
    if len(payload) < 2 or payload[1:2] != PROTOCOL_VERSION.encode("ascii"):
        raise ProtocolError("unsupported or missing envelope version")

    packet_type = payload[:1]
    if packet_type == b"C":
        body = _require_crc(payload, 5)
        command = chr(body[4])
        if not 0x20 <= body[4] <= 0x7E:
            raise ProtocolError("command is not printable ASCII")
        return CommandEnvelope(_parse_hex_byte(body[2:4], "sequence"), command)

    if packet_type == b"A":
        body = _require_crc(payload, 6)
        if body[5] not in b"0123":
            raise ProtocolError("unknown acknowledgement result")
        return AckEnvelope(
            sequence=_parse_hex_byte(body[2:4], "sequence"),
            command=chr(body[4]),
            result=body[5] - ord("0"),
        )

    if packet_type == b"F":
        if len(payload) < 10:
            raise ProtocolError("truncated fragment header")
        try:
            part = int(chr(payload[4]), 16)
            total = int(chr(payload[5]), 16)
        except ValueError as exc:
            raise ProtocolError("fragment index/count is not hexadecimal") from exc
        data_length = _parse_hex_byte(payload[6:8], "fragment length")
        if data_length > MAX_FRAGMENT_DATA:
            raise ProtocolError("fragment data exceeds 64 bytes")
        body_length = 8 + data_length
        body = _require_crc(payload, body_length)
        if not 1 <= total <= 15 or not 0 <= part < total:
            raise ProtocolError("invalid fragment index/count")
        return FragmentEnvelope(
            message_sequence=_parse_hex_byte(body[2:4], "message sequence"),
            part=part,
            total=total,
            data=body[8:],
        )

    raise ProtocolError("unknown envelope type")


@dataclass
class _PartialMessage:
    total: int
    parts: dict[int, bytes]
    updated_at: float


class FragmentReassembler:
    def __init__(self, timeout_s: float = 2.0) -> None:
        self.timeout_s = timeout_s
        self._partial: dict[tuple[int, int], _PartialMessage] = {}

    def add(self, sender_id: int, fragment: FragmentEnvelope, now: float | None = None) -> bytes | None:
        timestamp = time.monotonic() if now is None else now
        self.expire(timestamp)
        key = (sender_id, fragment.message_sequence)

        current = self._partial.get(key)
        if fragment.part == 0 or current is None or current.total != fragment.total:
            current = _PartialMessage(total=fragment.total, parts={}, updated_at=timestamp)
            self._partial[key] = current

        current.parts[fragment.part] = fragment.data
        current.updated_at = timestamp
        if len(current.parts) != current.total:
            return None

        message = b"".join(current.parts[index] for index in range(current.total))
        del self._partial[key]
        return message

    def expire(self, now: float | None = None) -> None:
        timestamp = time.monotonic() if now is None else now
        stale = [key for key, value in self._partial.items() if timestamp - value.updated_at > self.timeout_s]
        for key in stale:
            del self._partial[key]


@dataclass(frozen=True)
class PositionReport:
    gateway_sequence: int
    tag_id: int
    flags: int
    x_cm: int
    y_cm: int
    z_cm: int
    distances_cm: tuple[int, int, int, int]

    @property
    def position_valid(self) -> bool:
        return bool(self.flags & (1 << 16))


def parse_position_message(message: str) -> PositionReport:
    fields = message.split(",")
    if len(fields) != 11 or fields[0] != "POS":
        raise ProtocolError("invalid POS message")
    try:
        return PositionReport(
            gateway_sequence=int(fields[1]),
            tag_id=int(fields[2]),
            flags=int(fields[3], 16),
            x_cm=int(fields[4]),
            y_cm=int(fields[5]),
            z_cm=int(fields[6]),
            distances_cm=(int(fields[7]), int(fields[8]), int(fields[9]), int(fields[10])),
        )
    except ValueError as exc:
        raise ProtocolError("invalid numeric field in POS message") from exc
