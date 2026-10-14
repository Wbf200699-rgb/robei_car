from app.protocol import (
    AckEnvelope,
    CommandEnvelope,
    FragmentReassembler,
    ProtocolError,
    build_ack,
    build_command,
    build_fragment,
    crc8,
    parse_envelope,
    parse_position_message,
)


def test_crc8_known_vector() -> None:
    assert crc8(b"123456789") == 0xF4


def test_command_round_trip_and_zero_padding() -> None:
    payload = build_command(0x2A, "W")
    assert len(payload) == 7
    assert parse_envelope(payload + b"\x00\x00\x00") == CommandEnvelope(0x2A, "W")


def test_ack_round_trip() -> None:
    assert parse_envelope(build_ack(0xFE, "+", 1)) == AckEnvelope(0xFE, "+", 1)


def test_corrupt_crc_is_rejected() -> None:
    payload = bytearray(build_command(1, "C"))
    payload[-1] = ord("0") if payload[-1] != ord("0") else ord("1")
    try:
        parse_envelope(bytes(payload))
    except ProtocolError:
        pass
    else:
        raise AssertionError("corrupt command CRC was accepted")


def test_two_fragment_status_reassembly() -> None:
    line = b"@S" + bytes(range(ord("0"), ord("0") + 83))
    first = parse_envelope(build_fragment(7, 0, 2, line[:64]))
    second = parse_envelope(build_fragment(7, 1, 2, line[64:]))
    reassembler = FragmentReassembler()
    assert reassembler.add(1, first, now=1.0) is None
    assert reassembler.add(1, second, now=1.1) == line


def test_position_message() -> None:
    report = parse_position_message("POS,9,1,0001000F,100,200,0,101,102,103,104")
    assert report.position_valid
    assert report.distances_cm == (101, 102, 103, 104)
