# LinkPG UWB gateway protocol

This document defines the first wire-compatible protocol between the stationary
ESP32-CAM gateway, the LinkPG main anchor, the vehicle LinkPG tag, the FPGA, and
the server. It deliberately leaves anchor coordinates configurable because the
anchors have not yet been placed and measured.

## Physical topology

```text
mobile app <-> server <-> Wi-Fi/WebSocket <-> ESP32-CAM
                                               |
                                      115200 8N1 TTL UART
                                               |
                                     LinkPG main anchor A
                                               |
                                  UWB positioning + data pass
                                               |
                     vehicle tag 1 <-> 115200 8N1 <-> FPGA

vehicle tags 0, 2, and 3 are positioning-only and require power and ground.
```

The ESP32-CAM connects to the TTL pins of main anchor A. The FPGA connects to
the TTL pins of vehicle tag 1. Do not use the USB and TTL sides of the same
LinkPG UART concurrently. All crossed UART connections are TX-to-RX and all
devices must share ground.

## LinkPG framing used

The implementation follows the vendor Code V6.6 protocol:

- Main-anchor positioning frame marker: `CA DA`.
- Data-pass receive frame marker: `ED DA`.
- UART configuration: 115200 bit/s, 8 data bits, no parity, 1 stop bit.
- Data-pass command: `AT+DataSend="<payload>","<target>"\r\n`.
- Main-anchor target ID is `255`; the vehicle command target is protocol tag
  ID `1` by default. Direct USB register probing on 2026-08-04 confirmed that
  the physical module labelled tag 1 reports protocol ID 1.
- Positioning must be running before data pass is available.
- A data-pass payload is at most 80 bytes. This project limits it to 74 bytes.

Every received Modbus-style frame is checked with CRC-16/Modbus. CRC bytes on
the wire are low byte first.

## Vehicle application envelopes

Only printable ASCII is placed inside `AT+DataSend`. Hex fields are uppercase.
All application envelopes end with a two-digit CRC-8, polynomial `0x07`,
initial value `0x00`, no reflection, calculated over every preceding envelope
byte.

### Command

```text
C1SSKHH
```

| Field | Width | Meaning |
| --- | ---: | --- |
| `C` | 1 | command envelope |
| `1` | 1 | protocol version |
| `SS` | 2 | 8-bit command sequence |
| `K` | 1 | one existing vehicle command byte |
| `HH` | 2 | CRC-8 |

The server allocates a new sequence for every intentional command. A network
retry reuses the same sequence. The FPGA suppresses a duplicate sequence inside
a two-second window, which prevents `+` or `-` from being applied twice.

### Command acknowledgement

```text
A1SSKRHH
```

| `R` | Meaning |
| --- | --- |
| `0` | accepted and injected into the existing vehicle command UART |
| `1` | duplicate; the original command was already accepted |
| `2` | rejected because the local command queue was unavailable |
| `3` | command-UART collision or local injection failure |

This acknowledgement proves delivery to the FPGA bridge. Existing `@K`, `@S`,
and `@L` reports remain the authoritative application state; `@R` carries the
additive high-resolution lidar scan data.

### Existing vehicle telemetry fragment

```text
F1MMPTLL<data>HH
```

| Field | Width | Meaning |
| --- | ---: | --- |
| `F` | 1 | telemetry fragment |
| `1` | 1 | protocol version |
| `MM` | 2 | telemetry message sequence |
| `P` | 1 | zero-based fragment index |
| `T` | 1 | total fragment count |
| `LL` | 2 | data length in this fragment |
| `data` | 0-64 | bytes from an existing FPGA UART line |
| `HH` | 2 | CRC-8 |

The trailing newline from the original line is omitted. The server reassembles
the fragments and restores the line as `@S...`, `@L...`, `@R...`, `@K...`, or
`@PONG`. This keeps the current mobile protocol intact. The 85-byte `@S` body
uses two UWB fragments; the 55-byte `@L` body and 49-byte `@R` body each use
one. Six matching `@R` lines form one 72-point scan; their inner CRC-8 remains
mandatory after UWB fragment validation.

## ESP32-to-server WebSocket messages

The gateway uses UTF-8 text messages with comma-separated fields. No field may
contain a comma.

Gateway to server:

```text
HELLO,1,<gateway_id>,<firmware_version>
STATUS,<uptime_ms>,<rssi>,<good_frames>,<crc_errors>,<uwb_tx>,<queue_drops>,<rx_bytes>,<framing_errors>,<position_frames>,<pass_frames>,<raw_frames>,<rx_idle_ms>,<positioning_wanted>,<position_idle_ms>,<position_start_requests>
POS,<gateway_seq>,<tag>,<flags_hex>,<x_cm>,<y_cm>,<z_cm>,<A_cm>,<B_cm>,<C_cm>,<D_cm>
PASS,<gateway_seq>,<sender_id>,<payload_hex>
RAW,<gateway_seq>,<frame_hex>
SENDQ,<request_id>,<0-or-error>
PONG,<nonce>
```

Server to gateway:

```text
SEND,<request_id>,<target_tag>,<payload_hex>
UWB_START
UWB_STOP
UWB_READ,<request_id>,<register_address>,<register_count>
PING,<nonce>
```

`UWB_START` writes transient LinkPG register 59 value `4` (continuous automatic
output). It does not enable persistent boot-time measurement. `UWB_STOP` writes
value `0`. `UWB_READ` is a read-only diagnostic command using Modbus function
`0x03`; its response is reported as a CRC-checked `RAW` frame and retained in
the server state.

Gateway firmware 0.3.0 keeps `positioning_wanted` as an explicit target state.
While it is true, a start write is retried only when no position frame has been
received for 2.5 seconds, with at least 2 seconds between retries. `UWB_STOP`
sets it false before queuing the stop write, so intentional shutdown is not
undone by the watchdog. `position_start_requests` counts both the initial write
and later recovery writes.

Server command state is reported in three stages: acceptance by the network
API, `SENDQ` acceptance by the ESP32 UART queue, and the vehicle `A1` envelope.
Commands without a vehicle acknowledgement expire after 3 seconds. A queue ACK
or UWB ACK is transport evidence only; the latest `@S` remains the source of
truth for actual vehicle mode and motion state.

## Four-tag vehicle pose

Direct USB probing confirmed the four physical protocol tag IDs as 0, 1, 2,
and 3. They are currently placed randomly, so no tag has a valid vehicle-corner
role yet. Set the front-left, front-right, rear-left, and rear-right mapping only
after final mechanical mounting.
After the tags are mounted, set the actual rectangle length and width.

For a synchronized valid set, the server computes:

- center: mean of the four tag coordinates;
- front midpoint: mean of front-left and front-right;
- rear midpoint: mean of rear-left and rear-right;
- heading: `atan2(front_y - rear_y, front_x - rear_x)`;
- rigidity checks: measured front/rear widths and left/right lengths compared
  with the configured vehicle rectangle.

No pose is marked `full` until all four coordinates are fresh, the LinkPG
position-success bit is set, and the configured rectangle checks pass. Before
anchor placement, positioning frames are still forwarded, but zero coordinates
must not be treated as a valid map position.
