# Mobile App Handoff

Hardware baseline commit: Bluetooth manual drive, lidar control, eight-sector
telemetry, and rectangular-footprint obstacle protection.

## BLE transport

- CH9141K transparent BLE UART
- App writes FFF2 and subscribes to FFF1 notifications
- UART protocol is ASCII, 115200 8N1
- Send `P` every second; FPGA replies `@PONG\n`

## Operating modes

- `H`: enter manual mode and stop
- `O`: return to automatic route mode and stop manual motion
- `E`: enable lidar scanning
- `Q`: stop and disable lidar scanning

Accepted mode commands reply with `@K<key>\n`.

## Manual drive

Manual mode commands:

- `W` or `U`: forward
- `S`: backward
- `A` or `L`: turn left in place
- `D` or `R`: turn right in place
- `C` or `M`: stop

For a held direction, send the direction immediately and repeat every 100 ms.
Send `C` on touch release or cancellation. FPGA stops automatically if no
direction refresh arrives for 300 ms.

`C`/`M` is mode-preserving. In manual mode it stops manual motion and remains
in manual mode. In automatic route mode it also acts as the middle/confirm key
and remains in automatic mode, so confirming "edit route" cannot accidentally
transfer motor ownership to manual control.

Automatic mode keeps the existing route-command behavior. Physical car keys
remain independent and are only used for route editing/running.

## Parameter control

Parameter setup is Bluetooth-only:

- `V`: overall speed
- `T`: route step length
- `1`, `2`, `3`, `4`: LF, RF, LR, RR default wheel speed
- `5` or `Z`: lidar safe distance
- `+` / `-`: increase/decrease selected parameter
- `N` / `B`: next/previous parameter
- `G`: toggle parameter setting mode

Ultrasonic obstacle behavior is unchanged.

## Status frames

Route status is:

```text
@S<state><menu><active><done><now_x><now_y><target_x><target_y><route><step><speed><lf><rf><lr><rr><safe><setting><selected><lidar_online><manual><lidar_enable><manual_cmd><watchdog><motor_blocked><c5_count><c5_lf><c5_rf><c5_lr><c5_rr><stop_done><stop_left><c5_busy><seq><cal_state><cal_flags><learned_trim><yaw_corr><gyro_peak>\n
```

The line has no spaces and is exactly 99 bytes including `@` and `\n`:

| Field | Hex digits |
| --- | ---: |
| `state` | 2 |
| `menu` | 1 |
| `active`, `done` | 2 each |
| `now_x`, `now_y`, `target_x`, `target_y` | 2 each |
| `route` | 16 |
| `step` | 4 |
| `speed` | 3 |
| `lf`, `rf`, `lr`, `rr` default wheel speeds | 3 each |
| `safe` lidar distance | 4 |
| `setting`, `selected`, `lidar_online`, `manual`, `lidar_enable` | 1 each |
| `manual_cmd`, `watchdog`, `motor_blocked` | 1 each |
| `c5_count` | 4 |
| `c5_lf`, `c5_rf`, `c5_lr`, `c5_rr` last pulse widths | 3 each |
| `stop_done`, `stop_left`, `c5_busy` | 1 each |
| `seq` | 2 |
| `cal_state` | 1 |
| `cal_flags` | 2 |
| `learned_trim`, `yaw_corr` | 3 each, signed 12-bit hexadecimal |
| `gyro_peak` | 4 |

`seq` is an 8-bit sequence number encoded as two hexadecimal digits. A fresh
route frame is queued immediately when the route, current position, target,
state, or completed/active step count changes. Sending `?` always queues the
latest `@S` frame instead of waiting for periodic telemetry. Control and
configuration commands also queue a fresh `@S` after their `@K` reply.
`c5_count` increments after every complete C5 motor command frame. The four
`c5_*` pulse fields report the actual last transmitted pulse widths; neutral is
hex `5DC` (1500 decimal). A stop event sends the C5 controller's broadcast stop
command `#255PDST!` three times; motion still uses the four-wheel command frame.
`stop_done=1` means that burst completed, `stop_left` is the remaining frame
count, and `c5_busy=1` means a frame is currently being serialized. No motor
commands are periodically transmitted while idle.

`X` starts one complete FPGA-local calibration run. The phone must write `X`
once; it must not keep sending motion commands. The FPGA forces manual stop,
runs left/right polarity checks, learns straight-line yaw trim, reverses back,
and sends the protected stop sequence. `C`, `H`, `O`, or a manual direction
while calibration is busy aborts the run safely. `cal_state` values are:

| Value | State |
| --- | --- |
| `0` | idle |
| `1` | pre-stop/settle |
| `2`, `3` | spin left, settle |
| `4`, `5` | spin right, settle and verify |
| `6`, `7` | forward trim learning, settle |
| `8` | reverse return |
| `9` | final protected stop |
| `A` | completed |
| `B` | failed or aborted |

`cal_flags` bits are bit 0 busy, bit 1 valid result, bit 2 failed, bit 3
runtime correction active, bit 4 gyro polarity valid, bit 5 positive gyro is
physical right turn, bit 6 IMU sensors ready, and bit 7 obstacle active.
`learned_trim` is the retained runtime left/right baseline and `yaw_corr` is
the currently applied total correction. These values survive mode changes but
not FPGA power loss yet; persistent Flash storage is a later protocol version.

Lidar status:

```text
@L<online><lidar_stop><ultrasonic_stop><merged_stop><motor_blocked><ultrasonic_valid><ultrasonic_timeout><safe_mm><ultrasonic_cm><front_mm><s0>...<s7><seq>\n
```

The lidar line is exactly 56 bytes including `@` and `\n`. The first seven
status fields use one hexadecimal digit each. Every distance uses four digits,
and `seq` uses two digits. `motor_blocked` is the final decision applied by the
motor controller; the three obstacle fields identify the source. Ultrasonic
distance, valid, timeout, and stop fields hold the most recent completed
measurement until the next measurement arrives.

All distances are four hexadecimal digits in millimeters. `FFFF` means no
valid sample. `seq` is an independent 8-bit lidar-frame sequence number.
Distances use a three-scan median filter per sector. Sector order:

1. front
2. front-right
3. right
4. rear-right
5. rear
6. rear-left
7. left
8. front-left

UART output frames are atomic. `@K` and `@PONG` replies wait until an active
`@S`, `@L`, `@V`, `@M2`, or `@R` frame has finished, so frame bytes are never interleaved.
Both sequence counters wrap from `FF` to `00`.

## Five-channel VL53L1X discovery frames

The five fixed sensor indices are reported independently of their eventual
physical direction assignment:

```text
@VSSFFVVDDD0DDD1DDD2DDD3DDD4EEEE\n
```

The line is exactly 33 bytes including `@` and newline. All variable fields are
uppercase hexadecimal:

| Field | Hex digits | Meaning |
| --- | ---: | --- |
| `SS` | 2 | Independent 8-bit VL53 frame sequence |
| `FF` | 2 | Bit 7: initialization complete; bits 4:0: online mask for indices 4..0 |
| `VV` | 2 | Bits 4:0: valid-distance mask for indices 4..0 |
| `DDD0` ... `DDD4` | 4 each | Distance in millimeters for sensor indices 0 through 4 |
| `EEEE` | 4 | Saturating/rolling I2C transaction error count |

`VV` is authoritative for validity. `DDD0` through `DDD4` preserve the raw
16-bit distance even when the corresponding validity bit is clear: `0000`
means the sensor returned zero, a value above `0FA0` means it exceeded the
current 4000 mm acceptance limit, and `FFFF` is an unavailable/legacy-invalid
sample. The FPGA reports the frame at 2 Hz. During installation discovery,
cover one module at a time and observe which numbered distance changes; only
after that test should the index-to-direction mapping be stored. `@V` shares
the same UART frame scheduler as command replies, route status, lidar status,
and high-resolution map data.

## High-resolution lidar map frames

The 8-sector `@L` line remains the low-latency safety/status source. Detailed
display and mapping use this additive frame:

```text
@R1SSTTTTP6<DDD x 12>CC\n
```

Each line is exactly 50 bytes including `@` and newline. `SS` is an 8-bit scan
sequence, `TTTT` is a 16-bit FPGA tick in 10 ms units, and `P` is the part
index from `0` through `5`. All fields are uppercase hexadecimal except the
literal version/count characters. `CC` is CRC-8 polynomial `0x07`, initial
value `0x00`, calculated over bytes from `R` through the last point digit.

Each `DDD` is one 12-bit point:

```text
quality_class = (DDD >> 10) & 3
distance_code = DDD & 0x3FF
distance_mm = distance_code * 16
angle_deg = (P * 12 + point_index) * 5 + 2.5
```

`distance_code=0x3FF` means invalid and must not be plotted as a real range.
Commit a scan only after all six parts have matching `SS` and `TTTT` values and
valid CRCs. A new part `0`, a sequence/tick change, a timeout, or a missing
part invalidates the incomplete older scan; never mix parts from two scans.
The default complete-scan report rate is 2 Hz.

Manual command values are `0=stop`, `1=forward`, `2=reverse`, `3=spin left`,
and `4=spin right`. `watchdog=1` means the 300 ms manual command watchdog
stopped the previous movement. When an obstacle is active, manual forward is
blocked, while reverse and in-place turns remain available for escape.

The app can render a coarse low-latency radar from `@L` and a 72-point display
from complete `@R` scans. Permanent mapping still requires timestamped vehicle
pose, lidar extrinsic calibration, motion compensation, and multi-frame grid
fusion on the server.

## Dual BMI088 and PMW3901 motion frames

The FPGA reports a coherent snapshot at 10 Hz through the same atomic UART
scheduler:

```text
@M2SSSSFFQQXXXXYYYYAAAAAAAABBBBBBBBI0AXI0AYI0AZI0GXI0GYI0GZI1AXI1AYI1AZI1GXI1GYI1GZ\n
```

The line is exactly 84 bytes including `@` and newline. All variable fields are
uppercase hexadecimal:

| Field | Hex digits | Meaning |
| --- | ---: | --- |
| literal `M2` | 2 | Motion protocol version 2 |
| `SSSS` | 4 | 16-bit sensor sample sequence |
| `FF` | 2 | Health/status bitmap |
| `QQ` | 2 | PMW3901 SQUAL surface quality |
| `XXXX`, `YYYY` | 4 each | Signed 16-bit latest optical-flow delta |
| `AAAAAAAA`, `BBBBBBBB` | 8 each | Signed 32-bit accumulated flow X/Y |
| `I0A*`, `I1A*` | 4 each | Signed 16-bit BMI088 accelerometer raw axes |
| `I0G*`, `I1G*` | 4 each | Signed 16-bit bias-corrected gyro raw axes |

`FF` bits are: bit 0 PMW3901 online, bit 1 BMI0 online, bit 2 BMI1
online, bit 3 stationary gyro calibration complete, bit 4 fused sample ready,
bit 5 single-IMU degraded mode, bit 6 IMU disagreement, and bit 7 at least one
valid optical-flow motion sample seen.

The Android app converts acceleration with `5460 LSB/g` and gyro rate with
`131.2 LSB/(degree/s)`. It runs one complementary filter per BMI088 and shows
roll, pitch, and yaw relative to the pose at connection/calibration completion.
Yaw is deliberately labelled relative: BMI088 has no magnetometer, so this
frame cannot provide absolute north-referenced heading. PMW3901 values are
unscaled counts until the installed lens height and floor surface are calibrated.

Vehicle connector assignment is A14 SCLK, B15 MOSI, A13 MISO, B13 PMW3901 CS,
F20/F19 BMI0 accelerometer/gyro CS, and C20/E19 BMI1 accelerometer/gyro CS.
All sensor signals are 3.3 V LVCMOS and share ground. The separate UWB wiring
R2 to tag RX and tag TX to P5 is not consumed by this motion telemetry feature.

## Hybrid mapping profile (current)

The current profile keeps the 128 x 128 safety/occupancy grid and high-rate
IMU integration in FPGA, while Android owns editable world coordinates, UWB
translation, map visualization, profile management, and target planning.
Android still builds its 241 x 241 display map at 25 mm/cell from validated
`@R1` scans; `@G0` supplies the board-integrated heading and board health.

Before `J`, Android sends one 17-byte punctuation-only field frame twice:

```text
{XXYYHHHLLRRBBTT}
```

The 15 encoded nibbles use `!#$%&'()*,./:;<>` for hexadecimal `0..F`, so the
legacy single-byte command decoder ignores them. Fields are board-map start X/Y
cells, heading in 1/1024 turn, low/high X cells, and low/high Y cells. Cells are
50 mm. Android computes them from the current reliable UWB pose and the editable
A/C/D anchor coordinates; it reserves map padding outside the anchor rectangle
but shrinks the autonomous driving boundary by the vehicle circumradius plus
75 mm. Anchor coordinates are stored with each Android map profile and can be
edited for an unknown competition field without rebuilding FPGA RTL.

During mapping, Android displays UWB as the world translation and converts the
FPGA `@G0` local heading back to world heading. FPGA integrates every calibrated
BMI088 sample; Android must not integrate the low-rate `@M2` gyro snapshot for
mapping heading. UWB provides the absolute initial heading and remains the
fallback if `@G0` is stale or its calibration/polarity flags are invalid.

## FPGA-resident map and storage protocol

The following single-byte commands are additive and keep every older command
unchanged:

| Command | Operation |
| --- | --- |
| `J` | Start or resume FPGA-resident mapping |
| `K` | Pause mapping without clearing RAM |
| `6` | Clear the 128 x 128 map and reset local pose |
| `7` | Save map and metadata to the dedicated Flash slots |
| `8` | Load the newest valid Flash slot into map RAM |
| `9` | Request one complete 128-chunk map synchronization |

Each command first receives the normal atomic `@K<key>\n` acknowledgement.
Map status is a fixed 58-byte line including newline:

```text
@G0SSFFEEXXXXYYYYHHHNNNNOOOOQQQQDDDDPPORRJJIIIIIIWWWWWWWW\n
```

All fields are uppercase hexadecimal. `SS` is the status sequence, `FF` flags,
`EE` map epoch, `XXXX/YYYY` pose in millimeters, `HHH` heading in 1/1024 turn,
`NNNN` scan count, `OOOO/QQQQ` occupied/free cell counts, `DDDD` dropped
samples, `PP` pose flags, `O` Flash operation, `RR` progress, `JJ` error,
`IIIIII` JEDEC ID, and `WWWWWWWW` saved-map sequence.

`FF` bits 0..7 are map active, map paused, map valid, map RAM busy, Flash ready,
Flash busy, lidar online, and motion sensors ready. Flash operations are
`0=idle`, `1=boot check`, `2=save`, and `3=load`. Flash errors are `00=none`,
`01=JEDEC`, `02=not ready`, `03=address guard`, `04=no valid map`, `05=map
CRC`, and `06=timeout`.

One fixed 76-byte map chunk including newline is:

```text
@G1SSAAAA<64 hex data digits>CC\n
```

`AAAA` is a row-major cell base address from `0000` to `3FFF`. The 64 data
nibbles encode 128 cells low two bits first: `0=unknown`, `1=free`,
`2=occupied`; value `3` is reserved. `CC` is CRC-8 polynomial `0x07`, initial
`0x00`, over ASCII bytes from `G1` through the last data digit. The leading
`@`, CRC digits, and newline are excluded. A full transfer contains 128 unique
base addresses and intentionally takes tens of seconds under the BLE fairness
throttle; the app must accept wraparound sequence numbers and deduplicate by
base address within the current epoch.

The Android client automatically scans for the known vehicle address
`DC:32:62:C9:6B:C0`, connects after it is observed nearby, and retries with
bounded backoff after an unexpected disconnect. A manually selected address is
remembered. An explicit Disconnect action suppresses automatic reconnection
until the user connects again; this distinction must be preserved in future UI
changes.
