# Hardware parameter simulation suite

## Purpose and evidence boundary

This suite measures deterministic RTL parameters and protocol behavior without
changing the synthesized vehicle design. The results prove code-level timing,
mapping, boundaries, state transitions, filtering, and frame handling. They do
not replace physical measurements of motor RPM, wheel slip, sensor noise,
Bluetooth range, UWB RF accuracy, or HDMI signal integrity.

The Robei source tests are in `little_car/test/*.test` and are registered in:

- `little_car/little_car.proj`
- `robei_model_project/robei_model_project.proj`

Equivalent standalone Verilog is in `little_car/test/verilog/`. The checked
smoke-test evidence is in `little_car/test/results/hardware_parameters/`.

## VCD size policy

- Every simulation has a hard timeout.
- Long production waits are skipped only after their fixed production value is
  recorded; boundary clocks are still exercised.
- High-frequency clocks are omitted from long ultrasonic wave dumps.
- Only interface and decision signals are dumped, not the full DUT hierarchy.
- `run_hardware_parameter_tests.py` fails if a VCD exceeds 8 MiB.
- Current total for all nine VCD files is 26,951 bytes; largest is 11,494 bytes.

This policy avoids the previous failure mode where a full 500 ms UART timing
simulation produced a multi-gigabyte VCD.

## Test matrix

| Robei test | Main measurements | Current result | VCD bytes |
| --- | --- | --- | ---: |
| `power_on_reset_parameter.test` | POR length and asynchronous assertion | PASS | 942 |
| `hdmi_timing_parameter.test` | 1024x600 active/porch/sync boundaries | PASS | 1,703 |
| `ultrasonic_parameter.test` | trigger, distance conversion, threshold, timeout | PASS | 1,486 |
| `c5_motion_stabilizer_parameter.test` | polarity, PID slew/limit, stop and stale bypass | PASS | 2,503 |
| `dual_imu_calibration_parameter.test` | axes, means, peaks, online/noise rejection | PASS | 2,673 |
| `rplidar_protocol_parameter.test` | startup retry, descriptor, node decode, online gate | PASS | 1,775 |
| `lidar_map_sampler_parameter.test` | 72 bins, six chunks, CRC8, filtering | PASS | 2,258 |
| `onboard_exploration_parameter.test` | safety branches, boundary, pause/manual override | PASS | 2,117 |
| `linkpg_uart_parameter.test` | startup command, CRC16, position filter, bad CRC | PASS | 11,494 |

## 1. Power-on reset

Clock is 50 MHz (20 ns). `POR_CYCLES=1,000,000`, therefore the intended hold is
20,000 us (20 ms). The test skips to counter 999,990, observes the final 10
clocks, and separately asserts `ext_rst_n` between clocks.

Measured results:

- production POR: 1,000,000 clocks / 20,000 us
- accelerated observed tail: 10 clocks
- asynchronous reset assertion checked after 1 ns
- counter returns to zero and `system_rst_n` returns low

## 2. HDMI timing generator

The test probes every active/sync boundary and frame rollover rather than
dumping a complete 853,980-pixel frame.

| Parameter | Value |
| --- | ---: |
| Active resolution | 1024 x 600 |
| Horizontal front porch | 40 pixels |
| Horizontal sync | 100 pixels |
| Horizontal back porch | 160 pixels |
| Horizontal total | 1324 pixels |
| Vertical front porch | 12 lines |
| Vertical sync | 10 lines |
| Vertical back porch | 23 lines |
| Vertical total | 645 lines |
| Active pixels/frame | 614,400 |
| Blanking pixels/frame | 239,580 |
| Total pixels/frame | 853,980 |

Observed transitions: DE is high at `(0,0)` and `(1023,599)`, low at x=1024;
HSYNC is high from x=1064 through 1163; VSYNC is high from y=612 through 621;
the `(1323,644)` terminal count wraps to the first active line.

## 3. Ultrasonic guard

Fixed RTL parameters:

| Parameter | Cycles | Time |
| --- | ---: | ---: |
| Trigger pulse | 500 | 10 us |
| Echo timeout | 1,500,000 | 30 ms |
| Measurement period | 3,000,000 | 60 ms |
| Distance conversion | 2,900 cycles/cm | 58 us/cm |
| Default threshold | - | 20 cm |

Measured cases at a 20 cm threshold:

| Requested | Echo cycles | Decoded | Stop |
| ---: | ---: | ---: | ---: |
| 5 cm | 14,500 | 5 cm | 1 |
| 19 cm | 55,100 | 19 cm | 1 |
| 20 cm | 58,000 | 20 cm | 0 |
| 35 cm | 101,500 | 35 cm | 0 |

The comparison is strictly `< threshold`. Timeout returns `16'hFFFF`, asserts
`distance_valid` and `echo_timeout`, and does not assert obstacle stop.

## 4. C5 motion stabilizer

The fast-test macro shortens only waiting counters. Arithmetic constants and
decision logic remain the same:

- gyro deadband: 24 counts
- P/I/D right shifts: 5 / 7 / 7
- integral limit: +/-65,536
- yaw correction limit: +/-96 speed units
- yaw correction slew: 8 units per applied update
- learned trim limit: +/-80
- optical-flow correction limit: +/-32
- flow correction slew: 4
- polarity learning: four samples, minimum magnitude 80
- simulated runtime apply interval: 24 clocks

Observed behavior:

- four +116 samples during a right-spin command learn positive gyro as right
- +320 gyro while moving forward produces a bounded negative correction
- -320 gyro while reversing produces the opposite correction
- zero base command passes all four wheel outputs to zero immediately
- IMU disagreement does not disable a still-valid fallback stream
- stale fused samples disable correction and preserve 650/750/800/625 exactly

## 5. Dual-IMU static calibration

Acceptance constants from the DUT:

- gravity magnitude: 4,000 to 7,000 counts
- maximum secondary-axis acceleration: 2,500
- maximum absolute gyro mean: 128
- maximum gyro peak: 500

Nominal sample result:

- IMU0 vertical axis = Z, negative; acceleration mean = (30,-160,-5504)
- IMU1 vertical axis = Y, negative; acceleration mean = (1176,-5350,165)
- signed alternating gyro inputs average to zero
- one offline sample invalidates the run
- a 700-count stationary gyro sample fails the quietness gate

The Robei test uses 8 samples for a compact wave. The production model uses 256
samples and an 8-bit mean shift; the same accumulation/evaluation paths apply.

## 6. RPLIDAR C1 UART and safety data

Protocol parameters from the production RTL:

- clock / baud: 50 MHz / 460,800 bit/s
- startup sequence: STOP then SCAN
- descriptor retries: one bounded retry, then raw-node parsing fallback
- accepted quality: nonzero
- accepted range: 50 to 12,000 mm
- sectors: eight 45-degree sectors
- corner safety margin: 150 mm (`q2=600`)
- side guard: 220 mm (`q2=880`)
- nominal safe distance in this test: 450 mm

The standard 7-byte SCAN descriptor `A5 5A 05 00 00 40 81` is accepted. A valid
node at 0 degrees and 400 mm asserts online only after the sample is decoded;
receiving a descriptor alone is not treated as sensor health.

## 7. 72-bin lidar compression

- angular bins: 72, five degrees each
- report chunks: six
- words per chunk: 12
- encoded distance word: 12 bits
- invalid word: `FFF`
- minimum accepted quality: 8
- range filter: 50 to 12,000 mm
- CRC8 polynomial: `0x07`, initial value `0x00`

Observed representative bins:

- bin 0 = `84B`
- bin 1 = `C7D`
- bin 2 = `FFF` after quality rejection
- bin 5 = `464`
- bin 71 = `CBC`
- all six chunk CRC values matched an independent testbench function
- report drop count remained zero

## 8. Autonomous exploration safety

The compact test keeps the production decision branches but shortens action
leases. Production constants documented by the model are forward 90 ms,
reverse 120 ms, turn 300 ms, settle 650 ms, and scan timeout 1,000 ms.

Verified branches:

- clear fresh scan enters forward
- all-invalid front sectors never count as free space
- obstacle chooses reverse/turn escape rather than forward
- blocked front/rear/corner footprint does not command a scraping turn
- manual mode immediately removes auto command
- default 450..5950 mm boundary blocks outward travel
- configured 50 mm-cell field boundary is honored
- pause removes auto mode and commands stop

Command encoding is stop=0, forward=1, reverse=2, left=3, right=4. Stop reasons
are none=0, not armed=1, calibration=2, lidar=3, stale scan=4.

## 9. LinkPG UWB UART

Production UART is 115,200 8N1. The compact simulation changes clocks-per-bit
and wait counters only. It verifies the vendor register-59 start frame:

`01 10 00 3B 00 01 02 00 04 A3 18`

Measured protocol behavior:

- valid short coordinate frames decode signed X/Y/Z
- repeated samples are averaged; final example is (127,-42,11) cm
- the long AC DA coordinate form is accepted
- one intentionally corrupt CRC is rejected and increments the CRC counter
- CRC16 is Modbus polynomial `0xA001`, initial value `0xFFFF`
- position plausibility is bounded to +/-1000 cm with a 150 cm jump guard

## Reproduction

From `D:\Projects\Robei\little_car_new`:

```powershell
python tools\run_hardware_parameter_tests.py
```

The script regenerates all `.test` and standalone `.v` files, compiles with the
Icarus bundled by RobeiEDA when no PATH installation exists, runs every test,
requires a `RESULT TEST_PASS`, requires a VCD, checks the 8 MiB limit, and writes
`wave_manifest.csv` with SHA-256 hashes.

For RobeiEDA, open either project, select one `_parameter.test`, run simulation,
and stop after `RESULT TEST_PASS`. RobeiEDA chooses its own generated VCD path;
the standalone mirror is the reproducible small-wave reference.

## Handoff for waveform analysis

Give the other analysis session these files:

- `docs/HARDWARE_PARAMETER_SIMULATION.md`
- `little_car/test/results/hardware_parameters/wave_manifest.csv`
- the matching `.log` and `.vcd` for the subsystem being analyzed
- `little_car/test/verilog/<test-name>.v` for signal meaning and stimulus timing

The manifest hash proves which compact wave was analyzed. Use the log as the
parameter index and the VCD for edge/order verification; do not infer physical
sensor accuracy or motor mechanics from RTL simulation alone.
