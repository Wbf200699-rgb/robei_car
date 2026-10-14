# C5 four-wheel pulse mapping experiment

## Verdict

Final result: **PASS**

All requested signed-speed cases produced the expected four motor pulse widths.
The VCD UART waveform independently decoded into three valid startup stop frames
and five valid 62-byte movement frames.

## Provenance

- Date: 2026-07-12 (Asia/Shanghai)
- Git implementation: `938b646`
- Simulator: RobeiEDA bundled Icarus Verilog
- Timescale: 1 ps
- Original VCD size: 70,411,587 bytes
- Original VCD SHA-256: `766a3961250bc8de19a44d3f24c38d87cc0fdcd7cb29e43bb5963a8477d887aa`
- Selected VCD transitions retained: 1,845
- UART bytes decoded from VCD: 337

The full VCD is not committed. Its hash and size, selected signal transitions,
raw Robei results, and decoded UART frames are retained in this directory.

## Mapping rules

| Wheel side | Pulse formula |
| --- | --- |
| Left front/rear | `1500 + signed_speed` |
| Right front/rear | `1500 - signed_speed` |

The implemented input range is -1000 to +1000, corresponding to 500-2500 us.
All measured values are inside that valid range.

## Results

| Case | Signed speeds LF/RF/LR/RR | Pulses LF/RF/LR/RR (us) | Verdict |
| --- | --- | --- | --- |
| Forward 700 | +700/+700/+700/+700 | 2200/800/2200/800 | Pass |
| Reverse 700 | -700/-700/-700/-700 | 800/2200/800/2200 | Pass |
| Spin left 650 | -650/+650/-650/+650 | 850/850/850/850 | Pass |
| Spin right 650 | +650/-650/+650/-650 | 2150/2150/2150/2150 | Pass |
| Trim sample | +650/+750/+800/+625 | 2150/750/2300/875 | Pass |

The trim sample is mathematically consistent and valid. It should not yet be
treated as final mechanical calibration: final wheel defaults require a physical
straight-line and rotation test on the assembled chassis.

## UART audit

- Frames 1-3: `#255PDST!`
- Frame 4: forward 700, 62 bytes
- Frame 5: reverse 700, 62 bytes
- Frame 6: spin left 650, 62 bytes
- Frame 7: spin right 650, 62 bytes
- Frame 8: wheel trim sample, 62 bytes

Every movement frame starts with `{#006`, includes IDs 006-009 exactly once,
uses `T0000!`, and ends with `}`. Full frame text is retained in
`decoded_uart_frames.txt`.

## Console result

```text
RESULT forward_700 pulses=2200,800,2200,800
RESULT reverse_700 pulses=800,2200,800,2200
RESULT spin_left_650 pulses=850,850,850,850
RESULT spin_right_650 pulses=2150,2150,2150,2150
RESULT wheel_trim_sample pulses=2150,750,2300,875
RESULT TEST_PASS c5_mapping
```
