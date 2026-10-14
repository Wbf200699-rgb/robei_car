# C5 motor timing corrected experiment

## Verdict

Final result: **PASS**

This is the RobeiEDA rerun after correcting the first movement-frame start byte.
The strengthened test checks UART frame boundary bytes and the total byte count,
in addition to timing and pulse values.

## Provenance

- Date: 2026-07-12 (Asia/Shanghai)
- Git implementation: `d88bc09`
- Simulator: RobeiEDA bundled Icarus Verilog
- Timescale: 1 ps
- Original VCD size: 2,491,290,755 bytes
- Original VCD SHA-256: `88ee5cef99c5029a2f17328aca6aff01ba483381680a6256d32443465c3e807a`
- Extracted selected transitions: 1,053
- UART bytes checked: 178

The full VCD is not committed because it is 2.49 GB. The source hash and size,
raw Robei CSV, and compact selected-signal extraction are committed alongside
this report.

## Measurements

| Measurement | Result | Verdict |
| --- | ---: | --- |
| Startup wait to first stop frame | 500.000010 ms | Pass |
| Three startup stop frames complete | 502.345730 ms | Pass |
| One 9-byte stop frame | 0.781560 ms | Pass |
| One 62-byte movement frame | 5.384080 ms | Pass |
| Movement refresh start interval | 105.384080 ms | Pass |
| Frames transmitted during 250 ms idle | 0 | Pass |
| Forward LF/RF/LR/RR pulses | 2200/800/2200/800 us | Pass |

## UART audit

Frames 1-3 and 6-8 all decode as:

```text
#255PDST!
```

Both movement frames decode identically and correctly:

```text
{#006P2200T0000!#007P0800T0000!#008P2200T0000!#009P0800T0000!}
```

The malformed `##006...}` frame recorded in the earlier defect-finding run is
absent. The first movement frame now has the required opening brace, and the
periodic movement refresh is byte-for-byte identical.

## Console result

```text
RESULT startup_wait_ns=500000010
RESULT startup_stop_complete_ns=502345730
RESULT forward_pulses=2200,800,2200,800
RESULT movement_refresh_start_interval_ns=105384080
RESULT idle_frame_delta_250ms=0
RESULT TEST_PASS c5_timing
```
