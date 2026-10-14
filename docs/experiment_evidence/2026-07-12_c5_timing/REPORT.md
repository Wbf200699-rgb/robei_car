# C5 motor timing experiment evidence

## Provenance

- Date: 2026-07-12 (Asia/Shanghai)
- Git baseline under test: `c80dc4d`
- Simulator: RobeiEDA bundled Icarus Verilog
- Timescale: 1 ps
- Original VCD size: 2,491,289,433 bytes
- Original VCD SHA-256: see `source_vcd_metadata.txt`
- Compact VCD extraction: `c5_timing_key_transitions.csv`
- Robei test output: `c5_timing_robei_raw.csv`

The full VCD is intentionally not committed because it is 2.49 GB. Its hash,
size, raw Robei event log, and 1,053 selected signal transitions are retained so
the evidence remains traceable.

## Measured results

| Measurement | Result |
| --- | ---: |
| Startup wait to first stop frame | 500.000010 ms |
| Three startup stop frames complete | 502.345730 ms |
| One 9-byte broadcast stop frame | 0.781560 ms |
| One 62-byte movement frame | 5.384080 ms |
| Movement frame start-to-start refresh | 105.384080 ms |
| Idle frames during 250 ms | 0 |
| Forward pulses LF/RF/LR/RR | 2200/800/2200/800 us |
| UART bytes captured | 178 |

All three startup and all three explicit stop frames decode as:

```text
#255PDST!
```

The second movement frame decodes correctly:

```text
{#006P2200T0000!#007P0800T0000!#008P2200T0000!#009P0800T0000!}
```

## Audit finding

Robei printed `RESULT TEST_PASS c5_timing`, but independent byte reconstruction
from its raw UART log found that the first movement frame after the startup stop
burst was:

```text
##006P2200T0000!#007P0800T0000!#008P2200T0000!#009P0800T0000!}
```

Its first byte was `#` instead of `{`. The cause was the first-byte expression
reading the previous stop frame's `frame_is_stop` value in the same clock cycle
that a movement frame started. This run is therefore retained as a defect-finding
run, not as final passing evidence.

The RTL was corrected to select `{` directly when starting a movement frame,
and the Robei test was strengthened to check UART frame boundary bytes and the
total byte count. A corrected Robei rerun is required before this experiment is
marked final pass.
