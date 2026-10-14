# C5 route and manual obstacle-control experiment

## Verdict

Final result: **PASS**

Manual movement, per-wheel trim, directional obstacle handling, stop behavior,
and automatic forward control all match the intended controller behavior. The
complete 8,936-byte VCD is committed with the raw Robei CSV.

## Provenance

- Date: 2026-07-12 (Asia/Shanghai)
- Git implementation: `eceda6b`
- Simulator: RobeiEDA bundled Icarus Verilog
- Timescale: 1 ps
- Simulation end time: 0.900 us
- Original VCD SHA-256: `71f75daff5bbadd6a4c9bbea2b3de3f73c146dd14dd1b4d89a0994d564ae7198`
- Console verdict: `RESULT TEST_PASS c5_route`

## Input calibration values

| Wheel | Default speed |
| --- | ---: |
| LF | 650 |
| RF | 750 |
| LR | 800 |
| RR | 625 |

Overall speed and calibration base are both 700, so forward/reverse commands
preserve these four trim values directly.

## Results

| Case | LF/RF/LR/RR signed speeds | Blocked | Verdict |
| --- | --- | ---: | --- |
| Manual forward | 650/750/800/625 | 0 | Pass |
| Forward with obstacle | 0/0/0/0 | 1 | Pass |
| Reverse escape | -650/-750/-800/-625 | 0 | Pass |
| Spin-left escape | -600/700/-750/575 | 0 | Pass |
| Spin-right escape | 600/-700/750/-575 | 0 | Pass |
| Manual stop | 0/0/0/0 | 0 | Pass |
| Automatic forward | 650/750/800/625 | 0 | Pass |

## Behavioral audit

- An obstacle blocks manual forward and asserts `motor_blocked`.
- Reverse and in-place turns remain available while an obstacle is active, so
  the vehicle can escape instead of being locked in place.
- `motor_blocked` clears when the requested direction is permitted.
- Manual stop drives all four signed speed outputs to zero.
- Automatic forward uses the same four-wheel trim values as manual forward.
- Turn magnitude applies the configured 50-unit reduction independently after
  wheel trim: 650/750/800/625 becomes 600/700/750/575.

The VCD shows each command/obstacle input transition followed by the expected
four speed outputs and block flag, with no intermediate unintended motion state.

## Scope

This test validates `c5_route_controller`. The 300 ms manual-command watchdog
is implemented in `bt_route_uart` and is outside this module's scope; it is not
claimed as verified by this run.

## Console result

```text
RESULT manual_forward_trimmed speeds=650,750,800,625 blocked=0
RESULT forward_blocked speeds=0,0,0,0 blocked=1
RESULT reverse_escape speeds=-650,-750,-800,-625 blocked=0
RESULT left_escape speeds=-600,700,-750,575 blocked=0
RESULT right_escape speeds=600,-700,750,-575 blocked=0
RESULT manual_stop speeds=0,0,0,0 blocked=0
RESULT automatic_forward speeds=650,750,800,625 blocked=0
RESULT TEST_PASS c5_route
```
