# Dual BMI088 Recovery and C5 Self-Calibration Evidence

Date: 2026-08-10

## Scope

This record covers the dual-BMI088 startup/runtime recovery fix, static board
calibration, and the first closed-loop C5 yaw self-calibration test. No wheel
command was issued during the hardware sensor capture. The diagnostic image was
loaded to FPGA SRAM only, and the normal main bitstream was restored afterward.

## Hardware Result

Final capture files:

- `little_car/build/motion_sensor_capture/dual_imu_signed_mean_fixed_20260810_232148.log`
- `little_car/build/motion_sensor_capture/dual_imu_signed_mean_fixed_20260810_232148.csv`
- `little_car/build/motion_sensor_capture/dual_imu_signed_mean_fixed_20260810_232148.json`

Observed over 273 decoded frames:

| Item | Result |
| --- | --- |
| UART sequence gaps | 0 |
| Runtime sensor status | `9F` throughout; both BMI088 devices online |
| Static calibration | `FF`; done and passed |
| Dominant gravity axes | BMI0 `-Z`, BMI1 `-Z` |
| BMI0 accel mean | `(155.352, 157.059, -5506.077)` |
| BMI1 accel mean | `(451.758, 498.304, -5318.223)` |
| BMI0 gyro mean | `(1, 1, 0)` internal calibration result |
| BMI1 gyro mean | `(-1, 3, 2)` internal calibration result |
| BMI0/BMI1 gyro peak | `32 / 33` counts |

Both devices are now mounted with the same vertical axis. The acceleration
vector implies about 2.3 degrees residual tilt for BMI0 and 7.2 degrees for
BMI1. This is acceptable for yaw-rate feedback after bias calibration, but a
more level mechanical mount will improve later pose estimation.

## Recovery Fixes

- Retry the first BMI088 accelerometer chip-ID check after the SPI mode switch.
- Treat 32 consecutive implausible samples as a runtime dropout.
- Recover after 8 consecutive plausible live samples without depending on a
  stale startup chip-ID cache.
- Accumulate signed 16-bit samples with explicit 32-bit sign extension. The
  earlier zero extension produced false large gyro means whenever a sample was
  negative.
- Automatically restart static calibration after a failed stationary window.

The static calibrator uses 256 samples in production and shift-only averaging.
It does not instantiate an IP core, multiplier, DSP, or BRAM.

## Closed-Loop C5 Test

Testbench:

`little_car/tb/tb_c5_self_calibration_closed_loop.v`

The deterministic chassis model feeds the DUT wheel outputs back as the next
gyro-Z sample. A fixed rightward chassis bias is therefore corrected by the
actual controller, instead of replaying a precomputed sensor trace.

Source RTL and model-generated RTL produced the same result:

```text
RESULT baseline_yaw_abs=320
RESULT calibrated_yaw_abs_avg=16 max=16
RESULT learned_yaw_trim=-12 final_correction=-19 gyro_peak=5600
RESULT TEST_PASS c5_self_calibration_closed_loop
```

The average simulated yaw error fell from 320 to 16 counts, a 20x reduction.
The selected conservative fixed-point gains are proportional `1/32`, integral
`1/128`, derivative `1/128`, with a correction limit of 96 speed units and an
8-unit-per-sample slew limit. These operations use shifts and additions only.

The current onboard calibration learns gyro-Z polarity and a persistent
left/right yaw trim. It does not yet identify distance scale, absolute velocity,
or four independent wheel gains. Those require an external displacement
reference such as reliable UWB position, optical flow, or accessible wheel
encoder feedback.

## Integration Checks

- Robei source models were regenerated for `motion_sensor_subsystem`,
  `dual_imu_static_calibrator`, and `c5_motion_stabilizer`.
- The embedded code in `sys_route_gui_fsm_top.model` was synchronized.
- Robei converter regression: 6 of 6 tests passed.
- Source RTL and model-generated RTL static-calibrator tests passed.
- Source RTL and model-generated RTL closed-loop C5 tests passed.
- Vivado 2025.2 top-level synthesis read 41 Verilog files and completed with
  0 errors and 0 critical warnings.
- Synthesized top utilization: 22,209 LUTs (68.13%), 10,006 registers (15.35%),
  3 BRAM tiles (4.00%), and 9 inferred DSP primitives (7.50%). No `.xci`, `.bd`,
  or prebuilt `.dcp` was read. The new `motion_sensor_subsystem` and
  `c5_motion_stabilizer` hierarchies each report zero DSP use.
- Pre-route setup WNS is +1.445 ns. The synthesis-only report still has
  cross-clock hold violations (WHS -0.194 ns); implementation/route is required
  before timing closure can be claimed.

This run intentionally stopped after synthesis. Native Robei placement/routing
and a new production bitstream were not generated, so this evidence is not yet
an on-vehicle motion-calibration result.
