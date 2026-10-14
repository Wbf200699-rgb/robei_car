# Car parameter controller experiment

## Verdict

Final result: **PASS**

The default values, parameter selection, adjustment increments, and tested
saturation limits all match the RTL specification. The complete raw VCD is
committed because it is only 23,488 bytes.

## Provenance

- Date: 2026-07-12 (Asia/Shanghai)
- Git implementation: `b656992`
- Simulator: RobeiEDA bundled Icarus Verilog
- Timescale: 1 ps
- Simulation end time: 7.020 us
- Original VCD SHA-256: `76d71b8d3e8e66d3b63d8f2f110403205b4cad378066812b642b2c94a19a93bd`
- Console verdict: `RESULT TEST_PASS car_parameters`

## Default values

| Parameter | Measured default | Verdict |
| --- | ---: | --- |
| Overall speed | 700 | Pass |
| Step length | 20 | Pass |
| LF speed | 700 | Pass |
| RF speed | 700 | Pass |
| LR speed | 700 | Pass |
| RR speed | 700 | Pass |
| Lidar safe distance | 450 mm | Pass |

## Adjustment behavior

| Operation | Before | After | Increment | Verdict |
| --- | ---: | ---: | ---: | --- |
| Overall speed increase | 700 | 725 | +25 | Pass |
| Step length increase | 20 | 25 | +5 | Pass |
| LF speed decrease | 700 | 675 | -25 | Pass |
| RF speed increase | 700 | 725 | +25 | Pass |
| LR speed increase twice | 700 | 750 | +50 | Pass |
| RR speed decrease twice | 700 | 650 | -50 | Pass |
| Lidar distance increase | 450 | 500 | +50 mm | Pass |

The VCD shows that each select pulse changes `selected_param` before the adjust
pulse, and only the selected output changes. No cross-parameter update was
observed.

## Saturation limits

| Parameter | Minimum | Maximum | Step | Verdict |
| --- | ---: | ---: | ---: | --- |
| Speed | 100 | 900 | 25 | Pass |
| Lidar safe distance | 150 mm | 1500 mm | 50 mm | Pass |

The wheel speed controls share the same `inc_speed` and `dec_speed` saturation
functions as overall speed. This run directly exercises the overall speed
limits and individual wheel increments. Step-length increment is directly
tested; its 5-80 saturation endpoints are defined by RTL but are not swept in
this test run.

## Calibration status

These results validate digital parameter storage and adjustment behavior. They
do not establish final physical wheel calibration. The LF/RF/LR/RR values must
still be finalized from chassis straight-line and rotation measurements.

## Console result

```text
RESULT defaults overall=700 step=20 lf=700 rf=700 lr=700 rr=700 lidar=450
RESULT overall_plus_25 overall=725 step=20 lf=700 rf=700 lr=700 rr=700 lidar=450
RESULT step_plus_5 overall=725 step=25 lf=700 rf=700 lr=700 rr=700 lidar=450
RESULT lf_minus_25 overall=725 step=25 lf=675 rf=700 lr=700 rr=700 lidar=450
RESULT rf_plus_25 overall=725 step=25 lf=675 rf=725 lr=700 rr=700 lidar=450
RESULT lr_plus_50 overall=725 step=25 lf=675 rf=725 lr=750 rr=700 lidar=450
RESULT rr_minus_50 overall=725 step=25 lf=675 rf=725 lr=750 rr=650 lidar=450
RESULT lidar_plus_50 overall=725 step=25 lf=675 rf=725 lr=750 rr=650 lidar=500
RESULT speed_max overall=900 step=25 lf=675 rf=725 lr=750 rr=650 lidar=500
RESULT speed_min overall=100 step=25 lf=675 rf=725 lr=750 rr=650 lidar=500
RESULT lidar_max overall=100 step=25 lf=675 rf=725 lr=750 rr=650 lidar=1500
RESULT lidar_min overall=100 step=25 lf=675 rf=725 lr=750 rr=650 lidar=150
RESULT TEST_PASS car_parameters
```
