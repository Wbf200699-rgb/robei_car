# Robei parameter experiments

Open and simulate these files in RobeiEDA:

- `c5_motor_frame_driver_robei_test.test`: real 50 MHz/115200 timing, startup
  protection, UART bytes, movement refresh, stop burst, and idle silence.
- `c5_motor_mapping_robei_test.test`: forward/reverse/turn/trim pulse mapping.
- `car_param_controller_robei_test.test`: defaults, increments, and limits.
- `c5_route_controller_robei_test.test`: manual/automatic wheel commands and
  obstacle escape behavior.

Run one test at a time until it prints `RESULT TEST_PASS ...` or
`RESULT TEST_FAIL ...`. Each test writes a CSV in RobeiEDA's simulation working
directory:

| Test | CSV |
| --- | --- |
| C5 timing | `c5_timing_results.csv` |
| C5 mapping | `c5_mapping_results.csv` |
| Parameters | `car_parameter_results.csv` |
| Route controller | `c5_route_results.csv` |

After simulation, search under the project and Robei installation directories:

```powershell
Get-ChildItem D:\Projects\Robei\little_car_new,D:\Apps\RobeiEDA -Recurse `
  -Include c5_timing_results.csv,c5_mapping_results.csv,car_parameter_results.csv,c5_route_results.csv `
  -ErrorAction SilentlyContinue
```

Leave the CSV files in the workspace or provide the Robei console log. The
results can then be converted into a dated evidence report containing the raw
measurements, derived timing values, pass/fail decisions, tool version, Git
commit, and the final calibrated constants.

The `verilog` directory contains equivalent standalone testbenches for
command-line Icarus Verilog checks. Regenerate all files with:

```powershell
python tools/generate_robei_experiments.py
```

## Compact hardware-parameter suite

Nine additional Robei tests cover reset, HDMI timing, ultrasonic ranging,
closed-loop C5 correction, dual-IMU calibration, RPLIDAR startup/decoding,
72-bin scan compression, autonomous exploration safety, and LinkPG UWB UART.
Their filenames end in `_parameter.test` and they are registered in both
`little_car.proj` and the standard `robei_model_project.proj`.

Run all standalone mirrors, create compact VCD files, and enforce the 8 MiB
per-wave limit with:

```powershell
python tools/run_hardware_parameter_tests.py
```

Detailed metrics and interpretation are in
`docs/HARDWARE_PARAMETER_SIMULATION.md`. Logs, waves, hashes, and sizes are in
`little_car/test/results/hardware_parameters/`.
