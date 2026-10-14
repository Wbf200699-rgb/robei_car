# PMW3901 + dual BMI088 bring-up

## Scope

This is a standalone sensor-validation image. It does not drive the C5 chassis,
change the Bluetooth protocol, or modify the HDMI/lidar control path. The first
hardware pass is deliberately separated from motor control so that sensor IDs,
axis directions, scale, mounting quality, noise, and dual-IMU agreement can be
measured before these values are used in a closed loop.

Canonical Robei modules:

- `little_car/model/motion_sensor_subsystem.model`
- `little_car/model/motion_sensor_bringup_top.model`

Generated/working Verilog:

- `little_car/build/motion_sensor_subsystem.v`
- `little_car/build/motion_sensor_bringup_top.v`

Standalone bitstream:

- `little_car/build/motion_sensor_bringup/motion_sensor_bringup_top.bit`
- Current 2026-08-04 17:37 build SHA-256:
  `2275590B9E9936F1CC86A7C31814D20C6B9858ADF1C55B576F167A1AFAB0F831`

## Electrical rules

- All FPGA I/O in this design is `LVCMOS33`. Sensor logic levels must be 3.3 V.
- Power the board off before adding or moving Dupont wires.
- The workbook names H3-13 as `VCC` but does not state its voltage. Measure
  H3-13 to H3-14 first and only use it when the reading is approximately 3.3 V.
- All three sensor boards and the FPGA must share ground.
- Keep SCLK/MOSI/MISO short. Start with wires below about 15 cm.
- Never connect a 5 V sensor output directly to an FPGA pin.

## H3 wiring

Locate H3 pin 1 from the board silkscreen. Do not infer connector orientation
from a rotated photograph.

| H3 pin | FPGA ball | Signal | Connect to |
|---:|---|---|---|
| 1 | A14 | shared SCLK | PMW3901 SCLK; both BMI088 SCK pins |
| 2 | B15 | shared MOSI | PMW3901 MOSI/SDI; both BMI088 SDI pins |
| 3 | A13 | shared MISO | PMW3901 MISO/SDO; every BMI088 SDO1 and SDO2 |
| 4 | B13 | PMW3901 CS, active low | PMW3901 CS/NCS |
| 5 | F20 | BMI088 #0 accelerometer CS | BMI #0 CSB1/ACC_CS |
| 8 | F19 | BMI088 #0 gyroscope CS | BMI #0 CSB2/GYRO_CS |
| 9 | C20 | BMI088 #1 accelerometer CS | BMI #1 CSB1/ACC_CS |
| 12 | E19 | BMI088 #1 gyroscope CS | BMI #1 CSB2/GYRO_CS |
| 13 | VCC | sensor 3.3 V | All sensor VCC/3V3 pins, after meter check |
| 14 | GND | common ground | All sensor GND pins |

BMI088 breakout labels differ between vendors. A board exposing separate
accelerometer and gyroscope SPI ports has two chip selects and often two SDO
pins. Join both SDO pins to H3-3; only the selected internal device drives the
line. If the board has `PS`, `PS1`, `PS2`, or an I2C/SPI solder jumper, select
SPI according to that board's schematic. Leave all interrupt pins unconnected
for this test.

For the PMW3901 board, leave `INT` unconnected. The provided CJMCU-3901 archive
indicates that `EN` is pulled up on the module; leave it open initially. The
optical lens faces the ground and must have an unobstructed view.

Mount both BMI088 boards rigidly, close to the chassis center, with matching X,
Y, and Z axis directions. A mirrored or 90-degree rotated board will correctly
trigger the disagreement flag even though both device IDs are valid.

## Implemented behavior

- One shared SPI mode-3 bus at about 1.923 MHz and five independent chip selects.
- PMW3901 soft reset, product-ID check (`49/B6`), reference initialization table,
  retry, and approximately 100 Hz delta-X/delta-Y/SQUAL sampling.
- Two BMI088 instances with independent accelerometer/gyroscope ID checks
  (`1E/0F` for each board).
- BMI accelerometers: 200 Hz OSR4, +/-6 g.
- BMI gyroscopes: normal mode, 200 Hz/23 Hz bandwidth, +/-250 degrees/s.
- 512-sample stationary startup gyro-bias calibration, about 2.5 to 2.7 seconds.
- Dual-IMU average when both agree, single-IMU fallback when one fails, and an
  explicit disagreement status when two valid sensors diverge beyond limits.
- No inferred motion command and no motor output in this validation image.

The three board LEDs indicate PMW3901, BMI088 #0, and BMI088 #1 ID/online state.

## UART telemetry

The FPGA transmits on package pin J1 into the onboard USB-UART RX path at
115200 bit/s, 8 data bits, no parity, and 1 stop bit. One 99-byte ASCII line is
emitted every 100 ms:

```text
@Mssss,tt,qq,pp,ii,a0,g0,a1,g1,dddd,dddd,xxxxxxxx,xxxxxxxx,aaaa,aaaa,aaaa,gggg,gggg,gggg,uuuu,vvvv\n
```

| Field | Width | Meaning |
|---|---:|---|
| `ssss` | 16 bit | sample sequence |
| `tt` | 8 bit | status flags |
| `qq` | 8 bit | PMW3901 SQUAL |
| `pp,ii` | 8 bit each | PMW3901 product and inverse IDs |
| `a0,g0,a1,g1` | 8 bit each | BMI #0/#1 accelerometer and gyro IDs |
| first two `dddd` | signed 16 bit | current optical-flow delta X/Y |
| two `xxxxxxxx` | signed 32 bit | accumulated optical-flow X/Y |
| three `aaaa` | signed 16 bit | fused accelerometer X/Y/Z raw counts |
| three `gggg` | signed 16 bit | fused, bias-corrected gyro X/Y/Z raw counts |
| `uuuu,vvvv` | signed 16 bit | raw gyro-Z from BMI #0 and BMI #1 |

All values are big-endian hexadecimal text. Signed fields use two's complement.
Status bits are:

| Bit | Meaning when 1 |
|---:|---|
| 0 | PMW3901 online |
| 1 | BMI088 #0 online |
| 2 | BMI088 #1 online |
| 3 | gyro startup calibration complete |
| 4 | fused IMU data available |
| 5 | IMU degraded to one board |
| 6 | two valid IMUs disagree |
| 7 | at least one optical-flow sample received |

Expected ID segment after initialization is `49,B6,1E,0F,1E,0F`.

## Build and capture

Build with:

```powershell
.\tools\vivado_build_motion_sensor.bat
```

The batch file first regenerates `power_on_reset.v`,
`motion_sensor_subsystem.v`, and `motion_sensor_bringup_top.v` from their Robei
models, then runs Vivado. This keeps the Robei model as the build source of
truth instead of relying on an old ignored file under `build/`.

After programming the standalone bitstream, capture for 20 seconds with:

```powershell
.\tools\read_motion_sensor_uart.ps1 -Seconds 20
```

The script probes FT4232H virtual COM ports for `@M` frames. A specific port can
be forced with `-PortName COMxx`. It writes the exact serial stream and decoded
CSV to `little_car/build/motion_sensor_capture/`. These files are the evidence
used for axis, stationary-noise, gyro-bias, disagreement, and optical scale
analysis.

## Verification completed before wiring

Behavioral simulation, direct Verilog compilation, and Robei model-to-Verilog
round-trip simulation all produced:

```text
RESULT ids=49/b6,1e/0f,1e/0f
RESULT fused_accel=110,-190,5510 fused_gyro=300,100,300
RESULT flow=32,-8 accum=96,-40 squal=45
RESULT TEST_PASS motion_sensor_subsystem
```

Vivado 2025.2 implementation for XC7A50T-FGG484-1 completed fully routed with:

- 2196 LUTs (6.74%)
- 1398 registers (2.14%)
- 0 BRAM and 0 DSP
- setup WNS +7.447 ns; no failing setup or hold endpoints
- one nonfunctional DRC warning: board configuration voltage properties are not
  described in the available constraints

## Hardware validation completed

The standalone image was programmed through JTAG on 2026-08-04. COM20 produced
185 consecutive valid frames over 20 seconds with no sequence gap:

```text
PMW3901 IDs       49/B6
BMI088 #0 IDs     1E/0F
BMI088 #1 IDs     1E/0F
status            0x9F throughout
SQUAL             min 64, mean 72.405, max 80
last 50 gyro XYZ  mean -1.42 / 3.44 / 2.36 raw counts
last 50 gyro XYZ  std  4.754 / 9.267 / 4.902 raw counts
last 50 accel XYZ mean 189.06 / 56.36 / -5415.10 raw counts
sequence          238 through 422, zero gaps
```

`0x9F` confirms all three devices online, calibration complete, fused data
available, no single-IMU fallback, no dual-IMU disagreement, and at least one
optical-flow sample. Flow deltas remained zero because the chassis was not moved;
therefore optical scale, axis signs, rotation response, and installed vibration
are not yet calibrated. The retained evidence and interpretation are documented
in `docs/MOTION_SENSOR_VALIDATION_2026-08-04.md`.

## Source basis

- Bosch BMI088 datasheet:
  `https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf`
- Bosch BMI08x reference API, inspected commit
  `c1ed227e7bb7da1fa600bbd4e5c82d0da1eb416a`:
  `https://github.com/boschsensortec/BMI08x_SensorAPI`
- Bitcraze PMW3901 reference driver, inspected commit
  `d322d98d6f61757352d11d922cd194539e165231`:
  `https://github.com/bitcraze/Bitcraze_PMW3901`
- User-provided CJMCU-3901 archive SHA-256:
  `A1986EC87FB0FDD21FB139CE54525E1262A1EFC3C11A5CF1657340EA0871A812`
