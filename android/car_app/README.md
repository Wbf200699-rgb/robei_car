# Little Car Android controller

> Public portfolio snapshot: the UWB default hostname is an example placeholder.
> Configure your own endpoint and token before using UWB. Current active obstacle
> sensing uses RPLIDAR C1 and ultrasonic sensors; VL53 fields are compatibility only.

The app controls the vehicle only through the onboard CH9141K BLE transparent
UART service:

- service: `FFF0`
- FPGA-to-phone notifications: `FFF1`
- phone-to-FPGA writes: `FFF2`

The shared parser handles `@S`, `@L`, `@V`, `@R`, `@K`, `@PONG`, the five
indexed VL53L1X readings, and the six-part 72-point lidar map frames. The app retains manual drive, route editing,
parameter adjustment, radar display, local motion calibration, and occupancy
map views.

The reusable calibration and mapping reference core is under:

- `geometry/VehicleGeometry.java`: generated project geometry;
- `localization/UwbRigidBodyEstimator.java`: four-tag rigid fit;
- `localization/UwbWindowedPoseEstimator.java`: stationary median filtering;
- `calibration/MotionCalibrationEngine.java`: robust motion constants;
- `mapping/OccupancyGridMapper.java`: 25 mm log-odds mapping and local scan matching.

The live mapping view uses the same `241 x 241` grid and `25 mm/cell`
resolution, so replay results and the on-phone display use one spatial scale.

UWB position data is supplied by the ESP32-S3/server pipeline. It supports the
map and localization views but remains separate from the BLE vehicle command
and emergency-stop path.

Debug APKs also run `VehicleAlgorithmSelfCheck` inside the Android process at
startup. The result is written to the app log and to logcat with tag
`VehicleAlgorithm`; it replays the shared four-tag geometry, robust motion
calibration, 25 mm occupancy mapping, and local scan matching.

UWB is not a phone-to-vehicle command channel. Server layout changes clear the
previous coordinate-frame state before the app accepts new fixes.

## Selecting the UWB server

Open **设备 → UWB 服务器** and choose one route explicitly:

- **公网服务器（默认）** uses
  `wss://relay.example.invalid/ws/mobile`.
- **局域网电脑** accepts a private IPv4 or Tailscale address and a port
  (normally `18765`), then connects to
  `ws://<IP>:<port>/ws/mobile`.

The choice is saved and takes effect immediately. The app does not
automatically fail over between routes. Before choosing LAN mode, start the
portable relay on the computer and make sure the phone and S3 can reach it.

## Build

```powershell
.\gradlew.bat testDebugUnitTest assembleDebug
```

Debug builds use package `car.car_app.debug`, so they can remain installed
beside the older signed `car.car_app` package.

## On-device checks

The connection page distinguishes BLE connection from live FPGA data. A usable
session requires all of the following:

1. CH9141K GATT connected and services discovered;
2. FFF1 notifications enabled and FFF2 writable;
3. `@PONG` remaining fresh;
4. active `@S` and `@L` telemetry continuing to update;
5. complete lidar map frames passing sequence, part, and CRC checks.

Do not require inactive VL53L1X channels or `@V` to become valid. Build prerequisites,
reproduction and validation results are documented in `../../docs/GETTING_STARTED.md`
and `../../docs/VALIDATION.md`.
