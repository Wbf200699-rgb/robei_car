# Current Project Scope

This file defines the feature boundary for documentation generated from this
repository.

## Five VL53L1X Channels

The five VL53L1X laser-ranging modules are **not used by the current vehicle**.
Their active I2C controller, ranging state machine, fusion logic, and dedicated
tests have been removed from the current project. A constant inactive wrapper
keeps the existing top-level pins and Bluetooth frame layout compatible: every
channel reports offline/invalid and every distance reports `FFFF`.

Documentation for the current version must therefore:

- omit five-channel VL53L1X ranging from the implemented-feature list;
- omit it from current wiring, sensing, obstacle-avoidance, mapping, scoring,
  and validation claims;
- not present `@V` frames as active runtime telemetry;
- describe `vl53l1x_array5` only as an inactive protocol-compatibility wrapper
  when its existence must be discussed.

The active obstacle-sensing description should cover RPLIDAR C1 and ultrasonic
sensing. The Bluetooth frame shape and top-level constraint interface remain
unchanged by this retirement.
